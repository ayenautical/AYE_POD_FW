// =========================================================================
// AYE POD — AYE_POD_v37_DEV.ino  (file principale)
// Release: V2.0.7 — Fix setTimeout(8s corr.) + Stack2 PRIMA di Stack1
// Changelog V2.0.0:
//   - Versionamento SemVer (MAJOR.MINOR.PATCH) sostituisce schema V45.x.y
//     Adottato per supportare confronto numerico nella logica OTA futura
//   - Partition scheme: passaggio a custom 2-slot (no SPIFFS)
//     app0/app1 da ~1.99MB ciascuno (era partizione singola 2.75MB No-OTA)
//     Verificato: firmware non usa SPIFFS/FFat/LittleFS — nessun impatto funzionale
//     Margine attuale: 1.209.043 byte sketch / 2.080.768 byte slot = 42% libero
//   - VMG Wind: clamp rimosso (ereditato da V45.4.10) — invariato in questa release
//   - NESSUNA logica di download/installazione OTA in questa versione
//     Questa release prepara solo lo schema partizione (Fase 1).
//     Il client OTA (download, verify, install) sarà introdotto in Fase 2.
//
// Storico versioni precedenti (schema V45.x.y, ora migrato a SemVer):
// Hardware: Adafruit ESP32-S3 Feather 2MB PSRAM
//           BNO085 I2C 0x4A + GPS PA1616S I2C 0x10 + NeoPixel pin33
//           Anemometro NMEA 0183 @ 4800 baud (RX=38, TX=39)
//
// ARCHITETTURA DEFINITIVA — storia completa:
//
//   V44.4.0  GPS.read() 1x/ciclo + delay(100) → SOG latenza ~60s
//   V45.0.0  TaskGPS Core1 prio2 → race condition Wire con BNO → tutti 0
//   V45.1.0  TaskGPS Core0 + mutex Wire → starvation → GPS age:99999
//   V45.2.0  Loop drain GPS completo → ESP-NOW flood 50Hz → link Visore KO
//   V45.4.6  STABILE: loop 50Hz, GPS incrementale con break, ESP-NOW 10Hz
//
//   REGOLE ARCHITETTURALI (non modificare):
//   1. Wire usato SOLO dal loop() Core1 — mai da task separati
//   2. GPS: pattern Adafruit con break dopo parse (no sovrascrittura lastNMEA)
//   3. ESP-NOW: timer 100ms in inviaDatiVisore() — mai chiamare a >10Hz
//   4. TaskCloud Core0: SOLO HTTP, ZERO Wire
//
//   ┌─ Core 0 ────────────────────────────────────────────────────────────┐
//   │  TaskCloud — HTTP Supabase ogni 2s, ZERO Wire                      │
//   └─────────────────────────────────────────────────────────────────────┘
//   ┌─ Core 1 — loop() ogni 20ms (50Hz) ─────────────────────────────────┐
//   │  leggiBussola()     Wire BNO085 → heading/roll/pitch 50Hz          │
//   │  leggiGPS()         Wire GPS   → SOG/COG/coord ≤120ms latenza      │
//   │  leggiVento()       Serial1    → AWA/AWS 10Hz NMEA                 │
//   │  calcolaVentoVero() CPU only   → TWA/TWS/TWD                       │
//   │  calcolaVMG()       CPU only   → VMG vento                         │
//   │  inviaDatiVisore()  ESP-NOW    → 10Hz (timer 100ms interno)        │
//   └─────────────────────────────────────────────────────────────────────┘
//
// File del progetto V45.4.6 — tutti i file necessari:
//   AYE_POD_v45_3.ino     ← questo file (main)
//   GPS_v45_4.ino         ← leggiGPS() + guardia SOG dual-sensor BNO+GPS
//   Bussola_v45_3.ino     ← BNO085 semplice senza mutex
//   Visore_v45_3.ino      ← ESP-NOW con timer 100ms
//   Rete_Cloud_v45_3.ino  ← TaskCloud Core0, HTTP 2s
//   Utils_v45_3.ino       ← LED + telemetria
//   Vento.ino             ← INVARIATO (Serial1, no Wire)
//   AYE_Visore_V45_3.ino  ← INVARIATO (no smooth, raw values)
// =========================================================================

#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_MAX1704X.h>
#include <Adafruit_NeoPixel.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <math.h>

// ── Versionamento Semantico (SemVer) — V2.0.0 introduce schema OTA ────────
// Formato: MAJOR.MINOR.PATCH
//   MAJOR: breaking change struct ESP-NOW o partition scheme (raro, manuale)
//   MINOR: nuove funzionalità retrocompatibili (es. Anchor FASE 4)
//   PATCH: bugfix, tuning, nessuna nuova funzione (es. fix VMG)
#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 2
#define FW_VERSION_PATCH 6
#define FW_VERSION "2.2.6"

#ifndef PIN_NEOPIXEL
  #define PIN_NEOPIXEL 33
#endif

#define SWAP_ROLL_PITCH  true
#define INVERT_ROLL      false
#define INVERT_PITCH     false
#define INVERT_HEADING   true

#define WIND_BAUD_RATE 4800
#define RX_PIN 38
#define TX_PIN 39

String  fwVisoreAttuale = "n.d.";
uint8_t macVisore[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ── Struct ESP-NOW Pod→Visore (82 bytes packed — INVARIATA da V44.4.0) ─────
// NON MODIFICARE senza aggiornare contestualmente AYE_Visore
typedef struct __attribute__((packed)) struct_nautica {
  int   roll, pitch, heading;
  float sog, awa, aws, twa, tws, twd;
  int   batteria_pod;
  bool  cloud_connesso, gps_fix;
  float lat, lon, cog, vmg, vmg_wind;
  char  codice_crew[8];   // PIN crew aggiornato dal DB ogni 30s
  char  fw_str[12];       // es. "2.0.0" (SemVer) — buffer invariato, retrocompatibile
  uint32_t unix_timestamp;  // epoch UTC da NMEA GPS (+4 byte)
  bool     anchor_alert;    // ancora allerta (+1 byte)
} struct_nautica;
// Verifica al boot: sizeof deve essere 87 (era 82)

struct_nautica datiNautici;

// ── Struct Visore→Pod (22 bytes packed — INVARIATA) ───────────────────────
typedef struct __attribute__((packed)) struct_messaggio_visore {
  float batteriaVisore;
  char  fwVisore[10];
  float session_dist;
  int   cmd_sessione;
} struct_messaggio_visore;

struct_messaggio_visore datiDalVisore;

// ── Variabili sensori — scritte da loop(), lette da TaskCloud ─────────────
volatile int    g_roll=0, g_pitch=0, g_head=0, g_raw_head=0, g_acc=0;
volatile float  g_sog=0.0f, g_cog=0.0f;
volatile double g_lat=0.0,  g_lon=0.0;
volatile uint8_t g_sats=0;
volatile float  g_drift=0.0f;
volatile float  g_awa=-1.0f, g_aws=0.0f;
volatile float  g_twa=0.0f,  g_tws=0.0f;
volatile int    g_twd=0;
volatile float  g_vmg=0.0f,  g_vmg_wind=0.0f;
volatile float  g_accel_mag=0.0f;       // BNO085 campione istantaneo (m/s²) — telemetria
volatile float  g_accel_mag_media=0.0f; // BNO085 media 400ms — usata da leggiGPS()
volatile double g_wp_lat=0.0, g_wp_lon=0.0;
volatile int    g_wp_active=0;
// V2.2.3: extern per bloccare loop() durante OTA (evita Cache Miss Panic
// e race condition esp_now_send su Core1 mentre Core0 scrive flash)
extern volatile bool g_otaInCorso;
volatile bool   wifiConnesso=false, calibrazioneRichiesta=false;
volatile bool   autoCalibAttiva=false;
volatile int    remoteOffset=0;

// ── Anchor Alert (V45.4.7) ───────────────────────────────────────────────
volatile double  g_anchor_lat = 0.0;    // lat ancora (WGS84)
volatile double  g_anchor_lon = 0.0;    // lon ancora (WGS84)
volatile float   g_anchor_raggio_m = 0.0f; // raggio allerta in metri
volatile bool    g_anchor_attivo = false;   // ancora calata e allerta ON
volatile bool    g_anchor_alert = false;    // TRUE = barca fuori dal raggio

// ── Timestamp GPS (V45.4.7) ─────────────────────────────────────────────
volatile uint32_t g_unix_timestamp = 0; // secondi epoch UTC da NMEA

float sommaOffset=0.0f;
int   campioniOffset=0;
unsigned long lastWifiAttempt=0;

// ── Hardware ───────────────────────────────────────────────────────────────
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB+NEO_KHZ800);
Adafruit_BNO08x   bno08x;
sh2_SensorValue_t sensorValue;
Adafruit_MAX17048 maxlipo;
Adafruit_GPS      GPS(&Wire);  // GPS su Wire — usato SOLO dal loop() Core1
Preferences       prefsOffset;

// ── Prototipi ──────────────────────────────────────────────────────────────
void impostaLED(uint32_t, bool=false);
void setupUtils();
void setupGPS();
void leggiGPS();
void setupBussola();
void leggiBussola();
void avviaCalibrazioneBNO();
void leggiVento();
void calcolaVentoVero();
void calcolaVMG();
void inviaDatiVisore();
void stampaTelemetria();
void gestisciWiFi();
void setupWiFi();
void setupESPNOW();
void eseguiAutoCalibrazioneGPS();
void TaskInvioCloud(void*);
unsigned long gpsAgeMs();

// =========================================================================
// SETUP
// =========================================================================
void setup() {
  Serial.begin(115200);
  Serial1.begin(WIND_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(500);

  Serial.println("\n================================");
  Serial.printf(" AYE POD %s\n", FW_VERSION);
  Serial.println("================================");

  // Verifica struct prima di toccare qualsiasi hardware
  int sz_nav = (int)sizeof(struct_nautica);
  int sz_vis = (int)sizeof(datiDalVisore);
  Serial.printf("[SYS] struct_nautica=%d bytes (atteso 87)\n", sz_nav);
  if (sz_nav != 87) { Serial.println("[SYS] ERRORE CRITICO: struct size mismatch!"); while(1) delay(1000); }
  Serial.printf("[SYS] datiDalVisore=%d bytes (atteso 22)\n",  sz_vis);
  if (sz_nav != 82 || sz_vis != 22) {
    Serial.println("[ERRORE CRITICO] Struct ESP-NOW non allineata — il Visore ricevera' spazzatura!");
  }

  setupUtils();          // Wire.begin() + NeoPixel + MAX17048
  Wire.setClock(400000); // I2C Fast Mode: riduce tempo drain GPS da ~5ms a ~1ms
  Serial.println("[SYS] I2C @ 400kHz (Fast Mode)");

  setupBussola();        // BNO085 @ 0x4A — nessun mutex, solo Core1
  setupGPS();            // GPS @ 0x10 — nessun task, legge nel loop()
  setupWiFi();
  setupESPNOW();

  prefsOffset.begin("calib_data", true);
  remoteOffset = prefsOffset.getInt("prua_offset", 0);
  prefsOffset.end();
  Serial.printf("[SYS] Offset bussola: %d gradi\n", remoteOffset);

  strncpy(datiNautici.fw_str,      FW_VERSION, 12);
  strncpy(datiNautici.codice_crew, "----",      8);

  // TaskCloud su Core 0 — l'unico task aggiuntivo, NON tocca Wire
  // V2.2.3: stack portato a 16384 (da 8192) per supportare handshake SSL
  // OTA + buffer streaming heap + mbedtls durante download firmware.
  xTaskCreatePinnedToCore(TaskInvioCloud, "TaskCloud", 16384, NULL, 1, NULL, 0);

  Serial.println("[SYS] === BOOT OK ===");
  Serial.printf("[SYS] Core0: TaskCloud  Core1: loop(BNO@50Hz + GPS@5Hz+SBAS + ESPNOW@10Hz) V%s\n\n", FW_VERSION);
}

// =========================================================================
// LOOP PRINCIPALE — Core 1, 50Hz (delay 20ms)
//
// Wire usato SOLO qui: BNO085 e GPS in sequenza, stesso task, stesso core
// → zero race condition, zero mutex, zero task separati
//
// TaskCloud (Core 0) NON usa Wire → nessuna interferenza possibile
// =========================================================================
void loop() {

  // V2.2.3: blocca Core1 durante OTA — evita Cache Miss Panic e
  // race condition esp_now_send mentre Core0 scrive la flash.
  if (g_otaInCorso) {
    delay(100);
    return;
  }

  // Calibrazione BNO richiesta via cloud: blocca brevemente il loop
  if (calibrazioneRichiesta) {
    calibrazioneRichiesta = false;
    avviaCalibrazioneBNO();
    return;
  }

  // Gestione WiFi reconnect (non-blocking se già connesso)
  gestisciWiFi();

  // ── Sensori I2C (Wire — Core1 esclusivo) ──────────────────────────────
  leggiBussola();    // BNO085: quaternione → heading/roll/pitch
  leggiGPS();        // GPS: parser incrementale con break → SOG/COG/coord

  // ── Auto-calibrazione bussola via GPS ─────────────────────────────────
  if (autoCalibAttiva) eseguiAutoCalibrazioneGPS();

  // ── Scarroccio COG-HDG ────────────────────────────────────────────────
  if (g_sog >= 0.5f) {
    float d = (float)g_cog - (float)g_head;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    g_drift = d;
  } else {
    g_drift = 0.0f;
  }

  // ── Vento (Serial1 — no Wire, no conflitti) ───────────────────────────
  leggiVento();        // parser NMEA MWV → g_awa, g_aws, poi calcolaVentoVero()

  // ── Calcoli derivati (CPU only) ───────────────────────────────────────
  calcolaVMG();        // VMG vento (waypoint = 0 lato Pod)

  // ── Trasmissione al Visore via ESP-NOW @ 10Hz ─────────────────────────
  // inviaDatiVisore() ha timer interno: TX avviene solo ogni 100ms
  // Il loop gira a 50Hz ma la radio viene usata solo ogni 5 cicli
  inviaDatiVisore();

  // ── Telemetria debug seriale ──────────────────────────────────────────
  stampaTelemetria();

  // 20ms: 50Hz per BNO fluido e GPS incrementale
  delay(20);
}

// ── Auto-calibrazione bussola via GPS ─────────────────────────────────────
void eseguiAutoCalibrazioneGPS() {
  if (g_sog < 3.0f) return;
  float diff = (float)g_cog - (float)g_raw_head;
  while (diff <    0.0f) diff += 360.0f;
  while (diff >= 360.0f) diff -= 360.0f;
  sommaOffset += diff; campioniOffset++;
  if (campioniOffset >= 50) {
    int newOff = (int)(sommaOffset / campioniOffset);
    if (newOff > 180) newOff -= 360;
    remoteOffset = newOff;
    prefsOffset.begin("calib_data", false);
    prefsOffset.putInt("prua_offset", remoteOffset);
    prefsOffset.end();
    Serial.printf("[CALIB GPS] Nuovo offset bussola: %d gradi\n", remoteOffset);
    autoCalibAttiva = false;
    sommaOffset = 0.0f; campioniOffset = 0;
  }
}
