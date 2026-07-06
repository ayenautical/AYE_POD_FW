// =========================================================================
// AYE POD — AYE_POD_v37_DEV.ino  (file principale)
// Release: V2.3.0
//
// Changelog V2.3.1 — Fix compilazione (PATCH):
//   - Rete_Cloud.ino: aggiunto codice reale setupWiFi/gestisciWiFi/setupWebServer
//     (erano placeholder — causava undeclared in setup/loop/Bussola.ino)
//   - AYE_POD_v37_DEV.ino: g_otaInCorso → extern (OTA.ino la definisce già)
//   - Rete_Cloud.ino: controllaOTA — otaForce/hasTsRetry da bool a const char*
// Changelog V2.3.0:
//   - Filtro SOG ricalibrato per navigazione in acque mosse (onde, rollio)
//     Soglie ridotte: BNO_FERMO 0.06, gate finale 0.50 kn (era 1.00 kn)
//   - calcolaWaypoint(): BTW, DTW, VMG_WP, WP_BEARING_REL, ETA, arrive alert
//     Calcolo CPU-only dopo leggiGPS(), zero Wire, zero latenza bus
//   - struct_nautica: +19 bytes (106 totale) — aggiunti campi WP in fondo
//     BREAKING CHANGE: richiede aggiornamento contestuale AYE_Visore
//   - vmg_target (era vmg, sempre 0) ora popolato con VMG verso waypoint
//   - Nuove variabili globali: g_btw, g_dtw, g_vmg_wp, g_wp_bearing_rel,
//     g_eta_wp_sec, g_wp_arrive_alert
//   - DB Supabase: telemetria invia btw, dtw, vmg_wp (Rete_Cloud.ino)
//
// Changelog V2.2.6 (precedente):
//   - Fix setTimeout(8s corr.) + Stack2 PRIMA di Stack1
//
// ARCHITETTURA DEFINITIVA (invariata):
//   1. Wire usato SOLO dal loop() Core1 — mai da task separati
//   2. GPS: pattern Adafruit con break dopo parse
//   3. ESP-NOW: timer 100ms in inviaDatiVisore() — mai >10Hz
//   4. TaskCloud Core0: SOLO HTTP, ZERO Wire
//
//   ┌─ Core 0 ────────────────────────────────────────────────────────────┐
//   │  TaskCloud — HTTP Supabase ogni 2s, ZERO Wire                      │
//   └─────────────────────────────────────────────────────────────────────┘
//   ┌─ Core 1 — loop() ogni 20ms (50Hz) ─────────────────────────────────┐
//   │  leggiBussola()     Wire BNO085 → heading/roll/pitch 50Hz          │
//   │  leggiGPS()         Wire GPS   → SOG/COG/coord ≤120ms latenza      │
//   │  calcolaWaypoint()  CPU only   → BTW/DTW/VMG_WP/ETA               │
//   │  leggiVento()       Serial1    → AWA/AWS 10Hz NMEA                 │
//   │  calcolaVentoVero() CPU only   → TWA/TWS/TWD                       │
//   │  calcolaVMG()       CPU only   → VMG vento + VMG waypoint          │
//   │  inviaDatiVisore()  ESP-NOW    → 10Hz (timer 100ms interno)        │
//   └─────────────────────────────────────────────────────────────────────┘
//
// Hardware: Adafruit ESP32-S3 Feather 2MB PSRAM
//           BNO085 I2C 0x4A + PA1010D I2C 0x10 + NeoPixel pin33
//           Anemometro NMEA 0183 @ 4800 baud (RX=38, TX=39)
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

// ── Versionamento SemVer ──────────────────────────────────────────────────
#define FW_VERSION_MAJOR 2
#define FW_VERSION_MINOR 3
#define FW_VERSION_PATCH 1
#define FW_VERSION "2.3.1"

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

// ── Struct ESP-NOW Pod→Visore (106 bytes packed) ───────────────────────────
// V2.3.0: +21 bytes rispetto alla V2.2.6 (87 bytes) → totale 108 bytes
// I nuovi campi sono AGGIUNTI IN FONDO: gli offset di tutti i campi
// precedenti rimangono identici — la rottura è solo nel sizeof totale.
//
// CAMPI INVARIATI (offset 0-86, identici a V2.2.6):
//   roll, pitch, heading, sog, awa, aws, twa, tws, twd
//   batteria_pod, cloud_connesso, gps_fix, lat, lon, cog
//   vmg_target (era 'vmg', ora popolato con VMG verso waypoint)
//   vmg_wind, codice_crew[8], fw_str[12], unix_timestamp, anchor_alert
//
// CAMPI NUOVI (offset 87-105):
//   btw            — bearing to waypoint (gradi 0-359, Nord = riferimento)
//   dtw            — distance to waypoint (miglia nautiche)
//   wp_bearing_rel — BTW relativo alla prora corretto (-180/+180°)
//                    Positivo = WP a dritta, Negativo = WP a sinistra
//   vmg_wp         — SOG × cos(COG - BTW), VMG effettivo verso WP
//   eta_wp_sec     — ETA in secondi (0 = N/A, barca ferma o fuori rotta)
//   wp_arrive_alert — true quando DTW < 50m (arrive alarm)
//
// BREAKING CHANGE: AYE_Visore deve essere aggiornato contestualmente.
// Il Visore vecchio (87 bytes) scarterà i pacchetti silenziosamente.
// ──────────────────────────────────────────────────────────────────────────
typedef struct __attribute__((packed)) struct_nautica {
  // ── Assetto (invariato) ────────────────────────────────────────────────
  int   roll, pitch, heading;          // 12 bytes
  // ── Navigazione (invariato) ───────────────────────────────────────────
  float sog, awa, aws, twa, tws, twd; // 24 bytes
  int   batteria_pod;                  // 4 bytes
  bool  cloud_connesso, gps_fix;       // 2 bytes
  float lat, lon, cog;                 // 12 bytes
  float vmg_target;                    // 4 bytes — era 'vmg' (sempre 0), ora VMG verso WP
  float vmg_wind;                      // 4 bytes
  // ── Identificativi (invariato) ────────────────────────────────────────
  char  codice_crew[8];                // 8 bytes
  char  fw_str[12];                    // 12 bytes
  // ── Timestamp e alert (invariato) ─────────────────────────────────────
  uint32_t unix_timestamp;             // 4 bytes
  bool     anchor_alert;              // 1 byte  ← offset 86
  // ── Waypoint navigation (NUOVO V2.3.0) ────────────────────────────────
  float    btw;                        // 4 bytes — bearing to waypoint (0-359°)
  float    dtw;                        // 4 bytes — distance to waypoint (nm)
  float    wp_bearing_rel;             // 4 bytes — BTW relativo alla prora (signed)
  float    vmg_wp;                     // 4 bytes — VMG effettivo verso WP
  uint32_t eta_wp_sec;                 // 4 bytes — ETA in secondi
  bool     wp_arrive_alert;            // 1 byte  ← offset 105
} struct_nautica;
// sizeof atteso: 87 (V2.2.6) + 21 (WP: 5×float/uint32 + 1×bool) = 108 bytes

struct_nautica datiNautici;

// ── Struct Visore→Pod (22 bytes packed — INVARIATA) ───────────────────────
typedef struct __attribute__((packed)) struct_messaggio_visore {
  float batteriaVisore;
  char  fwVisore[10];
  float session_dist;
  int   cmd_sessione;
} struct_messaggio_visore;

struct_messaggio_visore datiDalVisore;

// ── Variabili sensori — Core1 scrive, TaskCloud legge ─────────────────────
volatile int    g_roll=0, g_pitch=0, g_head=0, g_raw_head=0, g_acc=0;
volatile float  g_sog=0.0f, g_cog=0.0f;
volatile double g_lat=0.0,  g_lon=0.0;
volatile uint8_t g_sats=0;
volatile float  g_drift=0.0f;
volatile float  g_awa=-1.0f, g_aws=0.0f;
volatile float  g_twa=0.0f,  g_tws=0.0f;
volatile int    g_twd=0;
volatile float  g_vmg=0.0f,  g_vmg_wind=0.0f;
volatile float  g_accel_mag=0.0f;
volatile float  g_accel_mag_media=0.0f;

// ── Waypoint navigation — NUOVE V2.3.0 ───────────────────────────────────
volatile double  g_wp_lat=0.0, g_wp_lon=0.0;
volatile int     g_wp_active=0;
volatile float   g_btw=0.0f;            // bearing to waypoint (0-359°)
volatile float   g_dtw=0.0f;            // distance to waypoint (nm)
volatile float   g_vmg_wp=0.0f;         // VMG verso waypoint (kn)
volatile float   g_wp_bearing_rel=0.0f; // BTW relativo alla prora (±180°)
volatile uint32_t g_eta_wp_sec=0;       // ETA in secondi
volatile bool    g_wp_arrive_alert=false;

// ── Variabili timestamp e anchor ─────────────────────────────────────────
volatile uint32_t g_unix_timestamp=0;
volatile bool     g_anchor_alert=false;
volatile double   g_anchor_lat=0.0, g_anchor_lon=0.0;
volatile float    g_anchor_raggio_m=30.0f;
volatile bool     g_anchor_attivo=false;

// ── Librerie oggetti ──────────────────────────────────────────────────────
Adafruit_BNO08x  bno08x;
sh2_SensorValue_t sensorValue;
Adafruit_GPS GPS(&Wire);
Adafruit_MAX17048 maxlipo;
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ── Calibrazione bussola ─────────────────────────────────────────────────
Preferences prefsOffset;
int         remoteOffset = 0;
bool        autoCalibAttiva = false;
float       sommaOffset = 0.0f;
int         campioniOffset = 0;
bool        calibrazioneRichiesta = false;

// ── Flag OTA ─────────────────────────────────────────────────────────────
extern volatile bool g_otaInCorso;  // definita in OTA.ino — no ridefinizione

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.printf("\n[SYS] === AYE POD V%s BOOT ===\n", FW_VERSION);

  // Verifica dimensione struct — CRITICA per ESP-NOW
  int sz_nav = sizeof(struct_nautica);
  int sz_vis = sizeof(datiDalVisore);
  Serial.printf("[SYS] struct_nautica=%d bytes (atteso 108)\n", sz_nav);
  Serial.printf("[SYS] datiDalVisore=%d bytes (atteso 22)\n",   sz_vis);
  if (sz_nav != 108) {
    Serial.println("[ERRORE CRITICO] struct_nautica size errata! Il Visore riceverà spazzatura.");
  }
  if (sz_vis != 22) {
    Serial.println("[ERRORE CRITICO] struct_messaggio_visore size errata!");
  }

  setupUtils();          // Wire.begin() + NeoPixel + MAX17048
  Wire.setClock(400000); // I2C Fast Mode 400kHz — riduce drain GPS da ~5ms a ~1ms
  Serial.println("[SYS] I2C @ 400kHz (Fast Mode)");

  setupBussola();        // BNO085 @ 0x4A
  setupGPS();            // PA1010D @ 0x10 — 5Hz+SBAS
  setupWiFi();
  setupESPNOW();

  prefsOffset.begin("calib_data", true);
  remoteOffset = prefsOffset.getInt("prua_offset", 0);
  prefsOffset.end();
  Serial.printf("[SYS] Offset bussola: %d gradi\n", remoteOffset);

  strncpy(datiNautici.fw_str,      FW_VERSION, 12);
  strncpy(datiNautici.codice_crew, "----",      8);

  // TaskCloud su Core 0 — NON tocca Wire
  xTaskCreatePinnedToCore(TaskInvioCloud, "TaskCloud", 16384, NULL, 1, NULL, 0);

  Serial.println("[SYS] === BOOT OK ===");
  Serial.printf("[SYS] Core0:TaskCloud  Core1:loop(BNO@50Hz+GPS@5Hz+WP_calc+ESPNOW@10Hz) V%s\n\n",
                FW_VERSION);
}

// ============================================================================
// LOOP PRINCIPALE — Core 1, 50Hz (delay 20ms)
//
// Wire usato SOLO qui: BNO085 e GPS in sequenza, stesso task, stesso core.
// calcolaWaypoint() è CPU-only: zero Wire, chiamata dopo leggiGPS().
// TaskCloud (Core 0) NON usa Wire.
// ============================================================================
void loop() {

  if (g_otaInCorso) {
    delay(100);
    return;
  }

  if (calibrazioneRichiesta) {
    calibrazioneRichiesta = false;
    avviaCalibrazioneBNO();
    return;
  }

  gestisciWiFi();

  // ── Sensori I2C (Wire — Core1 esclusivo) ─────────────────────────────
  leggiBussola();     // BNO085: quaternione → heading/roll/pitch
  leggiGPS();         // PA1010D: parser incrementale → SOG/COG/coord

  // ── Auto-calibrazione bussola via GPS ────────────────────────────────
  if (autoCalibAttiva) eseguiAutoCalibrazioneGPS();

  // ── Scarroccio COG-HDG ───────────────────────────────────────────────
  if (g_sog >= 0.5f) {
    float d = (float)g_cog - (float)g_head;
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    g_drift = d;
  } else {
    g_drift = 0.0f;
  }

  // ── Waypoint navigation (CPU only, zero Wire) ────────────────────────
  calcolaWaypoint();  // BTW, DTW, VMG_WP, WP_BEARING_REL, ETA, arrive alert

  // ── Vento (Serial1 — no Wire) ────────────────────────────────────────
  leggiVento();       // NMEA MWV → g_awa, g_aws, calcolaVentoVero()

  // ── Calcoli derivati ─────────────────────────────────────────────────
  calcolaVMG();       // VMG vento + VMG waypoint

  // ── Trasmissione Visore via ESP-NOW @ 10Hz ────────────────────────────
  inviaDatiVisore();  // timer interno 100ms

  // ── Telemetria debug seriale @ 1Hz ───────────────────────────────────
  stampaTelemetria();

  delay(20); // 50Hz
}

// ── Auto-calibrazione bussola via GPS (INVARIATA) ────────────────────────
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
