// =========================================================================
// AYE POD — Visore.ino (modulo ESP-NOW lato Pod)
// Release: V2.0.2
//
// Changelog V2.0.2 — FIX H3:
//   Aggiunta cattura snapshot atomica di cmd_sessione nella callback
//   OnDataRecvDalVisore. Quando il Visore manda cmd=1 o cmd=2, il valore
//   viene scritto IMMEDIATAMENTE in g_cmdSessSnap e g_cmdSessSnapDist
//   (volatile, visibili al TaskCloud su Core0) prima che qualsiasi latch
//   sul Visore possa azzerare cmd_sessione nel pacchetto successivo.
//
//   La logica di cattura è "edge triggered":
//   - cmd=1: cattura solo se lo snapshot era 0 (transizione 0→1)
//   - cmd=2: cattura solo se lo snapshot era 1 (transizione 1→2)
//             E salva session_dist nel momento esatto (valore più fresco)
//   - cmd=0: NON resetta g_cmdSessSnap — questo è compito di Rete_Cloud.ino
//             dopo la conferma DB, non del Visore dopo il latch
//
//   Questo garantisce che anche se il Visore manda cmd=2 per soli 100ms
//   (un singolo pacchetto ESP-NOW), il POD lo cattura e lo processa.
//
// Changelog V45.4.7 (precedente):
//   FIX CRITICO: timer 100ms per TX ESP-NOW
//   Il loop() gira a 50Hz. Senza timer, TX saturava la coda ESP-NOW.
// =========================================================================

#define ESPNOW_INTERVAL_MS  100UL   // 10Hz — non scendere sotto 80ms su ESP32

// Dichiarazioni extern per le variabili snapshot definite in Rete_Cloud.ino
extern volatile int   g_cmdSessSnap;
extern volatile float g_cmdSessSnapDist;
extern volatile bool  g_pendingChiudiSessione;
extern char           g_sessione_attiva_id[];  // per verifica sessione recuperata da boot precedente

void OnDataRecvDalVisore(const uint8_t* mac, const uint8_t* data, int len) {
  if (len == sizeof(datiDalVisore)) {
    memcpy(&datiDalVisore, data, sizeof(datiDalVisore));
    fwVisoreAttuale = String(datiDalVisore.fwVisore);

    // ── Cattura snapshot cmd — FIX H3 ──────────────────────────────
    // Legge cmd e dist UNA VOLTA dalla struct appena ricevuta.
    // La scrittura su g_cmdSessSnap è volatile → visibile subito a Core0.
    // Logica edge-triggered: cattura le transizioni significative,
    // non il valore corrente ogni 100ms.
    int   rxCmd  = datiDalVisore.cmd_sessione;
    float rxDist = datiDalVisore.session_dist;

    if (rxCmd == 1 && g_cmdSessSnap == 0) {
      // Transizione STOP→AVVIO: cattura start
      g_cmdSessSnap     = 1;
      g_cmdSessSnapDist = 0.0f;
      Serial.println("[SNAP] cmd=1 catturato (avvio sessione)");
    } else if (rxCmd == 2 && (g_cmdSessSnap == 1 || g_sessione_attiva_id[0] != '\0')) {
      // Cattura cmd=2 in due casi:
      //   1. Flusso normale: snapshot era 1 (avvio e stop nello stesso boot)
      //   2. Sessione recuperata da boot precedente: g_sessione_attiva_id non vuoto
      //      (es. POD riflashato con sessione già aperta sul DB — FASE 1 la recupera)
      // In entrambi i casi aggiorna snapshot e distanza, attiva il pending.
      g_cmdSessSnap     = 2;
      g_cmdSessSnapDist = rxDist;
      if (!g_pendingChiudiSessione) {
        g_pendingChiudiSessione = true;
        Serial.printf("[SNAP] cmd=2 catturato — dist=%.2f nm, pending attivato\n", rxDist);
      }
    }
    // cmd=0: ignorato — il reset di g_cmdSessSnap avviene in Rete_Cloud.ino
    // dopo conferma DB, non qui, per evitare reset prematuro

    Serial.printf("[RADIO-IN] Bat=%.0f%% FW=%s cmd=%d dist=%.2fnm\n",
      datiDalVisore.batteriaVisore, datiDalVisore.fwVisore,
      datiDalVisore.cmd_sessione, datiDalVisore.session_dist);
  } else {
    Serial.printf("[RADIO-WARN] sizeof errato: recv=%d atteso=%d\n",
      len, (int)sizeof(datiDalVisore));
  }
}

void setupESPNOW() {
  WiFi.mode(WIFI_AP_STA);
  if (esp_now_init() != ESP_OK) { Serial.println("[ERR] ESP-NOW init fallito"); return; }
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecvDalVisore);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, macVisore, 6);
  p.channel = 0; p.encrypt = false;
  esp_now_add_peer(&p);
  Serial.printf("[ESP-NOW] OK struct=%d datiDalVisore=%d TX@10Hz\n",
    (int)sizeof(struct_nautica), (int)sizeof(datiDalVisore));
}

void inviaDatiVisore() {
  static unsigned long lastSend = 0;
  unsigned long now = millis();
  if (now - lastSend < ESPNOW_INTERVAL_MS) return;
  lastSend = now;

  datiNautici.roll    = g_roll;
  datiNautici.pitch   = g_pitch;
  datiNautici.heading = g_head;

  datiNautici.sog = roundf((float)g_sog * 10.0f) / 10.0f;
  datiNautici.cog = roundf((float)g_cog);
  datiNautici.lat = (float)g_lat;
  datiNautici.lon = (float)g_lon;

  datiNautici.awa = roundf(g_awa);
  datiNautici.aws = roundf(g_aws * 10.0f) / 10.0f;

  datiNautici.twa = roundf(g_twa);
  datiNautici.tws = roundf(g_tws * 10.0f) / 10.0f;
  datiNautici.twd = (float)roundf((float)g_twd);

  datiNautici.vmg      = 0.0f;
  datiNautici.vmg_wind = roundf(g_vmg_wind * 10.0f) / 10.0f;

  datiNautici.batteria_pod   = (int)maxlipo.cellPercent();
  datiNautici.cloud_connesso = wifiConnesso;
  datiNautici.gps_fix        = (g_sats >= 4);

  strncpy(datiNautici.fw_str,      FW_VERSION,   12);
  extern char g_codice_crew[8];
  strncpy(datiNautici.codice_crew, g_codice_crew, 8);

  datiNautici.unix_timestamp = g_unix_timestamp;
  datiNautici.anchor_alert   = g_anchor_alert;

  esp_err_t r = esp_now_send(macVisore, (uint8_t*)&datiNautici, sizeof(datiNautici));
  if (r != ESP_OK) Serial.printf("[RADIO-ERR] esp_now_send=%d\n", (int)r);
}
