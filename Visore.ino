// =========================================================================
// AYE POD — Visore.ino
// Release: V2.3.1
//
// Changelog V2.3.0:
//   - inviaDatiVisore(): popola i nuovi campi waypoint nella struct_nautica
//     btw, dtw, wp_bearing_rel, vmg_wp, eta_wp_sec, wp_arrive_alert
//   - vmg_target ora inviato come g_vmg_wp (era sempre 0.0f)
//   - ESPNOW_INTERVAL_MS invariato: 100ms (10Hz)
//
// Nessuna altra modifica rispetto alla versione precedente.
// Callback OnDataRecvDalVisore: invariata (struct Visore→Pod a 22 bytes).
// =========================================================================

#define ESPNOW_INTERVAL_MS 100

void inviaDatiVisore() {
  static unsigned long lastSend = 0;
  unsigned long now = millis();
  if (now - lastSend < ESPNOW_INTERVAL_MS) return;
  lastSend = now;

  // ── Assetto ───────────────────────────────────────────────────────────
  datiNautici.roll    = g_roll;
  datiNautici.pitch   = g_pitch;
  datiNautici.heading = g_head;  // già con remoteOffset applicato in Bussola.ino

  // ── SOG/COG — arrotondati per display ────────────────────────────────
  datiNautici.sog = roundf((float)g_sog * 10.0f) / 10.0f;
  datiNautici.cog = roundf((float)g_cog);
  datiNautici.lat = (float)g_lat;
  datiNautici.lon = (float)g_lon;

  // ── Vento ────────────────────────────────────────────────────────────
  datiNautici.awa = roundf(g_awa);
  datiNautici.aws = roundf(g_aws * 10.0f) / 10.0f;
  datiNautici.twa = roundf(g_twa);
  datiNautici.tws = roundf(g_tws * 10.0f) / 10.0f;
  datiNautici.twd = (float)roundf((float)g_twd);

  // ── VMG ───────────────────────────────────────────────────────────────
  // vmg_target: ora popolato con VMG verso waypoint attivo (era sempre 0)
  datiNautici.vmg_target = roundf(g_vmg_wp * 10.0f) / 10.0f;
  datiNautici.vmg_wind   = roundf(g_vmg_wind * 10.0f) / 10.0f;

  // ── Sistema ──────────────────────────────────────────────────────────
  datiNautici.batteria_pod   = (int)maxlipo.cellPercent();
  datiNautici.cloud_connesso = wifiConnesso;
  datiNautici.gps_fix        = (g_sats >= 4);

  // ── Identificativi ───────────────────────────────────────────────────
  strncpy(datiNautici.fw_str,      FW_VERSION,   12);
  extern char g_codice_crew[8];
  strncpy(datiNautici.codice_crew, g_codice_crew, 8);

  // ── Timestamp e alert ────────────────────────────────────────────────
  datiNautici.unix_timestamp = g_unix_timestamp;
  datiNautici.anchor_alert   = g_anchor_alert;

  // ── Waypoint navigation (NUOVO V2.3.0) ───────────────────────────────
  datiNautici.btw            = roundf(g_btw);                        // 0-359°
  datiNautici.dtw            = roundf(g_dtw * 100.0f) / 100.0f;     // 2 dec (nm)
  datiNautici.wp_bearing_rel = roundf(g_wp_bearing_rel);             // ±180° intero
  datiNautici.vmg_wp         = roundf(g_vmg_wp * 10.0f) / 10.0f;   // 1 dec (kn)
  datiNautici.eta_wp_sec     = g_eta_wp_sec;                          // secondi
  datiNautici.wp_arrive_alert = g_wp_arrive_alert;

  esp_err_t r = esp_now_send(macVisore, (uint8_t*)&datiNautici, sizeof(datiNautici));
  if (r != ESP_OK) Serial.printf("[RADIO-ERR] esp_now_send=%d\n", (int)r);
}

// ── Callback ricezione dati dal Visore (INVARIATA) ────────────────────────
// Cattura snapshot cmd_sessione IMMEDIATAMENTE per evitare race condition
// con il latch del Visore.
void OnDataRecvDalVisore(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(datiDalVisore)) {
    memcpy(&datiDalVisore, data, sizeof(datiDalVisore));

    float rxDist = datiDalVisore.session_dist;

    if (datiDalVisore.cmd_sessione == 1) {
      g_cmdSessSnap     = 1;
      g_cmdSessSnapDist = rxDist;
    } else if (datiDalVisore.cmd_sessione == 2) {
      g_cmdSessSnap     = 2;
      g_cmdSessSnapDist = rxDist;
      if (!g_pendingChiudiSessione) {
        g_pendingChiudiSessione = true;
        Serial.printf("[SNAP] cmd=2 catturato — dist=%.2f nm\n", rxDist);
      }
    }

    // Aggiorna FW Visore per telemetria
    fwVisoreAttuale = String(datiDalVisore.fwVisore);

    Serial.printf("[RADIO-IN] Bat=%.0f%% FW=%s cmd=%d dist=%.2fnm\n",
      datiDalVisore.batteriaVisore, datiDalVisore.fwVisore,
      datiDalVisore.cmd_sessione, datiDalVisore.session_dist);
  } else {
    Serial.printf("[RADIO-WARN] sizeof errato: recv=%d atteso=%d\n",
      len, (int)sizeof(datiDalVisore));
  }
}

// ── setupESPNOW (INVARIATA) ───────────────────────────────────────────────
void setupESPNOW() {
  WiFi.mode(WIFI_AP_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init fallito");
    return;
  }
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecvDalVisore);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, macVisore, 6);
  p.channel = 0; p.encrypt = false;
  esp_now_add_peer(&p);
  Serial.printf("[ESP-NOW] OK struct=%d datiDalVisore=%d TX@10Hz\n",
    (int)sizeof(struct_nautica), (int)sizeof(datiDalVisore));
}
