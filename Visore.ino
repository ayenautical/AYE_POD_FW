// =========================================================================
// AYE POD — Visore.ino
// Release: V2.6.0
//
// Changelog V2.6.0 — Sessione a STATO: callback svuotata (MINOR):
//   La callback ESP-NOW non contiene piu' nessuna macchina a stati.
//   Con il protocollo a LIVELLO (sessione_attiva=0/1) ogni pacchetto porta
//   la verita' completa: non c'e' nessun fronte da catturare, quindi non
//   c'e' niente da ricordare tra un pacchetto e l'altro.
//
//   RIMOSSO (tutto quanto esisteva solo per non perdere/riconsumare i fronti):
//     - snapshot g_cmdSessSnap / g_cmdSessSnapDist
//     - g_pendingChiudiSessione e il suo reset (V2.4.5)
//     - guardia difensiva "!= 1" sull'avvio (V2.4.6 FIX 2)
//     - ramo "stop orfano" a 4 condizioni (V2.4.6 FIX 1) ← causa del bug
//       del 16/07: se g_pendingChiudiSessione restava true, la condizione
//       non scattava MAI e lo snapshot restava incollato a 2 a vita.
//   RESTA: memcpy + fwVisoreAttuale + timestamp RX + log.
//   La decisione crea/chiudi e' presa dal TaskCloud (Rete_Cloud.ino),
//   dove g_sessione_attiva_id — l'unica verita' — gia' vive.
//
//   FIX V2.6.0 — dereference di 'mac' in OnDataSentAlVisore():
//     La callback stampava mac[0..5] nei log [TX-ACK]. Su ESP32 Arduino
//     core 3.x il cast (esp_now_send_cb_t) maschera un mismatch di firma:
//     quei byte sono un puntatore di memoria, non un MAC (stesso bug del
//     15/07 che ha tenuto il Visore muto per 3 giorni). Ora si stampa
//     macVisore[] dal binding, che e' il peer reale e noto.
//
// Release: V2.5.1 / V2.4.6 / V2.4.5 / V2.3.2
//
// ── NOTA Task 5 (V45.5.18) — Serial senza mutex ───────────────────────────
// Core0 (TaskCloud) e Core1 (loop/inviaDatiVisore) scrivono su Serial
// concorrentemente senza mutex → righe troncate e sovrascritte nel log.
// Workaround immediato: ridurre Serial.printf nel TaskCloud durante runtime
// (lasciare solo errori critici).
// Fix definitivo in V2.5.0: SemaphoreHandle_t g_serialMux = xSemaphoreCreateMutex()
// + macro LOG_SAFE() che wrappa Serial.printf con presa/rilascio mutex.
// ─────────────────────────────────────────────────────────────────────────────



// Changelog V2.3.2 — Opzione A: silenzio TX quando Visore non abbinato (PATCH):
//   - inviaDatiVisore(): aggiunto guard all'inizio della funzione.
//     Se macVisore[] è broadcast (FF:FF:FF:FF:FF:FF) il POD NON trasmette
//     nessun pacchetto ESP-NOW. Questo evita che i dati vengano ricevuti
//     da qualsiasi Visore AYE nelle vicinanze quando il binding è assente.
//     Il timer lastSend NON viene aggiornato durante il silenzio: quando
//     il binding viene ripristinato, la prima trasmissione parte subito
//     senza attendere i 100ms del prossimo ciclo.
//     Log seriale ogni 30s: [RADIO] Nessun Visore abbinato — TX sospeso.
//     Telemetria cloud e sessioni: invariate, non toccate da questa modifica.
//
// Changelog V2.3.0:
//   - inviaDatiVisore(): popola i nuovi campi waypoint nella struct_nautica
//     btw, dtw, wp_bearing_rel, vmg_wp, eta_wp_sec, wp_arrive_alert
//   - vmg_target ora inviato come g_vmg_wp (era sempre 0.0f)
//   - ESPNOW_INTERVAL_MS invariato: 100ms (10Hz)
//
// Callback OnDataRecvDalVisore: invariata (struct Visore→Pod a 22 bytes).
// setupESPNOW(): invariata.
// =========================================================================

#define ESPNOW_INTERVAL_MS 100

void inviaDatiVisore() {
  static unsigned long lastSend    = 0;
  static unsigned long lastLogNoBind = 0;  // V2.3.2: log throttle
  unsigned long now = millis();

  // ── V2.3.2: Opzione A — silenzio TX se nessun Visore abbinato ──────────
  // macVisore[] = FF:FF:FF:FF:FF:FF significa che Rete_Cloud.ino non ha
  // ancora letto un mac_visore_bound valido dal DB (o è stato fatto unbind).
  // In questo stato il POD non trasmette nulla via ESP-NOW: un pacchetto
  // broadcast sarebbe ricevuto da qualsiasi Visore AYE nelle vicinanze.
  // Il timer lastSend non viene toccato: al bind successivo TX riparte subito.
  bool isBroadcast = (macVisore[0] == 0xFF && macVisore[1] == 0xFF &&
                      macVisore[2] == 0xFF && macVisore[3] == 0xFF &&
                      macVisore[4] == 0xFF && macVisore[5] == 0xFF);
  if (isBroadcast) {
    if (now - lastLogNoBind >= 30000) {
      lastLogNoBind = now;
      Serial.println("[RADIO] Nessun Visore abbinato — TX sospeso");
    }
    return;  // nessun TX, nessun aggiornamento lastSend
  }
  // ── Fine guard V2.3.2 ───────────────────────────────────────────────────

  if (now - lastSend < ESPNOW_INTERVAL_MS) return;
  lastSend = now;

  // ── Assetto ─────────────────────────────────────────────────────────────
  datiNautici.roll    = g_roll;
  datiNautici.pitch   = g_pitch;
  datiNautici.heading = g_head;  // già con remoteOffset applicato in Bussola.ino

  // ── SOG/COG — arrotondati per display ───────────────────────────────────
  datiNautici.sog = roundf((float)g_sog * 10.0f) / 10.0f;
  datiNautici.cog = roundf((float)g_cog);
  datiNautici.lat = (float)g_lat;
  datiNautici.lon = (float)g_lon;

  // ── Vento ────────────────────────────────────────────────────────────────
  datiNautici.awa = roundf(g_awa);
  datiNautici.aws = roundf(g_aws * 10.0f) / 10.0f;
  datiNautici.twa = roundf(g_twa);
  datiNautici.tws = roundf(g_tws * 10.0f) / 10.0f;
  datiNautici.twd = (float)roundf((float)g_twd);

  // ── VMG ──────────────────────────────────────────────────────────────────
  // vmg_target: ora popolato con VMG verso waypoint attivo (era sempre 0.0f)
  datiNautici.vmg_target = roundf(g_vmg_wp   * 10.0f) / 10.0f;
  datiNautici.vmg_wind   = roundf(g_vmg_wind * 10.0f) / 10.0f;

  // ── Sistema ──────────────────────────────────────────────────────────────
  datiNautici.batteria_pod   = (int)maxlipo.cellPercent();
  datiNautici.cloud_connesso = wifiConnesso;
  datiNautici.gps_fix        = (g_sats >= 4);

  // ── Identificativi ───────────────────────────────────────────────────────
  strncpy(datiNautici.fw_str,      FW_VERSION,   12);
  extern char g_codice_crew[8];
  strncpy(datiNautici.codice_crew, g_codice_crew, 8);

  // ── Timestamp e alert ────────────────────────────────────────────────────
  datiNautici.unix_timestamp = g_unix_timestamp;
  datiNautici.anchor_alert   = g_anchor_alert;

  // ── Waypoint navigation (NUOVO V2.3.0) ───────────────────────────────────
  datiNautici.btw             = roundf(g_btw);                        // 0-359°
  datiNautici.dtw             = roundf(g_dtw * 100.0f) / 100.0f;     // 2 dec (nm)
  datiNautici.wp_bearing_rel  = roundf(g_wp_bearing_rel);             // ±180° intero
  datiNautici.vmg_wp          = roundf(g_vmg_wp  * 10.0f) / 10.0f;  // 1 dec (kn)
  datiNautici.eta_wp_sec      = g_eta_wp_sec;                         // secondi
  datiNautici.wp_arrive_alert = g_wp_arrive_alert;

  esp_err_t r = esp_now_send(macVisore, (uint8_t*)&datiNautici, sizeof(datiNautici));
  if (r != ESP_OK) Serial.printf("[RADIO-ERR] esp_now_send=%d\n", (int)r);
}

// ── Callback ricezione dati dal Visore — V2.6.0: SOLO memcpy + log ─────────
// Con il protocollo a STATO non c'e' nessun fronte da catturare: ogni
// pacchetto porta il livello corrente (sessione_attiva=0/1) e vale da solo.
// Perdere pacchetti non ha conseguenze: il successivo rimette tutto a posto.
// Qui NON si prendono decisioni: la logica crea/chiudi vive nel TaskCloud
// (Rete_Cloud.ino), dove g_sessione_attiva_id — l'unica verita' — gia' esiste.
//
// ⚠ Il param 'mac' NON va MAI dereferenziato: su ESP32 Arduino core 3.x e'
//   un esp_now_recv_info_t*, non un uint8_t[6] (il cast in setupESPNOW()
//   maschera il mismatch). Leggerlo restituirebbe un puntatore di memoria.
void OnDataRecvDalVisore(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(datiDalVisore)) {
    memcpy(&datiDalVisore, data, sizeof(datiDalVisore));

    // V2.6.0: timestamp RX — alimenta il timeout 180s nel TaskCloud.
    // millis() non torna mai 0 se non all'overflow (~49gg): usiamo 1 come
    // valore minimo cosi' 0 resta il sentinella "mai ricevuto nulla".
    uint32_t nowMs = millis();
    g_ultimoPacchettoVisore = (nowMs == 0) ? 1 : nowMs;

    // Aggiorna FW Visore per telemetria
    fwVisoreAttuale = String(datiDalVisore.fwVisore);

    Serial.printf("[RADIO-IN] Bat=%.0f%% FW=%s sess_attiva=%d dist=%.2fnm\n",
      datiDalVisore.batteriaVisore, datiDalVisore.fwVisore,
      (int)datiDalVisore.sessione_attiva, datiDalVisore.session_dist);
  } else {
    // V2.4.6: il param 'mac' NON va dereferenziato — su ESP32 Arduino core 3.x
    // e' un esp_now_recv_info_t*, non un uint8_t[6]: i byte stampati sarebbero
    // un puntatore di memoria (stesso bug del cast risolto il 15/07, vedi
    // problema aperto #4). Il MAC atteso e' noto: macVisore[] dal binding.
    Serial.printf("[RADIO-WARN] sizeof errato: recv=%d atteso=%d\n",
      len, (int)sizeof(datiDalVisore));
  }
}

// ── V2.4.3 — Callback conferma invio ESP-NOW ────────────────────────────────
// Logga solo i CAMBI di stato (ok->fail o fail->ok) + un riepilogo ogni 10s,
// per non intasare il seriale a 10Hz.
//
// ⚠ FIX V2.6.0 — il param 'mac' NON viene piu' dereferenziato.
//   Il cast (esp_now_send_cb_t) in setupESPNOW() maschera un mismatch di
//   firma su core 3.x: i byte letti da mac[0..5] sono un puntatore di
//   memoria, non un MAC — i log [TX-ACK] mostravano garbage (stesso bug
//   del 15/07). Il peer reale e' noto: macVisore[] dal binding.
//   'status' (secondo param) e' invece corretto e resta usato.
void OnDataSentAlVisore(const uint8_t *mac, esp_now_send_status_t status) {
  static bool primaVolta = true;
  static esp_now_send_status_t ultimoStato = ESP_NOW_SEND_FAIL;
  static uint32_t nOk = 0, nFail = 0;
  static unsigned long ultimoRiepilogo = 0;

  if (status == ESP_NOW_SEND_SUCCESS) nOk++; else nFail++;

  if (primaVolta || status != ultimoStato) {
    primaVolta  = false;
    ultimoStato = status;
    Serial.printf("[TX-ACK] %s verso %02X:%02X:%02X:%02X:%02X:%02X\n",
      (status == ESP_NOW_SEND_SUCCESS) ? "CONSEGNATO" : "NESSUN ACK (peer non risponde)",
      macVisore[0],macVisore[1],macVisore[2],macVisore[3],macVisore[4],macVisore[5]);
  }

  if (millis() - ultimoRiepilogo > 10000) {
    ultimoRiepilogo = millis();
    Serial.printf("[TX-ACK] riepilogo 10s: ok=%lu fail=%lu\n",
                  (unsigned long)nOk, (unsigned long)nFail);
    nOk = 0; nFail = 0;
  }
}

// ── setupESPNOW (INVARIATA) ─────────────────────────────────────────────────
void setupESPNOW() {
  WiFi.mode(WIFI_AP_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init fallito");
    return;
  }
  // ATTENZIONE: il cast (esp_now_recv_cb_t) è latente su ESP32 Arduino core 3.x
  // (firma cambiata da uint8_t* a esp_now_recv_info_t*). Sicuro oggi perché
  // OnDataRecvDalVisore() NON dereferenzia il primo param. Non aggiungere
  // letture del MAC/info senza aggiornare la firma a core 3.x.
  // Fix root-cause analogo risolto nel Visore V45.5.18. (Task 4 — POD V2.4.3)
  esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecvDalVisore);
  // ── V2.4.3 — Callback conferma invio ─────────────────────────────────
  // esp_now_send() che ritorna ESP_OK significa solo "accodato", NON
  // "consegnato". La consegna reale (ACK a livello MAC) si sa SOLO qui.
  // Senza questa callback il TX rotto era invisibile.
 // ATTENZIONE: cast (esp_now_send_cb_t) latente su core 3.x.
  // OnDataSentAlVisore riceve il MAC come secondo-primo param: verifica che
  // status (secondo param) sia letto correttamente e non confuso col puntatore.
  // I log [TX-ACK] dei MAC potrebbero essere garbage se si legge il primo param.
  // Fix definitivo in V2.5.0. (Task 4 — POD V2.4.3)
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSentAlVisore);
  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, macVisore, 6);
  p.channel = 0; p.encrypt = false;
  esp_now_add_peer(&p);
  // ── V2.4.3 — Diagnostica MAC ESP-NOW ────────────────────────────────
  // Il Visore vede come sorgente 7A:EE:21:3C:74:EE, che non corrisponde
  // ne' allo STA (CC:BA:97:1C:BB:F0) ne' all'AP derivato (CE:BA:...).
  // Stampiamo TUTTI i MAC reali per capire da quale interfaccia esce
  // davvero il traffico ESP-NOW e su quale il POD ascolta.
  {
    uint8_t m[6];
    Serial.printf("[MAC] WiFi.macAddress()       = %s\n", WiFi.macAddress().c_str());
    Serial.printf("[MAC] WiFi.softAPmacAddress() = %s\n", WiFi.softAPmacAddress().c_str());
    if (esp_wifi_get_mac(WIFI_IF_STA, m) == ESP_OK)
      Serial.printf("[MAC] esp_wifi WIFI_IF_STA    = %02X:%02X:%02X:%02X:%02X:%02X\n",
                    m[0],m[1],m[2],m[3],m[4],m[5]);
    if (esp_wifi_get_mac(WIFI_IF_AP, m) == ESP_OK)
      Serial.printf("[MAC] esp_wifi WIFI_IF_AP     = %02X:%02X:%02X:%02X:%02X:%02X\n",
                    m[0],m[1],m[2],m[3],m[4],m[5]);
    uint8_t chP=0; wifi_second_chan_t chS;
    if (esp_wifi_get_channel(&chP,&chS) == ESP_OK)
      Serial.printf("[MAC] canale WiFi corrente    = %d\n", (int)chP);
    Serial.printf("[MAC] peer Visore atteso      = %02X:%02X:%02X:%02X:%02X:%02X\n",
                  macVisore[0],macVisore[1],macVisore[2],macVisore[3],macVisore[4],macVisore[5]);
  }

  Serial.printf("[ESP-NOW] OK struct=%d datiDalVisore=%d TX@10Hz\n",
    (int)sizeof(struct_nautica), (int)sizeof(datiDalVisore));
}
