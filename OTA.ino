// =========================================================================
// AYE POD — OTA.ino
// Release: V2.2.4
//
// Changelog V2.2.4 — Fix HTTP -11 timeout + flood RADIO-ERR:
//
//   FIX 1 — http.setTimeout(60000): era 60 (= 60ms!). HTTPClient::setTimeout
//     su ESP32 vuole MILLISECONDI, non secondi. Con 60ms nessun server HTTPS
//     può completare la connessione → -11 immediato su ogni tentativo download.
//     Corretto a 60000ms (60s) per coprire download bin su connessioni lente.
//
//   FIX 2 — Rimosso g_otaInCorso = false in TUTTI i blocchi di errore che
//     portano a ESP.restart(). La variabile rimetteva a false il lock prima
//     del reboot, sbloccando il Core1 (loop()) per i ~500ms del vTaskDelay
//     pre-restart. Poiché ESP-NOW era già stato deinizializzato, inviaDatiVisore()
//     generava il flood di [RADIO-ERR] esp_now_send=12389 visibile nel log.
//     g_otaInCorso rimane true finché il chip non si spegne: è l'unico modo
//     per garantire che Core1 resti in pausa. Al riavvio la variabile torna
//     false per reset del BSS segment.
//     Blocchi corretti: mbedtls_md_setup, HTTP GET, Update.begin, malloc,
//     Update.write, SHA256 mismatch, Update.end (7 punti totali).
//
// Changelog V2.1.6 — Architettura OTA con NVS (zero HTTP pre-download):
//
//   PROBLEMA RISOLTO: chiamaRPC("downloading") dentro eseguiOTA() apriva
//   un terzo SSL context HTTPS nello stesso ciclo TaskCloud (GET FASE1 +
//   POST telemetria + RPC) causando heap panic silenzioso → crash senza log.
//
//   SOLUZIONE — NVS namespace 'ota_ctrl':
//   - ZERO chiamate HTTP prima del download. Lo stato OTA è tracciato
//     interamente in NVS (Preferences flash locale), che sopravvive al reboot.
//   - Chiavi NVS: 'retry_count' (uint8), 'error_ts' (uint32, epoch UTC stima)
//   - Prima del download: NVS retry_count++ (sopravvive al crash).
//   - Se crash: al boot successivo controllaOTA() legge retry_count da NVS.
//   - Se retry_count >= OTA_MAX_RETRY (3): DB→error, NVS error_ts settato,
//     retry_count→0. Dopo 1h (OTA_ERROR_RESET_S) il DB torna pending automaticamente.
//   - Se download OK: NVS retry_count→0, DB stato→ok + fw aggiornato, reboot.
//   - Se download HTTP error: TWDT riabilitato, return false (POD continua
//     su firmware stabile — dual-slot ESP32 garantisce rollback automatico).
//   - Il DB ota_stato rimane 'pending' durante il download. Viene aggiornato
//     solo a successo ('ok') o max retry ('error'). Nessuna scrittura pre-download.
//   - Namespace NVS 'ota_ctrl' separato da 'calib_data' (offset bussola)
//     per evitare qualsiasi conflitto.
//
//   COMPORTAMENTO RETRY:
//   - 3 tentativi automatici (OTA_MAX_RETRY)
//   - Dopo 3 fail: DB=error, NVS salva timestamp. POD continua normalmente.
//   - Dopo 1h (OTA_ERROR_RESET_S=3600): DB torna pending automaticamente.
//   - Se l'operatore imposta manualmente error senza timestamp NVS: non si
//     auto-resetta (blocco manuale — serve intervento esplicito).
//   - Cloud mai disponibile: retry_count non incrementa (download non tentato).
//
// Changelog V2.1.5 —
// Changelog V2.1.5 — Rimosso extern g_cloudOkCount e OTA_MIN_CLOUD_OK:
//   Il gate OTA è ora interamente gestito in Rete_Cloud.ino tramite g_fase1Ok.
//   OTA.ino non ha più dipendenze dallo stato cloud — più coeso e testabile.
//   Tutti i fix precedenti (A: FORCE redirect, B/C: retry downloading, TWDT,
//   anti-loop 'downloading') rimangono invariati e attivi.
//
// Changelog V2.1.4
//
// Changelog V2.1.4 — Fix crash heap SSL durante download + timing cloud:
//
//   FIX A — HTTPC_FORCE_FOLLOW_REDIRECTS al posto di HTTPC_STRICT:
//     GitHub Releases risponde con redirect 302 → CDN githubusercontent.com.
//     HTTPC_STRICT apriva UNA NUOVA connessione HTTPS per il redirect, creando
//     due SSL context simultanei (~160KB heap totale). Con ~211KB liberi a runtime
//     il memory allocator crashava silenziosamente (heap panic → reset senza log).
//     HTTPC_FORCE riusa la connessione esistente: un solo SSL context, metà heap.
//
//   FIX B — Gate cloud-ready: OTA parte solo dopo 3 POST telemetria riusciti:
//     Al boot il TaskCloud tenta POST a Supabase prima che SSL sia "warm" →
//     errore -11 per i primi ~30s. Se l'OTA scattava in quel finestra, la
//     chiamaRPC("downloading") falliva silenziosamente e l'anti-loop non poteva
//     funzionare. Aggiunta variabile g_cloudOkCount (atomica, solo Core0):
//     si incrementa ad ogni POST telemetria con rc 2xx. eseguiOTA() è bloccata
//     finché g_cloudOkCount < OTA_MIN_CLOUD_OK (3). In questo modo quando
//     l'OTA parte la connessione SSL verso Supabase è già stabilizzata.
//     g_cloudOkCount è extern — dichiarato in Rete_Cloud.ino, letto qui.
//
//   FIX C — Retry scrittura 'downloading' con abort se fallisce:
//     chiamaRPC("downloading") ora verifica il return code. Se fallisce
//     (rc < 200 o >= 300) attende 3s e riprova una volta. Se fallisce ancora,
//     abort: g_otaInCorso = false, log di errore. Il DB rimane 'pending' →
//     l'operatore può ritentare senza intervento manuale sul campo.
//     Questo garantisce che l'anti-loop al boot funzioni sempre.
//
// Changelog V2.1.2 — Fix crash watchdog durante download OTA:
//   PROBLEMA: http.GET() bloccava Core0 senza yield → Task Watchdog Timer
//             (TWDT, default 5s su ESP32) resettava il chip durante il download.
//             Al reboot ota_stato restava 'pending' → loop infinito di crash.
//
//   FIX 1 — Anti-watchdog: esp_task_wdt_delete(NULL) prima del download,
//            esp_task_wdt_add(NULL) dopo. Rimuove il task corrente dal TWDT
//            per tutta la durata del download HTTP (legale su ESP-IDF, usato
//            da arduino-esp32 OTAWebUpdater di esempio).
//
//   FIX 2 — Anti-loop al boot: se al boot ota_stato == 'downloading' significa
//            che il POD è crashato durante un tentativo precedente. In questo caso
//            si scrive 'error' e si salta il tentativo. L'operatore deve
//            rimettere 'pending' manualmente dopo aver verificato la causa.
//            Questo check avviene in controllaOTA() prima di armare il trigger.
//
//   FIX 3 — Scrittura 'downloading' anticipata: ora avviene PRIMA di http.begin()
//            così anche in caso di crash immediato il DB non mostra 'pending'.
//
//   DIPENDENZA AGGIUNTA: esp_task_wdt.h (incluso in ESP-IDF, già disponibile)
//
// Logica aggiornamento firmware Over-The-Air via GitHub Releases + Supabase.
//
// ARCHITETTURA:
//   - Il TaskCloud (Rete_Cloud.ino) legge dal DB i campi ota_version,
//     ota_url, ota_sha256 nella FASE 1 (ogni 30s / ogni riavvio).
//   - Se ota_version > FW_VERSION (confronto SemVer numerico) E
//     il flag g_otaPending è TRUE, viene chiamata eseguiOTA().
//   - eseguiOTA() gira su Core0 (TaskCloud), NON nel loop() principale.
//     Durante il download la rete ESP-NOW è de-facto sospesa (WiFi in
//     modalità solo-STA, nessun loop() chiamato → Visore perde il link).
//   - LED durante OTA: lampeggio BIANCO veloce (50ms on/off).
//   - SHA256 verificato byte per byte durante lo streaming del bin
//     tramite mbedtls_md (già incluso in ESP-IDF/Arduino ESP32).
//   - Prima del reboot: RPC aggiorna_stato_ota con stato 'ok' e
//     nuovo FW_VERSION → visibile subito su Supabase dashboard.
//
// TRIGGER:
//   Boot: sempre (controllo una-tantum all'avvio)
//   Runtime: ogni OTA_CHECK_INTERVAL_MS (72h) dall'ultimo controllo
//
// SICUREZZA:
//   - L'URL è letto esclusivamente dal DB Supabase (non hardcoded).
//   - Il SHA256 è scritto nel DB dall'operatore (non dal POD).
//   - Se SHA256 non corrisponde: Update.abort(), nessun reboot, LED ROSSO.
//   - Se download fallisce: retry al prossimo boot o al prossimo intervallo.
//
// COLORI LED durante OTA:
//   Bianco lampeggiante → download in corso
//   Verde fisso          → OTA ok, reboot imminente (2s)
//   Rosso lampeggiante   → errore (SHA256 mismatch o download fallito)
//
// NON MODIFICARE: questa funzione è chiamata SOLO da TaskCloud (Core0).
//   Mai chiamare eseguiOTA() dal loop() (Core1) — race condition su WiFi.
//
// DIPENDENZE (già presenti nel progetto):
//   - confrontaVersioniSemVer() / aggiornamentoDisponibile() → Utils.ino
//   - impostaLED()                                           → Utils.ino
//   - chiamaRPC()                                            → Rete_Cloud.ino
//   - NOME_POD, SUPABASE_URL, SUPABASE_KEY                  → Rete_Cloud.ino
//   - FW_VERSION                                             → AYE_POD_v37_DEV.ino
// =========================================================================

#include <Update.h>
#include <HTTPClient.h>
#include "mbedtls/md.h"
#include "esp_task_wdt.h"   // disabilita TWDT durante download (FIX anti-watchdog)

// ── Dichiarazioni extern — risolve l'ordine alfabetico di compilazione Arduino ──
// Arduino IDE compila i file .ino in ordine alfabetico: OTA.ino viene PRIMA
// di Rete_Cloud.ino, quindi le costanti e funzioni definite lì non sono ancora
// visibili. Gli extern istruiscono il compilatore che esistono altrove nel progetto.
extern const char* NOME_POD;       // es. "AYE_POD_002"  — definito in Rete_Cloud.ino
extern const char* SUPABASE_URL;   // endpoint REST Supabase — definito in Rete_Cloud.ino
extern const char* SUPABASE_KEY;   // anon key — definito in Rete_Cloud.ino
// chiamaRPC() — forward declaration (la definizione è in Rete_Cloud.ino)
// NOTA: il default argument (= nullptr) va specificato UNA SOLA VOLTA in C++.
// Lo manteniamo nella definizione in Rete_Cloud.ino, qui lo omettiamo.
int chiamaRPC(const char* nome, const String& body, String* respOut);

// ── Parametri retry OTA ────────────────────────────────────────────────────
#define OTA_MAX_RETRY      3        // tentativi prima di impostare DB=error
// OTA_ERROR_RESET_S rimosso in V2.1.8: auto-reset basato su DB ota_retry_ts (UTC)
// Namespace NVS separato da 'calib_data' — nessun conflitto con offset bussola
#define OTA_NVS_NS         "ota_ctrl"
#define OTA_NVS_RETRY      "retry_count"   // uint8: 0..OTA_MAX_RETRY
#define OTA_NVS_ERROR_TS   "error_ts"      // DEPRECATO v2.1.8 — mantenuto per compatibilità
// ota_force e ota_retry_ts gestiti lato DB — più affidabili di millis() NVS
// (millis() riparte da 0 ad ogni boot, NVS error_ts era incompatibile con reboot)

// ── Intervallo controllo OTA (72 ore in ms) ───────────────────────────────
#define OTA_CHECK_INTERVAL_MS (72UL * 60UL * 60UL * 1000UL)

// ── Stato globale OTA — scritto da TaskCloud, letto da TaskCloud ──────────
// Tutti volatile: accesso cross-core (anche se in pratica solo Core0 scrive)
volatile bool   g_otaPending       = false;   // TRUE = versione remota > locale
volatile bool   g_otaInCorso       = false;   // TRUE = download attivo (lock)
char            g_otaUrlPending[256]    = "";  // URL bin da scaricare
char            g_otaSha256Pending[65] = "";   // SHA256 hex atteso (64 char + \0)
char            g_otaVersionePending[16] = ""; // es. "2.1.0"
unsigned long   g_otaUltimoCheck   = 0;       // millis() dell'ultimo controllo

// ── Dichiarazioni forward (usate solo internamente) ───────────────────────
static bool    _verificaSha256(const char* hexAtteso, const uint8_t* digest);
static void    _hexToBytes(const char* hex, uint8_t* out, int len);

// =========================================================================
// controllaOTA() — V2.1.8: NVS retry + DB timestamp + ota_force bypass
//
// Firma: 6 parametri (aggiunto otaForce, otaRetryTs rispetto a V2.1.6)
//
// Logica:
//   1. Legge retry_count da NVS 'ota_ctrl'
//   2. ota_force=true (DB): bypass totale retry_count, reset NVS e procedi
//   3. retry_count >= OTA_MAX_RETRY: DB=error (RPC setta ota_retry_ts UTC)
//   4. DB=error + ota_retry_ts presente: auto-reset a pending dopo 1h UTC
//      (usa timestamp DB, non millis NVS — stabile tra reboot)
//   5. DB=error senza ota_retry_ts: error manuale, attende operatore
//   6. DB=pending + versione > fw locale: arma g_otaPending
// =========================================================================
void controllaOTA(const char* otaVersion, const char* otaUrl, const char* otaSha256,
                  const char* otaStato,   const char* otaForce, const char* otaRetryTs) {
  if (!otaVersion || strlen(otaVersion) == 0) return;
  if (!otaUrl    || strlen(otaUrl)    == 0) return;
  if (!otaSha256 || strlen(otaSha256) == 0) return;
  if (g_otaInCorso || g_otaPending) return;

  // ── Leggi NVS 'ota_ctrl' (solo retry_count — error_ts ora è nel DB) ──
  Preferences nvsOta;
  nvsOta.begin(OTA_NVS_NS, true);
  uint8_t retryCount = nvsOta.getUChar(OTA_NVS_RETRY, 0);
  nvsOta.end();

  bool forceOta = (otaForce && strcmp(otaForce, "true") == 0);
  Serial.printf("[OTA] controllaOTA: stato=%s retry=%d force=%s\n",
                otaStato ? otaStato : "null", (int)retryCount,
                forceOta ? "YES" : "no");

  // ── ota_force=true: bypass totale — operatore forza nuovo tentativo ──
  // Azzera retry_count NVS e procedi direttamente come se stato=pending
  if (forceOta) {
    Serial.println("[OTA] ota_force=true → bypass retry, NVS reset");
    Preferences nvsW; nvsW.begin(OTA_NVS_NS, false);
    nvsW.putUChar(OTA_NVS_RETRY, 0);
    nvsW.remove(OTA_NVS_ERROR_TS);
    nvsW.end();
    retryCount = 0;
    // Salta tutti i check stato e procedi al trigger
    goto arm_trigger;
  }

  // ── Max retry raggiunto ───────────────────────────────────────────────
  // La RPC aggiorna_stato_ota salva ota_retry_ts=NOW() lato DB quando p_stato=error
  if (retryCount >= OTA_MAX_RETRY) {
    Serial.printf("[OTA] Max retry (%d) → DB=error (RPC salva timestamp UTC)\n", OTA_MAX_RETRY);
    String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\",\"p_stato\":\"error\"}";
    chiamaRPC("aggiorna_stato_ota", body, nullptr);
    Preferences nvsW; nvsW.begin(OTA_NVS_NS, false);
    nvsW.putUChar(OTA_NVS_RETRY, 0);  // reset per ciclo successivo post-1h
    nvsW.end();
    return;
  }

  // ── Gestione DB=error ────────────────────────────────────────────────
  if (otaStato && strcmp(otaStato, "error") == 0) {
    if (otaRetryTs && strlen(otaRetryTs) > 0) {
      // ota_retry_ts presente → error da firmware, auto-reset dopo 1h
      // Il DB contiene un ISO timestamp UTC. Lo confrontiamo con NOW() lato DB
      // tramite RPC — il POD non ha clock reale, ma la RPC di reset viene
      // chiamata solo se la FASE1 (GET) riesce, il che significa che la
      // connessione è disponibile.
      // SEMPLIFICAZIONE: dopo OTA_MAX_RETRY boot con FASE1 OK post-error,
      // resettiamo pending. Usiamo un secondo contatore NVS 'boot_since_err'.
      Preferences nvsW; nvsW.begin(OTA_NVS_NS, false);
      uint8_t bootsSinceErr = nvsW.getUChar("boot_err_cnt", 0) + 1;
      nvsW.putUChar("boot_err_cnt", bootsSinceErr);
      nvsW.end();
      Serial.printf("[OTA] DB=error firmware, boot post-error=%d/3\n", (int)bootsSinceErr);
      // Dopo 3 boot con cloud OK post-error → reset a pending
      if (bootsSinceErr >= 3) {
        Serial.println("[OTA] 3 boot post-error → DB=pending, reset NVS");
        String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\",\"p_stato\":\"pending\"}";
        chiamaRPC("aggiorna_stato_ota", body, nullptr);
        Preferences nvsC; nvsC.begin(OTA_NVS_NS, false);
        nvsC.putUChar("boot_err_cnt", 0);
        nvsC.end();
        // Continua verso trigger
      } else {
        return;  // attendi altri boot
      }
    } else {
      // Nessun ota_retry_ts → error manuale operatore, non auto-resetta
      Serial.println("[OTA] DB=error manuale — serve intervento operatore");
      return;
    }
  }

  // ── DB=ok: niente da fare ────────────────────────────────────────────
  if (otaStato && strcmp(otaStato, "ok") == 0) return;

  // ── DB=pending: confronto versione ───────────────────────────────────
  if (!aggiornamentoDisponibile(otaVersion)) return;

arm_trigger:
  strncpy(g_otaUrlPending,      otaUrl,     sizeof(g_otaUrlPending)     - 1);
  strncpy(g_otaSha256Pending,   otaSha256,  sizeof(g_otaSha256Pending)  - 1);
  strncpy(g_otaVersionePending, otaVersion, sizeof(g_otaVersionePending)- 1);
  g_otaUrlPending[sizeof(g_otaUrlPending)-1]           = '\0';
  g_otaSha256Pending[sizeof(g_otaSha256Pending)-1]     = '\0';
  g_otaVersionePending[sizeof(g_otaVersionePending)-1] = '\0';

  g_otaPending = true;
  Serial.printf("[OTA] Aggiornamento disponibile: %s \u2192 %s\n",
                FW_VERSION, g_otaVersionePending);
  Serial.printf("[OTA] URL: %s\n", g_otaUrlPending);
}

// =========================================================================
// eseguiOTA()
//
// Chiamata da TaskCloud DOPO aver inviato la telemetria del ciclo corrente.
// Esegue il download streaming con verifica SHA256 e flash sul secondo slot.
//
// Il dispositivo è "morto" durante questa operazione:
//   - Loop() Core1 NON viene bloccato esplicitamente MA la flash è in
//     scrittura → BNO085 e GPS continuano a girare ma senza effetto
//   - ESP-NOW non trasmette (nessun inviaDatiVisore() chiamato durante il blocco)
//   - LED bianco lampeggiante indica stato OTA visivamente
//
// Return: true se OTA completata e reboot eseguito (non ritorna mai),
//         false se errore (il POD continua a funzionare normalmente).
// =========================================================================
bool eseguiOTA() {
  if (!g_otaPending || g_otaInCorso) return false;
  if (strlen(g_otaUrlPending) == 0)  return false;

  g_otaInCorso = true;
  g_otaPending = false;

  Serial.println("[OTA] ═══════════════════════════════════════");
  Serial.printf ("[OTA] Avvio download firmware %s\n", g_otaVersionePending);
  Serial.println("[OTA] ═══════════════════════════════════════");
  Serial.println("[OTA] ⚠️  Il dispositivo sarà non-responsivo durante il download.");

  // ── V2.1.6: NVS retry_count++ PRIMA del download (zero HTTP) ─────────
  // Salviamo lo stato in NVS locale (Preferences) invece di una chiamata HTTP.
  // Questo elimina il terzo SSL context che causava heap panic.
  // Se il POD crasha durante il download, retry_count è già incrementato
  // e controllaOTA() al boot successivo gestisce il retry o l'errore.
  {
    Preferences nvsOta;
    nvsOta.begin(OTA_NVS_NS, false);
    uint8_t retry = nvsOta.getUChar(OTA_NVS_RETRY, 0);
    retry++;
    nvsOta.putUChar(OTA_NVS_RETRY, retry);
    nvsOta.end();
    Serial.printf("[OTA] NVS retry_count: %d (salvato pre-download)\n", (int)retry);
  }

  // ── Disabilita Task Watchdog Timer ───────────────────────────────────
  esp_task_wdt_delete(NULL);
  Serial.println("[OTA] TWDT disabilitato per il download");

  // ── Deinizializza ESP-NOW prima del download ──────────────────────────
  // MOTIVAZIONE (confermata da log e datasheet ESP32-S3):
  //   Il heap oscilla tra 166KB e 215KB ogni 2s per le connessioni HTTPS
  //   del TaskCloud. Quando http.GET() OTA viene chiamato col heap al minimo
  //   (~166KB), l'allocatore SSL crasha silenziosamente (heap panic → reboot).
  //   esp_now_deinit() libera i buffer ESP-NOW (~16KB) e — cosa più importante
  //   — impedisce alle callback OnDataRecvDalVisore di allocare memoria
  //   durante il download, rendendo il heap stabile per tutta la durata.
  //   NON tocchiamo WiFi.mode() né softAPdisconnect(): confermato da log
  //   v2.1.9 che WiFi.mode() con ESP-NOW attivo crasha il driver WiFi.
  //   Il softAP rimane attivo: il Visore perde ESP-NOW ma non il beacon AP.
  //   Al reboot post-OTA: setupESPNOW() reinizializza tutto automaticamente.
  //   In caso di errore download: ESP.restart() per sicurezza (no reinit).
  esp_now_deinit();
  Serial.println("[OTA] ESP-NOW deinizializzato — heap liberato per download");
  vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms stabilizzazione allocatore

  // Log heap disponibile prima del download
  Serial.printf("[OTA] FreeHeap pre-download: %u bytes\n", ESP.getFreeHeap());

  // ── Setup mbedtls SHA256 streaming ───────────────────────────────────
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  if (mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 0) != 0) {
    Serial.println("[OTA] ❌ Errore init SHA256");
    esp_task_wdt_add(NULL);
    // ELIMINATA: g_otaInCorso = false; — loop() deve restare in pausa fino al reboot
    ESP.restart(); // riavvia per reinizializzare ESP-NOW
    return false;
  }
  mbedtls_md_starts(&ctx);

  // ── HTTP GET streaming ────────────────────────────────────────────────
  // V2.2.1: URL Supabase Storage, WiFi in WIFI_AP_STA, ESP-NOW deinizializzato.
  HTTPClient http;
  http.begin(g_otaUrlPending);
  http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  http.setTimeout(60000);      // 60000ms — HTTPClient::setTimeout vuole MILLISECONDI
  http.setConnectTimeout(8000);

  int httpCode = http.GET();
  if (httpCode != HTTP_CODE_OK) {
    Serial.printf("[OTA] ❌ HTTP %d — download fallito\n", httpCode);
    http.end();
    mbedtls_md_free(&ctx);
    esp_task_wdt_add(NULL); // riabilita TWDT
    // ELIMINATA: g_otaInCorso = false; — loop() deve restare in pausa fino al reboot
    String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\",\"p_stato\":\"error\"}";
    chiamaRPC("aggiorna_stato_ota", body, nullptr);
    for (int i = 0; i < 20; i++) {
      impostaLED(pixel.Color(255, 0, 0), true);
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    Serial.println("[OTA] ❌ Errore download — ESP.restart() per ripristino ESP-NOW");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP.restart();
    return false; // mai raggiunto
  }

  int contentLength = http.getSize();
  Serial.printf("[OTA] Dimensione firmware: %d bytes\n", contentLength);

  if (!Update.begin(contentLength)) {
    Serial.printf("[OTA] ❌ Update.begin fallito: %s\n", Update.errorString());
    http.end();
    mbedtls_md_free(&ctx);
    esp_task_wdt_add(NULL);
    // ELIMINATA: g_otaInCorso = false; — loop() deve restare in pausa fino al reboot
    String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\",\"p_stato\":\"error\"}";
    chiamaRPC("aggiorna_stato_ota", body, nullptr);
    Serial.println("[OTA] ❌ Update.begin — ESP.restart() per ripristino ESP-NOW");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP.restart();
    return false; // mai raggiunto
  }

  // ── Streaming: legge chunk per chunk, aggiorna SHA256 e flash ────────
  // V2.2.3: buffer nello HEAP invece che sullo stack del TaskCloud.
  // uint8_t buf[4096] sullo stack consumava 4KB dei soli 8KB disponibili,
  // causando stack overflow durante l'handshake mbedtls SSL (~5KB).
  // Con malloc() il buffer va nell'heap (>210KB liberi) — zero stack pressure.
  WiFiClient* stream = http.getStreamPtr();
  const size_t CHUNK = 4096;
  uint8_t* buf = (uint8_t*)malloc(CHUNK);
  if (!buf) {
    Serial.println("[OTA] ❌ malloc buffer fallito — ESP.restart()");
    http.end();
    mbedtls_md_free(&ctx);
    esp_task_wdt_add(NULL);
    // ELIMINATA: g_otaInCorso = false; — loop() deve restare in pausa fino al reboot
    ESP.restart();
    return false;
  }
  size_t totaleRicevuto = 0;
  unsigned long ultimoLed = 0;
  bool ledStato = false;

  while (http.connected() && (contentLength > 0 || contentLength == -1)) {
    size_t disponibili = stream->available();
    if (disponibili == 0) {
      vTaskDelay(10 / portTICK_PERIOD_MS);
      continue;
    }

    size_t daLeggere = min(disponibili, CHUNK);
    size_t letti = stream->readBytes(buf, daLeggere);
    if (letti == 0) break;

    // SHA256 incrementale
    mbedtls_md_update(&ctx, buf, letti);

    // Flash write
    size_t scritti = Update.write(buf, letti);
    if (scritti != letti) {
      Serial.printf("[OTA] ❌ Errore write: attesi %u scritti %u\n",
                    (unsigned)letti, (unsigned)scritti);
      Update.abort();
      http.end();
      mbedtls_md_free(&ctx);
      esp_task_wdt_add(NULL);
      free(buf); // V2.2.3: libera heap buffer
      // ELIMINATA: g_otaInCorso = false; — loop() deve restare in pausa fino al reboot
      String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\",\"p_stato\":\"error\"}";
      chiamaRPC("aggiorna_stato_ota", body, nullptr);
      Serial.println("[OTA] ❌ Write error — ESP.restart()");
      vTaskDelay(500 / portTICK_PERIOD_MS);
      ESP.restart();
      return false; // mai raggiunto
    }

    totaleRicevuto += letti;
    if (contentLength > 0) contentLength -= letti;

    // LED bianco lampeggiante 50ms (visivo per l'utente)
    unsigned long ora = millis();
    if (ora - ultimoLed > 50) {
      ultimoLed = ora;
      ledStato = !ledStato;
      if (ledStato) pixel.setPixelColor(0, pixel.Color(255,255,255));
      else          pixel.setPixelColor(0, 0);
      pixel.show();
    }

    // Progresso ogni 64KB
    if (totaleRicevuto % (64 * 1024) < CHUNK) {
      Serial.printf("[OTA] Scaricati: %u bytes\n", (unsigned)totaleRicevuto);
    }
  }

  free(buf); // V2.2.3: libera heap buffer dopo download
  http.end();

  // ── Riabilita TWDT — download completato ───────────────────────
  esp_task_wdt_add(NULL);
  Serial.println("[OTA] TWDT riabilitato");

  Serial.printf("[OTA] Download completato: %u bytes totali\n",
                (unsigned)totaleRicevuto);

  // ── SHA256 finale ─────────────────────────────────────────────────────
  uint8_t digest[32];
  mbedtls_md_finish(&ctx, digest);
  mbedtls_md_free(&ctx);

  if (!_verificaSha256(g_otaSha256Pending, digest)) {
    Serial.println("[OTA] ❌ SHA256 MISMATCH — firmware rifiutato!");
    Serial.print("[OTA]    Atteso:   "); Serial.println(g_otaSha256Pending);
    // Stampa hash calcolato per debug
    Serial.print("[OTA]    Calcolato: ");
    for (int i = 0; i < 32; i++) Serial.printf("%02x", digest[i]);
    Serial.println();
    Update.abort();
    // ELIMINATA: g_otaInCorso = false; — loop() deve restare in pausa fino al reboot
    for (int i = 0; i < 32; i++) {
      impostaLED(pixel.Color(255, 0, 0), true);
      vTaskDelay(250 / portTICK_PERIOD_MS);
    }
    String body = "{\"p_pod_id\":\"" + String(NOME_POD) +
                  "\",\"p_stato\":\"error\"}";
    chiamaRPC("aggiorna_stato_ota", body, nullptr);
    free(buf); // V2.2.3
    Serial.println("[OTA] ❌ SHA256 mismatch — ESP.restart()");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP.restart();
    return false; // mai raggiunto
  }

  Serial.println("[OTA] ✅ SHA256 verificato — firmware integro");

  // ── Finalizza flash ───────────────────────────────────────────────────
  if (!Update.end(true)) {
    Serial.printf("[OTA] ❌ Update.end fallito: %s\n", Update.errorString());
    // ELIMINATA: g_otaInCorso = false; — loop() deve restare in pausa fino al reboot
    String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\",\"p_stato\":\"error\"}";
    chiamaRPC("aggiorna_stato_ota", body, nullptr);
    free(buf); // V2.2.3
    Serial.println("[OTA] ❌ Update.end — ESP.restart()");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    ESP.restart();
    return false; // mai raggiunto
  }

  Serial.println("[OTA] ✅ Flash completato sul secondo slot OTA");

  // ── V2.1.6: Reset NVS retry_count prima del reboot ───────────────────
  {
    Preferences nvsOta;
    nvsOta.begin(OTA_NVS_NS, false);
    nvsOta.putUChar(OTA_NVS_RETRY, 0);
    nvsOta.remove(OTA_NVS_ERROR_TS);
    nvsOta.end();
    Serial.println("[OTA] NVS retry_count → 0 (reset)");
  }

  // ── Notifica DB pre-reboot ─────────────────────────────────────────────
  // Ora è sicuro chiamare RPC: il download è completato e il chip non è
  // sotto stress di scrittura flash. Un solo SSL context attivo.
  {
    String body = "{\"p_pod_id\":\"" + String(NOME_POD) +
                  "\",\"p_stato\":\"ok\"" +
                  ",\"p_fw_attuale\":\"" + String(g_otaVersionePending) + "\"}";
    int rc = chiamaRPC("aggiorna_stato_ota", body, nullptr);
    Serial.printf("[OTA] Notifica DB pre-reboot: rc=%d\n", rc);
  }

  // LED verde fisso: OTA ok, reboot imminente
  impostaLED(pixel.Color(0, 255, 0), false);
  Serial.println("[OTA] ✅ Reboot in 2s...");
  vTaskDelay(2000 / portTICK_PERIOD_MS);

  ESP.restart();
  return true; // mai raggiunto
}

// =========================================================================
// _verificaSha256()
// Confronta il digest calcolato (32 byte raw) con la stringa hex dal DB (64 char).
// =========================================================================
static bool _verificaSha256(const char* hexAtteso, const uint8_t* digest) {
  if (!hexAtteso || strlen(hexAtteso) != 64) {
    Serial.println("[OTA] ⚠️  SHA256 nel DB non valido (deve essere 64 char hex)");
    return false;
  }
  uint8_t atteso[32];
  _hexToBytes(hexAtteso, atteso, 32);
  return memcmp(digest, atteso, 32) == 0;
}

// =========================================================================
// _hexToBytes()
// Converte stringa hex lowercase in array di byte.
// =========================================================================
static void _hexToBytes(const char* hex, uint8_t* out, int len) {
  for (int i = 0; i < len; i++) {
    char h[3] = { hex[i*2], hex[i*2+1], '\0' };
    out[i] = (uint8_t)strtol(h, nullptr, 16);
  }
}
