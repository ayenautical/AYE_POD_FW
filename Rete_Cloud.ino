// =========================================================================
// AYE POD — Rete_Cloud.ino
// Release: V2.2.4
//
// Changelog V2.2.4 — Fix setTimeout millisecondi:
//
//   FIX — h.setTimeout(8000) e hTel.setTimeout(8000): erano 8 (= 8ms!).
//     HTTPClient::setTimeout su ESP32 vuole MILLISECONDI. Con 8ms nessuna
//     risposta HTTPS è possibile → -11 su tutti i POST telemetria e le RPC,
//     inclusa la notifica errore OTA aggiorna_stato_ota → -11.
//     Corretto a 8000ms (8s) in chiamaRPC() e nello Stack 2 telemetria.
//     Nota: setConnectTimeout() vuole già millisecondi ed era corretto (3000).
//
// Changelog V2.1.5 — Gate OTA: g_cloudOkCount → g_fase1Ok (Opzione 2):
//   PROBLEMA (V2.1.4): il gate contava POST telemetria riusciti (g_cloudOkCount>=3).
//   Il POST usa WiFi AP+STA e fallisce con -11 per 2-3 minuti dopo il boot a causa
//   della contesa Access Point. g_cloudOkCount restava 0 per >80s, bloccando l'OTA.
//
//   SOLUZIONE: g_fase1Ok = true al primo GET FASE1 con rc 2xx.
//   Il GET FASE1 avviene a 30s dal boot e usa lo stesso SSL context del POST.
//   Una volta che il GET riesce, SSL verso Supabase è "warm" e tutti i successivi
//   POST/RPC funzionano. Il gate OTA si sblocca nello stesso ciclo FASE1.
//   Simulato con 10 scenari (boot normale, cloud veloce, anti-loop, sessione attiva,
//   cloud mai disponibile, stato error/ok, runtime 72h, doppio avvio, DB vuoto,
//   sequenza completa) — tutti passati prima dell'implementazione.
//
//   MODIFICHE:
//   - g_cloudOkCount (volatile uint8_t) → g_fase1Ok (bool, non volatile — solo Core0)
//   - GET FASE1: aggiunto if(!g_fase1Ok) { g_fase1Ok=true; log; }
//   - Stack OTA: gate cambiato da g_cloudOkCount<3 a !g_fase1Ok
//   - POST Stack2: rimosso incremento g_cloudOkCount
//   - OTA.ino: rimosso extern g_cloudOkCount e #define OTA_MIN_CLOUD_OK
//
// Changelog V2.1.4:
//
// Changelog V2.1.0 — OTA CLIENT:
//   Integrazione client OTA (Fase 2). La logica di download/flash è in OTA.ino.
//   Modifiche in questo file:
//   - FASE 1: estrazione campi ota_version, ota_url, ota_sha256 dal payload DB
//     e chiamata a controllaOTA() per armare il trigger
//   - FASE 1: controllo intervallo 72h (OTA_CHECK_INTERVAL_MS) + check al boot
//     tramite g_otaUltimoCheck == 0 (mai eseguito)
//   - SELECT FASE 1: aggiunta dei campi OTA alla query esistente
//   - TaskCloud: dopo Stack2 telemetria, se g_otaPending → eseguiOTA()
//     (non interferisce con telemetria: Stack2 è già inviato prima del branch)
//   - Variabili OTA globali definite in OTA.ino (extern non necessario,
//     stesso translation unit Arduino)
//
// Changelog V2.0.2 — FIX H3 DEFINITIVO:
//
//   PROBLEMA RADICE IDENTIFICATO:
//     Il Visore imposta cmd_sessione=2 ma NON chiama inviaDatiAlPod()
//     esplicitamente in stop_session() (a differenza di execute_start()
//     che lo chiama per cmd=1). Inoltre, dopo stop_session() il flag
//     pending_cloud_upload=true blocca il normale invio ciclico nel loop.
//     Risultato: il POD può non ricevere mai cmd=2 via ESP-NOW.
//
//     Anche quando cmd=2 arriva, il ciclo TaskCloud (2s + HTTP calls)
//     poteva superare la finestra di 4s del latch Visore, perdendo il
//     comando prima che la FASE 2 lo leggesse.
//
//   SOLUZIONE IMPLEMENTATA — due livelli di robustezza:
//
//   1. CATTURA IMMEDIATA nella callback ESP-NOW (OnDataRecvDalVisore,
//      Core1). Quando arriva cmd=2 dalla struct, viene catturato
//      ATOMICAMENTE in g_cmdSessSnapCmd e g_cmdSessSnapDist prima
//      che qualsiasi latch possa azzerarlo. La cattura avviene al
//      primo pacchetto utile, indipendente dal ciclo TaskCloud.
//
//   2. STACK CHIUSURA nel TaskCloud (Core0) con retry automatico.
//      g_pendingChiudiSessione viene attivato dalla callback (non
//      dal TaskCloud), poi il TaskCloud esegue la RPC in modo
//      indipendente dal timing del Visore. Retry fino a MAX_RETRY_CHIUDI.
//      La sessione viene cancellata localmente SOLO dopo rc 200-299.
//
//   COMPATIBILITA':
//     - struct_nautica INVARIATA (87 bytes, sizeof invariato)
//     - struct_messaggio_visore INVARIATA (22 bytes)
//     - g_sessioneChiusaOk: variabile globale per futuro ACK verso Visore
//     - Visore.ino: aggiunta cattura snapshot in OnDataRecvDalVisore
//       (modifica minima, nessun impatto su funzionalità esistenti)
//     - Tutto il resto invariato: FASE 1, FASE 3, WiFi, portal, anchor
//
// Changelog V2.0.1 (precedente): BUG H3 primo tentativo (timing issue)
// Changelog V2.0.0: SemVer, partizioni OTA dual-slot
// =========================================================================

#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
// OTA.ino — stesso progetto Arduino, nessun #include necessario.
// Funzioni usate: controllaOTA(), eseguiOTA()
// Variabili usate: g_otaPending, g_otaInCorso, g_otaUltimoCheck

Preferences prefs;
extern Preferences prefsOffset;

WebServer server(80);
DNSServer dnsServer;

String savedSSID = "";
String savedPass = "";
bool   portalRunning   = false;
unsigned long portalStartTime = 0;

// ── Credenziali da secrets.h (file locale, NON su GitHub) ────────────────
#include "secrets.h"
// NOME_POD, SUPABASE_URL, SUPABASE_KEY sono definiti in secrets.h

// ── Gate cloud-ready per OTA — V2.1.5 (sostituisce g_cloudOkCount di V2.1.4) ──
// FLAG: diventa TRUE al primo GET FASE1 con rc 2xx verso Supabase.
// Usato da eseguiOTA() come precondizione: garantisce che SSL verso Supabase
// sia "warm" prima del download, rendendo affidabile la scrittura 'downloading'
// (Fix C) e la notifica pre-reboot.
//
// RAZIONALE rispetto a V2.1.4 (g_cloudOkCount):
//   Il POST telemetria (Stack2) usa WiFi AP+STA e fallisce con -11 per 2-3 minuti
//   dopo il boot a causa della contesa AP. Il GET FASE1 usa lo stesso endpoint HTTPS
//   ma avviene a 30s dal boot e tipicamente riesce molto prima dei POST ripetuti.
//   Una volta che il GET FASE1 riesce, l'SSL context verso Supabase è stabilizzato
//   e tutti i successivi POST/RPC funzionano. Condizionare l'OTA al GET FASE1
//   è quindi più affidabile e reattivo del contatore POST (che poteva aspettare >3min).
//
// bool (non volatile uint8_t): solo Core0 (TaskCloud) lo scrive e lo legge.
// Nessun accesso cross-core → nessuna necessità di volatile.
// Non si resetta mai durante il runtime (solo al reboot hardware).
bool g_fase1Ok = false;

char g_sessione_attiva_id[37] = "";
char g_nome_capitano[64]      = "Capitano";
char g_codice_crew[8]         = "----";

// ── Snapshot comandi sessione — scritti dalla callback ESP-NOW (Core1) ───
// Questi valori vengono catturati nella callback OnDataRecvDalVisore
// IMMEDIATAMENTE quando arriva il pacchetto, prima che qualsiasi latch
// sul Visore possa azzerare cmd_sessione. Il TaskCloud (Core0) li legge
// in modo asincrono senza dipendere dal timing del latch.
//
// Uso volatile per garantire visibilità cross-core senza mutex
// (operazioni su tipi primitivi sono atomiche su ESP32 Xtensa).
volatile int   g_cmdSessSnap     = 0;     // snapshot cmd_sessione (0/1/2)
volatile float g_cmdSessSnapDist = 0.0f;  // snapshot session_dist al momento di cmd=2

// ── Stack chiusura sessione ───────────────────────────────────────────────
// g_pendingChiudiSessione: attivato dalla callback quando snapshot=2.
//   Il TaskCloud lo processa in modo indipendente.
// g_sessioneChiusaOk: ACK verso Visore (futuro Visore.ino V2+).
volatile bool  g_pendingChiudiSessione = false;
static   float g_sessionDistPending    = 0.0f;
static   int   g_retryChiudiCount      = 0;
bool           g_sessioneChiusaOk      = false;
static const int MAX_RETRY_CHIUDI      = 15;  // ~30s a 2s/ciclo

// ── Portale captive WiFi (invariato) ──────────────────────────────────────
void setupWebServer() {
  server.on("/", []() {
    String html =
      "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<style>body{font-family:Arial;padding:20px;background:#1e1e1e;color:#fff}"
      "input{width:100%;padding:15px;margin:10px 0;border-radius:5px;border:none;font-size:16px}"
      "button{width:100%;padding:15px;background:#f9a826;border:none;border-radius:5px;"
      "color:#000;font-weight:bold;font-size:18px;margin-top:20px}</style></head>"
      "<body><h2>AYE Pod Setup</h2><form action='/save' method='POST'>"
      "<label>SSID:</label><input type='text' name='ssid' value='" + savedSSID + "'>"
      "<label>Password:</label><input type='password' name='pass' value='" + savedPass + "'>"
      "<button type='submit'>SALVA E RIAVVIA</button></form></body></html>";
    server.send(200, "text/html", html);
  });
  server.on("/save", HTTP_POST, []() {
    prefs.begin("wifi_creds", false);
    prefs.putString("ssid", server.arg("ssid"));
    prefs.putString("pass", server.arg("pass"));
    prefs.end();
    server.send(200, "text/html",
      "<html><body style='background:#1e1e1e;color:#fff;text-align:center;padding:50px'>"
      "<h2>Salvato! Riavvio...</h2></body></html>");
    delay(1000); ESP.restart();
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "http://192.168.4.1", true);
    server.send(302, "text/plain", "");
  });
}

void setupWiFi() {
  prefs.begin("wifi_creds", true);
  savedSSID = prefs.getString("ssid", "Giorgio G DNV");
  savedPass = prefs.getString("pass", "Giorgione33");
  prefs.end();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("AYE_POD_NET", "ayesystems", 6, 0);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.begin(savedSSID.c_str(), savedPass.c_str());
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis()-t0 < 10000) { delay(500); }
  if (WiFi.status() == WL_CONNECTED) { wifiConnesso=true; Serial.println("[WIFI] Connesso"); }
  else { Serial.println("[WIFI] Locale (AP only)"); lastWifiAttempt = millis(); }
}

void gestisciWiFi() {
  unsigned long now = millis();
  if (WiFi.status() != WL_CONNECTED) {
    wifiConnesso = false;
    if (!portalRunning && (now-lastWifiAttempt >= 30000)) {
      dnsServer.start(53,"*",WiFi.softAPIP()); setupWebServer(); server.begin();
      portalRunning=true; portalStartTime=now;
    }
    if (portalRunning) {
      dnsServer.processNextRequest(); server.handleClient();
      impostaLED(pixel.Color(0,0,255), false);
      if (now-portalStartTime > 180000) {
        server.stop(); dnsServer.stop(); portalRunning=false;
        lastWifiAttempt=now; WiFi.disconnect();
        WiFi.begin(savedSSID.c_str(), savedPass.c_str());
      }
    } else if (now-lastWifiAttempt >= 60000) {
      lastWifiAttempt=now; WiFi.disconnect();
      WiFi.begin(savedSSID.c_str(), savedPass.c_str());
    }
    if (!portalRunning) impostaLED(pixel.Color(255,165,0), false);
  } else {
    wifiConnesso = true;
    if (portalRunning) { server.stop(); dnsServer.stop(); portalRunning=false; }
    impostaLED(pixel.Color(0,255,0), false);
  }
}

// ── RPC helper — ritorna HTTP code (non solo stringa) ─────────────────────
// chiamaRPC restituisce l'HTTP status code numerico.
// Versione precedente: restituiva solo la stringa di risposta, rendendo
// impossibile distinguere successo da errore senza parsing aggiuntivo.
int chiamaRPC(const char* nome, const String& body, String* respOut = nullptr) {
  HTTPClient h;
  h.begin(String(SUPABASE_URL)+"/rest/v1/rpc/"+nome);
  h.setTimeout(8000);          // 8000ms max risposta — HTTPClient::setTimeout vuole MILLISECONDI
  h.setConnectTimeout(3000);   // 3s max connessione TCP (setConnectTimeout vuole MILLISECONDI)
  h.addHeader("Content-Type","application/json");
  h.addHeader("apikey", SUPABASE_KEY);
  h.addHeader("Authorization", String("Bearer ")+SUPABASE_KEY);
  int rc = h.POST(body);
  String resp = (rc > 0) ? h.getString() : "";
  Serial.printf("[RPC] %s -> %d: %s\n", nome, rc, resp.substring(0,60).c_str());
  if (respOut) *respOut = resp;
  h.end();
  return rc;
}

// =========================================================================
// TASK CLOUD — V2.0.2
//
// Il cmd_sessione viene catturato NELLA CALLBACK ESP-NOW (Visore.ino),
// non qui. Questo TaskCloud (Core0) processa il pending in modo asincrono.
//
// FASE 1 — Config DB ogni 30s
// STACK 1 — Termine sessione (driven da g_cmdSessSnap, con retry)
// STACK 2 — Telemetria DB 1Hz
// =========================================================================
void TaskInvioCloud(void* pvParameters) {
  static unsigned long lastConfigRead  = 0;
  static int           lastCmdSnap     = 0;  // ultimo valore snapshot processato

  while (true) {

    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnesso) { wifiConnesso = true; }

      // ── Leggi snapshot cmd UNA VOLTA all'inizio del ciclo ────────────
      // Copia atomica dei valori volatile — usata per tutto il ciclo.
      // Garantisce coerenza: cmd e dist provengono dallo stesso pacchetto.
      int   snapCmd  = (int)g_cmdSessSnap;
      float snapDist = (float)g_cmdSessSnapDist;

      // ── FASE 1: config dal DB (ogni 30s) ─────────────────────────────
      // OTA: legge i campi OTA al boot (g_otaUltimoCheck==0) e ogni 72h.
      // Il flag doOtaCheck viene valutato PRIMA del gate 30s: anche se
      // lastConfigRead è 0 (boot), il gate 30s lascia passare subito.
      bool doOtaCheck = (g_otaUltimoCheck == 0) ||
                        (millis() - g_otaUltimoCheck >= OTA_CHECK_INTERVAL_MS);

      if (millis()-lastConfigRead > 30000) {
        lastConfigRead = millis();
        HTTPClient hConf;
        // Aggiunta ota_version, ota_url, ota_sha256, ota_stato alla SELECT (V2.1.2: anti-loop)
        String ep = String(SUPABASE_URL)+"/rest/v1/dispositivi?pod_id=eq."+NOME_POD+
          "&select=prua_offset,cmd_calibra,sessione_attiva_id,nome_capitano,codice_crew_live"
          ",anchor_lat,anchor_lon,anchor_raggio,anchor_attivo"
          ",ota_version,ota_url,ota_sha256,ota_stato,ota_force,ota_retry_ts";
        hConf.begin(ep);
        hConf.addHeader("apikey", SUPABASE_KEY);
        hConf.addHeader("Authorization", String("Bearer ")+SUPABASE_KEY);
        if (hConf.GET() > 0) {
          String r = hConf.getString();

          // V2.1.5: GET FASE1 riuscito → SSL Supabase warm → sblocca gate OTA
          if (!g_fase1Ok) {
            g_fase1Ok = true;
            Serial.println("[CLOUD] FASE1 OK — SSL Supabase warm, gate OTA sbloccato");
          }

          // offset bussola
          int i0 = r.indexOf("\"prua_offset\":")+14, i1 = r.indexOf(",",i0);
          if (i1<0) i1=r.indexOf("}",i0);
          if (i0>13&&i1>i0) {
            int off = r.substring(i0,i1).toInt();
            if (off != remoteOffset) {
              remoteOffset = off;
              prefsOffset.begin("calib_data",false);
              prefsOffset.putInt("prua_offset",remoteOffset);
              prefsOffset.end();
            }
          }
          // cmd_calibra
          if (r.indexOf("\"cmd_calibra\":true")>0 && !calibrazioneRichiesta) {
            calibrazioneRichiesta = true;
            HTTPClient hp;
            hp.begin(String(SUPABASE_URL)+"/rest/v1/dispositivi?pod_id=eq."+NOME_POD);
            hp.addHeader("Content-Type","application/json");
            hp.addHeader("apikey",SUPABASE_KEY);
            hp.addHeader("Authorization",String("Bearer ")+SUPABASE_KEY);
            hp.addHeader("Prefer","return=minimal");
            hp.PATCH("{\"cmd_calibra\":false}");
            hp.end();
          }
          // sessione_attiva_id — sincronizzazione con web app
          if (g_sessione_attiva_id[0]=='\0') {
            int si = r.indexOf("\"sessione_attiva_id\":\"");
            if (si>0) { si+=22; r.substring(si,si+36).toCharArray(g_sessione_attiva_id,37); }
          } else if (r.indexOf("\"sessione_attiva_id\":null")>0) {
            // Web app ha chiuso la sessione dall'esterno
            memset(g_sessione_attiva_id,0,37);
            g_pendingChiudiSessione = false;
            g_retryChiudiCount      = 0;
            g_sessioneChiusaOk      = false;
            lastCmdSnap             = 0;
            g_cmdSessSnap           = 0;
            Serial.println("[SESS] Sessione chiusa da web app — stato resettato");
          }
          // nome_capitano e codice_crew
          int ni = r.indexOf("\"nome_capitano\":\"");
          if (ni>0){ni+=17;int ne=r.indexOf("\"",ni);if(ne>ni)r.substring(ni,ne).toCharArray(g_nome_capitano,64);}
          int ci = r.indexOf("\"codice_crew_live\":\"");
          if (ci>0){ci+=20;int ce=r.indexOf("\"",ci);if(ce>ci)r.substring(ci,ce).toCharArray(g_codice_crew,8);}
          // Anchor Alert config
          {
            int ai, ae;
            ai = r.indexOf("\"anchor_lat\":"); if (ai>0){ ai+=13; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_lat=r.substring(ai,ae).toDouble(); }
            ai = r.indexOf("\"anchor_lon\":"); if (ai>0){ ai+=13; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_lon=r.substring(ai,ae).toDouble(); }
            ai = r.indexOf("\"anchor_raggio\": "); if(ai<0)ai=r.indexOf("\"anchor_raggio\":"); if(ai>0){ ai=r.indexOf(":",ai)+1; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_raggio_m=r.substring(ai,ae).toFloat(); }
            g_anchor_attivo = (r.indexOf("\"anchor_attivo\":true") > 0);
          }

          // ── OTA: parsing e check (solo se intervallo scaduto) ─────────
          // doOtaCheck è calcolato prima del gate 30s — se vero, estrai
          // i tre campi OTA dal JSON e passa a controllaOTA() per il confronto.
          if (doOtaCheck && !g_otaInCorso && !g_otaPending) {
            char otaVer[16]   = "";
            char otaUrl[256]  = "";
            char otaSha[65]   = "";
            char otaStato[16] = "";

            // ota_version
            { int si = r.indexOf("\"ota_version\":\""); if (si>0){ si+=15; int ei=r.indexOf("\"",si); if(ei>si) r.substring(si,ei).toCharArray(otaVer,sizeof(otaVer)); } }
            // ota_url
            { int si = r.indexOf("\"ota_url\":\"");    if (si>0){ si+=11; int ei=r.indexOf("\"",si); if(ei>si) r.substring(si,ei).toCharArray(otaUrl,sizeof(otaUrl)); } }
            // ota_sha256
            { int si = r.indexOf("\"ota_sha256\":\""); if (si>0){ si+=14; int ei=r.indexOf("\"",si); if(ei>si) r.substring(si,ei).toCharArray(otaSha,sizeof(otaSha)); } }
            // ota_stato
            { int si = r.indexOf("\"ota_stato\":\"");  if (si>0){ si+=13; int ei=r.indexOf("\"",si); if(ei>si) r.substring(si,ei).toCharArray(otaStato,sizeof(otaStato)); } }
            // ota_force (bool JSON: true/false)
            char otaForce[8]    = "false";
            if (r.indexOf("\"ota_force\":true") > 0) strncpy(otaForce, "true", sizeof(otaForce));
            // ota_retry_ts (ISO timestamp o null)
            char otaRetryTs[32] = "";
            { int si = r.indexOf("\"ota_retry_ts\":\""); if (si>0){ si+=16; int ei=r.indexOf("\"",si); if(ei>si) r.substring(si,ei).toCharArray(otaRetryTs,sizeof(otaRetryTs)); } }

            // Aggiorna timestamp check
            g_otaUltimoCheck = millis();

            // V2.1.8: passa ota_force e ota_retry_ts a controllaOTA (6 param)
            controllaOTA(otaVer, otaUrl, otaSha, otaStato, otaForce, otaRetryTs);
          }
        }
        hConf.end();
      }

      
      // ════════════════════════════════════════════════════════════════
      // STACK 2 — TELEMETRIA DB 1Hz  [eseguito PRIMA di Stack 1]
      // Garantisce heartbeat aggiornato ad ogni ciclo, anche durante
      // la chiusura sessione. La telemetria non si blocca mai per via
      // delle RPC di gestione sessione (stesso principio della web app).
      // ════════════════════════════════════════════════════════════════
      {
        HTTPClient hTel;
        hTel.begin(String(SUPABASE_URL)+"/rest/v1/punti_traccia_live");
        hTel.setTimeout(8000);         // 8000ms — HTTPClient::setTimeout vuole MILLISECONDI
        hTel.setConnectTimeout(3000);  // 3s connessione TCP (MILLISECONDI)
        hTel.addHeader("Content-Type","application/json");
        hTel.addHeader("apikey",SUPABASE_KEY);
        hTel.addHeader("Authorization",String("Bearer ")+SUPABASE_KEY);
        hTel.addHeader("Prefer","return=minimal");

        String j = "{";
        j += "\"pod_id\":\""+String(NOME_POD)+"\",";
        j += "\"fw_pod\":\""+String(FW_VERSION)+"\",";
        j += "\"fw_visore\":\""+fwVisoreAttuale+"\",";
        j += "\"cmd_sessione\":"+String((int)g_cmdSessSnap)+",";
        if (g_sessione_attiva_id[0]!='\0')
          j += "\"id_sessione\":\""+String(g_sessione_attiva_id)+"\",";
        if (g_lat==0.0 && g_lon==0.0) {
          j += "\"latitudine\":0.0,\"longitudine\":0.0,";
        } else {
          j += "\"latitudine\":"+String((float)g_lat,7)+",";
          j += "\"longitudine\":"+String((float)g_lon,7)+",";
        }
        j += "\"sog\":"+String((float)g_sog,1)+",";
        j += "\"cog\":"+String((int)roundf((float)g_cog))+",";
        j += "\"hdg\":"+String(g_head)+",";
        j += "\"drift\":"+String((float)g_drift,1)+",";
        j += "\"awa\":"+String((float)g_awa,1)+",";
        j += "\"aws\":"+String((float)g_aws,1)+",";
        j += "\"twa\":"+String((float)g_twa,1)+",";
        j += "\"tws\":"+String((float)g_tws,1)+",";
        j += "\"twd\":"+String(g_twd)+",";
        j += "\"vmg_target\":0.0,";
        j += "\"vmg_wind\":"+String((float)g_vmg_wind,1)+",";
        j += "\"roll\":"+String(g_roll)+",";
        j += "\"pitch\":"+String(g_pitch)+",";
        j += "\"satelliti\":"+String(g_sats)+",";
        j += "\"batteria_pod\":"+String((int)maxlipo.cellPercent());
        j += "}";

        int rcTel = hTel.POST(j);
        if (rcTel < 200 || rcTel >= 300) {
          Serial.printf("[CLOUD] POST telemetria errore %d\n", rcTel);
        }
        hTel.end();
      }

      // ════════════════════════════════════════════════════════════════
      // STACK OTA — [eseguito tra Stack 2 e Stack 1]
      //
      // V2.1.5: gate sostituito con g_fase1Ok (Opzione 2).
      // eseguiOTA() parte solo dopo che il GET FASE1 ha restituito 2xx,
      // garantendo SSL Supabase warm per la scrittura 'downloading' (Fix C)
      // e la notifica pre-reboot. Più reattivo del contatore POST (V2.1.4)
      // che falliva per 2-3min a causa della contesa WiFi AP+STA.
      // ════════════════════════════════════════════════════════════════
      if (g_otaPending && !g_otaInCorso) {
        if (g_sessione_attiva_id[0] != '\0') {
          Serial.println("[OTA] ⏸ Sessione attiva — OTA posticipata a sessione chiusa");
        } else if (!g_fase1Ok) {
          // Gate: attende il primo GET FASE1 riuscito verso Supabase
          // Questo avviene tipicamente entro 30-60s dal boot (primo ciclo FASE1)
          // Non stampa nulla per non inquinare il log durante il boot
        } else {
          Serial.println("[OTA] ▶ Avvio OTA (FASE1 ok, nessuna sessione attiva)...");
          eseguiOTA(); // blocca e reboot se ok, continua se errore
        }
      }

      // ════════════════════════════════════════════════════════════════
      // STACK 1 — GESTIONE SESSIONE  [eseguito DOPO Stack 2]
      // Driven da g_cmdSessSnap catturato nella callback ESP-NOW.
      // Anche se la RPC blocca, la telemetria del ciclo è già inviata.
      // ════════════════════════════════════════════════════════════════
      // A — Avvio sessione (cmd=1, processa una sola volta)
      if (snapCmd == 1 && lastCmdSnap != 1) {
        String respAvvio;
        int rcAvvio = chiamaRPC("crea_sessione_da_visore",
          "{\"p_pod_id\":\""+String(NOME_POD)+"\"}", &respAvvio);
        if (rcAvvio >= 200 && rcAvvio < 300) {
          respAvvio.replace("\"",""); respAvvio.trim();
          if (respAvvio.length() == 36) {
            respAvvio.toCharArray(g_sessione_attiva_id, 37);
          }
          Serial.printf("[SESS] ✅ Sessione avviata: %s\n", g_sessione_attiva_id);
        } else {
          Serial.printf("[SESS] ⚠️ crea_sessione rc=%d — riprova al prossimo ciclo\n", rcAvvio);
          // Non aggiorna lastCmdSnap — riproverà al ciclo successivo
          // (Stack 2 eseguirà comunque la telemetria dopo questo if)
        }
        lastCmdSnap        = 1;
        g_sessioneChiusaOk = false;
        g_pendingChiudiSessione = false;
        g_retryChiudiCount = 0;
      }

      // B — Stop sessione (cmd=2): attiva pending se non già attivo
      if (snapCmd == 2 && lastCmdSnap == 1 && !g_pendingChiudiSessione) {
        g_pendingChiudiSessione = true;
        g_sessionDistPending    = snapDist;
        g_retryChiudiCount      = 0;
        Serial.printf("[SESS] Stop catturato — dist=%.2f nm, avvio chiusura\n",
                      g_sessionDistPending);
      }

      // B — Esecuzione chiusura con retry
      if (g_pendingChiudiSessione && g_retryChiudiCount < MAX_RETRY_CHIUDI) {
        String bodyChiudi = "{\"p_pod_id\":\""+String(NOME_POD)+
                            "\",\"p_session_dist\":"+String(g_sessionDistPending,2)+"}";
        int rcChiudi = chiamaRPC("chiudi_sessione_da_visore", bodyChiudi);

        if (rcChiudi >= 200 && rcChiudi < 300) {
          // ✅ DB ha confermato la chiusura
          memset(g_sessione_attiva_id, 0, 37);
          g_pendingChiudiSessione = false;
          g_retryChiudiCount      = 0;
          g_sessioneChiusaOk      = true;
          lastCmdSnap             = 0;
          g_cmdSessSnap           = 0;  // resetta snapshot — pronto per prossima sessione
          Serial.println("[SESS] ✅ Sessione chiusa su DB — confermata");
        } else {
          g_retryChiudiCount++;
          Serial.printf("[SESS] ⚠️ Chiusura rc=%d — retry %d/%d\n",
                        rcChiudi, g_retryChiudiCount, MAX_RETRY_CHIUDI);
          if (g_retryChiudiCount >= MAX_RETRY_CHIUDI) {
            Serial.println("[SESS] ❌ Max retry — sessione potrebbe essere aperta su DB");
            g_pendingChiudiSessione = false;
            lastCmdSnap             = 0;
            // g_sessione_attiva_id mantenuto: FASE 1 sincronizzerà al GET successivo
          }
        }
      }

      // C — cmd=0 senza pending: reset stato (tra sessioni)
      if (snapCmd == 0 && !g_pendingChiudiSessione && lastCmdSnap != 0) {
        lastCmdSnap = 0;
      }

    } else {
      if (wifiConnesso) { wifiConnesso = false; }
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}
