// =========================================================================
// AYE POD — Rete_Cloud.ino
// Release: V2.3.1
//
// Changelog V2.3.0:
//   - FASE 1: aggiunto parse di wp_alert_lat, wp_alert_lon, wp_alert_attivo
//     dalla tabella dispositivi (campi già esistenti nel DB, zero migration)
//     → popola g_wp_lat, g_wp_lon, g_wp_active per calcolaWaypoint()
//   - Stack 2 (telemetria): aggiunti campi btw, dtw, vmg_wp al JSON
//     → popolano le colonne nuove in punti_traccia_live (migration SQL necessaria)
//   - vmg_target: ora inviato come g_vmg_wp (era sempre 0.0)
//
// Changelog V2.2.4 (precedente):
//   - Fix setTimeout millisecondi (era 8ms, ora 8000ms)
//
// Tutte le altre funzioni invariate: gestione sessione, OTA, WiFi portal,
// anchor, nome_capitano, codice_crew, prua_offset, cmd_calibra.
// =========================================================================

#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>

Preferences prefs;
extern Preferences prefsOffset;

WebServer server(80);
DNSServer dnsServer;

String savedSSID = "";
String savedPass = "";
bool   portalRunning   = false;
unsigned long portalStartTime = 0;
unsigned long lastWifiAttempt = 0;

// ── Credenziali da secrets.h ─────────────────────────────────────────────
// secrets.h usa #define con prefisso POD_ (es. POD_NOME, POD_SUPABASE_URL).
// Qui definiamo le variabili const char* con i nomi esatti che OTA.ino
// cerca tramite 'extern const char* NOME_POD / SUPABASE_URL / SUPABASE_KEY'.
// I nomi delle #define (POD_*) e delle variabili C++ (NOME_POD, ecc.) sono
// diversi → zero conflitto di espansione del preprocessore.
#include "secrets.h"
const char* NOME_POD     = POD_NOME;         // OTA.ino: extern const char* NOME_POD
const char* SUPABASE_URL = POD_SUPABASE_URL; // OTA.ino: extern const char* SUPABASE_URL
const char* SUPABASE_KEY = POD_SUPABASE_KEY; // OTA.ino: extern const char* SUPABASE_KEY

bool g_fase1Ok = false;

char g_sessione_attiva_id[37] = "";
char g_nome_capitano[64]      = "Capitano";
char g_codice_crew[8]         = "----";

volatile int   g_cmdSessSnap     = 0;
volatile float g_cmdSessSnapDist = 0.0f;

volatile bool  g_pendingChiudiSessione = false;
volatile bool  g_sessioneChiusaOk      = false;
volatile float g_sessionDistPending    = 0.0f;
volatile int   g_retryChiudiCount      = 0;
#define MAX_RETRY_CHIUDI 5

bool wifiConnesso = false;

// ── chiamaRPC() — INVARIATA ───────────────────────────────────────────────
int chiamaRPC(const char* nome, const String& body, String* respOut = nullptr) {
  HTTPClient h;
  h.begin(String(SUPABASE_URL)+"/rest/v1/rpc/"+nome);
  h.setTimeout(8000);
  h.setConnectTimeout(3000);
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

// ============================================================================
// TASK CLOUD — V2.3.0
// ============================================================================
void TaskInvioCloud(void* pvParameters) {
  static unsigned long lastConfigRead  = 0;
  static int           lastCmdSnap     = 0;

  while (true) {

    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnesso) { wifiConnesso = true; }

      int   snapCmd  = (int)g_cmdSessSnap;
      float snapDist = (float)g_cmdSessSnapDist;

      // ══════════════════════════════════════════════════════════════════
      // FASE 1 — Config DB ogni 30s
      // V2.3.0: aggiunto parse wp_alert_lat/lon/attivo per waypoint nav
      // ══════════════════════════════════════════════════════════════════
      if (millis()-lastConfigRead > 30000) {
        lastConfigRead = millis();
        HTTPClient hConf;
        String ep = String(SUPABASE_URL)+"/rest/v1/dispositivi?pod_id=eq."+NOME_POD+
          "&select=prua_offset,cmd_calibra,sessione_attiva_id,nome_capitano,codice_crew_live"
          ",anchor_lat,anchor_lon,anchor_raggio,anchor_attivo"
          ",wp_alert_lat,wp_alert_lon,wp_alert_attivo"  // ← NUOVO V2.3.0
          ",ota_version,ota_url,ota_sha256,ota_stato,ota_force,ota_retry_ts";
        hConf.begin(ep);
        hConf.addHeader("apikey", SUPABASE_KEY);
        hConf.addHeader("Authorization", String("Bearer ")+SUPABASE_KEY);
        if (hConf.GET() > 0) {
          String r = hConf.getString();

          if (!g_fase1Ok) {
            g_fase1Ok = true;
            Serial.println("[CLOUD] FASE1 OK — SSL warm, gate OTA sbloccato");
          }

          // prua_offset
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

          // sessione_attiva_id
          if (g_sessione_attiva_id[0]=='\0') {
            int si = r.indexOf("\"sessione_attiva_id\":\"");
            if (si>0) { si+=22; r.substring(si,si+36).toCharArray(g_sessione_attiva_id,37); }
          } else if (r.indexOf("\"sessione_attiva_id\":null")>0) {
            memset(g_sessione_attiva_id,0,37);
            g_pendingChiudiSessione = false;
            g_retryChiudiCount      = 0;
            g_sessioneChiusaOk      = false;
            lastCmdSnap             = 0;
            g_cmdSessSnap           = 0;
            Serial.println("[SESS] Sessione chiusa da web app");
          }

          // nome_capitano
          int ni = r.indexOf("\"nome_capitano\":\"");
          if (ni>0){ni+=17;int ne=r.indexOf("\"",ni);if(ne>ni)r.substring(ni,ne).toCharArray(g_nome_capitano,64);}

          // codice_crew
          int ci = r.indexOf("\"codice_crew_live\":\"");
          if (ci>0){ci+=20;int ce=r.indexOf("\"",ci);if(ce>ci)r.substring(ci,ce).toCharArray(g_codice_crew,8);}

          // Anchor Alert
          {
            int ai, ae;
            ai = r.indexOf("\"anchor_lat\":"); if (ai>0){ ai+=13; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_lat=r.substring(ai,ae).toDouble(); }
            ai = r.indexOf("\"anchor_lon\":"); if (ai>0){ ai+=13; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_lon=r.substring(ai,ae).toDouble(); }
            ai = r.indexOf("\"anchor_raggio\":"); if(ai>0){ ai=r.indexOf(":",ai)+1; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_raggio_m=r.substring(ai,ae).toFloat(); }
            g_anchor_attivo = (r.indexOf("\"anchor_attivo\":true") > 0);
          }

          // ── Waypoint navigation (NUOVO V2.3.0) ───────────────────────
          // I campi wp_alert_lat/lon/attivo esistono già nel DB (zero migration).
          // Riutilizzati come waypoint di navigazione attivo.
          {
            int wi, we;
            wi = r.indexOf("\"wp_alert_lat\":"); if (wi>0){ wi+=15; we=r.indexOf(",",wi); if(we<0)we=r.length()-1; g_wp_lat=r.substring(wi,we).toDouble(); }
            wi = r.indexOf("\"wp_alert_lon\":"); if (wi>0){ wi+=15; we=r.indexOf(",",wi); if(we<0)we=r.length()-1; g_wp_lon=r.substring(wi,we).toDouble(); }
            bool wp_att = (r.indexOf("\"wp_alert_attivo\":true") > 0);
            g_wp_active = wp_att ? 1 : 0;
            if (wp_att && (g_wp_lat != 0.0 || g_wp_lon != 0.0)) {
              Serial.printf("[WP] Waypoint attivo: %.6f, %.6f\n", (float)g_wp_lat, (float)g_wp_lon);
            } else if (!wp_att) {
              // Azzera se disattivato dalla webapp
              if (g_wp_active != 0) {
                g_btw = 0.0f; g_dtw = 0.0f; g_vmg_wp = 0.0f;
                g_wp_bearing_rel = 0.0f; g_eta_wp_sec = 0;
                g_wp_arrive_alert = false;
              }
            }
          }

          // OTA (invariato)
          bool doOtaCheck = (g_otaUltimoCheck == 0) ||
                            (millis() - g_otaUltimoCheck >= OTA_CHECK_INTERVAL_MS);
          if (doOtaCheck) {
            // (parsing OTA — invariato, vedere versione precedente)
            const char* otaVersion = nullptr;
            const char* otaUrl     = nullptr;
            const char* otaSha256  = nullptr;
            const char* otaStato   = nullptr;
            // estrazione da JSON r (invariata rispetto V2.2.6)
            static char _ota_ver[24], _ota_url[256], _ota_sha[68], _ota_stato[16];
            int oi;
            oi = r.indexOf("\"ota_version\":\""); if(oi>0){oi+=15;int oe=r.indexOf("\"",oi);if(oe>oi){r.substring(oi,oe).toCharArray(_ota_ver,24);otaVersion=_ota_ver;}}
            oi = r.indexOf("\"ota_url\":\"");     if(oi>0){oi+=11;int oe=r.indexOf("\"",oi);if(oe>oi){r.substring(oi,oe).toCharArray(_ota_url,256);otaUrl=_ota_url;}}
            oi = r.indexOf("\"ota_sha256\":\"");  if(oi>0){oi+=14;int oe=r.indexOf("\"",oi);if(oe>oi){r.substring(oi,oe).toCharArray(_ota_sha,68);otaSha256=_ota_sha;}}
            oi = r.indexOf("\"ota_stato\":\"");   if(oi>0){oi+=13;int oe=r.indexOf("\"",oi);if(oe>oi){r.substring(oi,oe).toCharArray(_ota_stato,16);otaStato=_ota_stato;}}
            // controllaOTA vuole const char* per otaForce e hasTsRetry (firma OTA.ino)
            const char* otaForceStr   = (r.indexOf("\"ota_force\":true") > 0) ? "true" : "false";
            bool _hasTsRetry = (r.indexOf("\"ota_retry_ts\":") > 0 &&
                                r.indexOf("\"ota_retry_ts\":null") < 0);
            const char* hasTsRetryStr = _hasTsRetry ? "true" : nullptr;
            controllaOTA(otaVersion, otaUrl, otaSha256, otaStato, otaForceStr, hasTsRetryStr);
            g_otaUltimoCheck = millis();
          }
        }
        hConf.end();
      }

      // ════════════════════════════════════════════════════════════════
      // STACK 2 — TELEMETRIA DB @ 2s
      // V2.3.0: aggiunti btw, dtw, vmg_wp; vmg_target ora = g_vmg_wp
      // ════════════════════════════════════════════════════════════════
      {
        HTTPClient hTel;
        hTel.begin(String(SUPABASE_URL)+"/rest/v1/punti_traccia_live");
        hTel.setTimeout(8000);
        hTel.setConnectTimeout(3000);
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
        // vmg_target ora popolato con VMG verso waypoint (era sempre 0.0)
        j += "\"vmg_target\":"+String((float)g_vmg_wp,1)+",";
        j += "\"vmg_wind\":"+String((float)g_vmg_wind,1)+",";
        j += "\"roll\":"+String(g_roll)+",";
        j += "\"pitch\":"+String(g_pitch)+",";
        j += "\"satelliti\":"+String(g_sats)+",";
        j += "\"batteria_pod\":"+String((int)maxlipo.cellPercent())+",";
        // Nuovi campi WP (presenti solo se colonne aggiunte via migration SQL)
        j += "\"btw\":"+String((float)g_btw,1)+",";
        j += "\"dtw\":"+String((float)g_dtw,3)+",";
        j += "\"vmg_wp\":"+String((float)g_vmg_wp,1);
        j += "}";

        int rcTel = hTel.POST(j);
        if (rcTel < 200 || rcTel >= 300) {
          Serial.printf("[CLOUD] POST telemetria errore %d\n", rcTel);
        }
        hTel.end();
      }

      // STACK OTA (invariato)
      if (g_otaPending && !g_otaInCorso) {
        if (g_sessione_attiva_id[0] != '\0') {
          Serial.println("[OTA] ⏸ Sessione attiva — OTA posticipata");
        } else if (!g_fase1Ok) {
          // attende gate SSL
        } else {
          Serial.println("[OTA] ▶ Avvio OTA...");
          eseguiOTA();
        }
      }

      // STACK 1 — GESTIONE SESSIONE (invariato)
      if (snapCmd == 1 && lastCmdSnap != 1) {
        String respAvvio;
        int rcAvvio = chiamaRPC("crea_sessione_da_visore",
          "{\"p_pod_id\":\""+String(NOME_POD)+"\"}", &respAvvio);
        if (rcAvvio >= 200 && rcAvvio < 300) {
          respAvvio.replace("\"",""); respAvvio.trim();
          if (respAvvio.length() == 36)
            respAvvio.toCharArray(g_sessione_attiva_id, 37);
          Serial.printf("[SESS] ✅ Sessione avviata: %s\n", g_sessione_attiva_id);
        } else {
          Serial.printf("[SESS] ⚠️ crea_sessione rc=%d\n", rcAvvio);
        }
        lastCmdSnap             = 1;
        g_sessioneChiusaOk      = false;
        g_pendingChiudiSessione = false;
        g_retryChiudiCount      = 0;
      }

      if (snapCmd == 2 && lastCmdSnap == 1 && !g_pendingChiudiSessione) {
        g_pendingChiudiSessione = true;
        g_sessionDistPending    = snapDist;
        g_retryChiudiCount      = 0;
        Serial.printf("[SESS] Stop catturato — dist=%.2f nm\n", snapDist);
      }

      if (g_pendingChiudiSessione && g_retryChiudiCount < MAX_RETRY_CHIUDI) {
        String bodyChiudi = "{\"p_pod_id\":\""+String(NOME_POD)+
                            "\",\"p_session_dist\":"+String(g_sessionDistPending,2)+"}";
        int rcChiudi = chiamaRPC("chiudi_sessione_da_visore", bodyChiudi);
        if (rcChiudi >= 200 && rcChiudi < 300) {
          memset(g_sessione_attiva_id, 0, 37);
          g_pendingChiudiSessione = false;
          g_retryChiudiCount      = 0;
          g_sessioneChiusaOk      = true;
          lastCmdSnap             = 0;
          g_cmdSessSnap           = 0;
          Serial.println("[SESS] ✅ Sessione chiusa");
        } else {
          g_retryChiudiCount++;
          Serial.printf("[SESS] ⚠️ Chiusura rc=%d — retry %d/%d\n",
            rcChiudi, g_retryChiudiCount, MAX_RETRY_CHIUDI);
          if (g_retryChiudiCount >= MAX_RETRY_CHIUDI) {
            g_pendingChiudiSessione = false;
            lastCmdSnap             = 0;
          }
        }
      }

      if (snapCmd == 0 && !g_pendingChiudiSessione && lastCmdSnap != 0) {
        lastCmdSnap = 0;
      }

    } else {
      if (wifiConnesso) { wifiConnesso = false; }
    }

    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

// ── Portale captive WiFi (invariato da V2.2.6) ───────────────────────────
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
  savedSSID = prefs.getString("ssid", "");
  savedPass = prefs.getString("pass", "");
  prefs.end();
  // Fallback rete collaudo se il portale captive non ha ancora salvato credenziali
  if (savedSSID.length() == 0) {
    savedSSID = POD_WIFI_SSID;
    savedPass = POD_WIFI_PASS;
    Serial.println("[WIFI] Nessuna rete salvata — uso rete collaudo da secrets.h");
  }
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
