// =========================================================================
// AYE POD — Rete_Cloud.ino
// Release: V2.3.2
//
// Changelog V2.3.2 — Binding POD↔Visore via Supabase (MINOR).
//   Fix compilazione: esp_read_mac()/ESP_MAC_WIFI_STA sostituiti con
//   WiFi.macAddress() — non richiede esp_mac.h, funziona su tutti i
//   core Arduino ESP32 (2.x e 3.x). Nessuna modifica funzionale.
//
//
//   NUOVE FUNZIONALITÀ:
//
//   1. REGISTRAZIONE MAC POD — una tantum al primo boot connesso
//      Al primo GET FASE1 riuscito, legge il MAC WiFi dell'ESP32 via
//      esp_read_mac() e chiama la RPC registra_mac_pod() su Supabase.
//      Idempotente: flag NVS namespace 'pod_prov' (chiave 'mac_ok')
//      evita la chiamata ai boot successivi anche senza risposta DB.
//      Il MAC viene scritto nella colonna mac_pod di dispositivi.
//
//   2. LETTURA mac_visore_bound DA DB — ogni 30s (dentro FASE1 esistente)
//      Aggiunto "mac_visore_bound" alla SELECT FASE1 esistente (una riga).
//      Se il valore nel DB cambia rispetto all'ultimo letto:
//        → parseMacString() converte "AA:BB:CC:DD:EE:FF" → uint8_t[6]
//        → aggiornaVisorePeer() aggiorna il peer ESP-NOW:
//            esp_now_del_peer(vecchioMac) + esp_now_add_peer(nuovoMac)
//        → macVisore[] viene aggiornato (globale usato da inviaDatiVisore)
//      Se mac_visore_bound è NULL nel DB → torna a broadcast FF:FF:FF:FF:FF:FF
//      Il POD funziona normalmente senza Visore: TX broadcast, nessun errore.
//
//   THREAD SAFETY:
//      aggiornaVisorePeer() è chiamata solo da TaskCloud (Core0).
//      inviaDatiVisore() in Visore.ino (Core1) chiama solo esp_now_send()
//      e legge macVisore[] ma non modifica la peer list.
//      esp_now_del_peer/add_peer sono safe da Core0 secondo ESP-IDF docs.
//      macVisore[] è un array globale: la scrittura da Core0 è atomica
//      per allineamento (6 byte) su Xtensa LX7. Nessun mutex necessario.
//
//   INVARIATO RISPETTO A V2.3.1:
//      Tutte le funzioni esistenti: setupWiFi, gestisciWiFi, setupWebServer,
//      chiamaRPC, TaskInvioCloud (FASE1 config, Stack1 sessione, Stack2
//      telemetria, OTA gate), anchor, waypoint, nome_capitano, codice_crew.
//      sizeof(struct_nautica) = 108 byte — INVARIATO.
//      sizeof(struct_messaggio_visore) = 22 byte — INVARIATO.
//      Nessuna modifica a Visore.ino, AYE_POD_v37_DEV.ino, OTA.ino.
//
// Changelog V2.3.1 — Fix compilazione (PATCH)
// Changelog V2.3.0 — Waypoint navigation, filtro SOG, telemetria btw/dtw/vmg_wp
// Changelog V2.2.4 — Fix setTimeout millisecondi
// =========================================================================

#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
// esp_wifi.h non necessario — usiamo WiFi.macAddress() che è in WiFi.h (già incluso)

Preferences prefs;
extern Preferences prefsOffset;

WebServer server(80);
DNSServer dnsServer;

String savedSSID = "";
String savedPass = "";
bool   portalRunning    = false;
unsigned long portalStartTime  = 0;
unsigned long lastWifiAttempt  = 0;

// ── Credenziali da secrets.h ──────────────────────────────────────────────
// secrets.h usa #define con prefisso POD_ (POD_NOME, POD_SUPABASE_URL, ...)
// Le variabili const char* sotto sono quelle cercate da OTA.ino via extern.
#include "secrets.h"
const char* NOME_POD     = POD_NOME;
const char* SUPABASE_URL = POD_SUPABASE_URL;
const char* SUPABASE_KEY = POD_SUPABASE_KEY;

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

// ── V2.3.2: Binding Visore ────────────────────────────────────────────────
// macVisore[] dichiarato in AYE_POD_v37_DEV.ino, usato da Visore.ino.
// Usiamo extern per accedervi da TaskCloud (Core0).
extern uint8_t macVisore[6];

// MAC broadcast — default quando nessun Visore è abbinato
static const uint8_t MAC_BROADCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Ultimo MAC letto dal DB — per rilevare cambiamenti ed evitare update inutili
static uint8_t g_macVisorePrecedente[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Flag persistente NVS: MAC POD già registrato su Supabase
static bool g_macPodRegistrato = false;

// ── V2.3.2: Parsing stringa MAC → array uint8_t[6] ───────────────────────
// Accetta "AA:BB:CC:DD:EE:FF" (con ':') o "AABBCCDDEEFF" (senza).
// Ritorna true se il parsing è riuscito e il MAC è valido.
static bool parseMacString(const String& macStr, uint8_t out[6]) {
  String clean = macStr;
  clean.replace(":", "");
  clean.replace("-", "");
  clean.toUpperCase();
  clean.trim();
  if (clean.length() != 12) return false;
  for (int i = 0; i < 6; i++) {
    char byteStr[3] = { clean[i*2], clean[i*2+1], '\0' };
    out[i] = (uint8_t)strtol(byteStr, nullptr, 16);
  }
  return true;
}

// ── V2.3.2: Aggiornamento peer ESP-NOW con nuovo MAC Visore ──────────────
// Chiamata da TaskCloud (Core0) solo quando mac_visore_bound cambia nel DB.
// Sequenza: rimuovi vecchio peer → aggiorna macVisore[] → aggiungi nuovo peer.
static void aggiornaVisorePeer(const uint8_t nuovoMac[6]) {
  // Rimuovi peer precedente (errore ignorato: potrebbe non esistere)
  esp_now_del_peer(g_macVisorePrecedente);

  // Aggiorna il globale macVisore[] usato da inviaDatiVisore() in Visore.ino
  memcpy(macVisore, nuovoMac, 6);
  memcpy(g_macVisorePrecedente, nuovoMac, 6);

  // Se è broadcast, non aggiungere come peer unicast (non necessario,
  // esp_now_send con FF:FF:FF:FF:FF:FF funziona senza peer registrato)
  bool isBroadcast = true;
  for (int i = 0; i < 6; i++) {
    if (nuovoMac[i] != 0xFF) { isBroadcast = false; break; }
  }

  if (isBroadcast) {
    Serial.println("[BIND] Visore rimosso — TX broadcast FF:FF:FF:FF:FF:FF");
    return;
  }

  // Aggiungi nuovo peer unicast
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, nuovoMac, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_err_t err = esp_now_add_peer(&peer);
  if (err == ESP_OK) {
    Serial.printf("[BIND] Peer aggiornato → %02X:%02X:%02X:%02X:%02X:%02X\n",
      nuovoMac[0],nuovoMac[1],nuovoMac[2],nuovoMac[3],nuovoMac[4],nuovoMac[5]);
  } else {
    Serial.printf("[BIND] WARN esp_now_add_peer err=%d\n", (int)err);
  }
}

// ── V2.3.2: Registra MAC POD su Supabase (una tantum) ────────────────────
// Chiamata dopo il primo GET FASE1 riuscito.
// NVS 'pod_prov'/'mac_ok' impedisce ripetizioni tra reboot.
static void registraMacPodSeNecessario() {
  if (g_macPodRegistrato) return;

  // Controlla NVS prima di fare la chiamata HTTP
  Preferences nvsProv;
  nvsProv.begin("pod_prov", true);
  bool giaOk = nvsProv.getBool("mac_ok", false);
  nvsProv.end();
  if (giaOk) { g_macPodRegistrato = true; return; }

  // Leggi MAC WiFi STA dell'ESP32.
  // WiFi.macAddress() è disponibile da WiFi.h (già incluso) su tutti i core
  // Arduino ESP32 (ESP32, ESP32-S3, ESP32-C3...) — più portabile di esp_read_mac().
  // Formato restituito: "AA:BB:CC:DD:EE:FF" — già pronto per il body della RPC.
  // WiFi è già in WIFI_AP_STA quando questa funzione viene chiamata (post-setupWiFi).
  String macString = WiFi.macAddress();
  macString.toUpperCase();
  const char* macStr = macString.c_str();

  // Chiama RPC registra_mac_pod
  String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\","
                 "\"p_mac_pod\":\"" + String(macStr) + "\","
                 "\"p_fw\":\"" + String(FW_VERSION) + "\"}";
  String resp;
  int rc = chiamaRPC("registra_mac_pod", body, &resp);

  if (rc >= 200 && rc < 300) {
    Serial.printf("[PROV] MAC POD registrato: %s\n", macStr);
    Preferences nvsW;
    nvsW.begin("pod_prov", false);
    nvsW.putBool("mac_ok", true);
    nvsW.end();
    g_macPodRegistrato = true;
  } else {
    // Non blocca: riproverà al prossimo ciclo FASE1 (30s)
    Serial.printf("[PROV] registra_mac_pod rc=%d — riprova al prossimo ciclo\n", rc);
  }
}

// ── chiamaRPC() — INVARIATA da V2.3.1 ────────────────────────────────────
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

// =========================================================================
// TASK CLOUD — V2.3.2
// Struttura invariata rispetto a V2.3.1.
// Aggiunta: lettura mac_visore_bound + registrazione MAC POD dentro FASE1.
// =========================================================================
void TaskInvioCloud(void* pvParameters) {
  static unsigned long lastConfigRead = 0;
  static int           lastCmdSnap   = 0;

  while (true) {

    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnesso) { wifiConnesso = true; }

      int   snapCmd  = (int)g_cmdSessSnap;
      float snapDist = (float)g_cmdSessSnapDist;

      // ════════════════════════════════════════════════════════════════
      // FASE 1 — Config DB ogni 30s
      // V2.3.2: aggiunto "mac_visore_bound" alla SELECT esistente
      // ════════════════════════════════════════════════════════════════
      if (millis()-lastConfigRead > 30000) {
        lastConfigRead = millis();
        HTTPClient hConf;
        String ep = String(SUPABASE_URL)+"/rest/v1/dispositivi?pod_id=eq."+NOME_POD+
          "&select=prua_offset,cmd_calibra,sessione_attiva_id,nome_capitano,codice_crew_live"
          ",anchor_lat,anchor_lon,anchor_raggio,anchor_attivo"
          ",wp_alert_lat,wp_alert_lon,wp_alert_attivo"
          ",ota_version,ota_url,ota_sha256,ota_stato,ota_force,ota_retry_ts"
          ",mac_visore_bound";    // ← V2.3.2: nuovo campo
        hConf.begin(ep);
        hConf.addHeader("apikey", SUPABASE_KEY);
        hConf.addHeader("Authorization", String("Bearer ")+SUPABASE_KEY);

        if (hConf.GET() > 0) {
          String r = hConf.getString();

          // Gate OTA — invariato
          if (!g_fase1Ok) {
            g_fase1Ok = true;
            Serial.println("[CLOUD] FASE1 OK — SSL warm, gate OTA sbloccato");
          }

          // ── V2.3.2: Registrazione MAC POD (una tantum) ──────────────
          registraMacPodSeNecessario();

          // ── V2.3.2: Lettura e aggiornamento mac_visore_bound ─────────
          {
            uint8_t nuovoMac[6];
            bool macValido = false;
            int ib = r.indexOf("\"mac_visore_bound\":");
            if (ib > 0) {
              ib += 19;
              if (r.charAt(ib) == '"') {
                // Valore stringa
                int ie = r.indexOf('"', ib+1);
                if (ie > ib) {
                  String macStr = r.substring(ib+1, ie);
                  macValido = parseMacString(macStr, nuovoMac);
                }
              }
              // Se null → macValido rimane false → broadcast
            }
            if (!macValido) {
              memcpy(nuovoMac, MAC_BROADCAST, 6);
            }
            // Aggiorna peer solo se il MAC è effettivamente cambiato
            if (memcmp(nuovoMac, g_macVisorePrecedente, 6) != 0) {
              aggiornaVisorePeer(nuovoMac);
            }
          }

          // ── prua_offset — invariato ──────────────────────────────────
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

          // ── cmd_calibra — invariato ──────────────────────────────────
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

          // ── sessione_attiva_id — invariato ───────────────────────────
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

          // ── nome_capitano — invariato ────────────────────────────────
          int ni = r.indexOf("\"nome_capitano\":\"");
          if (ni>0){ni+=17;int ne=r.indexOf("\"",ni);if(ne>ni)r.substring(ni,ne).toCharArray(g_nome_capitano,64);}

          // ── codice_crew — invariato ──────────────────────────────────
          int ci = r.indexOf("\"codice_crew_live\":\"");
          if (ci>0){ci+=20;int ce=r.indexOf("\"",ci);if(ce>ci)r.substring(ci,ce).toCharArray(g_codice_crew,8);}

          // ── Anchor Alert — invariato ─────────────────────────────────
          {
            int ai, ae;
            ai = r.indexOf("\"anchor_lat\":"); if (ai>0){ ai+=13; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_lat=r.substring(ai,ae).toDouble(); }
            ai = r.indexOf("\"anchor_lon\":"); if (ai>0){ ai+=13; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_lon=r.substring(ai,ae).toDouble(); }
            ai = r.indexOf("\"anchor_raggio\":"); if(ai>0){ ai=r.indexOf(":",ai)+1; ae=r.indexOf(",",ai); if(ae<0)ae=r.length()-1; g_anchor_raggio_m=r.substring(ai,ae).toFloat(); }
            g_anchor_attivo = (r.indexOf("\"anchor_attivo\":true") > 0);
          }

          // ── Waypoint navigation — invariato da V2.3.1 ────────────────
          {
            int wi, we;
            wi = r.indexOf("\"wp_alert_lat\":"); if (wi>0){ wi+=15; we=r.indexOf(",",wi); if(we<0)we=r.length()-1; g_wp_lat=r.substring(wi,we).toDouble(); }
            wi = r.indexOf("\"wp_alert_lon\":"); if (wi>0){ wi+=15; we=r.indexOf(",",wi); if(we<0)we=r.length()-1; g_wp_lon=r.substring(wi,we).toDouble(); }
            bool wp_att = (r.indexOf("\"wp_alert_attivo\":true") > 0);
            g_wp_active = wp_att ? 1 : 0;
            if (wp_att && (g_wp_lat != 0.0 || g_wp_lon != 0.0)) {
              Serial.printf("[WP] Waypoint attivo: %.6f, %.6f\n", (float)g_wp_lat, (float)g_wp_lon);
            } else if (!wp_att) {
              if (g_wp_active != 0) {
                g_btw = 0.0f; g_dtw = 0.0f; g_vmg_wp = 0.0f;
                g_wp_bearing_rel = 0.0f; g_eta_wp_sec = 0;
                g_wp_arrive_alert = false;
              }
            }
          }

          // ── OTA — invariato da V2.3.1 ────────────────────────────────
          bool doOtaCheck = (g_otaUltimoCheck == 0) ||
                            (millis() - g_otaUltimoCheck >= OTA_CHECK_INTERVAL_MS);
          if (doOtaCheck) {
            const char* otaVersion = nullptr;
            const char* otaUrl     = nullptr;
            const char* otaSha256  = nullptr;
            const char* otaStato   = nullptr;
            static char _ota_ver[24], _ota_url[256], _ota_sha[68], _ota_stato[16];
            int oi;
            oi = r.indexOf("\"ota_version\":\""); if(oi>0){oi+=15;int oe=r.indexOf("\"",oi);if(oe>oi){r.substring(oi,oe).toCharArray(_ota_ver,24);otaVersion=_ota_ver;}}
            oi = r.indexOf("\"ota_url\":\"");     if(oi>0){oi+=11;int oe=r.indexOf("\"",oi);if(oe>oi){r.substring(oi,oe).toCharArray(_ota_url,256);otaUrl=_ota_url;}}
            oi = r.indexOf("\"ota_sha256\":\"");  if(oi>0){oi+=14;int oe=r.indexOf("\"",oi);if(oe>oi){r.substring(oi,oe).toCharArray(_ota_sha,68);otaSha256=_ota_sha;}}
            oi = r.indexOf("\"ota_stato\":\"");   if(oi>0){oi+=13;int oe=r.indexOf("\"",oi);if(oe>oi){r.substring(oi,oe).toCharArray(_ota_stato,16);otaStato=_ota_stato;}}
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
      // STACK 2 — Telemetria DB @ 2s — INVARIATO da V2.3.1
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
        j += "\"vmg_target\":"+String((float)g_vmg_wp,1)+",";
        j += "\"vmg_wind\":"+String((float)g_vmg_wind,1)+",";
        j += "\"roll\":"+String(g_roll)+",";
        j += "\"pitch\":"+String(g_pitch)+",";
        j += "\"satelliti\":"+String(g_sats)+",";
        j += "\"batteria_pod\":"+String((int)maxlipo.cellPercent())+",";
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

      // ════════════════════════════════════════════════════════════════
      // STACK OTA — INVARIATO da V2.3.1
      // ════════════════════════════════════════════════════════════════
      if (g_otaPending && !g_otaInCorso) {
        if (g_sessione_attiva_id[0] != '\0') {
          Serial.println("[OTA] ⏸ Sessione attiva — OTA posticipata");
        } else if (!g_fase1Ok) {
          // Gate SSL: attende primo FASE1 ok
        } else {
          Serial.println("[OTA] ▶ Avvio OTA...");
          eseguiOTA();
        }
      }

      // ════════════════════════════════════════════════════════════════
      // STACK 1 — Gestione Sessione — INVARIATO da V2.3.1
      // ════════════════════════════════════════════════════════════════
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
            rcChiudi, (int)g_retryChiudiCount, MAX_RETRY_CHIUDI);
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

// ── setupWebServer() — INVARIATA da V2.3.1 ────────────────────────────────
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

// ── setupWiFi() — INVARIATA da V2.3.1 ─────────────────────────────────────
void setupWiFi() {
  prefs.begin("wifi_creds", true);
  savedSSID = prefs.getString("ssid", "");
  savedPass = prefs.getString("pass", "");
  prefs.end();
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

  // V2.3.2: inizializza stato NVS binding
  memcpy(g_macVisorePrecedente, MAC_BROADCAST, 6);
  Preferences nvsProv;
  nvsProv.begin("pod_prov", true);
  g_macPodRegistrato = nvsProv.getBool("mac_ok", false);
  nvsProv.end();
  if (g_macPodRegistrato) Serial.println("[PROV] MAC POD già registrato (NVS ok)");
}

// ── gestisciWiFi() — INVARIATA da V2.3.1 ──────────────────────────────────
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
