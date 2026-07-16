// =========================================================================
// AYE POD — Rete_Cloud.ino
//  Release: V2.6.1
//
// Changelog V2.6.1 — Fix ownership + POD offline (PATCH):
//   BUG A: il POD chiudeva le sessioni della web app (regola senza
//     ownership). FIX: agisce solo su avviata_da='visore' (ho_mia).
//     Opzione (a): il Visore si aggancia a una sessione web esistente e
//     ne assume l'ownership ("chiude chi apre").
//   BUG B: GET avviata_il rimosso — 1 TLS sul percorso caldo mandava il
//     POD offline dopo lo stop (gap 156s/209s). avviata_da ora arriva via
//     embedding nella GET di FASE1 (FK aggiunta a DB). Timeout 72h -> job DB.
//
// Changelog V2.6.0 — Sessione a STATO + ownership (MINOR):
//   STACK1 non reagisce piu' a fronti (cmd=1/cmd=2) ma confronta due livelli:
//
//     bool vuole = (datiDalVisore.sessione_attiva != 0);   // dice il Visore
//     bool ho    = (g_sessione_attiva_id[0] != '\0');      // dice il DB
//     if      (vuole && !ho)  crea_sessione_da_visore();
//     else if (!vuole && ho)  chiudi_sessione_da_visore();
//
//   Il retry e' implicito: se una RPC fallisce, il ciclo dopo (2s) ritenta
//   perche' la condizione e' ancora vera. Nessun flag, nessuno snapshot.
//
//   RITORNO ALLE RPC (inverte la decisione V2.5.1 "trigger-only"):
//     Il trigger gestisci_sessione_automatica e' CONCETTUALMENTE
//     INCOMPATIBILE con la logica a livello: reagisce a cmd_sessione != 0
//     su OGNI punto, cioe' a un livello, non a un cambiamento. Con
//     sessione_attiva=1 costante scatterebbe su ogni singolo punto.
//     → Il trigger va RIMOSSO dal DB (task 4, BLOCCANTE) e restano le RPC:
//       - crea_sessione_da_visore: idempotente (se trova una sessione
//         aperta la ritorna invece di duplicare), scrive avviata_da='visore'
//       - chiudi_sessione_da_visore: fa tutto quello che faceva il trigger
//         e meglio (fallback su p_session_dist se n_punti=0, aggregati per
//         id_sessione OR finestra, azzera dispositivi.sessione_attiva_id)
//     ⚠ Richiede il GRANT anon EXECUTE sulle due RPC, revocato in V2.5.1
//       (migration revoke_anon_rpc_sessione_ripristino_trigger_only).
//       Senza il grant le RPC rispondono 403 e la sessione non si apre.
//
//   LATENZA — il problema che aveva motivato V2.5.1 non si ripresenta:
//     V2.5.1 tolse le RPC perche' chiamate a OGNI ciclo con cmd armato
//     (handshake TLS 2-4s → ciclo >7s → gap telemetria → "POD offline").
//     Ora le RPC partono SOLO sulla TRANSIZIONE (vuole != ho): due volte
//     per sessione, non 10 volte al minuto. A regime il ciclo resta ~2s.
//
//   TIMEOUT (regole di ownership):
//     - 180s senza pacchetti dal Visore → il POD chiude la sessione
//     - 72h durata massima → rete di sicurezza (ricalcolata da avviata_il,
//       sopravvive al riavvio del POD)
//
//   INVARIATI: FASE1 config, STACK2 telemetria, OTA, binding NVS Visore,
//     registrazione MAC POD, struct_nautica=108, struct_messaggio_visore=22.
//
// Changelog V2.5.1 — Gestione sessione TRIGGER-ONLY (MINOR):
//   DECISIONE (task #3 storico 15/07): "trigger OPPURE RPC, non entrambi".
//   Scelto il TRIGGER: gestisci_sessione_automatica apre su cmd=1 e chiude
//   su cmd=2 dai punti telemetria, scrive avviata_da='visore' (migration
//   fix_avviata_da_visore_v45518) e calcola gli aggregati con la funzione
//   unificata calcola_miglia_sessione. Fa gia' tutto: le RPC erano ridondanti.
//
//   BUG RISOLTO: il POD smetteva di inviare telemetria durante la sessione
//   avviata dal Visore (portale: POD offline; ripartiva solo allo stop).
//   CAUSA: STACK1 chiamava crea_sessione_da_visore/chiudi_sessione_da_visore
//   in modo SINCRONO dentro il ciclo TaskCloud. Ogni chiamata apre una nuova
//   sessione TLS verso Supabase (handshake 2-4s su ESP32). Con cmd=1 il ciclo
//   diventava: POST telemetria (TLS) + GET config (TLS) + RPC (TLS) + retry
//   → >7s invece di 2s → gap telemetria. Allo stop lastCmdSnap tornava a 0,
//   le RPC non partivano piu' e il ciclo tornava a ~2s ("il POD torna live").
//
//   FIX 1: rimosse le chiamate RPC crea/chiudi_sessione_da_visore dal
//     TaskCloud. STACK1 ora aggiorna solo lo stato locale (nessun HTTP).
//     Il ciclo resta ~2s costanti: telemetria continua garantita.
//   FIX 2: FASE1 non azzera piu' g_cmdSessSnap su sessione_attiva_id=null.
//     Azzerarlo cancellava il cmd=1 appena armato dal Visore nei ~2-30s che
//     il trigger impiega ad aprire la sessione (regressione grave).
//     La pulizia dello stato orfano e' gestita dalla callback (Visore.ino
//     V2.4.6, ramo cmd==0) che ha il contesto per farlo in sicurezza.
//   FIX 3: id_sessione nel POST telemetria arriva da FASE1 (<=30s dall'avvio).
//     Nessun impatto sugli aggregati: il trigger li calcola per finestra
//     temporale (avviata_il..conclusa_il), non per id_sessione.
//
//   DB (contestuale): revocato EXECUTE anon su crea/chiudi_sessione_da_visore
//     (migration revoke_anon_rpc_sessione_ripristino_trigger_only).
//     Le funzioni restano per la web app (ruolo authenticated).
//   INVARIATI: struct_nautica=108, struct_messaggio_visore=22, FASE1 config,
//     STACK2 telemetria, OTA, binding NVS Visore, registrazione MAC POD.
//
//  Release: V2.4.0
// Changelog V2.4.0 — Telemetria debug filtro SOG (MINOR):
//   +6 campi nel POST punti_traccia_live: sog_raw, accel_media,
//   accel_x/y_min/max. Logging only — filtro SOG INVARIATO.
//   Binding NVS V2.3.3 preservato integralmente.
//
// Changelog V2.3.3 — Persistenza binding Visore in NVS (PATCH):
//   NVS namespace 'pod_bind', chiave 'vis_mac'. Il MAC del Visore
//   abbinato viene salvato al binding e letto al boot prima di
//   setupESPNOW() — link immediato anche offline.
//
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

// ── V2.6.0 — Stato sessione: UNA sola variabile di verita' ────────────────
// RIMOSSE: g_cmdSessSnap, g_cmdSessSnapDist, g_pendingChiudiSessione,
//          g_sessioneChiusaOk, g_sessionDistPending, g_retryChiudiCount,
//          MAX_RETRY_CHIUDI.
// Esistevano tutte per un solo motivo: catturare un FRONTE senza perderlo
// ne' riconsumarlo. Con il protocollo a LIVELLO il fronte non esiste piu':
//   vuole = (datiDalVisore.sessione_attiva != 0)   ← lo dice il Visore
//   ho    = (g_sessione_attiva_id[0] != '\0')      ← lo dice il DB
// Il retry e' implicito: se crea/chiudi fallisce, il ciclo dopo (2s)
// ritenta da solo perche' la condizione e' ancora vera. Nessun flag da
// azzerare, nessuno stato che puo' restare incollato.
//
// g_sessione_attiva_id (sopra) e' l'unica verita': e' l'ID della sessione
// realmente aperta, popolato da FASE1/crea_sessione e azzerato da chiudi.

// Distanza dichiarata dal Visore — usata SOLO come fallback dalla RPC di
// chiusura quando non ci sono punti a DB (n_punti=0, es. sessione senza
// copertura cloud). La verita' sulle miglia resta calcola_miglia_sessione().
extern struct_messaggio_visore datiDalVisore;

// ── V2.6.0 — Timeout sessione (regole di ownership) ───────────────────────
// 180s: assenza Visore → il POD chiude la sessione.
//   36x il LINK LOST del Visore (5s), sopravvive a un riavvio Visore (~15s).
// 72h: durata massima → rete di sicurezza. Ricalcolata da avviata_il letto
//   dal DB, cosi' sopravvive a un riavvio del POD.
#define VISORE_ASSENTE_TIMEOUT_MS  180000UL      // 180 s
#define SESSIONE_MAX_DURATA_SEC    (72UL*3600UL) // 72 h

extern volatile uint32_t g_ultimoPacchettoVisore;  // definita in AYE_POD_v37_DEV.ino

// Epoch di avvio della sessione attiva (0 = ignoto).
// V2.6.1: impostato SOLO da STACK1 alla creazione, dal clock GPS
// (g_unix_timestamp). Non si legge piu' avviata_il dal DB: vedi la nota
// sul GET rimosso in FASE1.
static uint32_t g_sessioneAvviataEpoch = 0;

// ── V2.6.1 — OWNERSHIP: la sessione aperta e' del Visore? ────────────────
// true  = l'ha aperta (o agganciata) il Visore → il POD la chiude
// false = l'ha aperta la web app → il POD NON la tocca
// Popolato da FASE1 leggendo avviata_da, e forzato a true da STACK1 quando
// il Visore apre/aggancia (opzione a: "chiude chi apre").
static bool g_sessioneMia = false;

bool wifiConnesso = false;

// ── V2.6.2 — WATCHDOG TaskCloud ──────────────────────────────────────────
// PERCHE': il 16/07 il TaskCloud (Core0) e' rimasto appeso dentro una
// chiamata TLS senza timeout. Il Core1 continuava a girare (ESP-NOW e
// telemetria seriale regolari) quindi il POD SEMBRAVA vivo, ma non parlava
// piu' col DB. gestisciWiFi() non se ne accorgeva: guarda solo
// WiFi.status(), che restava WL_CONNECTED. Nessun recovery, all'infinito:
// i gap misurati (34, 63, 77, 146, 198, 213, 406s) finivano solo quando
// l'utente riavviava il ROUTER — cioe' forzava la deassociazione.
//
// COME: il TaskCloud aggiorna g_cloudHeartbeat a OGNI giro del suo while.
// gestisciWiFi() gira nel loop() su Core1 (vivo anche quando Core0 e'
// appeso) e controlla da quanto e' fermo. Oltre la soglia: ciclo WiFi,
// che replica il riavvio del router fatto a mano.
//
// volatile: scritto da Core0, letto da Core1. Su ESP32 le letture/scritture
// a 32 bit allineate sono atomiche, quindi non serve un mutex: leggiamo un
// timestamp, non una struttura da tenere coerente.
volatile uint32_t g_cloudHeartbeat = 0;
// Diagnostica: quante volte il watchdog e' intervenuto dal boot.
volatile uint32_t g_cloudWdtTrigger = 0;

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


// V.2.3.3 Aggiungi defines e 3 funzioni NVS 
// V2.4.0: forward declaration — nvsBindLeggi() usa parseMacString()
// definita piu sotto. L'IDE Arduino auto-genera i prototipi, ma per le
// funzioni static l'auto-prototyping non e garantito: dichiarazione esplicita.
static bool parseMacString(const String& macStr, uint8_t out[6]);

#define NVS_BIND_NS  "pod_bind"
#define NVS_BIND_KEY "vis_mac"

static void nvsBindSalva(const uint8_t mac[6]) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
    mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  Preferences nvsBind;
  nvsBind.begin(NVS_BIND_NS, false);
  nvsBind.putString(NVS_BIND_KEY, macStr);
  nvsBind.end();
  Serial.printf("[BIND-NVS] Salvato: %s\n", macStr);
}

static void nvsBindCancella() {
  Preferences nvsBind;
  nvsBind.begin(NVS_BIND_NS, false);
  nvsBind.remove(NVS_BIND_KEY);
  nvsBind.end();
  Serial.println("[BIND-NVS] Cancellato (unbind)");
}

static bool nvsBindLeggi(uint8_t out[6]) {
  Preferences nvsBind;
  nvsBind.begin(NVS_BIND_NS, true);
  String macStr = nvsBind.getString(NVS_BIND_KEY, "");
  nvsBind.end();
  if (macStr.length() < 12) return false;
  return parseMacString(macStr, out);
}

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
    nvsBindCancella();   // ← AGGIUNTO V2.3.3
    Serial.println("[BIND] Visore rimosso — TX sospeso (Opzione A), NVS cancellato");
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
    nvsBindSalva(nuovoMac);   // ← dentro l'if, solo se add_peer OK
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
  // V2.6.0: rimossa lastCmdSnap — non ci sono piu' fronti da ricordare.
  // Lo stato vive in g_sessione_attiva_id e datiDalVisore.sessione_attiva.

  while (true) {

    // ── V2.6.2 — Battito del watchdog ───────────────────────────────────
    // Aggiornato a OGNI giro, PRIMA di qualunque HTTP. Se il task si blocca
    // dentro una chiamata, il battito si ferma qui e Core1 se ne accorge.
    // Va all'inizio del ciclo, non in fondo: in fondo non verrebbe mai
    // eseguito proprio nel caso che ci interessa (blocco a meta' ciclo).
    g_cloudHeartbeat = millis();

    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnesso) { wifiConnesso = true; }

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
          ",mac_visore_bound"     // ← V2.3.2: nuovo campo
          // ── V2.6.1: ownership della sessione attiva ───────────────────
          // Colonna denormalizzata su dispositivi, mantenuta dal trigger DB
          // trg_sync_sessione_attiva_da. Serve alla regola "chiude chi apre".
          //
          // Perche' NON l'embedding sessioni_navigazione(avviata_da):
          // la FK esiste, ma sessioni_navigazione ha RLS con unica policy
          // SELECT "auth.uid() = utente_id". Il POD usa la publishable key
          // (ruolo anon) -> auth.uid() e' NULL -> 0 righe -> l'embedding
          // tornerebbe SEMPRE vuoto -> g_sessioneMia sempre false -> il POD
          // non chiuderebbe mai nemmeno le proprie sessioni.
          // Verificato con "set local role anon" sul DB il 16/07.
          // Aprire una policy anon su sessioni_navigazione esporrebbe le
          // sessioni di TUTTI gli utenti: scartato.
          //
          // Viaggia dentro la GET che FASE1 fa gia': ZERO handshake TLS in
          // piu' - al contrario del GET dedicato rimosso in questa versione.
          ",sessione_attiva_da";
        hConf.begin(ep);
        // ── V2.6.2 — TIMEOUT (era la CAUSA del blocco del 16/07) ─────────
        // Senza setTimeout()/setConnectTimeout() HTTPClient puo' restare
        // appeso indefinitamente se il server accetta il TCP ma il TLS non
        // completa: la GET() non ritorna MAI e il TaskCloud muore in piedi.
        // Evidenza dal log seriale: heap che oscilla 206K<->157K a ogni TLS,
        // poi si CONGELA a 157300 identico per sempre = contesto TLS aperto
        // e mai chiuso, task fermo dentro la chiamata. Il Core1 continuava
        // (ESP-NOW e telemetria seriale ok) -> il Visore si aggiornava e il
        // POD sembrava vivo, ma non parlava piu' col DB.
        // gestisciWiFi() non interveniva: guarda solo WiFi.status(), che
        // restava WL_CONNECTED (il POD risultava agganciato al router).
        // Stessi valori di chiamaRPC/hTel, che infatti non si bloccavano mai.
        hConf.setTimeout(8000);
        hConf.setConnectTimeout(3000);
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
            // V2.6.2: timeout mancanti — stesso rischio di blocco di hConf.
            hp.setTimeout(8000);
            hp.setConnectTimeout(3000);
            hp.addHeader("Content-Type","application/json");
            hp.addHeader("apikey",SUPABASE_KEY);
            hp.addHeader("Authorization",String("Bearer ")+SUPABASE_KEY);
            hp.addHeader("Prefer","return=minimal");
            hp.PATCH("{\"cmd_calibra\":false}");
            hp.end();
          }

          // ── sessione_attiva_id — V2.6.0: recupero stato da DB ────────
          // Riallinea g_sessione_attiva_id ('ho') con la verita' del DB.
          // Serve al recupero dopo un riavvio del POD a sessione aperta:
          // il POD riprende la sessione invece di aprirne una seconda.
          // NOTA: FASE1 gira ogni 30s, STACK1 ogni 2s. Nella finestra tra
          // la crea_sessione e il FASE1 successivo, g_sessione_attiva_id
          // e' gia' popolato da STACK1 (dalla risposta della RPC), quindi
          // 'ho' e' vero e non si aprono sessioni doppie.
          if (r.indexOf("\"sessione_attiva_id\":null") > 0) {
            if (g_sessione_attiva_id[0] != '\0') {
              Serial.println("[SESS] Sessione chiusa lato DB — stato locale allineato");
            }
            memset(g_sessione_attiva_id, 0, 37);
            g_sessioneAvviataEpoch = 0;
            g_sessioneMia          = false;
          } else {
            int si = r.indexOf("\"sessione_attiva_id\":\"");
            if (si > 0) {
              si += 22;
              char idDaDb[37] = "";
              r.substring(si, si+36).toCharArray(idDaDb, 37);
              if (strncmp(idDaDb, g_sessione_attiva_id, 36) != 0) {
                strncpy(g_sessione_attiva_id, idDaDb, 37);
                g_sessione_attiva_id[36] = '\0';
                // Sessione NON aperta da noi in questa sessione di lavoro
                // (riavvio del POD, o apertura dalla web app): l'epoch non
                // lo conosciamo. Il timeout 72h e' del job DB, quindi 0 va
                // bene: la guardia (epoch != 0) semplicemente non scatta.
                g_sessioneAvviataEpoch = 0;
                // ── V2.6.1: OWNERSHIP da dispositivi.sessione_attiva_da ─
                // Campo denormalizzato mantenuto dal trigger DB, leggibile
                // dal POD (anon) — vedi nota sull'endpoint FASE1 sopra.
                // Se non riusciamo a leggerlo il default PRUDENTE e' false
                // (= non nostra -> non la tocchiamo): meglio una sessione
                // che resta aperta di una sessione web uccisa per errore.
                int ad = r.indexOf("\"sessione_attiva_da\":\"");
                if (ad > 0) {
                  ad += 22;   // strlen("\"sessione_attiva_da\":\"")
                  int ae = r.indexOf('"', ad);
                  g_sessioneMia = (ae > ad) &&
                                  (strcmp(r.substring(ad, ae).c_str(), "visore") == 0);
                } else {
                  g_sessioneMia = false;
                }
                Serial.printf("[SESS] Sessione recuperata da DB: %s (avviata_da=%s → %s)\n",
                              g_sessione_attiva_id,
                              g_sessioneMia ? "visore" : "web/ignoto",
                              g_sessioneMia ? "la gestisce il POD" : "la gestisce la web app");
              }
            }
          }

          // ── V2.6.1 — GET avviata_il RIMOSSO (fix "POD offline") ──────
          // BUG V2.6.0 (16/07): qui c'era un GET dedicato su
          // sessioni_navigazione per leggere avviata_il (serviva SOLO al
          // timeout 72h). Ogni GET = 1 handshake TLS (2-4s su ESP32).
          // La guardia era (id != '' && epoch == 0): dopo una chiusura,
          // FASE1 poteva rileggere dal DB un sessione_attiva_id non ancora
          // aggiornato, ripopolare l'id e azzerare l'epoch (ramo
          // "recuperata") → la guardia si riapriva → TLS extra a ogni
          // FASE1, sommato alla RPC di chiusura → il ciclo TaskCloud
          // sfondava i 2s e la telemetria si diradava fino a fermarsi.
          // Osservato: gap di 156s e 209s, POD muto fino al riavvio.
          //
          // Il timeout 72h resta coperto dal job DB chiudi_sessioni_orfane_72h()
          // (gia' attivo e testato), che e' la sede giusta: funziona anche
          // a POD spento, che e' proprio il caso da coprire.
          // g_sessioneAvviataEpoch e' ora impostato SOLO da STACK1 alla
          // creazione, dal clock GPS: zero rete sul percorso caldo.

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
        // V2.6.0: il campo DB cmd_sessione trasporta ora un LIVELLO (0/1),
        // non piu' un comando (0/1/2). Nome della colonna invariato per non
        // rompere la web app; la semantica e' quella di sessione_attiva.
        // ⚠ Richiede la RIMOZIONE del trigger trigger_gestisci_sessione
        //   (task 4): quel trigger scatta su cmd_sessione != 0 a OGNI punto
        //   e con un livello costante a 1 scatterebbe di continuo.
        j += "\"cmd_sessione\":"+String((int)datiDalVisore.sessione_attiva)+",";
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
        j += "\"vmg_wp\":"+String((float)g_vmg_wp,1)+",";
        // ── V2.4.0 — Debug filtro SOG (logging only) ──────────────────
        j += "\"sog_raw\":"+String((float)g_sog_raw,2)+",";
        j += "\"accel_media\":"+String((float)g_accel_mag_media,3)+",";
        j += "\"accel_x_min\":"+String((float)g_ax_min,3)+",";
        j += "\"accel_x_max\":"+String((float)g_ax_max,3)+",";
        j += "\"accel_y_min\":"+String((float)g_ay_min,3)+",";
        j += "\"accel_y_max\":"+String((float)g_ay_max,3)+",";
        // ── V2.4.2 — Qualita' fix GPS ─────────────────────────────────
        j += "\"hdop\":"+String((float)g_hdop,2)+",";
        j += "\"fix_quality\":"+String((int)g_fixq)+",";
        j += "\"fix_quality_3d\":"+String((int)g_fixq3d);
        g_acc_win_reset = true;   // riarma finestra min/max per i prossimi 2s
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
      // STACK 1 — Gestione Sessione — V2.6.0: CONFRONTO DI STATO
      //
      //   vuole = livello dichiarato dal Visore (sessione_attiva 0/1)
      //   ho    = sessione realmente aperta a DB (g_sessione_attiva_id)
      //
      // Due condizioni, una variabile di stato. Il retry e' implicito: se
      // una RPC fallisce, la condizione resta vera e il ciclo dopo (2s)
      // riprova da solo. Nessun flag da azzerare, niente che resti incollato.
      //
      // Le RPC partono SOLO sulla transizione (vuole != ho): 2 volte per
      // sessione. A regime (vuole == ho) questo blocco non fa HTTP: il ciclo
      // resta ~2s e la telemetria non si interrompe (regressione V2.5.1
      // non ripresentabile).
      // ════════════════════════════════════════════════════════════════
      {
        bool vuole = (datiDalVisore.sessione_attiva != 0);
        bool ho    = (g_sessione_attiva_id[0] != '\0');

        // ── V2.6.1 — REGOLA DI OWNERSHIP: "chiude chi apre" ─────────────
        // BUG V2.6.0 (16/07): la regola (!vuole && ho → chiudi) non
        // guardava CHI aveva aperto la sessione. La web app apriva, FASE1
        // popolava g_sessione_attiva_id (ho=1), il Visore diceva vuole=0
        // → il POD chiudeva la sessione web entro 2s. Osservato: sessioni
        // web durate 13s e 25s, con 0 e 4 punti.
        //
        // g_sessioneMia = la sessione aperta e' del Visore (avviata_da='visore').
        // Il POD agisce SOLO sulle proprie: le sessioni 'web' le chiude la
        // web app con chiudi_sessione_da_webapp(). Simmetrico e completo.
        bool ho_mia = ho && g_sessioneMia;
        bool ho_web = ho && !g_sessioneMia;

        // ── Timeout 1: Visore assente da >180s → forza vuole=false ──────
        // Copre "Visore spento a sessione aperta": il POD chiude da solo.
        // 180s = 36x il LINK LOST del Visore (5s), sopravvive a un riavvio
        // del Visore (~15s) senza chiudere la sessione per sbaglio.
        // g_ultimoPacchettoVisore==0 → nessun pacchetto dal boot: non
        // possiamo dire che il Visore "e' sparito", quindi non forziamo.
        uint32_t ultimoRx = g_ultimoPacchettoVisore;
        bool visoreAssente = (ultimoRx != 0) &&
                             ((uint32_t)(millis() - ultimoRx) > VISORE_ASSENTE_TIMEOUT_MS);
        if (visoreAssente && vuole) {
          Serial.printf("[SESS] ⏱ Visore assente da >%lus — forzo chiusura\n",
                        VISORE_ASSENTE_TIMEOUT_MS/1000UL);
          vuole = false;
        }
        // Se il Visore e' sparito PRIMA di dire 0, l'ultimo livello letto
        // resta 1 in datiDalVisore: azzeriamo anche il campo, cosi' al
        // rientro non riparte una sessione fantasma da un dato vecchio.
        if (visoreAssente) datiDalVisore.sessione_attiva = 0;

        // ── Timeout 2: durata massima 72h → rete di sicurezza ───────────
        // Epoch reale (da avviata_il o dal clock GPS alla creazione), non
        // millis(): sopravvive al riavvio del POD. Solo se abbiamo un
        // timestamp valido dal GPS, altrimenti non possiamo giudicare.
        if (ho && vuole && g_sessioneAvviataEpoch != 0 && g_unix_timestamp != 0) {
          uint32_t nowEp = g_unix_timestamp;
          if (nowEp > g_sessioneAvviataEpoch &&
              (nowEp - g_sessioneAvviataEpoch) > SESSIONE_MAX_DURATA_SEC) {
            Serial.println("[SESS] ⏱ Sessione oltre 72h — chiusura di sicurezza");
            vuole = false;
          }
        }

        // ── La decisione: due livelli, due condizioni + ownership ───────
        //
        // OPZIONE (a) — "il Visore si aggancia, poi chiude chi apre":
        //   Se la web app ha gia' una sessione aperta e l'utente preme START
        //   sul Visore, crea_sessione_da_visore() e' idempotente e ritorna
        //   QUELLA sessione invece di duplicarne una. Il Visore ci registra
        //   dentro. Da quel momento la sessione e' "sua" a tutti gli effetti:
        //   e' stato il Visore a (ri)aprirla, quindi sara' il Visore a
        //   chiuderla — anche se avviata_da resta 'web' (l'origine storica
        //   non cambia: la sessione l'ha creata davvero la web app).
        //   g_sessioneMia viene forzato a true qui: e' il POD che decide
        //   l'ownership operativa, non il campo avviata_da.
        if (vuole && !ho) {
          // Il Visore dichiara sessione attiva, a DB non c'e': aprila.
          // crea_sessione_da_visore e' idempotente: se una sessione aperta
          // esiste gia' (es. aperta dalla web app), la RITORNA invece di
          // duplicarla → il Visore si AGGANCIA (opzione a).
          String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\"}";
          String resp;
          int rc = chiamaRPC("crea_sessione_da_visore", body, &resp);
          if (rc >= 200 && rc < 300) {
            // Risposta: "uuid" (con virgolette) — estrai i 36 char
            int q1 = resp.indexOf('"');
            if (q1 >= 0) {
              int q2 = resp.indexOf('"', q1+1);
              if (q2 > q1 && (q2-q1-1) == 36) {
                resp.substring(q1+1, q2).toCharArray(g_sessione_attiva_id, 37);
                // Epoch di avvio dal clock GPS: evita il GET di recupero.
                g_sessioneAvviataEpoch = g_unix_timestamp;
                // V2.6.1 — OWNERSHIP: e' stato il Visore ad aprirla (o ad
                // agganciarsi), quindi sara' il Visore a chiuderla.
                // "Chiude chi apre": vale anche sull'aggancio a una
                // sessione web preesistente.
                g_sessioneMia = true;
                Serial.printf("[SESS] ▶ Sessione APERTA/AGGANCIATA: %s\n", g_sessione_attiva_id);
              } else {
                Serial.println("[SESS] WARN risposta crea_sessione non parsabile — ritento");
              }
            }
          } else {
            // Nessun flag di errore da gestire: la condizione (vuole && !ho)
            // e' ancora vera, il prossimo ciclo (2s) riprova da solo.
            Serial.printf("[SESS] crea_sessione rc=%d — ritento tra 2s\n", rc);
          }

        } else if (!vuole && ho_mia) {
          // V2.6.1: SOLO se la sessione e' NOSTRA (ho_mia, non ho).
          // Le sessioni della web app non si toccano: le chiude la web app
          // con chiudi_sessione_da_webapp(). Senza questa guardia il POD
          // uccideva le sessioni web entro 2s (bug V2.6.0 del 16/07).
          // chiudi_sessione_da_visore e' idempotente: esce subito se
          // dispositivi.sessione_attiva_id e' gia' NULL.
          // p_session_dist e' il valore del Visore, usato dalla RPC SOLO
          // come fallback quando n_punti=0 (sessione senza copertura cloud).
          String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\","
                        "\"p_session_dist\":" + String(datiDalVisore.session_dist, 2) + "}";
          String resp;
          int rc = chiamaRPC("chiudi_sessione_da_visore", body, &resp);
          if (rc >= 200 && rc < 300) {
            Serial.printf("[SESS] ■ Sessione CHIUSA: %s (dist Visore=%.2f nm)\n",
                          g_sessione_attiva_id, datiDalVisore.session_dist);
            memset(g_sessione_attiva_id, 0, 37);
            g_sessioneAvviataEpoch = 0;
            g_sessioneMia          = false;
          } else {
            // Idem: la condizione resta vera, il prossimo ciclo riprova.
            // NON azzeriamo g_sessione_attiva_id qui: se lo facessimo, la
            // sessione resterebbe aperta a DB con statistiche a zero e
            // nessuno la chiuderebbe piu' (divergenza silenziosa).
            Serial.printf("[SESS] chiudi_sessione rc=%d — ritento tra 2s\n", rc);
          }

        } else if (vuole && ho_web) {
          // ── V2.6.1 — OPZIONE (a): AGGANCIO ────────────────────────────
          // C'e' una sessione APERTA dalla web app e l'utente preme START
          // sul Visore. Non ne apriamo una seconda (sarebbe un doppione):
          // il Visore si aggancia a quella e ci registra dentro — i punti
          // ci finiscono gia' (id_sessione nel POST STACK2).
          //
          // L'OWNERSHIP operativa passa al Visore, ma NON e' il FW a
          // deciderlo: e' crea_sessione_da_visore() che, sull'aggancio,
          // scrive gestita_da='visore' sul DB (V2.6.1). Qui allineiamo
          // solo lo specchio locale; al prossimo FASE1 il campo
          // dispositivi.sessione_attiva_da confermera' 'visore'.
          //
          // avviata_da a DB resta 'web': l'origine STORICA non si falsifica,
          // l'ha creata davvero la web app. gestita_da dice chi la governa
          // ORA ed e' quello che decide chi puo' chiuderla.
          //
          // NOTA (bug 16/07, trovato in test prima del flash): una versione
          // precedente forzava g_sessioneMia=true SOLO nel FW mentre la RPC
          // guardava avviata_da: il FW credeva di poter chiudere, il DB
          // rifiutava -> sessione mai chiusa. Una sola verita', nel DB.
          // Nessuna chiamata di rete: solo un flag locale.
          // La rivendicazione DEVE passare dal DB: e' li' che vive
          // gestita_da, ed e' quella che la RPC di chiusura guarda.
          // crea_sessione_da_visore() e' idempotente: vede la sessione
          // aperta, non ne crea una seconda, e scrive gestita_da='visore'.
          // Una sola chiamata, sulla transizione (non a ogni ciclo):
          // dopo, g_sessioneMia=true e questo ramo non rientra piu'.
          String body = "{\"p_pod_id\":\"" + String(NOME_POD) + "\"}";
          String resp;
          int rc = chiamaRPC("crea_sessione_da_visore", body, &resp);
          if (rc >= 200 && rc < 300) {
            g_sessioneMia          = true;
            g_sessioneAvviataEpoch = g_unix_timestamp;
            Serial.printf("[SESS] ⇄ Visore agganciato a sessione web %s — ownership al Visore\n",
                          g_sessione_attiva_id);
          } else {
            // Ritenta al prossimo ciclo: la condizione (vuole && ho_web)
            // e' ancora vera. NON impostiamo g_sessioneMia: senza la
            // scrittura di gestita_da a DB, lo STOP farebbe no-op e la
            // sessione non si chiuderebbe piu' (bug trovato in simulazione).
            Serial.printf("[SESS] aggancio rc=%d — ritento tra 2s\n", rc);
          }

        } else if (!vuole && ho_web) {
          // Sessione della web app, il Visore non c'entra: NON la tocchiamo.
          // La telemetria continua e i punti restano agganciati alla
          // sessione web (id_sessione nel POST STACK2) — e' esattamente
          // il comportamento voluto: registrare la sessione avviata da web.
          // Log throttled a 30s: qui si passa a ogni ciclo (2s).
          static unsigned long lastLogWeb = 0;
          if (millis() - lastLogWeb >= 30000) {
            lastLogWeb = millis();
            Serial.printf("[SESS] Sessione web attiva (%s) — la chiude la web app\n",
                          g_sessione_attiva_id);
          }
        }
        // vuole == ho → stato allineato, nessuna azione, zero HTTP.
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
  // V2.3.3: leggi MAC Visore da NVS → link immediato al boot
  {
    uint8_t macDaNvs[6];
    if (nvsBindLeggi(macDaNvs)) {
      memcpy(macVisore, macDaNvs, 6);
      memcpy(g_macVisorePrecedente, macDaNvs, 6);
      Serial.printf("[BIND-NVS] Binding ripristinato: %02X:%02X:%02X:%02X:%02X:%02X\n",
        macDaNvs[0],macDaNvs[1],macDaNvs[2],macDaNvs[3],macDaNvs[4],macDaNvs[5]);
    } else {
      Serial.println("[BIND-NVS] Nessun binding in NVS — TX sospeso");
    }
  }
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

    // ── V2.6.2 — WATCHDOG TaskCloud ───────────────────────────────────
    // Siamo nel ramo WL_CONNECTED: il WiFi e' associato (il POD si vede nel
    // router) ma questo NON garantisce che lo stack TLS sia vivo. Il caso
    // del 16/07: TaskCloud appeso in una GET senza timeout, heap congelato,
    // zero dati al DB, WiFi regolarmente associato -> nessun recovery.
    //
    // Se il battito del TaskCloud e' fermo da oltre la soglia, ricicliamo il
    // WiFi: e' esattamente cio' che faceva il riavvio del router a mano
    // (deassociazione -> i socket appesi cadono -> il task si sblocca ed
    // esce con errore -> il ciclo riparte).
    //
    // SOGLIA 15s: il ciclo cloud normale e' ~2s; una chiusura sessione
    // (POST telemetria + RPC) costa ~3s misurati. 15s = 5x il caso peggiore
    // noto: nessun falso positivo su rete lenta, ma recupero rapido.
    //
    // g_cloudHeartbeat==0 -> il task non ha ancora fatto un giro (boot):
    // non e' un blocco, non intervenire.
    // Aritmetica unsigned 32 bit: (now - hb) e' corretta anche al rollover
    // di millis() (~49 giorni). uint32_t esplicito = stesso tipo su ESP32
    // (ILP32) e in simulazione.
    const uint32_t CLOUD_WDT_TIMEOUT_MS = 15000UL;
    // Dopo un intervento diamo tempo al WiFi di riassociarsi e al task di
    // completare un giro, altrimenti ricicleremmo in loop ogni 15s.
    const uint32_t CLOUD_WDT_COOLDOWN_MS = 20000UL;
    static uint32_t lastWdtAction = 0;

    // ECCEZIONE OTA: eseguiOTA() gira DENTRO il TaskCloud ed e' bloccante
    // per minuti (download del firmware). Il battito si ferma legittimamente:
    // senza questa guardia il watchdog ciclerebbe il WiFi a meta' download,
    // abortendo l'aggiornamento. g_otaInCorso e' gia' volatile e definita in
    // OTA.ino. Durante l'OTA il watchdog e' sospeso e il battito riparte da
    // solo al primo giro utile dopo la fine.
    uint32_t hb = g_cloudHeartbeat;
    if (g_otaInCorso) {
      hb = 0;   // sospendi il watchdog: nessun falso positivo durante l'OTA
    }
    if (hb != 0) {
      uint32_t fermoDa = (uint32_t)millis() - hb;
      bool inCooldown = (lastWdtAction != 0) &&
                        (((uint32_t)millis() - lastWdtAction) < CLOUD_WDT_COOLDOWN_MS);
      if (fermoDa > CLOUD_WDT_TIMEOUT_MS && !inCooldown) {
        lastWdtAction = (uint32_t)millis();
        g_cloudWdtTrigger++;
        Serial.printf("[WDT] TaskCloud fermo da %lus con WiFi associato — "
                      "ciclo WiFi (intervento #%lu)\n",
                      (unsigned long)(fermoDa/1000UL),
                      (unsigned long)g_cloudWdtTrigger);
        // Ciclo WiFi: replica il riavvio del router. NON tocchiamo il task
        // (creato con handle NULL: non e' ne' uccidibile ne' resettabile
        // dall'esterno) e NON riavviamo il POD: la sessione in corso e i
        // dati del Visore restano. Se il task e' appeso in un socket, la
        // deassociazione lo fa uscire con errore e il ciclo riparte.
        WiFi.disconnect();
        delay(100);
        WiFi.begin(savedSSID.c_str(), savedPass.c_str());
        lastWifiAttempt = millis();   // non far ripartire subito il portale
        impostaLED(pixel.Color(255,0,255), false);  // magenta = watchdog
      }
    }
  }
}
