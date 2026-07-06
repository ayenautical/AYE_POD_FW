// =========================================================================
// AYE POD — GPS.ino
// Release: V2.3.1
//
// CHANGELOG V2.3.0:
//
//   1. FILTRO SOG RICALIBRATO PER NAVIGAZIONE IN ACQUE MOSSE
//      Problema: in barca con onde e rollio il filtro precedente (tarato per
//      edificio/asfalto) azzerava SOG durante navigazione reale perché:
//        a) BNO085 2D ha picchi intermittenti tra un'onda e l'altra (accel scende
//           momentaneamente sotto 0.12) → strato 2a forzava SOG=0
//        b) GPS mini PA1010D con antenna integrata è più soggetto a variazione
//           SOG con onde (SD > 0.50 kn anche a 4 nodi)
//        c) Gate 1.0 kn troppo alto: impediva la visualizzazione a velocità basse
//      Soluzione — soglie adattate per uso nautico reale:
//        BNO_FERMO_MEDIA:  0.12 → 0.06 m/s² (barca ferma su onde: ax,ay ~0.04)
//        BNO_MOTO_MEDIA:   0.40 → 0.30 m/s² (risposta più rapida in acque mosse)
//        GPS_SD_MAX:       0.50 → 0.65 kn   (variazione SOG con onde = reale)
//        GPS_MEDIA_FERMO:  0.30 → 0.20 kn   (soglia fermo più conservativa)
//        SOG_MIN_KN:       1.00 → 0.50 kn   (gate ridotto, bolina vento leggero)
//        GPS_DIRETTA_KN:   2.00 → 1.50 kn   (moto certo a velocità più bassa)
//
//   2. CALCOLO WAYPOINT — calcolaWaypoint() nuova funzione
//      Calcola in puro CPU (zero I2C, zero latenza bus):
//        BTW  — bearing to waypoint (gradi 0-359, riferito al Nord)
//        DTW  — distance to waypoint (miglia nautiche, double precision)
//        VMG_WP — SOG × cos(COG - BTW) verso il waypoint
//        WP_BEARING_REL — BTW relativo alla prora (heading corretto)
//                         Positivo = WP a dritta, Negativo = WP a sinistra
//        ETA_WP — secondi stimati all'arrivo (0 se VMG_WP ≤ 0.1 kn)
//        WP_ARRIVE_ALERT — attivo quando DTW < WP_RAGGIO_ARRIVO (default 50m)
//      IMPORTANTE:
//        - BTW usa COG GPS (non heading bussola): il vettore di percorso reale
//        - WP_BEARING_REL usa g_head (heading con remoteOffset applicato)
//        - COG non è influenzato da remoteOffset (viene dal GPS, non dal BNO)
//
//   3. haversineNM() — versione double precision per waypoint lontani
//      La haversineM() float32 esistente è mantenuta per Anchor Alert (<1km).
//      La nuova haversineNM() double è usata per distanze fino a 100 nm.
//
//   REGOLE ARCHITETTURALI INVARIATE:
//     Wire usato SOLO da loop() Core1
//     GPS: pattern Adafruit con break dopo parse
//     calcolaWaypoint() chiamata da loop() dopo leggiGPS() — CPU only
//
//   ALGORITMO SOG — 4 strati + gate finale (logica invariata, soglie modificate):
//
//   STRATO 1 — Kalman GPS freddo (< 3 min fix con sats>=6):
//     SOG = raw GPS. Il Kalman HW non ha ancora convergito.
//
//   STRATO 0 — GPS Consistency Check:
//     COG GPS stabile + spostamento reale → navigazione direzionale → raw
//
//   STRATO 2a — BNO085 fermo (< 0.06 m/s²):
//     Accelerazione 2D sotto soglia → barca ferma (anche con onde verticali)
//
//   STRATO 2b — BNO085 moto certo (> 0.30 m/s²) + GPS > 1.50 kn:
//     Moto fisico certo → raw immediato
//
//   STRATO 3 — SD buffer GPS 8 campioni:
//     SD > 0.65 kn → multipath o jitter anomalo → SOG = 0
//     Media < 0.20 kn → fermo → SOG = 0
//
//   STRATO 4 — moto lento confermato (zona grigia):
//     raw_sog > GPS_MEDIA_FERMO → SOG raw
//
//   GATE FINALE — SOG < 0.50 kn → forza 0:
//     Elimina drift GPS/GNSS noise floor in porto.
//     Latenza partenza ≤ 400ms (sopra 0.50 kn risposta immediata).
//
// Hardware: PA1010D I2C 0x10, 5Hz+SBAS, Kalman HW
// =========================================================================

// ── Soglie filtro SOG ─────────────────────────────────────────────────────
#define BNO_FERMO_MEDIA     0.06f  // m/s²: media 2D sotto → fermo certo
#define BNO_MOTO_MEDIA      0.30f  // m/s²: media 2D sopra → moto fisico
#define GPS_DIRETTA_KN      1.50f  // kn: con BNO_MOTO → raw immediato
#define GPS_SD_MAX          0.65f  // kn: SD buffer sopra → multipath/jitter
#define GPS_MEDIA_FERMO     0.20f  // kn: media buffer sotto → fermo
#define GPS_BUF_N           8      // campioni buffer SOG
#define COG_CONFIRM_MIN     3      // cicli SOG>0 per aggiornare COG
#define KALMAN_WARM_MS   180000UL  // ms: 3 min per Kalman HW convergito
#define SOG_MIN_KN          0.50f  // kn: gate finale — sotto questa soglia SOG=0

// ── GPS Consistency Check ─────────────────────────────────────────────────
#define COG_BUF_N           3      // campioni COG
#define COG_STD_SOGLIA     30.0f  // gradi: sotto = navigazione direzionale
#define GPS_DIST_MIN_M      2.0f   // m: spostamento minimo tra fix

// ── Anchor Alert — buffer media mobile posizione ──────────────────────────
#define ANCHOR_POS_BUF_N    3

// ── Waypoint — raggio arrivo di default ──────────────────────────────────
#define WP_RAGGIO_ARRIVO_NM  0.0270f  // 50 m in miglia nautiche

static unsigned long _gps_first_fix_ms = 0;
static unsigned long _gps_last_fix_ms  = 0;

unsigned long gpsAgeMs() {
  if (_gps_last_fix_ms == 0) return 99999UL;
  return millis() - _gps_last_fix_ms;
}

// Buffer circolare SOG
static float   _sog_buf[GPS_BUF_N];
static uint8_t _sog_idx = 0;
static uint8_t _sog_cnt = 0;

// Buffer COG per consistency check
static float   _cog_buf[COG_BUF_N];
static float   _lat_buf[COG_BUF_N];
static float   _lon_buf[COG_BUF_N];
static uint8_t _cog_idx = 0;
static uint8_t _cog_cnt = 0;

// Buffer media mobile posizione per Anchor Alert
static double  _anc_lat_buf[ANCHOR_POS_BUF_N];
static double  _anc_lon_buf[ANCHOR_POS_BUF_N];
static uint8_t _anc_idx = 0;
static uint8_t _anc_cnt = 0;
static double  _lat_smooth = 0.0;
static double  _lon_smooth = 0.0;

static void sogPush(float v) {
  _sog_buf[_sog_idx] = v;
  _sog_idx = (_sog_idx + 1) % GPS_BUF_N;
  if (_sog_cnt < GPS_BUF_N) _sog_cnt++;
}

static void cogPosPush(float cog_deg, float lat, float lon) {
  _cog_buf[_cog_idx] = cog_deg;
  _lat_buf[_cog_idx] = lat;
  _lon_buf[_cog_idx] = lon;
  _cog_idx = (_cog_idx + 1) % COG_BUF_N;
  if (_cog_cnt < COG_BUF_N) _cog_cnt++;
}

static void anchorPosPush(double lat, double lon) {
  _anc_lat_buf[_anc_idx] = lat;
  _anc_lon_buf[_anc_idx] = lon;
  _anc_idx = (_anc_idx + 1) % ANCHOR_POS_BUF_N;
  if (_anc_cnt < ANCHOR_POS_BUF_N) _anc_cnt++;
  double slat = 0.0, slon = 0.0;
  for (uint8_t i = 0; i < _anc_cnt; i++) {
    slat += _anc_lat_buf[i];
    slon += _anc_lon_buf[i];
  }
  _lat_smooth = slat / _anc_cnt;
  _lon_smooth = slon / _anc_cnt;
}

// ── Haversine float32 — per distanze < 1 km (Anchor Alert) ───────────────
static float haversineM(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0f;
  float dLat = (lat2 - lat1) * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  float a = sinf(dLat * 0.5f) * sinf(dLat * 0.5f) +
            cosf(lat1 * DEG_TO_RAD) * cosf(lat2 * DEG_TO_RAD) *
            sinf(dLon * 0.5f) * sinf(dLon * 0.5f);
  return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

// ── Haversine double — per waypoint fino a 100 nm ────────────────────────
// Usa double precision per evitare errori di troncamento su distanze lunghe.
static double haversineNM(double lat1, double lon1, double lat2, double lon2) {
  const double R = 3440.065; // Raggio Terra in miglia nautiche
  double dLat = (lat2 - lat1) * (M_PI / 180.0);
  double dLon = (lon2 - lon1) * (M_PI / 180.0);
  double a = sin(dLat * 0.5) * sin(dLat * 0.5) +
             cos(lat1 * (M_PI / 180.0)) * cos(lat2 * (M_PI / 180.0)) *
             sin(dLon * 0.5) * sin(dLon * 0.5);
  return R * 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
}

// ── Bearing iniziale da posizione barca a waypoint (gradi 0-359) ─────────
// Calcolo basato su formula del bearing orthodromico (great-circle).
// Usa COG GPS come riferimento per VMG_WP — indipendente da offset bussola.
static float bearingTo(double lat1, double lon1, double lat2, double lon2) {
  double dLon = (lon2 - lon1) * (M_PI / 180.0);
  double y = sin(dLon) * cos(lat2 * (M_PI / 180.0));
  double x = cos(lat1 * (M_PI / 180.0)) * sin(lat2 * (M_PI / 180.0)) -
             sin(lat1 * (M_PI / 180.0)) * cos(lat2 * (M_PI / 180.0)) * cos(dLon);
  float brng = (float)(atan2(y, x) * (180.0 / M_PI));
  while (brng <   0.0f) brng += 360.0f;
  while (brng >= 360.0f) brng -= 360.0f;
  return brng;
}

// ── Deviazione angolare COG ───────────────────────────────────────────────
static float cogAngularStd() {
  if (_cog_cnt < 2) return 999.0f;
  float sum = 0, sumSq = 0;
  for (uint8_t i = 0; i < _cog_cnt; i++) { sum += _cog_buf[i]; }
  float mean = sum / _cog_cnt;
  for (uint8_t i = 0; i < _cog_cnt; i++) {
    float d = _cog_buf[i] - mean;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    sumSq += d * d;
  }
  return sqrtf(sumSq / _cog_cnt);
}

static float sogSD() {
  if (_sog_cnt < 2) return 99.0f;
  float m = 0;
  for (uint8_t i = 0; i < _sog_cnt; i++) m += _sog_buf[i];
  m /= _sog_cnt;
  float v = 0;
  for (uint8_t i = 0; i < _sog_cnt; i++) v += (_sog_buf[i]-m)*(_sog_buf[i]-m);
  return sqrtf(v / (_sog_cnt - 1));
}

static float sogMean() {
  if (_sog_cnt == 0) return 0.0f;
  float m = 0;
  for (uint8_t i = 0; i < _sog_cnt; i++) m += _sog_buf[i];
  return m / _sog_cnt;
}

// ── Calcolo Unix timestamp da NMEA UTC ───────────────────────────────────
static uint32_t nmea_to_unix(uint16_t year2k, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second) {
  int y = year2k + 2000;
  int days = (y - 1970) * 365;
  for (int i = 1970; i < y; i++) {
    if ((i % 4 == 0 && i % 100 != 0) || (i % 400 == 0)) days++;
  }
  const uint8_t md[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  bool leap = ((y % 4 == 0 && y % 100 != 0) || (y % 400 == 0));
  for (int m = 0; m < month - 1; m++) {
    days += md[m];
    if (m == 1 && leap) days++;
  }
  days += day - 1;
  return (uint32_t)days * 86400UL +
         (uint32_t)hour * 3600UL +
         (uint32_t)minute * 60UL +
         (uint32_t)second;
}

// ── setupGPS() — V2.3.0: invariato rispetto V45.4.7 ──────────────────────
void setupGPS() {
  Serial.println("[GPS] Init PA1010D I2C 0x10...");
  if (!GPS.begin(0x10)) {
    Serial.println("[GPS] ERRORE: modulo non trovato!");
    return;
  }
  // Output: RMC + GGA — SOG, COG, lat, lon, sats, HDOP, timestamp
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // 5Hz — limite SBAS/EGNOS per PA1010D
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  // SBAS/EGNOS: ~1-3m in campo aperto (lago)
  GPS.sendCommand("$PMTK301,2*2E");
  GPS.sendCommand("$PMTK313,1*2E");
  Serial.printf("[GPS] OK — PA1010D 5Hz+SBAS, SOG gate %.1fkn, WP calcolo attivo\n",
                SOG_MIN_KN);
}

// ── leggiGPS() — V2.3.0 ──────────────────────────────────────────────────
void leggiGPS() {
  // Pattern Adafruit: leggi byte disponibili, interrompi dopo il primo parse.
  // Garantisce che il loop() non si blocchi su burst di NMEA.
  while (GPS.available()) {
    GPS.read();
    if (GPS.newNMEAreceived()) {
      GPS.parse(GPS.lastNMEA());
      break;
    }
  }

  g_sats = GPS.satellites;

  if (!GPS.fix || g_sats < 4) {
    g_sog = 0.0f;
    _gps_first_fix_ms = 0;
    return;
  }

  unsigned long now = millis();
  if (g_sats >= 6) {
    if (_gps_first_fix_ms == 0) _gps_first_fix_ms = now;
  } else {
    _gps_first_fix_ms = 0;
  }
  _gps_last_fix_ms = now;

  // ── Timestamp UTC da NMEA ────────────────────────────────────────────
  if (GPS.year > 0 && GPS.month >= 1 && GPS.month <= 12) {
    g_unix_timestamp = nmea_to_unix(GPS.year, GPS.month, GPS.day,
                                    GPS.hour, GPS.minute, GPS.seconds);
  }

  float raw_sog = GPS.speed;         // knots da $GPRMC
  float raw_lat = GPS.latitudeDegrees;
  float raw_lon = GPS.longitudeDegrees;
  float raw_cog = GPS.angle;         // COG da $GPRMC (track made good)

  sogPush(raw_sog);
  cogPosPush(raw_cog, raw_lat, raw_lon);
  anchorPosPush((double)raw_lat, (double)raw_lon);

  // ── STRATO 1: Kalman GPS non ancora convergito ───────────────────────
  bool kalman_caldo = (_gps_first_fix_ms > 0 &&
                       (now - _gps_first_fix_ms) >= KALMAN_WARM_MS);
  if (!kalman_caldo) {
    g_sog = (raw_sog < SOG_MIN_KN) ? 0.0f : raw_sog;
    if (raw_sog >= 0.3f) g_cog = GPS.angle;
    g_lat = raw_lat;
    g_lon = raw_lon;
    goto anchor_check;
  }

  // ── STRATO 0: GPS Consistency Check ─────────────────────────────────
  {
    static uint8_t cog_confirm = 0;
    bool gps_direzionale = false;
    if (_cog_cnt >= 2) {
      float cog_std = cogAngularStd();
      uint8_t oldest = (_cog_idx) % COG_BUF_N;
      float dist_m = haversineM(_lat_buf[oldest], _lon_buf[oldest], raw_lat, raw_lon);
      gps_direzionale = (cog_std < COG_STD_SOGLIA && dist_m > GPS_DIST_MIN_M);
    }

    if (gps_direzionale) {
      g_sog = (raw_sog < SOG_MIN_KN) ? 0.0f : raw_sog;
      cog_confirm++;
      if (cog_confirm >= COG_CONFIRM_MIN) g_cog = raw_cog;
      g_lat = raw_lat;
      g_lon = raw_lon;
      goto anchor_check;
    }

    // ── STRATO 2a: BNO085 fermo (accel 2D sotto soglia) ─────────────
    // Soglia abbassata a 0.06: barca ferma su onde verticali ha ax,ay ≈ 0.03-0.05
    // Le onde verticali agiscono su az (escluso dal calcolo 2D)
    if (g_accel_mag_media < BNO_FERMO_MEDIA) {
      g_sog = 0.0f;
      cog_confirm = 0;
      g_lat = raw_lat;
      g_lon = raw_lon;
      goto anchor_check;
    }

    // ── STRATO 2b: BNO085 moto certo + GPS sopra soglia ─────────────
    // Soglia BNO abbassata a 0.30: in acque mosse la media 2D è più variabile
    // Soglia GPS abbassata a 1.50: risposta più rapida in vento leggero
    if (g_accel_mag_media > BNO_MOTO_MEDIA && raw_sog >= GPS_DIRETTA_KN) {
      g_sog = (raw_sog < SOG_MIN_KN) ? 0.0f : raw_sog;
      cog_confirm++;
      if (cog_confirm >= COG_CONFIRM_MIN) g_cog = GPS.angle;
      g_lat = raw_lat;
      g_lon = raw_lon;
      goto anchor_check;
    }

    {
      // ── STRATO 3: SD buffer GPS ────────────────────────────────────
      // GPS_SD_MAX alzata a 0.65: con onde la SOG oscilla naturalmente
      // ma il moto è reale. Filtra solo multipath anomalo (SD > 0.65).
      float sd   = sogSD();
      float mean = sogMean();

      if (sd > GPS_SD_MAX || mean < GPS_MEDIA_FERMO) {
        g_sog = 0.0f;
        cog_confirm = 0;
        g_lat = raw_lat;
        g_lon = raw_lon;
        goto anchor_check;
      }

      // ── STRATO 4: moto lento confermato ───────────────────────────
      if (raw_sog > GPS_MEDIA_FERMO) {
        g_sog = (raw_sog < SOG_MIN_KN) ? 0.0f : raw_sog;
        cog_confirm++;
        if (cog_confirm >= COG_CONFIRM_MIN) g_cog = GPS.angle;
      } else {
        g_sog = 0.0f;
        cog_confirm = 0;
      }
      g_lat = raw_lat;
      g_lon = raw_lon;
    }
  }

anchor_check:
  // ── ANCHOR ALERT ─────────────────────────────────────────────────────
  if (g_anchor_attivo && g_anchor_raggio_m > 0.0f && _anc_cnt >= 2) {
    float dist = haversineM(
      (float)g_anchor_lat, (float)g_anchor_lon,
      (float)_lat_smooth,  (float)_lon_smooth);
    g_anchor_alert = (dist > g_anchor_raggio_m);
  } else {
    g_anchor_alert = false;
  }
}

// ── calcolaWaypoint() — V2.3.0 ───────────────────────────────────────────
// Calcola BTW, DTW, VMG_WP, WP_BEARING_REL, ETA, arrive alert.
// Chiamata da loop() dopo leggiGPS() — CPU only, zero Wire.
//
// NOTE CRITICHE:
//   - BTW e VMG_WP usano COG (da GPS, indipendente da offset bussola)
//   - WP_BEARING_REL usa g_head (heading BNO con remoteOffset già applicato)
//   - COG è aggiornato solo con SOG > 0 (COG_CONFIRM_MIN cicli) — stabile
//   - Se wp_active = 0 o pos = 0,0: azzera tutti i dati WP
//   - ETA = 0 se VMG_WP ≤ 0.1 kn (barca ferma o allontanamento dal WP)
void calcolaWaypoint() {
  // Nessun waypoint attivo: azzera e ritorna
  if (!g_wp_active || (g_wp_lat == 0.0 && g_wp_lon == 0.0)) {
    g_btw            = 0.0f;
    g_dtw            = 0.0f;
    g_vmg_wp         = 0.0f;
    g_wp_bearing_rel = 0.0f;
    g_eta_wp_sec     = 0;
    g_wp_arrive_alert = false;
    return;
  }
  // Fix GPS necessario per calcoli validi
  if (g_lat == 0.0 && g_lon == 0.0) return;

  // Distanza (double precision per waypoint fino a 100 nm)
  double dtw_nm = haversineNM((double)g_lat, (double)g_lon,
                               (double)g_wp_lat, (double)g_wp_lon);
  g_dtw = (float)dtw_nm;

  // Bearing verso waypoint (0-359°, riferito al Nord geografico)
  g_btw = bearingTo((double)g_lat, (double)g_lon,
                    (double)g_wp_lat, (double)g_wp_lon);

  // VMG verso waypoint: SOG × cos(COG - BTW)
  // Usa g_cog (track made good GPS) — non influenzato da offset bussola
  if (g_sog > 0.1f) {
    float diff = (float)g_cog - g_btw;
    while (diff >  180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    g_vmg_wp = (float)g_sog * cosf(diff * DEG_TO_RAD);
  } else {
    g_vmg_wp = 0.0f;
  }

  // Bearing relativo alla prora (heading con offset bussola già applicato)
  // Positivo = WP a dritta, Negativo = WP a sinistra
  // Utile per capire se girare a dritta o sinistra per puntare il WP
  float rel = g_btw - (float)g_head;
  while (rel >  180.0f) rel -= 360.0f;
  while (rel < -180.0f) rel += 360.0f;
  g_wp_bearing_rel = rel;

  // ETA: distanza residua / VMG_WP convertita in secondi
  if (g_vmg_wp > 0.1f) {
    g_eta_wp_sec = (uint32_t)((dtw_nm / (double)g_vmg_wp) * 3600.0);
  } else {
    g_eta_wp_sec = 0; // barca ferma, fuori rotta o in allontanamento
  }

  // Arrive alert: dentro raggio 50m
  g_wp_arrive_alert = (dtw_nm < (double)WP_RAGGIO_ARRIVO_NM);
}

// ── calcolaVMG() — INVARIATA ─────────────────────────────────────────────
// VMG Vento = SOG × cos(TWA)
// Positivo = bolina (avanzamento verso il vento)
// Negativo = portante (informativo, non clampato)
void calcolaVMG() {
  g_vmg = 0.0f;
  if (g_sog > 0.2f && g_tws > 0.1f) {
    float twa = g_twa;
    if (twa >  180.0f) twa -= 360.0f;
    if (twa < -180.0f) twa += 360.0f;
    g_vmg_wind = g_sog * cosf(twa * DEG_TO_RAD);
  } else {
    g_vmg_wind = 0.0f;
  }
}

String formattaDatoGeografico(double val) {
  if (val == 0.0) return "0,0000000";
  String s = String(val, 7);
  s.replace('.', ',');
  return s;
}
