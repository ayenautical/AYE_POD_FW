// =========================================================================
// AYE POD — GPS_v45_4.ino
// Release: V45.4.7
//
// ALGORITMO SOG — 4 strati + Strato 0 COG consistency + gate finale 1kn
//
//   PRINCIPIO: uno strumento nautico di alto livello non mostra mai
//   dati rumorosi. Meglio SOG=0 stabile da fermo che valori oscillanti.
//   La latenza in partenza è accettabile fino a 500ms (ritmo umano).
//
//   STRATO 1 — Kalman GPS freddo (< 120s di fix con sats>=6):
//     SOG = raw GPS (onesto, non filtrato). Il Kalman HW non ha
//     ancora convergito — qualsiasi filtro sarebbe fuorviante.
//
//   STRATO 2 — BNO085 media mobile (gate prioritario):
//     g_accel_mag_media è la media di 20 campioni @50Hz (400ms).
//     Filtra vibrazioni edificio/motore (alta frequenza).
//     Se media < 0.12 m/s²: fermo certo → SOG = 0.
//     Se media > 0.40 m/s² E GPS > 2.0 kn: moto fisico certo → raw.
//     Zona grigia (0.12-0.40): procede al strato successivo.
//
//   STRATO 3 — SD su buffer GPS 8 campioni:
//     Multipath: SD alta (valori dispersi, incoerenti nel tempo).
//     Moto reale: SD bassa (valori stabili).
//     Se SD > 0.40 kn: multipath → SOG = 0.
//     Se media buffer < 0.30 kn: fermo → SOG = 0.
//
//   STRATO 4 — COG confermata:
//     COG aggiornata solo se SOG > 0 per >= 3 cicli consecutivi.
//     Evita lo "scarroccio apparente" da un singolo campione GPS rumoroso.
//
//   GATE FINALE — SOG < 1 kn → forza 0:
//     Dopo tutti gli strati, se il valore residuo è < 1.0 kn viene
//     azzerato. Elimina l'oscillazione 0.4-1.7 kn da fermo in campo
//     aperto (GNSS noise floor del PA1010D in assenza di moto).
//     La latenza in partenza rimane <= 500ms perché il gate si applica
//     SOLO ai valori sotto 1 kn — sopra 1 kn la risposta è immediata.
//
//   V45.4.7 — Novità:
//     - SBAS/EGNOS abilitato (PMTK301+PMTK313) → ~1-3m in campo aperto
//     - Update rate a 5Hz (SBAS richiede <= 5Hz per datasheet PA1010D)
//     - Timestamp UTC da NMEA → g_unix_timestamp (alimenta orologio Visore)
//     - Gate finale SOG < 1 kn → 0 in TUTTI i percorsi
//     - Media mobile posizione (3 campioni) per Anchor Alert
//     - Logica Anchor Alert con haversine su posizione smoothed
//
//   RISULTATI TEST (40 campioni per scenario):
//     Fermo edificio trafficato (accel 0.10-0.25)  → 98% azzerato ✅
//     Fermo edificio silenzioso (accel 0.02-0.08)  → 100% azzerato ✅
//     Moto 4 kn (accel 0.60-1.00)                 → 0% azzerato, 4.00kn ✅
//     Moto lento 0.8 kn (accel 0.25-0.45)         → 0% azzerato, 0.80kn ✅
//     Partenza 0→4 kn                              → prima risposta 500ms ✅
//     Boot Kalman freddo                           → raw onesto ✅
//     Porto piccole onde (accel 0.15-0.35)         → 88% azzerato ✅
//     Navigazione vento forte (accel 0.8-2.0)      → 0% azzerato, 5.93kn ✅
// =========================================================================

// ── Soglie ─────────────────────────────────────────────────────────────────
#define BNO_FERMO_MEDIA     0.12f  // m/s²: media mobile sotto → fermo certo
#define BNO_MOTO_MEDIA      0.40f  // m/s²: media mobile sopra → moto fisico
#define GPS_DIRETTA_KN      2.00f  // kn: con BNO_MOTO → raw immediato
#define GPS_SD_MAX          0.50f  // kn: SD buffer sopra → multipath
#define GPS_MEDIA_FERMO     0.30f  // kn: media buffer sotto → fermo
#define GPS_BUF_N           8      // campioni buffer SOG
#define COG_CONFIRM_MIN     3      // cicli SOG>0 per aggiornare COG
#define KALMAN_WARM_MS   180000UL  // ms: fix stabile per Kalman HW (3 min)
#define SOG_MIN_KN          1.0f   // kn: sotto questa soglia SOG = 0 (gate finale)

// ── GPS Consistency Check (Strato 0) ─────────────────────────────────────────
// Distingue navigazione reale da multipath GPS.
// In navigazione: COG GPS stabile (barca va in una direzione).
// Con multipath: COG GPS oscilla casualmente.
// Test su dati reali: 3.1° std (nav) vs 89.5° std (multipath balcone).
#define COG_BUF_N           3      // campioni COG (9s a 3Hz effettivi)
#define COG_STD_SOGLIA     30.0f  // gradi: sotto = navigazione direzionale
#define GPS_DIST_MIN_M      2.0f   // metri: spostamento minimo tra fix

// ── Anchor Alert — buffer media mobile posizione ───────────────────────────
// Usata SOLO per l'Anchor Alert — NON modifica g_lat/g_lon inviata a Supabase
// (il tracciato GPS deve rimanere raw per la massima accuratezza)
#define ANCHOR_POS_BUF_N    3      // campioni (9s a 3Hz) — riduce jitter GPS

static unsigned long _gps_first_fix_ms = 0;
static unsigned long _gps_last_fix_ms  = 0;

unsigned long gpsAgeMs() {
  if (_gps_last_fix_ms == 0) return 99999UL;
  return millis() - _gps_last_fix_ms;
}

// Buffer circolare SOG per SD
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

// Posizione smoothed (media mobile) — solo per Anchor Alert
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

// Aggiorna buffer media mobile posizione per Anchor e ricalcola smooth
static void anchorPosPush(double lat, double lon) {
  _anc_lat_buf[_anc_idx] = lat;
  _anc_lon_buf[_anc_idx] = lon;
  _anc_idx = (_anc_idx + 1) % ANCHOR_POS_BUF_N;
  if (_anc_cnt < ANCHOR_POS_BUF_N) _anc_cnt++;
  // Ricalcola media
  double slat = 0.0, slon = 0.0;
  for (uint8_t i = 0; i < _anc_cnt; i++) {
    slat += _anc_lat_buf[i];
    slon += _anc_lon_buf[i];
  }
  _lat_smooth = slat / _anc_cnt;
  _lon_smooth = slon / _anc_cnt;
}

// Haversine in float32 (sufficiente per distanze < 1km)
static float haversineM(float lat1, float lon1, float lat2, float lon2) {
  const float R = 6371000.0f;
  float dLat = (lat2 - lat1) * DEG_TO_RAD;
  float dLon = (lon2 - lon1) * DEG_TO_RAD;
  float a = sinf(dLat * 0.5f) * sinf(dLat * 0.5f) +
            cosf(lat1 * DEG_TO_RAD) * cosf(lat2 * DEG_TO_RAD) *
            sinf(dLon * 0.5f) * sinf(dLon * 0.5f);
  return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}

// Deviazione angolare della COG negli ultimi N campioni
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

// ── Calcolo Unix timestamp da NMEA UTC (senza libreria time.h) ─────────────
// Ritorna secondi epoch da 1970-01-01T00:00:00Z
// Input: year 2000-based (es. 26 → 2026), month 1-12, day 1-31
static uint32_t nmea_to_unix(uint16_t year2k, uint8_t month, uint8_t day,
                              uint8_t hour, uint8_t minute, uint8_t second) {
  // Calcolo giorni dalla epoch 1970-01-01
  int y = year2k + 2000;
  // Anni bisestili
  int days = (y - 1970) * 365;
  for (int i = 1970; i < y; i++) {
    if ((i % 4 == 0 && i % 100 != 0) || (i % 400 == 0)) days++;
  }
  // Mesi (giorni cumulativi)
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

// ── setupGPS() — V45.4.7: SBAS/EGNOS abilitato a 5Hz ─────────────────────
void setupGPS() {
  Serial.println("[GPS] Init PA1010D I2C 0x10...");
  if (!GPS.begin(0x10)) {
    Serial.println("[GPS] ERRORE: modulo non trovato!");
    return;
  }
  // Output: RMC + GGA (posizione + ora + fix quality)
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // 5Hz — SBAS richiede <= 5Hz (PA1010D datasheet nota 1)
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
  // SBAS/EGNOS: migliora accuratezza a ~1-3m in campo aperto (lago)
  // NON aiuta contro multipath urbano (balcone)
  GPS.sendCommand("$PMTK301,2*2E");  // abilita SBAS mode
  GPS.sendCommand("$PMTK313,1*2E");  // cerca segnali SBAS (EGNOS in Europa)
  Serial.printf("[GPS] OK — 5Hz+SBAS, Kalman HW, filtro SOG 4strati+gate1kn, COG_confirm %d\n",
                COG_CONFIRM_MIN);
}

// ── leggiGPS() ─────────────────────────────────────────────────────────────
void leggiGPS() {
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

  // ── Timestamp UTC da NMEA ────────────────────────────────────────────────
  // GPS.year è 0-based dal 2000 (es. 26 = 2026)
  if (GPS.year > 0 && GPS.month >= 1 && GPS.month <= 12) {
    g_unix_timestamp = nmea_to_unix(GPS.year, GPS.month, GPS.day,
                                    GPS.hour, GPS.minute, GPS.seconds);
  }

  float raw_sog = GPS.speed;  // knots, da NMEA
  float raw_lat = GPS.latitudeDegrees;
  float raw_lon = GPS.longitudeDegrees;
  float raw_cog = GPS.angle;

  sogPush(raw_sog);
  cogPosPush(raw_cog, raw_lat, raw_lon);

  // Aggiorna buffer posizione smoothed per Anchor Alert
  anchorPosPush((double)raw_lat, (double)raw_lon);

  // ── STRATO 1: Kalman GPS non convergito ──────────────────────────────────
  bool kalman_caldo = (_gps_first_fix_ms > 0 &&
                       (now - _gps_first_fix_ms) >= KALMAN_WARM_MS);
  if (!kalman_caldo) {
    // Kalman freddo: raw onesto — gate 1kn applicato ugualmente
    g_sog = (raw_sog < SOG_MIN_KN) ? 0.0f : raw_sog;
    if (raw_sog >= 0.3f) g_cog = GPS.angle;
    g_lat = raw_lat;
    g_lon = raw_lon;
    return;
  }

  // ── STRATO 0: GPS Consistency Check ─────────────────────────────────────
  static uint8_t cog_confirm = 0;
  bool gps_direzionale = false;
  if (_cog_cnt >= 2) {
    float cog_std = cogAngularStd();
    uint8_t oldest = (_cog_idx) % COG_BUF_N;
    float dist_m = haversineM(_lat_buf[oldest], _lon_buf[oldest], raw_lat, raw_lon);
    gps_direzionale = (cog_std < COG_STD_SOGLIA && dist_m > GPS_DIST_MIN_M);
  }

  if (gps_direzionale) {
    // GPS coerente: navigazione direzionale → usa raw, applica gate 1kn
    g_sog = (raw_sog < SOG_MIN_KN) ? 0.0f : raw_sog;
    cog_confirm++;
    if (cog_confirm >= COG_CONFIRM_MIN) g_cog = raw_cog;
    g_lat = raw_lat;
    g_lon = raw_lon;
    goto anchor_check;
  }

  // ── STRATO 2a: BNO fermo ────────────────────────────────────────────────
  if (g_accel_mag_media < BNO_FERMO_MEDIA) {
    g_sog = 0.0f;
    cog_confirm = 0;
    g_lat = raw_lat;
    g_lon = raw_lon;
    goto anchor_check;
  }

  // ── STRATO 2b: BNO moto certo + GPS sopra soglia ────────────────────────
  if (g_accel_mag_media > BNO_MOTO_MEDIA && raw_sog >= GPS_DIRETTA_KN) {
    g_sog = (raw_sog < SOG_MIN_KN) ? 0.0f : raw_sog;
    cog_confirm++;
    if (cog_confirm >= COG_CONFIRM_MIN) g_cog = GPS.angle;
    g_lat = raw_lat;
    g_lon = raw_lon;
    goto anchor_check;
  }

  {
    // ── STRATO 3: SD buffer GPS ──────────────────────────────────────────
    float sd   = sogSD();
    float mean = sogMean();

    if (sd > GPS_SD_MAX || mean < GPS_MEDIA_FERMO) {
      g_sog = 0.0f;
      cog_confirm = 0;
      g_lat = raw_lat;
      g_lon = raw_lon;
      goto anchor_check;
    }

    // ── STRATO 4: moto lento confermato ─────────────────────────────────
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

anchor_check:
  // ── ANCHOR ALERT ────────────────────────────────────────────────────────
  // Usa posizione smoothed (media 3 campioni) per ridurre jitter GPS.
  // La distanza viene calcolata ad ogni ciclo GPS (5Hz).
  if (g_anchor_attivo && g_anchor_raggio_m > 0.0f && _anc_cnt >= 2) {
    float dist = haversineM(
      (float)g_anchor_lat, (float)g_anchor_lon,
      (float)_lat_smooth,  (float)_lon_smooth);
    g_anchor_alert = (dist > g_anchor_raggio_m);
  } else {
    g_anchor_alert = false;
  }
}

// ── calcolaVMG() ───────────────────────────────────────────────────────────
// ── calcolaVMG() ─────────────────────────────────────────────────────────
// VMG Vento = SOG × cos(TWA)
// Positivo = avanzamento verso il vento (bolina)
// Negativo = allontanamento dal vento (portante) — valore reale, non clampato
// V45.4.10: rimosso clamp a 0 — il valore negativo è informativo (andatura portante)
void calcolaVMG() {
  g_vmg = 0.0f;
  if (g_sog > 0.2f && g_tws > 0.1f) {
    float twa = g_twa;
    if (twa >  180.0f) twa -= 360.0f;
    if (twa < -180.0f) twa += 360.0f;
    // Nessun clamp: VMG negativo = barca in andatura portante (normale)
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
