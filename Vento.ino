// =========================================================================
// AYE POD — Vento.ino
// Release: V45.4.5
//
// NOVITA':
//   1. Timeout sensore vento: se nessuna sentenza MWV valida in >3s
//      → g_awa = -999.0 (sentinella "sensore non disponibile")
//      → g_aws = 0.0
//      La webapp interpreta g_awa < -900 come "--" su display.
//
//   2. Soglia rumore anemometro: aws < 1.0 kn → 0.0
//      Il sensore a coppe in assenza di vento reale misura rumore
//      rotazionale (frizione, convezione termica) di 2-4 nodi.
//      La soglia 1.0 kn è conservativa: elimina il rumore senza
//      perdere vento leggero significativo (brezza 1.5kn è già rilevante).
//      NOTA: 1 kn = 0.51 m/s — il vento sotto 1 kn non è utile per la vela.
// =========================================================================

#define AWS_SOGLIA_KN        1.0f   // kn: sotto = rumore meccanico anemometro
#define VENTO_TIMEOUT_MS  3000UL   // ms: silenzio → sensore non connesso

static unsigned long _ultimo_vento_ms = 0;  // ultimo timestamp MWV valida

void calcolaVentoVero() {
  // g_awa < -900 = sentinella "sensore non disponibile"
  if (g_aws <= 0.05 || g_awa < -900.0) {
    g_tws = 0.0; g_twa = 0.0;
    g_twd = g_head;
    return;
  }

  float awa = g_awa;
  if (awa > 180.0) awa -= 360.0;

  if (g_sog < 0.1) {
    g_tws = g_aws; g_twa = awa;
    float twd = g_head + g_twa;
    while (twd >= 360.0) twd -= 360.0;
    while (twd < 0.0)   twd += 360.0;
    g_twd = twd;
    return;
  }

  float awaRad = awa * DEG_TO_RAD;
  float aw_x   = g_aws * sin(awaRad);
  float aw_y   = g_aws * cos(awaRad);
  float tw_x   = aw_x;
  float tw_y   = aw_y - g_sog;
  g_tws = sqrt(tw_x*tw_x + tw_y*tw_y);
  g_twa = atan2(tw_x, tw_y) * RAD_TO_DEG;

  float twd = g_head + g_twa;
  while (twd >= 360.0) twd -= 360.0;
  while (twd < 0.0)   twd += 360.0;
  g_twd = twd;
}

void leggiVento() {
  static String buffer = "";

  // ── Timeout sensore ────────────────────────────────────────────────────
  // Se non riceviamo una sentenza MWV valida da più di VENTO_TIMEOUT_MS,
  // segnaliamo "sensore non disponibile" con la sentinella -999
  if (_ultimo_vento_ms > 0 &&
      (millis() - _ultimo_vento_ms) > VENTO_TIMEOUT_MS) {
    if (g_awa > -900.0) {  // solo al primo timeout, non ogni ciclo
      Serial.println("[VENTO] Timeout: sensore non risponde → --");
      g_awa = -999.0;
      g_aws =  0.0;
      calcolaVentoVero();
    }
  }

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n' || c == '\r') {
      buffer.trim();
      if (buffer.length() > 10 && buffer.indexOf("MWV") >= 0) {
        int startIdx = buffer.indexOf('$');
        if (startIdx >= 0) buffer = buffer.substring(startIdx);

        int p1=buffer.indexOf(',');
        int p2=buffer.indexOf(',',p1+1);
        int p3=buffer.indexOf(',',p2+1);
        int p4=buffer.indexOf(',',p3+1);
        int p5=buffer.indexOf(',',p4+1);
        int p6=buffer.indexOf('*',p5+1);

        if (p1>0 && p6>p5) {
          String s_awa    = buffer.substring(p1+1, p2);
          String s_aws    = buffer.substring(p3+1, p4);
          String s_unit   = buffer.substring(p4+1, p5);
          String s_status = buffer.substring(p5+1, p6);

          if (s_status.indexOf('A')>=0 || s_status.indexOf('a')>=0) {
            float awa = s_awa.toFloat();
            float aws = s_aws.toFloat();

            // Conversione unità → nodi
            if      (s_unit.indexOf('M')>=0||s_unit.indexOf('m')>=0) aws*=1.94384f;
            else if (s_unit.indexOf('K')>=0||s_unit.indexOf('k')>=0) aws*=0.539957f;

            // Soglia rumore meccanico anemometro
            if (aws < AWS_SOGLIA_KN) aws = 0.0f;

            // Antispike (nessun anemometro supera 200kn)
            if (aws >= 0.0f && aws < 200.0f) {
              g_awa = awa;
              g_aws = aws;
              _ultimo_vento_ms = millis();  // aggiorna timestamp
              calcolaVentoVero();
            }
          } else {
            // Status 'V' (Void): segnale non valido — non aggiornare timestamp
            g_awa = -999.0;
            g_aws =  0.0;
          }
        }
      }
      buffer = "";
    } else {
      buffer += c;
      if (buffer.length() > 100) buffer = "";
    }
  }
}
