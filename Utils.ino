// =========================================================================
// AYE POD — Utils.ino
// Release: V2.3.1
//
// Changelog V2.3.0:
//   - stampaTelemetria: aggiunta riga WP con BTW/DTW/VMG_WP/ETA/WP_REL
//     Visibile nel monitor seriale per debug waypoint navigation.
//
// Changelog V2.0.0 (precedente):
//   - confrontaVersioniSemVer() per OTA
//   - stampaTelemetria: FreeHeap + batteria_pod%/voltage
// =========================================================================

int confrontaVersioniSemVer(const char* v1, const char* v2) {
  int major1=0, minor1=0, patch1=0;
  int major2=0, minor2=0, patch2=0;
  sscanf(v1, "%d.%d.%d", &major1, &minor1, &patch1);
  sscanf(v2, "%d.%d.%d", &major2, &minor2, &patch2);
  if (major1 != major2) return major1 - major2;
  if (minor1 != minor2) return minor1 - minor2;
  return patch1 - patch2;
}

bool aggiornamentoDisponibile(const char* versioneRemota) {
  return confrontaVersioniSemVer(versioneRemota, FW_VERSION) > 0;
}

void setupUtils() {
  pixel.begin();
  pixel.setBrightness(30);
  impostaLED(pixel.Color(0, 0, 255), false);
  Wire.begin();
  if (!maxlipo.begin()) Serial.println("[ERR] MAX17048 non trovato!");
}

void impostaLED(uint32_t colore, bool lampeggio) {
  if (lampeggio && (millis() % 500 < 250)) pixel.setPixelColor(0, 0);
  else pixel.setPixelColor(0, colore);
  pixel.show();
}

void stampaTelemetria() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();

  Serial.printf(
    "[TEL V%s] HDG:%3d | SOG:%4.1f | COG:%3d | Dr:%+.0f | "
    "AWA:%+4.0f AWS:%4.1f | TWA:%+4.0f TWS:%4.1f | "
    "VMGw:%.1f | SATS:%d | GPS_age:%lums | BNOacc:%.2f | ACC:%d\n",
    FW_VERSION,
    g_head, (float)g_sog, (int)g_cog, (float)g_drift,
    (float)g_awa, (float)g_aws,
    (float)g_twa, (float)g_tws,
    (float)g_vmg_wind,
    g_sats, gpsAgeMs(),
    (float)g_accel_mag_media,
    g_acc
  );

  // ── Waypoint navigation (V2.3.0) ────────────────────────────────────
  if (g_wp_active && (g_wp_lat != 0.0 || g_wp_lon != 0.0)) {
    uint32_t eta = g_eta_wp_sec;
    Serial.printf(
      "[WP] BTW:%.0f° DTW:%.3fnm VMG_WP:%.1fkn WP_REL:%+.0f° ETA:%dm%ds %s\n",
      (float)g_btw,
      (float)g_dtw,
      (float)g_vmg_wp,
      (float)g_wp_bearing_rel,
      (int)(eta / 60), (int)(eta % 60),
      g_wp_arrive_alert ? "*** ARRIVE ***" : ""
    );
  } else {
    Serial.println("[WP] nessun waypoint attivo");
  }

  Serial.printf("[MEM] FreeHeap:%u bytes\n", ESP.getFreeHeap());
  Serial.printf(
    "[BATT] %.0f%% | %.2fV | uptime:%lus\n",
    maxlipo.cellPercent(), maxlipo.cellVoltage(), millis() / 1000
  );
}
