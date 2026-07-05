// =========================================================================
// AYE POD — Utils.ino
// Release: V2.0.0
// Changelog V2.0.0:
//   - confrontaVersioniSemVer(): nuova funzione per confronto MAJOR.MINOR.PATCH
//     Preparazione per la logica OTA (Fase 2) — non ancora chiamata in produzione
//     in questa release. Disponibile per i test preliminari.
//   - stampaTelemetria: aggiunto FreeHeap (test I2 — verifica memory leak) e
//     batteria_pod%/voltage (test I3 — monitoraggio autonomia LiPo+boost 6106)
//   - Fix: rimosso argomento g_accel_mag orfano nel printf (era passato senza
//     placeholder corrispondente — innocuo ma disallineato, ora pulito)
// Changelog V45.4.6 (precedente):
//   - stampaTelemetria: refresh 1s, mostra GPS age (ms dall'ultimo fix)
//     Utile per verificare la latenza in console durante i test
//   - Formato: angoli interi, velocita' 1 dec — coerente con display
// =========================================================================

// ── Confronto versioni SemVer (MAJOR.MINOR.PATCH) ──────────────────────────
// Ritorna: >0 se v1 > v2, <0 se v1 < v2, 0 se uguali
// Uso futuro (Fase 2): il POD confronta la versione disponibile sul DB
// con FW_VERSION corrente per decidere se avviare il download OTA.
// Esempio: confrontaVersioniSemVer("2.1.0", "2.0.0") > 0 → aggiornamento disponibile
int confrontaVersioniSemVer(const char* v1, const char* v2) {
  int major1=0, minor1=0, patch1=0;
  int major2=0, minor2=0, patch2=0;
  sscanf(v1, "%d.%d.%d", &major1, &minor1, &patch1);
  sscanf(v2, "%d.%d.%d", &major2, &minor2, &patch2);
  if (major1 != major2) return major1 - major2;
  if (minor1 != minor2) return minor1 - minor2;
  return patch1 - patch2;
}

// Wrapper: true se la versione remota è più recente della corrente
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

  // gpsAgeMs() definita in GPS_v45.ino — ms dall'ultimo fix valido
  // Valore atteso a 10Hz: < 120ms. Se > 500ms: problema di drenaggio buffer.
  // g_acc: accuratezza calibrazione BNO085 (0=non cal., 1=bassa, 2=media, 3=alta)
  //   Se HDG resta fisso a 0 nonostante movimento fisico, controllare g_acc:
  //   un valore 0-1 persistente indica che il sensore non si è mai calibrato
  //   (serve muovere il POD a figura-8 — vedi avviaCalibrazioneBNO())
  Serial.printf(
    "[TEL v2.0.0] HDG:%3d | SOG:%4.1f | COG:%3d | Dr:%+.0f | "
    "AWA:%+4.0f AWS:%4.1f | TWA:%+4.0f TWS:%4.1f | "
    "VMGw:%.1f | SATS:%d | GPS_age:%lums | ACC:%d\n",
    g_head, (float)g_sog, (int)g_cog, (float)g_drift,
    (float)g_awa, (float)g_aws,
    (float)g_twa, (float)g_tws,
    (float)g_vmg_wind,
    g_sats, gpsAgeMs(), g_acc
  );

  // ── Test I2: memoria stabile (free heap) ─────────────────────────────────
  // Valore atteso: oscillazioni piccole (±1-2KB) per allocazioni temporanee
  // (buffer HTTP/JSON) che vengono liberate. Un calo COSTANTE e progressivo
  // senza mai risalire indica un memory leak.
  Serial.printf("[MEM] FreeHeap:%u bytes\n", ESP.getFreeHeap());

  // ── Test I3: monitoraggio batteria (MAX17048 via Adafruit 6106 boost) ───
  // cellPercent(): percentuale stimata dal fuel gauge
  // cellVoltage(): tensione LiPo reale, più utile per seguire il decadimento
  // lineare prima del cutoff (~3.2V) rispetto alla sola percentuale stimata
  Serial.printf(
    "[BATT] %.0f%% | %.2fV | uptime:%lus\n",
    maxlipo.cellPercent(), maxlipo.cellVoltage(), millis() / 1000
  );
}
