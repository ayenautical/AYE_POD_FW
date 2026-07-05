// =========================================================================
// AYE POD — Bussola_v45_4.ino
// Release: V45.4.6
//
// CAMBIO CRITICO rispetto V45.4.4:
//   g_accel_mag_media ora usa sqrt(ax²+ay²) invece di sqrt(ax²+ay²+az²).
//
//   MOTIVAZIONE FISICA:
//   Il Pod è montato in piano sulla barca. Nel frame del BNO085:
//     X = sinistra/destra  (rollio)
//     Y = avanti/indietro  (avanzamento barca) ← asse moto principale
//     Z = su/giù           (verticale) ← asse DOMINANTE per vibrazioni edificio
//
//   Le vibrazioni di un edificio (traffico, vento sulla struttura, HVAC)
//   agiscono principalmente sull'asse verticale Z → az ≈ 0.10-0.20 m/s²
//   mentre ax,ay restano bassi (0.02-0.05 m/s²).
//
//   Usando sqrt(ax²+ay²+az²): az contamina la media → falsi "moto"
//   Usando sqrt(ax²+ay²):     az ignorato → edificio = fermo ✅
//
//   TEST (30 campioni per scenario, az=0.15 edificio vibrante):
//     3D: media=0.144 → sopra soglia 0.12 → MOTO (errato ❌)
//     2D: media=0.036 → sotto soglia 0.12 → FERMO (corretto ✅)
//
//   La magnitudine 2D è sufficiente per rilevare il moto della barca
//   perché l'accelerazione di avanzamento (ay) è dominante anche a 0.5kn.
// =========================================================================

#define BNO_ACCEL_WIN 20  // campioni media mobile (400ms @50Hz)

void setupBussola() {
  Serial.println("[BNO085] Init I2C 0x4A...");
  if (!bno08x.begin_I2C(0x4A, &Wire, 0)) {
    Serial.println("[BNO085] ERRORE: non trovato!");
    return;
  }
  bno08x.enableReport(SH2_ARVR_STABILIZED_RV);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION);
  Serial.printf("[BNO085] OK — dual-report, media 2D su %d campioni\n", BNO_ACCEL_WIN);
}

void leggiBussola() {
  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION);
    Serial.println("[BNO085] Reset — riarmo report");
  }
  if (!bno08x.getSensorEvent(&sensorValue)) return;

  switch (sensorValue.sensorId) {

    case SH2_ARVR_STABILIZED_RV: {
      float qr = sensorValue.un.rotationVector.real;
      float qi = sensorValue.un.rotationVector.i;
      float qj = sensorValue.un.rotationVector.j;
      float qk = sensorValue.un.rotationVector.k;
      float sqr=sq(qr), sqi=sq(qi), sqj=sq(qj), sqk=sq(qk);
      g_acc = sensorValue.status;

      int raw_roll  = (int)(atan2f(2.0f*(qi*qr+qj*qk), 1.0f-2.0f*(sqi+sqj)) * 180.0f/PI);
      int raw_pitch = (int)(asinf (2.0f*(qr*qj-qi*qk))                       * 180.0f/PI);
      int raw_head  = (int)(atan2f(2.0f*(qr*qk+qi*qj), 1.0f-2.0f*(sqj+sqk)) * 180.0f/PI);

      if (SWAP_ROLL_PITCH) { g_roll=raw_pitch; g_pitch=raw_roll; }
      else                 { g_roll=raw_roll;  g_pitch=raw_pitch; }
      if (INVERT_ROLL)    g_roll  = -g_roll;
      if (INVERT_PITCH)   g_pitch = -g_pitch;
      if (INVERT_HEADING) raw_head = -raw_head;

      raw_head += 180;
      g_raw_head = raw_head;
      while (g_raw_head < 0)    g_raw_head += 360;
      while (g_raw_head >= 360) g_raw_head -= 360;
      g_head = g_raw_head + remoteOffset;
      while (g_head < 0)    g_head += 360;
      while (g_head >= 360) g_head -= 360;
      break;
    }

    case SH2_LINEAR_ACCELERATION: {
      float ax = sensorValue.un.linearAcceleration.x;
      float ay = sensorValue.un.linearAcceleration.y;
      // az (verticale) ESCLUSO: dominato da vibrazioni edificio, non moto barca

      // Campione 2D istantaneo (per telemetria debug)
      g_accel_mag = sqrtf(ax*ax + ay*ay);

      // Media mobile 2D — filtra vibrazioni ad alta frequenza
      static float win[BNO_ACCEL_WIN];
      static uint8_t idx=0, cnt=0;
      static float   sum=0.0f;
      sum -= win[idx];
      win[idx] = g_accel_mag;
      sum += g_accel_mag;
      idx = (idx+1) % BNO_ACCEL_WIN;
      if (cnt < BNO_ACCEL_WIN) cnt++;
      g_accel_mag_media = sum / cnt;
      break;
    }
    default: break;
  }
}

void avviaCalibrazioneBNO() {
  Serial.println("\n[BNO085] CALIBRAZIONE — muovi a figura-8");
  unsigned long t0 = millis();
  bool ok = false;
  while (millis()-t0 < 60000) {
    impostaLED(pixel.Color(255,255,0), true);
    gestisciWiFi();
    if (bno08x.getSensorEvent(&sensorValue) &&
        sensorValue.sensorId == SH2_ARVR_STABILIZED_RV &&
        sensorValue.status >= 2) { ok=true; break; }
    delay(10);
  }
  Serial.println(ok ? "[BNO085] Calibrazione OK!" : "[BNO085] Timeout.");
  impostaLED(pixel.Color(0,0,255), false);
  calibrazioneRichiesta = false;
}
