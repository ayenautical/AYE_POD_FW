# AYE POD Firmware

[![Version](https://img.shields.io/badge/version-2.3.3-blue)](https://github.com/AYE-Nautical/AYE_POD_FW/releases)
[![License](https://img.shields.io/badge/license-Proprietary-red)](LICENSE)
[![Hardware](https://img.shields.io/badge/hardware-ESP32--S3%20Feather-green)](https://www.adafruit.com/product/5477)

Firmware ufficiale per il **AYE POD** — sistema di telemetria nautica in tempo reale sviluppato da **AYE Nautical Systems**.

---

## Architettura

```
┌─ Core 0 ────────────────────────────────────────────┐
│  TaskCloud — HTTP Supabase ogni 2s, ZERO Wire       │
│  OTA       — Download + SHA256 verify (on demand)   │
└─────────────────────────────────────────────────────┘
┌─ Core 1 — loop() 50Hz ──────────────────────────────┐
│  leggiBussola()     BNO085 I2C → heading/roll/pitch  │
│  leggiGPS()         PA1010D I2C → SOG/COG/coord      │
│  calcolaWaypoint()  CPU only → BTW/DTW/VMG_WP/ETA     │
│  leggiVento()       Serial1 NMEA → AWA/AWS           │
│  calcolaVentoVero() CPU → TWA/TWS/TWD                │
│  inviaDatiVisore()  ESP-NOW 10Hz → AYE Visore        │
└─────────────────────────────────────────────────────┘
```

## Hardware

| Componente | Modello | Interfaccia |
|---|---|---|
| MCU | Adafruit ESP32-S3 Feather 2MB PSRAM | — |
| IMU | BNO085 | I2C 0x4A |
| GPS | PA1010D | I2C 0x10 |
| Anemometro | NMEA 0183 | Serial1  |
| LED | NeoPixel | Pin 33 |

## Moduli

| File | Funzione |
|---|---|
| `AYE_POD_v37_DEV.ino` | Main — setup(), loop(), struct ESP-NOW |
| `Bussola.ino` | BNO085 quaternione → heading/roll/pitch |
| `GPS.ino` | Parser incrementale + calcolaWaypoint() — BTW/DTW/VMG_WP/ETA |
| `Vento.ino` | NMEA MWV + calcolo vento vero |
| `Visore.ino` | ESP-NOW TX/RX con AYE Visore |
| `Rete_Cloud.ino` | TaskCloud Core0 — HTTP Supabase |
| `OTA.ino` | Over-The-Air update via GitHub Releases |
| `Utils.ino` | LED, telemetria seriale, SemVer |

## Versionamento

Schema **SemVer** (MAJOR.MINOR.PATCH):

| Tipo | Quando |
|---|---|
| PATCH | Bugfix, tuning — nessuna nuova funzione |
| MINOR | Nuove funzionalità retrocompatibili |
| MAJOR | Breaking change struct ESP-NOW o partition scheme |

## OTA (Over-The-Air Update)

Il firmware supporta aggiornamenti remoti via **GitHub Releases + Supabase**:
- Il DB Supabase contiene `ota_version`, `ota_url`, `ota_sha256`
- Il POD confronta la versione al boot e ogni 72h
- Download con verifica SHA256 byte-per-byte
- Dual-slot ESP32: rollback automatico in caso di errore
- Max 3 retry automatici, poi stato `error` nel DB

## Licenza

© 2026 AYE Nautical Systems. Tutti i diritti riservati.
Vedere [LICENSE](LICENSE) per i termini d'uso.

---

> ⚠️ Questo repository è **source available** ma non open source.
> Il codice è visibile a scopo di audit e trasparenza tecnica.
> Qualsiasi uso commerciale richiede autorizzazione scritta.