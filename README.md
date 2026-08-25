# GeoParcela GroundStation

ESP32/ESP8266 LoRa gateway: receives collar CSV over LoRa and publishes JSON to AWS IoT Core.

## Setup

1. Copy `include/credentials.example.h` → `include/credentials.h` and fill WiFi + AWS endpoint.
2. Put device certs in `data/` (`ca-cert.pem`, `device-cert.pem`, `private-key.pem`).
3. Build/upload:

```powershell
pio run -e groundstation_esp32 -t upload
pio run -e groundstation_esp32 -t uploadfs
```

## Canonical uplink JSON

`publishParsed()` emits webapp field names (`msg_id`, `timestamp`, `speed_kmh`, `battery_v`, …).
See the webapp `Documents/INTEGRATION.md` for the full contract.
