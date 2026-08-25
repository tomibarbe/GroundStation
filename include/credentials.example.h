// credentials.example.h — copy to credentials.h and fill in locally (credentials.h is gitignored)
#pragma once

// WiFi
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASS     "YOUR_WIFI_PASSWORD"

// WiFi connection timeout (ms)
#define WIFI_TIMEOUT 30000

// ========================================
// LoRa Configuration (SX1278)
// ========================================
#define LORA_FREQUENCY 433E6  // 433 MHz
#define LORA_SS_PIN    5      // NSS/CS pin  → IO5
#define LORA_RST_PIN   14     // Reset pin   → IO14
#define LORA_DI0_PIN   32     // DIO0 pin    → IO32

// LoRa settings
#define LORA_BANDWIDTH 125E3       // 125 kHz
#define LORA_SPREADING_FACTOR 9    // SF9 — longer range
#define LORA_CODING_RATE 5         // 4/5
#define LORA_SYNC_WORD 0x12        // Private network sync word

// ========================================
// AWS IoT Core Configuration
// ========================================
#define AWS_IOT_ENDPOINT "YOUR-ATS-ENDPOINT.iot.REGION.amazonaws.com"
#define AWS_IOT_PORT 8883

// Device information
#define DEVICE_ID "lora-ground-station-001"
#define AWS_IOT_CLIENT_ID DEVICE_ID

// Topics
#define MQTT_TOPIC_UPLINK_RAW    "geoparcela/uplink/raw"
#define MQTT_TOPIC_UPLINK_PARSED "geoparcela/uplink/%s/%s" // dev_id/msg_type
#define MQTT_TOPIC_UPLINK_PARSED_FALLBACK "geoparcela/uplink/parsed" // fallback if dynamic topics fail
#define MQTT_TOPIC_STATUS        "geoparcela/gs/status"

// MQTT settings
#define MQTT_TIMEOUT 5000
#define MQTT_RECONNECT_DELAY 5000

// ========================================
// Certificate file paths (in data folder)
// ========================================
#define AWS_CERT_CA "/ca-cert.pem"
#define AWS_CERT_CRT "/device-cert.pem"
#define AWS_CERT_PRIVATE "/private-key.pem"

// ========================================
// System Configuration
// ========================================
#define SERIAL_BAUD 115200
#define STATUS_LED_PIN 2        // Built-in LED
#define HEARTBEAT_INTERVAL 30000 // Send status every 30 seconds

// Emulation of collars (1 to enable, 0 to disable)
#ifndef EMU_COLLARS
#define EMU_COLLARS 0
#endif

// Skip LoRa init for testing (1 to skip, 0 to init normally)
#ifndef SKIP_LORA
#define SKIP_LORA 0
#endif

// Debug settings
#define DEBUG_ENABLED 1
#if DEBUG_ENABLED
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_PRINTF(x, ...) Serial.printf(x, __VA_ARGS__)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_PRINTF(x, ...)
#endif
