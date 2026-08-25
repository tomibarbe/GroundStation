#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <time.h>
#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <WiFiClientSecureBearSSL.h>
#else
#include <WiFi.h>
#include <WiFiClientSecure.h>
#endif
#include <PubSubClient.h>
#include <FS.h>
#include <LittleFS.h>

#include "credentials.h"

// Increase MQTT buffer size for larger JSON messages
#define MQTT_MAX_PACKET_SIZE 512

// Keep cert content alive (TLS client keeps pointers)
static String g_ca, g_crt, g_key;

#if defined(ESP8266)
BearSSL::WiFiClientSecure net;
static std::unique_ptr<BearSSL::X509List> g_ca_list;
static std::unique_ptr<BearSSL::X509List> g_crt_list;
static std::unique_ptr<BearSSL::PrivateKey> g_private_key;
#else
WiFiClientSecure net;
#endif
PubSubClient mqtt(net);

unsigned long lastHeartbeat = 0;
unsigned long lastEmuTx = 0;
uint32_t emuSeq = 0;

// Simple LRU cache for duplicate detection
struct DupEntry {
  String dev_id;
  uint32_t seq;
  unsigned long ts;
};
const int MAX_DUP_ENTRIES = 20;
DupEntry dupCache[MAX_DUP_ENTRIES];
int dupCacheIdx = 0;

bool loadFileToString(const char* path, String& out) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  out = f.readString();
  f.close();
  return out.length() > 0;
}

void connectWiFi() {
  DEBUG_PRINTLN("[WiFi] Connecting...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT) {
    delay(250);
    DEBUG_PRINT(".");
  }
  DEBUG_PRINTLN("");

  if (WiFi.status() != WL_CONNECTED) {
    DEBUG_PRINTLN("[WiFi] Failed, rebooting in 5s");
    delay(5000);
    ESP.restart();
  }
  DEBUG_PRINTF("[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());
}

void connectMQTT() {
  mqtt.setServer(AWS_IOT_ENDPOINT, AWS_IOT_PORT);
  mqtt.setBufferSize(MQTT_MAX_PACKET_SIZE); // Set larger buffer for JSON messages

  DEBUG_PRINTLN("[MQTT] Connecting to AWS IoT...");
  while (!mqtt.connected()) {
    if (mqtt.connect(AWS_IOT_CLIENT_ID)) {
      DEBUG_PRINTLN("[MQTT] Connected.");
      DEBUG_PRINTF("[MQTT] Buffer size: %d bytes\n", mqtt.getBufferSize());
    } else {
      DEBUG_PRINTF("[MQTT] Failed rc=%d. Retrying in %d ms\n", mqtt.state(), MQTT_RECONNECT_DELAY);
      delay(MQTT_RECONNECT_DELAY);
    }
  }
}

static void syncTime() {
#if defined(ESP8266) || defined(ESP32)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  const unsigned long maxWaitMs = 10000;
  unsigned long start = millis();
  time_t t = 0;
  while (((t = time(nullptr)) < 1609459200UL) && (millis() - start < maxWaitMs)) { // >= 2021-01-01
    delay(200);
  }
  DEBUG_PRINTF("[Time] now=%lu\n", (unsigned long)t);
#if defined(ESP8266)
  if (t > 0) { net.setX509Time(t); }
#endif
#endif
}

void sendStatus(const char* status, const char* detail = nullptr) {
  char buf[256];
  snprintf(buf, sizeof(buf),
           "{\"device\":\"%s\",\"status\":\"%s\",\"detail\":\"%s\",\"ip\":\"%s\",\"ts\":%lu}",
           DEVICE_ID, status, detail ? detail : "", WiFi.localIP().toString().c_str(), (unsigned long)time(nullptr));
  mqtt.publish(MQTT_TOPIC_STATUS, buf);
}

// Check if packet is duplicate
bool isDuplicate(const String& dev_id, uint32_t seq) {
  for (int i = 0; i < MAX_DUP_ENTRIES; i++) {
    if (dupCache[i].dev_id == dev_id && dupCache[i].seq == seq) {
      return true;
    }
  }
  return false;
}

// Add to duplicate cache
void addToDupCache(const String& dev_id, uint32_t seq) {
  dupCache[dupCacheIdx].dev_id = dev_id;
  dupCache[dupCacheIdx].seq = seq;
  dupCache[dupCacheIdx].ts = millis();
  dupCacheIdx = (dupCacheIdx + 1) % MAX_DUP_ENTRIES;
}

// Parse CSV format: v,t,id,seq,ts,lat,lon,spd,hdop,bat,act
struct ParsedPacket {
  int proto_ver;
  String msg_type;
  String dev_id;
  uint32_t seq;
  uint32_t ts;
  float lat, lon, spd_kmh, hdop, bat_v;
  String act;
  bool valid;
};

ParsedPacket parseCSV(const String& payload) {
  ParsedPacket p = {0};
  p.valid = false;
  
  int fields[11];
  String parts[11];
  int fieldCount = 0;
  int start = 0;
  
  // Split by comma
  for (int i = 0; i <= payload.length() && fieldCount < 11; i++) {
    if (i == payload.length() || payload[i] == ',') {
      parts[fieldCount] = payload.substring(start, i);
      fieldCount++;
      start = i + 1;
    }
  }
  
  if (fieldCount < 11) return p; // Need all 11 fields
  
  p.proto_ver = parts[0].toInt();
  p.msg_type = parts[1];
  p.dev_id = parts[2];
  p.seq = parts[3].toInt();
  p.ts = parts[4].toInt();
  p.lat = parts[5].toFloat();
  p.lon = parts[6].toFloat();
  p.spd_kmh = parts[7].toFloat();
  p.hdop = parts[8].toFloat();
  p.bat_v = parts[9].toFloat();
  p.act = parts[10];
  p.valid = true;
  
  return p;
}

// Publish RAW packet
void publishRaw(const String& payload, int rssi, float snr) {
  char msg[512];
  unsigned long gw_ts = (unsigned long)time(nullptr);
  snprintf(msg, sizeof(msg),
    "{\"gs\":\"%s\",\"ts\":%lu,\"rssi\":%d,\"snr\":%.1f,\"raw\":\"%s\"}",
    DEVICE_ID, gw_ts, rssi, snr, payload.c_str());
  
  DEBUG_PRINTF("[RAW] rssi=%d snr=%.1f payload=%s\n", rssi, snr, payload.c_str());
  mqtt.publish(MQTT_TOPIC_UPLINK_RAW, msg, false); // QoS 0
}

// Publish PARSED packet — canonical JSON contract (webapp field names).
// Collar LoRa CSV stays positional; names are assigned here once.
void publishParsed(const ParsedPacket& p, int rssi, float snr) {
  char topic[128];
  char msg[512];
  unsigned long gw_ts = (unsigned long)time(nullptr);
  // Prefer collar GPS Unix time; fall back to gateway receive time when ts==0
  unsigned long timestamp = (p.ts > 0) ? (unsigned long)p.ts : gw_ts;

  snprintf(topic, sizeof(topic), MQTT_TOPIC_UPLINK_PARSED, p.dev_id.c_str(), p.msg_type.c_str());

  snprintf(msg, sizeof(msg),
    "{\"proto_ver\":%d,\"msg_type\":\"%s\",\"dev_id\":\"%s\","
    "\"msg_id\":%u,\"timestamp\":%lu,\"lat\":%.6f,\"lon\":%.6f,"
    "\"speed_kmh\":%.1f,\"hdop\":%.1f,\"battery_v\":%.2f,\"act\":\"%s\","
    "\"gs\":\"%s\",\"rssi\":%d,\"snr\":%.1f,\"gw_ts\":%lu}",
    p.proto_ver, p.msg_type.c_str(), p.dev_id.c_str(),
    p.seq, timestamp, p.lat, p.lon,
    p.spd_kmh, p.hdop, p.bat_v, p.act.c_str(),
    DEVICE_ID, rssi, snr, gw_ts);

  DEBUG_PRINTF("[PARSED] Topic: %s (%d chars)\n", topic, strlen(topic));
  DEBUG_PRINTF("[PARSED] Msg: %s (%d chars)\n", msg, strlen(msg));

  bool result = mqtt.publish(topic, msg, false); // QoS 0
  DEBUG_PRINTF("[PARSED] Publish result: %s\n", result ? "SUCCESS" : "FAILED");

  // If dynamic topic fails, try fallback topic
  if (!result) {
    DEBUG_PRINTF("[PARSED] Retrying with fallback topic: %s\n", MQTT_TOPIC_UPLINK_PARSED_FALLBACK);
    result = mqtt.publish(MQTT_TOPIC_UPLINK_PARSED_FALLBACK, msg, false);
    DEBUG_PRINTF("[PARSED] Fallback result: %s\n", result ? "SUCCESS" : "FAILED");
  }
}

// Handle LoRa packet - both RAW and PARSED
void handleLoRaPacket(const String& payload, int rssi, float snr) {
  // Always publish RAW
  publishRaw(payload, rssi, snr);
  
  // Try to parse and publish PARSED
  ParsedPacket parsed = parseCSV(payload);
  if (parsed.valid) {
    // Check for duplicates
    if (isDuplicate(parsed.dev_id, parsed.seq)) {
      DEBUG_PRINTF("[DUP] Duplicate packet from %s seq=%u\n", parsed.dev_id.c_str(), parsed.seq);
      return;
    }
    
    // Add to cache and publish
    addToDupCache(parsed.dev_id, parsed.seq);
    publishParsed(parsed, rssi, snr);
  } else {
    DEBUG_PRINTLN("[PARSE] Failed to parse CSV");
  }
}

void setupLoRa() {
#if !SKIP_LORA
  LoRa.setPins(LORA_SS_PIN, LORA_RST_PIN, LORA_DI0_PIN);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    DEBUG_PRINTLN("[LoRa] init failed. Rebooting in 5s");
    delay(5000);
    ESP.restart();
  }
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  DEBUG_PRINTLN("[LoRa] Ready.");
#else
  DEBUG_PRINTLN("[LoRa] Skipped (SKIP_LORA=1)");
#endif
}

void setup() {
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);

  Serial.begin(SERIAL_BAUD);
  DEBUG_PRINTLN("\n[Boot] Groundstation starting…");

  // FS for certs
  if (!LittleFS.begin()) {
    DEBUG_PRINTLN("[FS] LittleFS mount failed. Reformatting...");
    LittleFS.format();
    if (!LittleFS.begin()) {
      DEBUG_PRINTLN("[FS] Mount failed after format.");
      while (true) { delay(1000); }
    }
  }

  // Load certs
  if (!loadFileToString(AWS_CERT_CA, g_ca) ||
      !loadFileToString(AWS_CERT_CRT, g_crt) ||
      !loadFileToString(AWS_CERT_PRIVATE, g_key)) {
    DEBUG_PRINTLN("[FS] Failed to read certs from /data. Did you upload with `pio run -t uploadfs`?");
    while (true) { delay(1000); }
  }

#if defined(ESP8266)
  g_ca_list.reset(new BearSSL::X509List(g_ca.c_str()));
  g_crt_list.reset(new BearSSL::X509List(g_crt.c_str()));
  g_private_key.reset(new BearSSL::PrivateKey(g_key.c_str()));
  net.setTrustAnchors(g_ca_list.get());
  net.setClientRSACert(g_crt_list.get(), g_private_key.get());
#else
  net.setCACert(g_ca.c_str());
  net.setCertificate(g_crt.c_str());
  net.setPrivateKey(g_key.c_str());
#endif

  connectWiFi();
  syncTime();
  connectMQTT();
  setupLoRa();
  sendStatus("online");

  DEBUG_PRINTLN("[Boot] Done.");
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) connectWiFi();
  if (!mqtt.connected()) connectMQTT();
  mqtt.loop();

  // Heartbeat
  if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
    sendStatus("alive");
    lastHeartbeat = millis();
  }

#if !SKIP_LORA
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String payload;
    while (LoRa.available()) payload += (char)LoRa.read();
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    handleLoRaPacket(payload, rssi, snr);
  }
#endif

#if EMU_COLLARS
  // Emulate two collars by periodically generating compact CSV lines
  // Format matches device uplink expectations (opaque to GS, forwarded as raw)
  if (millis() - lastEmuTx > 5000) {
    lastEmuTx = millis();
    emuSeq++;

    // Collar A
    String a = String("1,POS,COLLAR-A,") + emuSeq + "," + (unsigned long)time(nullptr) +
               ",-33.123456,-64.123456,3.2,0.9,3.95,WALK";
    handleLoRaPacket(a, -72, 7.5);

    // Collar B
    String b = String("1,POS,COLLAR-B,") + emuSeq + "," + (unsigned long)time(nullptr) +
               ",-33.654321,-64.654321,0.4,1.5,3.88,REST";
    handleLoRaPacket(b, -86, 5.0);
  }
#endif
}