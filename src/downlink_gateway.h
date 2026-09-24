#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>
#include <FS.h>
#include <LittleFS.h>
#include <LoRa.h>
#include <PubSubClient.h>
#include <time.h>
#include "../contracts/downlink_v2.h"

namespace GatewayDownlink {
using namespace GeoDownlink;
struct DeviceKey { const char* id; uint8_t key[KEY_SIZE]; };
#if __has_include("../include/lora_downlink_keys.h")
#include "../include/lora_downlink_keys.h"
#else
#define LORA_DOWNLINK_KEYS { { "", {0} } }
#endif
static const DeviceKey keys[] = LORA_DOWNLINK_KEYS;
inline const uint8_t* keyFor(const char* id) {
  for (const auto& row : keys)
    if (!strcmp(row.id, id) && keyPresent(row.key)) return row.key;
  return nullptr;
}

constexpr uint32_t QUEUE_MAGIC = 0x32514C47; // GLQ2
constexpr size_t QUEUE_CAPACITY = 128;
struct Pending {
  char dev_id[9];
  uint32_t cmd_id;
  uint32_t expires_at;
  uint8_t frame_len;
  uint8_t frame[MAX_FRAME_SIZE];
  uint8_t acked;
  uint8_t ack_body[7];
};
struct Queue {
  uint32_t magic;
  uint32_t generation;
  uint16_t count;
  Pending entries[QUEUE_CAPACITY];
  uint32_t checksum;
};
static Queue queue;
static bool queueHealthy = false;

inline uint32_t checksum(const Queue& q) {
  const uint8_t* b = reinterpret_cast<const uint8_t*>(&q);
  uint32_t h = 2166136261u;
  for (size_t i = 0; i < offsetof(Queue, checksum); ++i) h = (h ^ b[i]) * 16777619u;
  return h;
}
inline bool readSlot(const char* path, Queue& out) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  bool ok = f.size() == sizeof(out) && f.readBytes(reinterpret_cast<char*>(&out), sizeof(out)) == sizeof(out);
  f.close();
  return ok && out.magic == QUEUE_MAGIC && out.count <= QUEUE_CAPACITY &&
         out.checksum == checksum(out);
}
inline bool load() {
  static Queue candidate;
  memset(&queue, 0, sizeof(queue));
  bool found = false;
  if (readSlot("/dlq_a", candidate)) { queue = candidate; found = true; }
  if (readSlot("/dlq_b", candidate) && (!found || candidate.generation > queue.generation)) {
    queue = candidate; found = true;
  }
  if (!found) queue.magic = QUEUE_MAGIC;
  queueHealthy = true;
  Serial.printf("[DL] queue restored: %u command(s) generation=%lu\n",
                queue.count, (unsigned long)queue.generation);
  return true;
}
inline bool save() {
  if (!queueHealthy) return false;
  queue.magic = QUEUE_MAGIC;
  ++queue.generation;
  queue.checksum = checksum(queue);
  const char* path = (queue.generation & 1) ? "/dlq_a" : "/dlq_b";
  File f = LittleFS.open(path, "w");
  bool ok = f && f.write(reinterpret_cast<const uint8_t*>(&queue), sizeof(queue)) == sizeof(queue);
  if (f) { f.flush(); f.close(); }
  if (!ok) {
    queueHealthy = false;
    Serial.println("[DL] Queue persistence FAILED; downlink paused");
  }
  return ok;
}
inline void eraseAt(size_t i) {
  if (i >= queue.count) return;
  for (size_t j = i + 1; j < queue.count; ++j) queue.entries[j - 1] = queue.entries[j];
  memset(&queue.entries[--queue.count], 0, sizeof(Pending));
}
inline time_t parseUtc(const char* iso) {
  if (!iso) return 0;
  int year, month, day, hour, minute, second, used = 0;
  if (sscanf(iso, "%4d-%2d-%2dT%2d:%2d:%2dZ%n", &year, &month, &day,
             &hour, &minute, &second, &used) != 6 || !used || iso[used]) return 0;
  if (year < 2021 || month < 1 || month > 12 || day < 1 || day > 31 ||
      hour > 23 || minute > 59 || second > 59) return 0;
  static const int daysInMonth[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  const bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  if (day > daysInMonth[month - 1] + (month == 2 && leap ? 1 : 0)) return 0;
  int y = year - (month <= 2);
  int era = y / 400;
  unsigned yoe = y - era * 400;
  unsigned mp = month + (month > 2 ? -3 : 9);
  unsigned doy = (153 * mp + 2) / 5 + day - 1;
  unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  int64_t days = era * 146097LL + doe - 719468LL;
  int64_t seconds = days * 86400 + hour * 3600 + minute * 60 + second;
  return seconds > 0 && seconds <= UINT32_MAX ? static_cast<time_t>(seconds) : 0;
}
inline bool number(JsonVariantConst v, uint32_t max, uint32_t& out) {
  if (!(v.is<uint32_t>() || v.is<int32_t>() || v.is<uint64_t>())) return false;
  int64_t signedValue = v.as<int64_t>();
  if (signedValue < 0 || static_cast<uint64_t>(signedValue) > max) return false;
  out = static_cast<uint32_t>(signedValue);
  return true;
}
inline bool signedCoord(JsonVariantConst v, int32_t min, int32_t max, int32_t& out) {
  if (!(v.is<int32_t>() || v.is<int64_t>())) return false;
  int64_t value = v.as<int64_t>();
  if (value < min || value > max) return false;
  out = static_cast<int32_t>(value);
  return true;
}
inline bool buildFrame(JsonObjectConst root, const char* id, const uint8_t* key,
                       Frame& f, uint32_t& expiry) {
  if (strcmp(root["msg_type"] | "", "CFG")) return false;
  uint32_t idNumber, group;
  if (!number(root["msg_id"], UINT32_MAX - 1, idNumber) || idNumber == 0) return false;
  time_t exp = parseUtc(root["expires_at"] | "");
  if (exp < 1609459200 || exp > UINT32_MAX) return false;
  expiry = static_cast<uint32_t>(exp);
  JsonObjectConst p = root["payload"].as<JsonObjectConst>();
  if (p.isNull() || p.containsKey("escalate") || !number(p["group_id"], 65535, group)) return false;
  memset(&f, 0, sizeof(f));
  strncpy(f.dev_id, id, 8);
  f.cmd_id = idNumber;
  f.group_id = group;
  const char* kind = p["kind"] | "";
  if (!strcmp(kind, "FENCE")) {
    uint32_t active, version;
    if (!number(p["active_paddock_id"], 65535, active) ||
        !number(p["version"], 65535, version) ||
        ((active == 0) != (version == 0))) return false;
    f.type = FENCE; f.body_len = 4;
    write16(f.body, active); write16(f.body + 2, version);
  } else if (!strcmp(kind, "SHIFT")) {
    uint32_t source, sourceVersion, target, targetVersion;
    if (!number(p["source_paddock_id"], 65535, source) || !source ||
        !number(p["source_version"], 65535, sourceVersion) || !sourceVersion ||
        !number(p["target_paddock_id"], 65535, target) || !target ||
        !number(p["target_version"], 65535, targetVersion) || !targetVersion ||
        (source == target && sourceVersion == targetVersion)) return false;
    f.type = SHIFT; f.body_len = 8;
    write16(f.body, source); write16(f.body + 2, sourceVersion);
    write16(f.body + 4, target); write16(f.body + 6, targetVersion);
  } else if (!strcmp(kind, "PADDOCK")) {
    uint32_t paddock, version, border, shock, grace, n;
    if (!number(p["paddock_id"], 65535, paddock) || !paddock ||
        !number(p["version"], 65535, version) || !version ||
        !number(p["border_m"], 65535, border) ||
        !number(p["max_shock"], 3, shock) ||
        !number(p["transition_grace_s"], 65535, grace) ||
        !number(p["n_vertices"], 12, n) || n < 3) return false;
    JsonArrayConst coords = p["coords"].as<JsonArrayConst>();
    if (coords.isNull() || coords.size() != n * 2) return false;
    f.type = PADDOCK; f.body_len = 10 + n * 8;
    write16(f.body, paddock); write16(f.body + 2, version);
    write16(f.body + 4, border); f.body[6] = shock;
    write16(f.body + 7, grace); f.body[9] = n;
    for (size_t i = 0; i < n; ++i) {
      int32_t lon, lat;
      if (!signedCoord(coords[i * 2], -180000000, 180000000, lon) ||
          !signedCoord(coords[i * 2 + 1], -90000000, 90000000, lat)) return false;
      write32(f.body + 10 + i * 8, static_cast<uint32_t>(lon));
      write32(f.body + 14 + i * 8, static_cast<uint32_t>(lat));
    }
  } else return false;
  uint8_t raw[MAX_FRAME_SIZE]; size_t len;
  return encode(f, key, raw, sizeof(raw), len);
}
inline bool ingest(const char* topic, const uint8_t* payload, size_t length) {
  if (!queueHealthy || strncmp(topic, "geoparcela/downlink/", 20)) return false;
  const char* id = topic + 20;
  if (!validId(id)) return false;
  const uint8_t* key = keyFor(id);
  if (!key) { Serial.printf("[DL] No provisioned key for %s\n", id); return false; }
  StaticJsonDocument<2048> doc;
  if (deserializeJson(doc, payload, length)) return false;
  Frame f{}; uint32_t expiry = 0;
  if (!buildFrame(doc.as<JsonObjectConst>(), id, key, f, expiry)) return false;
  const time_t now = time(nullptr);
  if (now < 1609459200 || expiry <= static_cast<uint32_t>(now)) return false;
  uint8_t raw[MAX_FRAME_SIZE]; size_t len = 0;
  if (!encode(f, key, raw, sizeof(raw), len)) return false;
  for (size_t i = 0; i < queue.count; ++i) {
    Pending& old = queue.entries[i];
    if (!strcmp(old.dev_id, id) && old.cmd_id == f.cmd_id)
      return old.frame_len == len && !memcmp(old.frame, raw, len); // exact MQTT retry
  }
  if (f.type == FENCE && read16(f.body) == 0) {
    for (size_t i = 0; i < queue.count;)
      if (!strcmp(queue.entries[i].dev_id, id) && !queue.entries[i].acked &&
          queue.entries[i].cmd_id < f.cmd_id) eraseAt(i);
      else ++i;
  }
  if (queue.count >= QUEUE_CAPACITY) return false;
  Pending& item = queue.entries[queue.count++];
  memset(&item, 0, sizeof(item));
  strcpy(item.dev_id, id);
  item.cmd_id = f.cmd_id; item.expires_at = expiry;
  item.frame_len = len; memcpy(item.frame, raw, len);
  if (!save()) return false;
  Serial.printf("[DL] queued %s id=%lu type=%u\n", id, (unsigned long)f.cmd_id, f.type);
  return true;
}
inline int nextFor(const char* id) {
  if (!queueHealthy || !keyFor(id)) return -1;
  time_t now = time(nullptr);
  if (now < 1609459200) return -1;
  int selected = -1;
  for (size_t i = 0; i < queue.count; ++i) {
    Pending& p = queue.entries[i];
    if (strcmp(p.dev_id, id) || p.acked || p.expires_at <= static_cast<uint32_t>(now)) continue;
    if (selected < 0 || p.cmd_id < queue.entries[selected].cmd_id) selected = i;
  }
  return selected;
}
inline void pruneExpired() {
  static unsigned long lastPrune = 0;
  if (!queueHealthy || (lastPrune && millis() - lastPrune < 60000)) return;
  lastPrune = millis();
  time_t now = time(nullptr);
  if (now < 1609459200) return;
  bool changed = false;
  for (size_t i = 0; i < queue.count;)
    if (!queue.entries[i].acked && queue.entries[i].expires_at <= static_cast<uint32_t>(now)) {
      Serial.printf("[DL] expired %s id=%lu\n", queue.entries[i].dev_id,
                    (unsigned long)queue.entries[i].cmd_id);
      eraseAt(i);
      changed = true;
    } else ++i;
  if (changed) save();
}
inline void replyToCheckin(const char* id) {
  int index = nextFor(id);
  if (index < 0) return;
  Pending& p = queue.entries[index];
  Serial.printf("[DL] sending to %s id=%lu\n", id, (unsigned long)p.cmd_id);
  delay(80); // let the just-transmitting collar switch the SX1278 into RX
  LoRa.beginPacket();
  LoRa.write(p.frame, p.frame_len);
  LoRa.endPacket();
  LoRa.receive();
  unsigned long started = millis();
  while (millis() - started < 1500) {
    int size = LoRa.parsePacket();
    if (!size) { delay(5); continue; }
    uint8_t raw[MAX_FRAME_SIZE];
    if (size > static_cast<int>(sizeof(raw))) {
      while (LoRa.available()) LoRa.read();
      continue;
    }
    int n = 0;
    while (LoRa.available() && n < size) raw[n++] = LoRa.read();
    Frame ack{};
    if (n != size || !decode(raw, n, keyFor(id), ack) || ack.type != ACK ||
        strcmp(ack.dev_id, id) || ack.cmd_id != p.cmd_id || ack.body_len != 7 ||
        ack.body[0] > UNSUPPORTED || read16(raw + 17) != read16(p.frame + 17)) continue;
    p.acked = 1;
    memcpy(p.ack_body, ack.body, 7);
    if (save()) Serial.printf("[DL] verified ACK id=%lu result=%u\n",
                              (unsigned long)p.cmd_id, ack.body[0]);
    return;
  }
  Serial.printf("[DL] no ACK id=%lu; retry on next check-in\n", (unsigned long)p.cmd_id);
}
inline void publishAcks(PubSubClient& mqtt, const char* stationId) {
  if (!queueHealthy || !mqtt.connected()) return;
  for (size_t i = 0; i < queue.count;) {
    Pending& p = queue.entries[i];
    if (!p.acked) { ++i; continue; }
    char topic[96], body[320];
    uint32_t now = static_cast<uint32_t>(time(nullptr));
    snprintf(topic, sizeof(topic), "geoparcela/uplink/%s/ACK", p.dev_id);
    snprintf(body, sizeof(body),
      "{\"proto_ver\":2,\"msg_type\":\"ACK\",\"dev_id\":\"%s\",\"ack_id\":%lu,"
      "\"success\":%s,\"result\":%u,\"ref_paddock_id\":%u,\"ref_version\":%u,"
      "\"active_paddock_id\":%u,\"timestamp\":%lu,\"gs\":\"%s\",\"gw_ts\":%lu}",
      p.dev_id, (unsigned long)p.cmd_id, p.ack_body[0] == APPLIED ? "true" : "false",
      p.ack_body[0], read16(p.ack_body + 1), read16(p.ack_body + 3),
      read16(p.ack_body + 5), (unsigned long)now, stationId, (unsigned long)now);
    if (!mqtt.publish(topic, body, false)) break;
    Serial.printf("[DL] published verified ACK %s id=%lu\n", p.dev_id, (unsigned long)p.cmd_id);
    eraseAt(i);
    if (!save()) break;
  }
}
}
