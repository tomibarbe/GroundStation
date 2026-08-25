#pragma once
#include <Arduino.h>
#include <time.h>
#include <TinyGPS++.h>

#define LORA_FREQ   433E6
#define LORA_NSS    5
#define LORA_RST    14
#define LORA_DIO0   2
#define SYNC_WORD   0x12  // change from default to avoid neighbors

// GPS time conversion to Unix timestamp
inline time_t gpsTimeToUnixTimestamp(TinyGPSPlus& gps) {
  if (!gps.time.isValid() || !gps.date.isValid()) {
    return 0;
  }
  
  struct tm timeinfo;
  timeinfo.tm_year = gps.date.year() - 1900;  // tm_year is years since 1900
  timeinfo.tm_mon = gps.date.month() - 1;     // tm_mon is 0-11
  timeinfo.tm_mday = gps.date.day();
  timeinfo.tm_hour = gps.time.hour();
  timeinfo.tm_min = gps.time.minute();
  timeinfo.tm_sec = gps.time.second();
  timeinfo.tm_isdst = 0;  // No daylight saving time
  
  return mktime(&timeinfo);
}

// Sync ESP32 RTC with GPS time
inline void syncRTCWithGPS(TinyGPSPlus& gps) {
  time_t gps_time = gpsTimeToUnixTimestamp(gps);
  if (gps_time > 0) {
    struct timeval tv = { .tv_sec = gps_time };
    settimeofday(&tv, NULL);
  }
}

// Get current timestamp - GPS synced or fallback to millis
inline unsigned long now() {
  time_t current_time = time(nullptr);
  // If RTC has valid time (after year 2020), use it
  if (current_time > 1577836800) { // Jan 1, 2020 timestamp
    return (unsigned long)current_time;
  }
  // Fallback to millis if no valid time set
  return millis() / 1000;
}

// very compact CSV: v,t,id,seq,ts,lat,lon,spd,hdop,bat,act
inline String buildPOS(const char* id, uint16_t seq,
                       double lat, double lon,
                       float spd_kmh, float hdop, float bat_v,
                       const char* act) {
  unsigned long ts = now();
  String s = "1,POS,";
  s += id; s += ','; s += seq; s += ','; s += ts; s += ',';
  s += String(lat, 6); s += ','; s += String(lon, 6); s += ',';
  s += String(spd_kmh, 1); s += ','; s += String(hdop, 1); s += ',';
  s += String(bat_v, 2); s += ','; s += act;
  return s;
}

struct Parsed {
  String type; String id;
  uint16_t seq; unsigned long ts;
  double lat, lon; float spd, hdop, bat; String act;
};
inline bool parseCSV(const String& in, Parsed& p) {
  // very forgiving split
  int idx = 0; int start = 0; int field = 0;
  String f[12];
  while ((idx = in.indexOf(',', start)) >= 0 && field < 11) {
    f[field++] = in.substring(start, idx); start = idx + 1;
  }
  f[field++] = in.substring(start);
  if (field < 10) return false;          // minimal fields present?
  // f0=v, f1=t, f2=id, f3=seq, f4=ts, f5=lat, f6=lon, f7=spd, f8=hdop, f9=bat, f10=act
  p.type = f[1]; p.id = f[2];
  p.seq = (uint16_t)f[3].toInt(); p.ts = (unsigned long)f[4].toInt();
  p.lat = f[5].toDouble(); p.lon = f[6].toDouble();
  p.spd = f[7].toFloat(); p.hdop = f[8].toFloat(); p.bat = f[9].toFloat();
  p.act = (field > 10) ? f[10] : "NONE";
  return true;
}