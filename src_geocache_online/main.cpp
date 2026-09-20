#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <ArduinoJson.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <esp_wps.h>
#include <math.h>
#include "okapi_key.h"

static constexpr uint16_t COLOR_DARKGREY = 0x7BEF;

// ---------------- T-Embed CC1101 hardware ----------------
static constexpr int PIN_SPI_SCK   = 11;
static constexpr int PIN_SPI_MISO  = 10;
static constexpr int PIN_SPI_MOSI  = 9;
static constexpr int PIN_TFT_CS    = 41;
static constexpr int PIN_TFT_DC    = 16;
static constexpr int PIN_TFT_BL    = 21;
static constexpr int PIN_CC1101_CS = 12;
static constexpr int PIN_SD_CS     = 13;
static constexpr int PIN_PWR_EN    = 15;

static constexpr int PIN_ENC_A     = 4;
static constexpr int PIN_ENC_B     = 5;
static constexpr int PIN_ENC_KEY   = 0;
static constexpr int PIN_BACK      = 6;

static constexpr int PIN_GPS_RX    = 44;
static constexpr int PIN_GPS_TX    = 43;
static constexpr uint32_t GPS_BAUD = 115200;

// ---------------- App settings ----------------
static constexpr uint32_t GPS_FIX_MAX_AGE_MS = 6000;
static constexpr uint32_t DRAW_INTERVAL_MS = 500;
static constexpr uint32_t CENTER_LONG_MS = 1300;
static constexpr uint32_t BACK_LONG_MS = 2500;
static constexpr int MAX_CACHES = 20;
static constexpr double SEARCH_RADIUS_KM = 25.0;

static const char *OKAPI_BASE = "https://www.opencaching.de/okapi/";
static const char *OKAPI_KEY = OKAPI_CONSUMER_KEY;

Adafruit_ST7789 tft(&SPI, PIN_TFT_CS, PIN_TFT_DC, -1);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
Preferences prefs;

struct CacheEntry {
  String code;
  String name;
  String type;
  String size;
  String hint;
  String attribution;
  double lat = 0.0;
  double lon = 0.0;
  float difficulty = 0.0f;
  float terrain = 0.0f;
};

CacheEntry caches[MAX_CACHES];
int cacheCount = 0;
int selectedCache = 0;

enum ScreenMode : uint8_t {
  SCREEN_LIST = 0,
  SCREEN_NAV = 1,
  SCREEN_DETAIL = 2,
  SCREEN_GPS = 3,
  SCREEN_INFO = 4
};

ScreenMode screenMode = SCREEN_LIST;
bool screenDirty = true;
uint32_t lastDrawMs = 0;
int lastEncA = HIGH;
bool fetching = false;
bool wpsRunning = false;
bool autoFetchTried = false;
uint32_t bootMs = 0;
String statusText = "Start";
uint32_t statusUntil = 0;

struct ButtonTracker {
  int pin;
  bool last = HIGH;
  uint32_t downSince = 0;
  bool longHandled = false;
};

ButtonTracker centerBtn;
ButtonTracker backBtn;

static void setStatus(const String &s, uint32_t ms = 4000) {
  statusText = s;
  statusUntil = millis() + ms;
  screenDirty = true;
}

static bool gpsFixValid() {
  return gps.location.isValid() && gps.location.age() <= GPS_FIX_MAX_AGE_MS;
}

static bool keyConfigured() {
  return OKAPI_KEY && strlen(OKAPI_KEY) >= 8;
}

static double normalize360(double deg) {
  while (deg < 0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
  return deg;
}

static const char *cardinal16(double bearing) {
  static const char *dirs[16] = {
    "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
    "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
  };
  int idx = (int)floor((normalize360(bearing) + 11.25) / 22.5) & 15;
  return dirs[idx];
}

static String cleanField(String s, size_t maxLen) {
  s.replace("~", "-");
  s.replace("\n", " ");
  s.replace("\r", " ");
  s.trim();
  if (s.length() > maxLen) s = s.substring(0, maxLen);
  return s;
}

static void saveCaches() {
  prefs.putInt("count", cacheCount);
  prefs.putInt("sel", selectedCache);
  for (int i = 0; i < MAX_CACHES; ++i) {
    String key = "c" + String(i);
    if (i >= cacheCount) {
      prefs.remove(key.c_str());
      continue;
    }
    const CacheEntry &c = caches[i];
    String v;
    v.reserve(420);
    v += cleanField(c.code, 16); v += "~";
    v += cleanField(c.name, 48); v += "~";
    v += String(c.lat, 7); v += "~";
    v += String(c.lon, 7); v += "~";
    v += cleanField(c.type, 24); v += "~";
    v += cleanField(c.size, 16); v += "~";
    v += String(c.difficulty, 1); v += "~";
    v += String(c.terrain, 1); v += "~";
    v += cleanField(c.hint, 160); v += "~";
    v += cleanField(c.attribution, 160);
    prefs.putString(key.c_str(), v);
  }
}

static String fieldAt(const String &s, int idx) {
  int start = 0;
  for (int i = 0; i < idx; ++i) {
    int p = s.indexOf('~', start);
    if (p < 0) return "";
    start = p + 1;
  }
  int end = s.indexOf('~', start);
  if (end < 0) end = s.length();
  return s.substring(start, end);
}

static void loadCaches() {
  cacheCount = constrain(prefs.getInt("count", 0), 0, MAX_CACHES);
  for (int i = 0; i < cacheCount; ++i) {
    String v = prefs.getString(("c" + String(i)).c_str(), "");
    caches[i].code = fieldAt(v, 0);
    caches[i].name = fieldAt(v, 1);
    caches[i].lat = fieldAt(v, 2).toDouble();
    caches[i].lon = fieldAt(v, 3).toDouble();
    caches[i].type = fieldAt(v, 4);
    caches[i].size = fieldAt(v, 5);
    caches[i].difficulty = fieldAt(v, 6).toFloat();
    caches[i].terrain = fieldAt(v, 7).toFloat();
    caches[i].hint = fieldAt(v, 8);
    caches[i].attribution = fieldAt(v, 9);
  }
  selectedCache = prefs.getInt("sel", 0);
  if (cacheCount == 0) selectedCache = 0;
  else selectedCache = constrain(selectedCache, 0, cacheCount - 1);
}

static void drawHeader(const char *title) {
  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(7, 5);
  tft.print(title);

  tft.setTextSize(1);
  tft.setCursor(230, 7);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("WLAN OK");
  } else {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("WLAN --");
  }
}

static void drawFooter() {
  tft.fillRect(0, 154, 320, 16, ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setCursor(5, 158);
  if (millis() < statusUntil) {
    tft.setTextColor(ST77XX_YELLOW);
    String s = statusText;
    if (s.length() > 50) s = s.substring(0, 50);
    tft.print(s);
  } else {
    tft.setTextColor(COLOR_DARKGREY);
    tft.print("Lang Druck: Suche/WPS   Seite 2.5s: Launcher");
  }
}

static void drawWrapped(String text, int16_t x, int16_t y, int charsPerLine, int maxLines, uint16_t color) {
  text.replace("\n", " ");
  text.replace("\r", " ");
  tft.setTextSize(1);
  tft.setTextColor(color);

  int pos = 0;
  for (int line = 0; line < maxLines && pos < (int)text.length(); ++line) {
    int take = min(charsPerLine, (int)text.length() - pos);
    int end = pos + take;
    if (end < (int)text.length()) {
      int space = text.lastIndexOf(' ', end);
      if (space > pos + charsPerLine / 2) end = space;
    }
    String part = text.substring(pos, end);
    part.trim();
    tft.setCursor(x, y + line * 12);
    tft.print(part);
    pos = end;
    while (pos < (int)text.length() && text[pos] == ' ') ++pos;
  }
}

static double cacheDistance(const CacheEntry &c) {
  if (!gpsFixValid()) return -1.0;
  return TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), c.lat, c.lon);
}

static void drawListScreen() {
  drawHeader("CACHES");

  if (fetching) {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(15, 68);
    tft.print("Suche in Umgebung...");
    drawFooter();
    return;
  }

  if (cacheCount == 0) {
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(12, 42);
    tft.print("Keine Caches geladen");

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(12, 78);
    if (!keyConfigured()) {
      tft.print("OKAPI Consumer Key fehlt.");
      tft.setCursor(12, 96);
      tft.print("In GitHub Secret eintragen.");
    } else if (!gpsFixValid()) {
      tft.print("Warte auf GPS-Fix...");
    } else if (WiFi.status() != WL_CONNECTED) {
      tft.print("Lang druecken -> WPS starten");
      tft.setCursor(12, 96);
      tft.print("oder gespeichertes WLAN nutzen");
    } else {
      tft.print("Lang druecken -> Caches suchen");
    }
    drawFooter();
    return;
  }

  int first = selectedCache - 2;
  if (first < 0) first = 0;
  if (first > cacheCount - 5) first = max(0, cacheCount - 5);

  tft.setTextSize(1);
  for (int row = 0; row < 5; ++row) {
    int i = first + row;
    if (i >= cacheCount) break;
    int y = 31 + row * 24;
    bool sel = i == selectedCache;
    if (sel) {
      tft.fillRect(3, y - 3, 314, 21, ST77XX_BLUE);
      tft.setTextColor(ST77XX_WHITE);
    } else {
      tft.setTextColor(ST77XX_WHITE);
    }

    String n = caches[i].name;
    if (n.length() > 29) n = n.substring(0, 29);
    tft.setCursor(7, y);
    tft.printf("%c %s", sel ? '>' : ' ', n.c_str());

    double d = cacheDistance(caches[i]);
    tft.setCursor(238, y);
    if (d >= 0) {
      if (d < 1000) tft.printf("%.0fm", d);
      else tft.printf("%.1fkm", d / 1000.0);
    } else {
      tft.print("--");
    }

    tft.setCursor(17, y + 11);
    tft.setTextColor(sel ? ST77XX_CYAN : COLOR_DARKGREY);
    tft.printf("%s  D%.1f/T%.1f", caches[i].type.c_str(), caches[i].difficulty, caches[i].terrain);
  }
  drawFooter();
}

static void drawArrow(int16_t cx, int16_t cy, int16_t length, double angleDeg, uint16_t color) {
  const double a = angleDeg * DEG_TO_RAD;
  const int16_t tx = cx + (int16_t)lround(sin(a) * length);
  const int16_t ty = cy - (int16_t)lround(cos(a) * length);
  tft.drawLine(cx, cy, tx, ty, color);
  tft.drawLine(cx + 1, cy, tx + 1, ty, color);
  const double left = (angleDeg + 150.0) * DEG_TO_RAD;
  const double right = (angleDeg - 150.0) * DEG_TO_RAD;
  const int16_t hx1 = tx + (int16_t)lround(sin(left) * 15.0);
  const int16_t hy1 = ty - (int16_t)lround(cos(left) * 15.0);
  const int16_t hx2 = tx + (int16_t)lround(sin(right) * 15.0);
  const int16_t hy2 = ty - (int16_t)lround(cos(right) * 15.0);
  tft.fillTriangle(tx, ty, hx1, hy1, hx2, hy2, color);
  tft.fillCircle(cx, cy, 3, color);
}

static void drawNavDynamic() {
  if (cacheCount == 0) return;

  CacheEntry &c = caches[selectedCache];

  // Only repaint the two dynamic navigation areas, never the full display.
  tft.fillRect(5, 31, 110, 119, ST77XX_BLACK);
  tft.fillRect(116, 42, 204, 101, ST77XX_BLACK);

  if (!gpsFixValid()) {
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(12, 65);
    tft.print("Warte auf GPS-Fix");
    return;
  }

  double lat = gps.location.lat();
  double lon = gps.location.lng();
  double distance = TinyGPSPlus::distanceBetween(lat, lon, c.lat, c.lon);
  double bearing = TinyGPSPlus::courseTo(lat, lon, c.lat, c.lon);
  bool moving = gps.speed.isValid() && gps.course.isValid() && gps.speed.kmph() >= 2.0;
  double arrowAngle = moving ? normalize360(bearing - gps.course.deg()) : normalize360(bearing);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(25, 40);
  tft.print(moving ? "REL KURS" : "N OBEN");

  drawArrow(62, 101, 40, arrowAngle, distance <= 15.0 ? ST77XX_GREEN : ST77XX_CYAN);

  tft.setTextSize(3);
  tft.setTextColor(distance <= 15.0 ? ST77XX_GREEN : ST77XX_WHITE);
  tft.setCursor(120, 55);
  if (distance < 1000) tft.printf("%.0f m", distance);
  else if (distance < 10000) tft.printf("%.2f km", distance / 1000.0);
  else tft.printf("%.1f km", distance / 1000.0);

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(120, 91);
  tft.printf("%03.0f deg %s", bearing, cardinal16(bearing));

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(120, 118);
  tft.printf("%s  D%.1f/T%.1f", c.code.c_str(), c.difficulty, c.terrain);
}

static void drawNavScreen() {
  drawHeader("NAV");
  if (cacheCount == 0) {
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(12, 60);
    tft.print("Kein Ziel geladen");
    drawFooter();
    return;
  }

  CacheEntry &c = caches[selectedCache];
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(52, 7);
  String name = c.name;
  if (name.length() > 25) name = name.substring(0, 25);
  tft.print(name);

  drawNavDynamic();
  drawFooter();
}

static void drawDetailScreen() {
  drawHeader("DETAIL");
  if (cacheCount == 0) {
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(12, 60);
    tft.print("Kein Cache");
    drawFooter();
    return;
  }

  CacheEntry &c = caches[selectedCache];
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, 31);
  String n = c.name;
  if (n.length() > 46) n = n.substring(0, 46);
  tft.print(n);

  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(8, 48);
  tft.printf("%s | %s | %s | D%.1f/T%.1f", c.code.c_str(), c.type.c_str(), c.size.c_str(), c.difficulty, c.terrain);

  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(8, 68);
  tft.print("Hint:");
  drawWrapped(c.hint.length() ? c.hint : "(kein Hint hinterlegt)", 8, 81, 50, 4, ST77XX_WHITE);

  tft.setTextColor(COLOR_DARKGREY);
  tft.setCursor(8, 137);
  tft.print("Daten: Opencaching.de");
  drawFooter();
}

static void drawGpsDynamic() {
  // Small body refresh only. Header and footer stay untouched.
  tft.fillRect(5, 28, 315, 122, ST77XX_BLACK);

  tft.setTextSize(1);
  tft.setTextColor(gpsFixValid() ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(10, 32);
  tft.print(gpsFixValid() ? "FIX OK" : "KEIN FRISCHER FIX");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 53);
  if (gps.location.isValid()) {
    tft.printf("%.6f", gps.location.lat());
    tft.setCursor(10, 77);
    tft.printf("%.6f", gps.location.lng());
  } else {
    tft.print("Position --");
  }

  tft.setTextSize(1);
  tft.setCursor(10, 109);
  tft.printf("Sat: %lu", (unsigned long)(gps.satellites.isValid() ? gps.satellites.value() : 0));
  tft.setCursor(90, 109);
  if (gps.hdop.isValid()) tft.printf("HDOP %.1f", gps.hdop.hdop());
  tft.setCursor(180, 109);
  if (gps.altitude.isValid()) tft.printf("%.0fm", gps.altitude.meters());

  tft.setCursor(10, 132);
  if (WiFi.status() == WL_CONNECTED) {
    tft.setTextColor(ST77XX_GREEN);
    tft.print("WLAN: ");
    tft.print(WiFi.SSID());
  } else {
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("WLAN nicht verbunden");
  }
}

static void drawGpsScreen() {
  drawHeader("GPS");
  drawGpsDynamic();
  drawFooter();
}

static void drawInfoScreen() {
  drawHeader("ONLINE");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(8, 32);
  tft.printf("Caches gespeichert: %d/%d", cacheCount, MAX_CACHES);
  tft.setCursor(8, 50);
  tft.printf("Suchradius: %.0f km", SEARCH_RADIUS_KM);
  tft.setCursor(8, 68);
  tft.print("Quelle: Opencaching.de / OKAPI");

  tft.setCursor(8, 91);
  tft.setTextColor(keyConfigured() ? ST77XX_GREEN : ST77XX_RED);
  tft.print(keyConfigured() ? "OKAPI-Key: vorhanden" : "OKAPI-Key: FEHLT");

  tft.setCursor(8, 112);
  tft.setTextColor(ST77XX_WHITE);
  if (WiFi.status() == WL_CONNECTED) {
    tft.print("WLAN: ");
    tft.print(WiFi.SSID());
  } else if (wpsRunning) {
    tft.print("WPS laeuft - Routertaste druecken");
  } else {
    tft.print("Langdruck startet WPS");
  }

  tft.setCursor(8, 136);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Encoder kurz = Ansicht wechseln");
  drawFooter();
}

static void refreshListDistances() {
  if (fetching || cacheCount == 0) return;

  int first = selectedCache - 2;
  if (first < 0) first = 0;
  if (first > cacheCount - 5) first = max(0, cacheCount - 5);

  tft.setTextSize(1);
  for (int row = 0; row < 5; ++row) {
    int i = first + row;
    if (i >= cacheCount) break;
    int y = 31 + row * 24;
    bool sel = i == selectedCache;
    tft.fillRect(236, y - 1, 80, 10, sel ? ST77XX_BLUE : ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE);
    tft.setCursor(238, y);
    double d = cacheDistance(caches[i]);
    if (d >= 0) {
      if (d < 1000) tft.printf("%.0fm", d);
      else tft.printf("%.1fkm", d / 1000.0);
    } else {
      tft.print("--");
    }
  }
}

static void drawScreen() {
  // A full clear happens only for a real screen/content change.
  tft.fillScreen(ST77XX_BLACK);
  switch (screenMode) {
    case SCREEN_NAV: drawNavScreen(); break;
    case SCREEN_DETAIL: drawDetailScreen(); break;
    case SCREEN_GPS: drawGpsScreen(); break;
    case SCREEN_INFO: drawInfoScreen(); break;
    case SCREEN_LIST:
    default: drawListScreen(); break;
  }
}

static void refreshDynamic() {
  switch (screenMode) {
    case SCREEN_NAV:
      drawNavDynamic();
      break;
    case SCREEN_GPS:
      drawGpsDynamic();
      break;
    case SCREEN_LIST:
      refreshListDistances();
      break;
    default:
      break;
  }
  drawFooter();
}

static bool httpGetJson(const String &url, String &payload) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  http.setTimeout(12000);
  if (!http.begin(client, url)) return false;
  int code = http.GET();
  if (code != HTTP_CODE_OK) {
    Serial.printf("HTTP %d\n", code);
    payload = http.getString();
    http.end();
    return false;
  }
  payload = http.getString();
  http.end();
  return true;
}

static bool fetchNearbyCaches() {
  if (fetching) return false;
  if (!keyConfigured()) {
    setStatus("OKAPI-Key fehlt", 6000);
    return false;
  }
  if (WiFi.status() != WL_CONNECTED) {
    setStatus("Kein WLAN - Langdruck startet WPS", 6000);
    return false;
  }
  if (!gpsFixValid()) {
    setStatus("Noch kein GPS-Fix", 5000);
    return false;
  }

  fetching = true;
  screenMode = SCREEN_LIST;
  screenDirty = false;
  lastDrawMs = millis();
  drawScreen();

  String center = String(gps.location.lat(), 6) + "%7C" + String(gps.location.lng(), 6);
  String url1 = String(OKAPI_BASE) +
    "services/caches/search/nearest?center=" + center +
    "&radius=" + String(SEARCH_RADIUS_KM, 0) +
    "&limit=" + String(MAX_CACHES) +
    "&status=Available&consumer_key=" + String(OKAPI_KEY);

  String payload;
  if (!httpGetJson(url1, payload)) {
    fetching = false;
    setStatus("OKAPI Suche fehlgeschlagen", 6000);
    return false;
  }

  JsonDocument searchDoc;
  DeserializationError err = deserializeJson(searchDoc, payload);
  if (err) {
    fetching = false;
    setStatus("Antwort nicht lesbar", 6000);
    return false;
  }

  JsonArray results = searchDoc["results"].as<JsonArray>();
  if (results.size() == 0) {
    cacheCount = 0;
    saveCaches();
    fetching = false;
    setStatus("Keine OC-Caches im Radius", 6000);
    return true;
  }

  String codes;
  int wanted = min((int)results.size(), MAX_CACHES);
  for (int i = 0; i < wanted; ++i) {
    const char *code = results[i] | "";
    if (i) codes += "%7C";
    codes += code;
  }

  String fields = "code%7Cname%7Clocation%7Ctype%7Csize2%7Cdifficulty%7Cterrain%7Chint2%7Cattribution_note";
  String url2 = String(OKAPI_BASE) +
    "services/caches/geocaches?cache_codes=" + codes +
    "&fields=" + fields +
    "&langpref=de%7Cen&attribution_append=none&consumer_key=" + String(OKAPI_KEY);

  payload = "";
  if (!httpGetJson(url2, payload)) {
    fetching = false;
    setStatus("Cache-Details fehlgeschlagen", 6000);
    return false;
  }

  JsonDocument detailsDoc;
  err = deserializeJson(detailsDoc, payload);
  if (err) {
    fetching = false;
    setStatus("Details nicht lesbar", 6000);
    return false;
  }

  cacheCount = 0;
  for (int i = 0; i < wanted && cacheCount < MAX_CACHES; ++i) {
    String code = results[i].as<String>();
    JsonVariant v = detailsDoc[code];
    if (v.isNull()) continue;
    JsonObject o = v.as<JsonObject>();

    String loc = o["location"] | "";
    int sep = loc.indexOf('|');
    if (sep < 1) continue;

    CacheEntry &c = caches[cacheCount];
    c.code = code;
    c.name = cleanField(o["name"].as<String>(), 48);
    c.lat = loc.substring(0, sep).toDouble();
    c.lon = loc.substring(sep + 1).toDouble();
    c.type = cleanField(o["type"].as<String>(), 24);
    c.size = cleanField(o["size2"].as<String>(), 16);
    c.difficulty = o["difficulty"] | 0.0f;
    c.terrain = o["terrain"] | 0.0f;
    c.hint = cleanField(o["hint2"].as<String>(), 160);
    c.attribution = cleanField(o["attribution_note"].as<String>(), 160);
    ++cacheCount;
  }

  selectedCache = 0;
  saveCaches();
  fetching = false;
  setStatus(String(cacheCount) + " Caches geladen", 5000);
  return true;
}

static void stopWps() {
  esp_wifi_wps_disable();
  wpsRunning = false;
  screenDirty = true;
}

static void startWps() {
  if (wpsRunning) return;
  WiFi.mode(WIFI_MODE_STA);

  esp_wps_config_t config = WPS_CONFIG_INIT_DEFAULT(WPS_TYPE_PBC);
  esp_err_t e = esp_wifi_wps_enable(&config);
  if (e != ESP_OK) {
    setStatus("WPS konnte nicht starten", 6000);
    return;
  }
  e = esp_wifi_wps_start(0);
  if (e != ESP_OK) {
    esp_wifi_wps_disable();
    setStatus("WPS Startfehler", 6000);
    return;
  }
  wpsRunning = true;
  setStatus("WPS: Routertaste druecken", 120000);
}

static void WiFiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      wpsRunning = false;
      setStatus("WLAN verbunden: " + WiFi.SSID(), 5000);
      break;
    case ARDUINO_EVENT_WPS_ER_SUCCESS:
      stopWps();
      delay(20);
      WiFi.begin();
      setStatus("WPS erfolgreich - verbinde...", 8000);
      break;
    case ARDUINO_EVENT_WPS_ER_FAILED:
    case ARDUINO_EVENT_WPS_ER_TIMEOUT:
      stopWps();
      setStatus("WPS fehlgeschlagen", 6000);
      break;
    default:
      break;
  }
}

static void returnToLauncher() {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(20, 70);
  tft.print("Zurueck zum Launcher...");
  delay(150);
  esp_sleep_enable_timer_wakeup(1000000ULL);
  delay(50);
  esp_deep_sleep_start();
}

static void nextScreen(int delta) {
  int s = (int)screenMode + delta;
  while (s < 0) s += 5;
  while (s >= 5) s -= 5;
  screenMode = (ScreenMode)s;
  screenDirty = true;
}

static void serviceEncoder() {
  int a = digitalRead(PIN_ENC_A);
  if (a != lastEncA && a == LOW) {
    int b = digitalRead(PIN_ENC_B);
    int dir = (b == HIGH) ? 1 : -1;
    if (screenMode == SCREEN_LIST && cacheCount > 0 && !fetching) {
      selectedCache += dir;
      if (selectedCache < 0) selectedCache = cacheCount - 1;
      if (selectedCache >= cacheCount) selectedCache = 0;
      prefs.putInt("sel", selectedCache);
      screenDirty = true;
    }
  }
  lastEncA = a;
}

static void handleCenterLong() {
  if (!keyConfigured()) {
    screenMode = SCREEN_INFO;
    setStatus("GitHub Secret OKAPI_CONSUMER_KEY fehlt", 8000);
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    screenMode = SCREEN_INFO;
    startWps();
    return;
  }
  fetchNearbyCaches();
}

static void serviceButton(ButtonTracker &btn, uint32_t longMs, bool isCenter) {
  bool now = digitalRead(btn.pin);

  if (btn.last == HIGH && now == LOW) {
    btn.downSince = millis();
    btn.longHandled = false;
  }

  if (now == LOW && !btn.longHandled && btn.downSince != 0 &&
      (uint32_t)(millis() - btn.downSince) >= longMs) {
    btn.longHandled = true;
    if (isCenter) handleCenterLong();
    else returnToLauncher();
  }

  if (btn.last == LOW && now == HIGH) {
    if (!btn.longHandled) {
      if (isCenter) nextScreen(+1);
      else nextScreen(-1);
    }
    btn.downSince = 0;
  }
  btn.last = now;
}

static void initHardware() {
  centerBtn.pin = PIN_ENC_KEY;
  backBtn.pin = PIN_BACK;

  pinMode(PIN_PWR_EN, OUTPUT);
  digitalWrite(PIN_PWR_EN, HIGH);
  delay(10);

  pinMode(PIN_CC1101_CS, OUTPUT);
  digitalWrite(PIN_CC1101_CS, HIGH);
  pinMode(PIN_SD_CS, OUTPUT);
  digitalWrite(PIN_SD_CS, HIGH);
  pinMode(PIN_TFT_CS, OUTPUT);
  digitalWrite(PIN_TFT_CS, HIGH);
  pinMode(PIN_TFT_BL, OUTPUT);
  digitalWrite(PIN_TFT_BL, HIGH);

  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  pinMode(PIN_ENC_KEY, INPUT_PULLUP);
  pinMode(PIN_BACK, INPUT_PULLUP);

  lastEncA = digitalRead(PIN_ENC_A);
  centerBtn.last = digitalRead(PIN_ENC_KEY);
  backBtn.last = digitalRead(PIN_BACK);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);
  tft.init(170, 320);
  tft.setRotation(1);
  tft.setTextWrap(false);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  initHardware();

  prefs.begin("geocache2", false);
  loadCaches();

  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin();

  bootMs = millis();

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(18, 48);
  tft.print("Geocache Online");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(18, 82);
  tft.print("GPS + WLAN + Opencaching.de");
  tft.setCursor(18, 103);
  tft.print(cacheCount ? "Offline-Caches geladen" : "Warte auf GPS/WLAN...");
  delay(900);
  screenDirty = true;
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode((char)gpsSerial.read());
  }

  serviceEncoder();
  serviceButton(centerBtn, CENTER_LONG_MS, true);
  serviceButton(backBtn, BACK_LONG_MS, false);

  if (!autoFetchTried && millis() - bootMs > 2500 &&
      keyConfigured() && gpsFixValid() && WiFi.status() == WL_CONNECTED) {
    autoFetchTried = true;
    fetchNearbyCaches();
  }

  uint32_t now = millis();
  if (screenDirty) {
    screenDirty = false;
    lastDrawMs = now;
    drawScreen();
  } else if ((uint32_t)(now - lastDrawMs) >= DRAW_INTERVAL_MS) {
    lastDrawMs = now;
    refreshDynamic();
  }

  delay(5);
}
