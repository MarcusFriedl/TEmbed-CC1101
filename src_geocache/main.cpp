#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <TinyGPSPlus.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>

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
static constexpr uint32_t GPS_FIX_MAX_AGE_MS = 5000;
static constexpr uint32_t DRAW_INTERVAL_MS = 300;
static constexpr uint32_t CENTER_LONG_MS = 1200;
static constexpr uint32_t BACK_LONG_MS = 2500;
static constexpr int MAX_CACHES = 12;

static const char *AP_SSID = "TEmbed-Geocache";
static const char *AP_PASS = "geocache1";

Adafruit_ST7789 tft(&SPI, PIN_TFT_CS, PIN_TFT_DC, -1);
HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
Preferences prefs;
WebServer server(80);

struct CacheEntry {
  String name;
  double lat = 0.0;
  double lon = 0.0;
};

CacheEntry caches[MAX_CACHES];
int cacheCount = 0;
int activeCache = -1;

enum ScreenMode : uint8_t {
  SCREEN_NAV = 0,
  SCREEN_GPS = 1,
  SCREEN_HELP = 2
};

ScreenMode screenMode = SCREEN_NAV;
bool apRunning = false;
bool screenDirty = true;
uint32_t lastDrawMs = 0;

int lastEncA = HIGH;

struct ButtonTracker {
  int pin;
  bool last = HIGH;
  uint32_t downSince = 0;
  bool longHandled = false;
};

ButtonTracker centerBtn;
ButtonTracker backBtn;


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

static bool gpsFixValid() {
  return gps.location.isValid() && gps.location.age() <= GPS_FIX_MAX_AGE_MS;
}

static String sanitizeName(String s) {
  s.trim();
  s.replace("|", "-");
  s.replace("\n", " ");
  s.replace("\r", " ");
  if (s.length() == 0) s = "Cache";
  if (s.length() > 28) s = s.substring(0, 28);
  return s;
}

static String htmlEscape(String s) {
  s.replace("&", "&amp;");
  s.replace("<", "&lt;");
  s.replace(">", "&gt;");
  s.replace("\"", "&quot;");
  return s;
}

static void saveCaches() {
  String blob;
  for (int i = 0; i < cacheCount; ++i) {
    blob += sanitizeName(caches[i].name);
    blob += "|";
    blob += String(caches[i].lat, 7);
    blob += "|";
    blob += String(caches[i].lon, 7);
    blob += "\n";
  }
  prefs.putString("caches", blob);
  prefs.putInt("active", activeCache);
}

static void loadCaches() {
  cacheCount = 0;
  String blob = prefs.getString("caches", "");

  int start = 0;
  while (start < (int)blob.length() && cacheCount < MAX_CACHES) {
    int end = blob.indexOf('\n', start);
    if (end < 0) end = blob.length();

    String line = blob.substring(start, end);
    line.trim();

    if (line.length()) {
      int p1 = line.indexOf('|');
      int p2 = (p1 >= 0) ? line.indexOf('|', p1 + 1) : -1;
      if (p1 > 0 && p2 > p1) {
        String name = line.substring(0, p1);
        double lat = line.substring(p1 + 1, p2).toDouble();
        double lon = line.substring(p2 + 1).toDouble();

        if (lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0) {
          caches[cacheCount].name = name;
          caches[cacheCount].lat = lat;
          caches[cacheCount].lon = lon;
          ++cacheCount;
        }
      }
    }

    start = end + 1;
  }

  activeCache = prefs.getInt("active", cacheCount ? 0 : -1);
  if (cacheCount == 0) {
    activeCache = -1;
  } else if (activeCache < 0 || activeCache >= cacheCount) {
    activeCache = 0;
  }
}

static void selectCache(int index) {
  if (cacheCount <= 0) {
    activeCache = -1;
    return;
  }

  while (index < 0) index += cacheCount;
  while (index >= cacheCount) index -= cacheCount;

  activeCache = index;
  prefs.putInt("active", activeCache);
  screenDirty = true;
}

static void addCache(const String &name, double lat, double lon) {
  if (cacheCount >= MAX_CACHES) return;
  caches[cacheCount].name = sanitizeName(name);
  caches[cacheCount].lat = lat;
  caches[cacheCount].lon = lon;
  activeCache = cacheCount;
  ++cacheCount;
  saveCaches();
  screenDirty = true;
}

static void deleteCache(int index) {
  if (index < 0 || index >= cacheCount) return;
  for (int i = index; i < cacheCount - 1; ++i) {
    caches[i] = caches[i + 1];
  }
  --cacheCount;

  if (cacheCount == 0) {
    activeCache = -1;
  } else if (activeCache >= cacheCount) {
    activeCache = cacheCount - 1;
  }

  saveCaches();
  screenDirty = true;
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

static void drawHeader(const char *title) {
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(7, 5);
  tft.print(title);

  tft.setTextSize(1);
  tft.setTextColor(apRunning ? ST77XX_GREEN : ST77XX_YELLOW);
  tft.setCursor(257, 7);
  if (apRunning) {
    tft.print("WLAN ON");
  } else {
    tft.print("WLAN --");
  }
}

static void drawNoCache() {
  drawHeader("GEOCACHE");
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 45);
  tft.print("Noch kein Ziel");

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 80);
  tft.print("Encoder 1.2s halten");
  tft.setCursor(10, 96);
  tft.print("WLAN TEmbed-Geocache");
  tft.setCursor(10, 112);
  tft.print("Safari: 192.168.4.1");
}

static void drawNavScreen() {
  if (activeCache < 0 || activeCache >= cacheCount) {
    drawNoCache();
    return;
  }

  drawHeader("NAV");

  CacheEntry &c = caches[activeCache];

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(55, 5);
  String name = c.name;
  if (name.length() > 17) name = name.substring(0, 17);
  tft.print(name);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(8, 28);
  tft.printf("Ziel %d/%d   Drehen = anderes Ziel", activeCache + 1, cacheCount);

  if (!gpsFixValid()) {
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.setCursor(10, 63);
    tft.print("Warte auf GPS Fix");

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_YELLOW);
    tft.setCursor(10, 92);
    tft.printf("Satelliten: %lu", (unsigned long)(gps.satellites.isValid() ? gps.satellites.value() : 0));
    tft.setCursor(10, 108);
    tft.print("GPS-Modul RX44/TX43");
    return;
  }

  const double lat = gps.location.lat();
  const double lon = gps.location.lng();
  const double distance = TinyGPSPlus::distanceBetween(lat, lon, c.lat, c.lon);
  const double bearing = TinyGPSPlus::courseTo(lat, lon, c.lat, c.lon);

  const bool moving = gps.speed.isValid() && gps.course.isValid() && gps.speed.kmph() >= 2.0;
  const double arrowAngle = moving
      ? normalize360(bearing - gps.course.deg())
      : normalize360(bearing);

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(24, 45);
  tft.print(moving ? "REL KURS" : "N OBEN");

  uint16_t arrowColor = distance <= 15.0 ? ST77XX_GREEN : ST77XX_CYAN;
  drawArrow(61, 105, 42, arrowAngle, arrowColor);

  tft.setTextColor(distance <= 15.0 ? ST77XX_GREEN : ST77XX_WHITE);
  tft.setTextSize(3);
  tft.setCursor(118, 52);
  if (distance < 1000.0) {
    tft.printf("%.0f m", distance);
  } else if (distance < 10000.0) {
    tft.printf("%.2f km", distance / 1000.0);
  } else {
    tft.printf("%.1f km", distance / 1000.0);
  }

  tft.setTextSize(2);
  tft.setTextColor(ST77XX_CYAN);
  tft.setCursor(118, 87);
  tft.printf("%03.0f deg %s", bearing, cardinal16(bearing));

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(118, 114);
  if (moving) {
    double turn = normalize360(bearing - gps.course.deg());
    if (turn > 180.0) turn -= 360.0;
    tft.printf("Kurs %.0f  Turn %+.0f", gps.course.deg(), turn);
  } else {
    tft.print("Im Stand: Peilung N-oben");
  }

  tft.setCursor(8, 148);
  tft.setTextColor(ST77XX_YELLOW);
  double hdop = gps.hdop.isValid() ? gps.hdop.hdop() : 99.9;
  tft.printf("Sat:%lu  HDOP:%.1f", (unsigned long)(gps.satellites.isValid() ? gps.satellites.value() : 0), hdop);

  if (distance <= 15.0) {
    tft.setCursor(190, 148);
    tft.setTextColor(ST77XX_GREEN);
    tft.print("ZIELBEREICH");
  }
}

static void drawGpsScreen() {
  drawHeader("GPS");

  tft.setTextSize(1);
  tft.setTextColor(gpsFixValid() ? ST77XX_GREEN : ST77XX_RED);
  tft.setCursor(10, 32);
  tft.print(gpsFixValid() ? "FIX OK" : "KEIN FRISCHER FIX");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 52);

  if (gps.location.isValid()) {
    tft.printf("%.6f", gps.location.lat());
    tft.setCursor(10, 76);
    tft.printf("%.6f", gps.location.lng());
  } else {
    tft.print("Position --");
  }

  tft.setTextSize(1);
  tft.setCursor(10, 108);
  tft.printf("Satelliten: %lu", (unsigned long)(gps.satellites.isValid() ? gps.satellites.value() : 0));

  tft.setCursor(10, 124);
  if (gps.hdop.isValid()) tft.printf("HDOP: %.1f", gps.hdop.hdop());
  else tft.print("HDOP: --");

  tft.setCursor(120, 108);
  if (gps.altitude.isValid()) tft.printf("Hoehe: %.0f m", gps.altitude.meters());
  else tft.print("Hoehe: --");

  tft.setCursor(120, 124);
  if (gps.speed.isValid()) tft.printf("Speed: %.1f km/h", gps.speed.kmph());
  else tft.print("Speed: --");

  tft.setCursor(120, 140);
  if (gps.course.isValid()) tft.printf("Kurs: %.0f deg", gps.course.deg());
  else tft.print("Kurs: --");

  tft.setCursor(10, 156);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("Encoder kurz = naechste Ansicht");
}

static void drawHelpScreen() {
  drawHeader("HILFE");

  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);

  tft.setCursor(10, 35);
  tft.print("Drehen: Ziel wechseln");

  tft.setCursor(10, 53);
  tft.print("Encoder kurz: Ansicht wechseln");

  tft.setCursor(10, 71);
  tft.print("Encoder 1.2s: WLAN an/aus");

  tft.setCursor(10, 89);
  tft.print("Seitentaste kurz: Ansicht zurueck");

  tft.setCursor(10, 107);
  tft.print("Seitentaste 2.5s: Launcher");

  tft.setCursor(10, 132);
  tft.setTextColor(ST77XX_YELLOW);
  tft.print("WLAN: TEmbed-Geocache");

  tft.setCursor(10, 148);
  tft.print("192.168.4.1  PW: geocache1");
}

static void drawApScreen() {
  drawHeader("CACHE SETUP");

  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.setCursor(10, 43);
  tft.print("WLAN aktiv");

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(10, 76);
  tft.print("SSID: TEmbed-Geocache");
  tft.setCursor(10, 94);
  tft.print("Passwort: geocache1");
  tft.setCursor(10, 112);
  tft.print("Safari: http://192.168.4.1");

  tft.setTextColor(ST77XX_YELLOW);
  tft.setCursor(10, 145);
  tft.print("Encoder 1.2s = WLAN aus");
}

static void drawScreen() {
  if (apRunning) {
    drawApScreen();
    return;
  }

  switch (screenMode) {
    case SCREEN_GPS:
      drawGpsScreen();
      break;
    case SCREEN_HELP:
      drawHelpScreen();
      break;
    case SCREEN_NAV:
    default:
      drawNavScreen();
      break;
  }
}

static String makePage() {
  String html;
  html.reserve(6500);

  html += "<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<meta charset='utf-8'><title>T-Embed Geocache</title>";
  html += "<style>body{font-family:-apple-system,BlinkMacSystemFont,sans-serif;margin:18px;background:#111;color:#eee}";
  html += "input,button{font-size:18px;padding:10px;margin:4px 0;width:100%;box-sizing:border-box}";
  html += ".c{border:1px solid #555;border-radius:12px;padding:12px;margin:10px 0}.a{border-color:#36d17c}";
  html += "a{color:#7fd7ff}small{color:#aaa}</style></head><body>";
  html += "<h2>T-Embed Geocache</h2>";

  if (gpsFixValid()) {
    html += "<p>GPS: <b style='color:#36d17c'>Fix OK</b> &middot; ";
    html += String(gps.location.lat(), 6);
    html += ", ";
    html += String(gps.location.lng(), 6);
    html += " &middot; Sat ";
    html += String(gps.satellites.isValid() ? gps.satellites.value() : 0);
    html += "</p>";
  } else {
    html += "<p>GPS: noch kein frischer Fix</p>";
  }

  html += "<h3>Neues Ziel</h3><form method='post' action='/add'>";
  html += "<input name='name' maxlength='28' placeholder='Cachename' required>";
  html += "<input name='lat' inputmode='decimal' placeholder='Breitengrad, z.B. 49.79123' required>";
  html += "<input name='lon' inputmode='decimal' placeholder='Laengengrad, z.B. 9.26789' required>";
  html += "<button type='submit'>Speichern & aktivieren</button></form>";

  if (gpsFixValid() && cacheCount < MAX_CACHES) {
    html += "<form method='post' action='/here'><input name='name' maxlength='28' placeholder='Name fuer aktuelle Position' value='Hier'>";
    html += "<button type='submit'>Aktuelle GPS-Position als Ziel speichern</button></form>";
  }

  html += "<h3>Gespeicherte Ziele (" + String(cacheCount) + "/" + String(MAX_CACHES) + ")</h3>";

  if (cacheCount == 0) {
    html += "<p><small>Noch keine Ziele gespeichert.</small></p>";
  }

  for (int i = 0; i < cacheCount; ++i) {
    html += "<div class='c";
    if (i == activeCache) html += " a";
    html += "'><b>";
    html += htmlEscape(caches[i].name);
    html += "</b><br><small>";
    html += String(caches[i].lat, 6);
    html += ", ";
    html += String(caches[i].lon, 6);
    html += "</small><br>";

    if (i != activeCache) {
      html += "<a href='/select?i=" + String(i) + "'>Als Ziel waehlen</a> &nbsp; ";
    } else {
      html += "<b style='color:#36d17c'>AKTIV</b> &nbsp; ";
    }

    html += "<a href='/delete?i=" + String(i) + "' onclick='return confirm(\"Loeschen?\")'>Loeschen</a></div>";
  }

  html += "<p><small>Am T-Embed: Encoder drehen = Ziel wechseln. Encoder 1.2 s = WLAN aus.</small></p>";
  html += "</body></html>";
  return html;
}

static void redirectHome() {
  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "");
}

static void setupWebServer() {
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html; charset=utf-8", makePage());
  });

  server.on("/add", HTTP_POST, []() {
    if (cacheCount >= MAX_CACHES) {
      server.send(400, "text/plain", "Maximale Anzahl Ziele erreicht.");
      return;
    }

    String name = server.arg("name");
    double lat = server.arg("lat").toDouble();
    double lon = server.arg("lon").toDouble();

    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
      server.send(400, "text/plain", "Ungueltige Koordinaten.");
      return;
    }

    addCache(name, lat, lon);
    redirectHome();
  });

  server.on("/here", HTTP_POST, []() {
    if (!gpsFixValid()) {
      server.send(400, "text/plain", "Kein frischer GPS Fix.");
      return;
    }
    if (cacheCount >= MAX_CACHES) {
      server.send(400, "text/plain", "Maximale Anzahl Ziele erreicht.");
      return;
    }

    addCache(server.arg("name"), gps.location.lat(), gps.location.lng());
    redirectHome();
  });

  server.on("/select", HTTP_GET, []() {
    selectCache(server.arg("i").toInt());
    redirectHome();
  });

  server.on("/delete", HTTP_GET, []() {
    deleteCache(server.arg("i").toInt());
    redirectHome();
  });

  server.onNotFound([]() {
    redirectHome();
  });
}

static void startAp() {
  if (apRunning) return;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  delay(50);
  server.begin();

  apRunning = true;
  screenDirty = true;

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

static void stopAp() {
  if (!apRunning) return;

  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);

  apRunning = false;
  screenDirty = true;
}

static void toggleAp() {
  if (apRunning) stopAp();
  else startAp();
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
  while (s < 0) s += 3;
  while (s >= 3) s -= 3;
  screenMode = (ScreenMode)s;
  screenDirty = true;
}

static void serviceEncoder() {
  int a = digitalRead(PIN_ENC_A);
  if (a != lastEncA && a == LOW) {
    int b = digitalRead(PIN_ENC_B);
    int dir = (b == HIGH) ? 1 : -1;
    if (cacheCount > 0 && !apRunning) {
      selectCache(activeCache + dir);
    }
  }
  lastEncA = a;
}

static void serviceButton(ButtonTracker &btn, uint32_t longMs, bool isCenter) {
  bool now = digitalRead(btn.pin);

  if (btn.last == HIGH && now == LOW) {
    btn.downSince = millis();
    btn.longHandled = false;
  }

  if (now == LOW && !btn.longHandled && btn.downSince != 0) {
    if ((uint32_t)(millis() - btn.downSince) >= longMs) {
      btn.longHandled = true;
      if (isCenter) {
        toggleAp();
      } else {
        returnToLauncher();
      }
    }
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

  prefs.begin("geocache", false);
  loadCaches();

  setupWebServer();

  WiFi.mode(WIFI_OFF);

  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_CYAN);
  tft.setTextSize(2);
  tft.setCursor(20, 55);
  tft.print("T-Embed Geocache");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE);
  tft.setCursor(20, 88);
  tft.print("M5Stack GPS wird gestartet...");
  delay(800);

  screenDirty = true;
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode((char)gpsSerial.read());
  }

  serviceEncoder();
  serviceButton(centerBtn, CENTER_LONG_MS, true);
  serviceButton(backBtn, BACK_LONG_MS, false);

  if (apRunning) {
    server.handleClient();
  }

  uint32_t now = millis();
  if (screenDirty || (uint32_t)(now - lastDrawMs) >= DRAW_INTERVAL_MS) {
    screenDirty = false;
    lastDrawMs = now;
    drawScreen();
  }

  delay(5);
}
