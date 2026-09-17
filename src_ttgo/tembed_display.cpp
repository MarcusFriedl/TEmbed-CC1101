#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"

#ifdef TEMBED_CC1101

namespace {

static constexpr int TEMBED_CC1101_CS = 12;
static constexpr int TEMBED_SD_CS      = 13;
static constexpr int TEMBED_TFT_CS     = 41;
static constexpr int TEMBED_TFT_DC     = 16;
static constexpr int TEMBED_TFT_BL     = 21;

static constexpr double DEG_TO_RAD_D = 0.017453292519943295769;
static constexpr double RAD_TO_DEG_D = 57.295779513082320876;
static constexpr double EARTH_RADIUS_M = 6371000.0;

static Adafruit_ST7789 tembedTft(&SPI, TEMBED_TFT_CS, TEMBED_TFT_DC, -1);

struct DisplaySnapshot {
    uint32_t freqHz;
    float rssi;
    float rssiMax;
    bool btConnected;
    bool scannerActive;
    bool sondeValid;
    bool positionValid;
    bool altitudeValid;
    bool climbValid;
    char sondeId[16];
    double lat;
    double lon;
    double alt;
    float climb;

    bool ownGpsValid;
    double ownLat;
    double ownLon;
    double ownAlt;
    bool ownSpeedValid;
    float ownSpeedKmh;
    bool ownCourseValid;
    float ownCourseDeg;
    uint32_t ownSatellites;
    uint32_t ownGpsAgeMs;
};

static portMUX_TYPE displayStateMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t liveFreqHz = 0;
static float liveRssi = -128.0f;
static float liveRssiMax = -128.0f;
static bool liveBtConnected = false;
static bool liveScannerActive = false;
static bool liveDirty = true;

static bool liveSondeValid = false;
static bool livePositionValid = false;
static bool liveAltitudeValid = false;
static bool liveClimbValid = false;
static char liveSondeId[16] = {0};
static double liveLat = 0.0;
static double liveLon = 0.0;
static double liveAlt = 0.0;
static float liveClimb = 0.0f;

static bool liveOwnGpsValid = false;
static double liveOwnLat = 0.0;
static double liveOwnLon = 0.0;
static double liveOwnAlt = NAN;
static bool liveOwnSpeedValid = false;
static float liveOwnSpeedKmh = NAN;
static bool liveOwnCourseValid = false;
static float liveOwnCourseDeg = NAN;
static uint32_t liveOwnSatellites = 0;
static uint32_t liveOwnGpsAgeMs = UINT32_MAX;

static bool previousSondeSampleValid = false;
static char previousSondeId[16] = {0};
static double previousAltitude = 0.0;
static uint32_t previousFrame = 0;
static uint32_t previousSampleMs = 0;

static bool displayReady = false;
static bool lastRenderedChase = false;
static uint32_t lastDrawMs = 0;

static void clearSondeStateLocked()
{
    liveSondeValid = false;
    livePositionValid = false;
    liveAltitudeValid = false;
    liveClimbValid = false;
    liveSondeId[0] = 0;
    liveLat = 0.0;
    liveLon = 0.0;
    liveAlt = 0.0;
    liveClimb = 0.0f;
    liveRssiMax = -128.0f;

    previousSondeSampleValid = false;
    previousSondeId[0] = 0;
    previousAltitude = 0.0;
    previousFrame = 0;
    previousSampleMs = 0;
}

static DisplaySnapshot getSnapshot()
{
    DisplaySnapshot s{};

    portENTER_CRITICAL(&displayStateMux);
    s.freqHz = liveFreqHz;
    s.rssi = liveRssi;
    s.rssiMax = liveRssiMax;
    s.btConnected = liveBtConnected;
    s.scannerActive = liveScannerActive;
    s.sondeValid = liveSondeValid;
    s.positionValid = livePositionValid;
    s.altitudeValid = liveAltitudeValid;
    s.climbValid = liveClimbValid;
    strncpy(s.sondeId, liveSondeId, sizeof(s.sondeId) - 1);
    s.sondeId[sizeof(s.sondeId) - 1] = 0;
    s.lat = liveLat;
    s.lon = liveLon;
    s.alt = liveAlt;
    s.climb = liveClimb;

    s.ownGpsValid = liveOwnGpsValid;
    s.ownLat = liveOwnLat;
    s.ownLon = liveOwnLon;
    s.ownAlt = liveOwnAlt;
    s.ownSpeedValid = liveOwnSpeedValid;
    s.ownSpeedKmh = liveOwnSpeedKmh;
    s.ownCourseValid = liveOwnCourseValid;
    s.ownCourseDeg = liveOwnCourseDeg;
    s.ownSatellites = liveOwnSatellites;
    s.ownGpsAgeMs = liveOwnGpsAgeMs;
    portEXIT_CRITICAL(&displayStateMux);

    return s;
}

static void clearRow(int16_t y, int16_t h = 24)
{
    tembedTft.fillRect(0, y, 320, h, ST77XX_BLACK);
}

static double normalize360(double deg)
{
    while (deg < 0.0) deg += 360.0;
    while (deg >= 360.0) deg -= 360.0;
    return deg;
}

static double distanceMeters(double lat1, double lon1, double lat2, double lon2)
{
    const double p1 = lat1 * DEG_TO_RAD_D;
    const double p2 = lat2 * DEG_TO_RAD_D;
    const double dp = (lat2 - lat1) * DEG_TO_RAD_D;
    const double dl = (lon2 - lon1) * DEG_TO_RAD_D;

    const double a = sin(dp * 0.5) * sin(dp * 0.5) +
                     cos(p1) * cos(p2) * sin(dl * 0.5) * sin(dl * 0.5);
    const double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    return EARTH_RADIUS_M * c;
}

static double bearingDegrees(double lat1, double lon1, double lat2, double lon2)
{
    const double p1 = lat1 * DEG_TO_RAD_D;
    const double p2 = lat2 * DEG_TO_RAD_D;
    const double dl = (lon2 - lon1) * DEG_TO_RAD_D;

    const double y = sin(dl) * cos(p2);
    const double x = cos(p1) * sin(p2) -
                     sin(p1) * cos(p2) * cos(dl);
    return normalize360(atan2(y, x) * RAD_TO_DEG_D);
}

static const char *cardinal16(double bearing)
{
    static const char *dirs[16] = {
        "N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
        "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"
    };
    int idx = (int)floor((normalize360(bearing) + 11.25) / 22.5) & 15;
    return dirs[idx];
}

static void drawArrow(int16_t cx, int16_t cy, int16_t length, double angleDeg, uint16_t color)
{
    const double a = angleDeg * DEG_TO_RAD_D;
    const int16_t tx = cx + (int16_t)lround(sin(a) * length);
    const int16_t ty = cy - (int16_t)lround(cos(a) * length);

    for (int offset = -1; offset <= 1; ++offset) {
        tembedTft.drawLine(cx + offset, cy, tx + offset, ty, color);
    }

    const double left = (angleDeg + 150.0) * DEG_TO_RAD_D;
    const double right = (angleDeg - 150.0) * DEG_TO_RAD_D;
    const int16_t hx1 = tx + (int16_t)lround(sin(left) * 16.0);
    const int16_t hy1 = ty - (int16_t)lround(cos(left) * 16.0);
    const int16_t hx2 = tx + (int16_t)lround(sin(right) * 16.0);
    const int16_t hy2 = ty - (int16_t)lround(cos(right) * 16.0);

    tembedTft.fillTriangle(tx, ty, hx1, hy1, hx2, hy2, color);
    tembedTft.fillCircle(cx, cy, 3, color);
}

static void drawNormalBase()
{
    tembedTft.fillScreen(ST77XX_BLACK);
    tembedTft.setTextColor(ST77XX_CYAN);
    tembedTft.setTextSize(3);
    tembedTft.setCursor(10, 8);
    tembedTft.print("Ra-TEmbed");
}

static void drawNormalRows(const DisplaySnapshot &s)
{
    char line[48];

    clearRow(42);
    tembedTft.setTextSize(2);
    tembedTft.setTextColor(s.btConnected ? ST77XX_GREEN : ST77XX_YELLOW);
    tembedTft.setCursor(10, 45);
    snprintf(line, sizeof(line), "CC1101 OK iRa:%s G:%s",
             s.btConnected ? "ON" : "--",
             s.ownGpsValid ? "OK" : "--");
    tembedTft.print(line);

    clearRow(67);
    tembedTft.setTextColor(ST77XX_WHITE);
    tembedTft.setCursor(10, 70);
    if (s.scannerActive) {
        tembedTft.print("400-406 MHz scan");
    }
    else if (s.freqHz >= 100000000UL) {
        if (isfinite(s.rssi) && s.rssi > -127.5f) {
            tembedTft.printf("%.3f MHz  %.0f dBm", (double)s.freqHz / 1000000.0, (double)s.rssi);
        }
        else {
            tembedTft.printf("%.3f MHz  --- dBm", (double)s.freqHz / 1000000.0);
        }
    }
    else {
        tembedTft.print("Frequency: --");
    }

    clearRow(92);
    tembedTft.setCursor(10, 95);
    if (s.scannerActive) {
        tembedTft.setTextColor(ST77XX_CYAN);
        tembedTft.print("RSSI: spectrum");
    }
    else if (s.sondeValid) {
        tembedTft.setTextColor(ST77XX_CYAN);
        snprintf(line, sizeof(line), "Sonde: %s", s.sondeId);
        tembedTft.print(line);
    }
    else {
        tembedTft.setTextColor(ST77XX_CYAN);
        tembedTft.print("RS41 / RS92");
    }

    clearRow(117);
    tembedTft.setCursor(10, 120);
    tembedTft.setTextColor(ST77XX_WHITE);
    if (s.scannerActive) {
        tembedTft.print("Scanner active");
    }
    else if (s.sondeValid && s.positionValid) {
        tembedTft.printf("%.5f  %.5f", s.lat, s.lon);
    }
    else if (s.sondeValid) {
        tembedTft.print("Sonde GPS: waiting");
    }
    else if (s.ownGpsValid) {
        tembedTft.printf("Own GPS: FIX  Sat:%lu", (unsigned long)s.ownSatellites);
    }
    else {
        tembedTft.print("Waiting for sonde");
    }

    clearRow(142, 28);
    tembedTft.setCursor(10, 145);
    tembedTft.setTextColor(ST77XX_YELLOW);
    if (!s.scannerActive && s.sondeValid && s.altitudeValid) {
        if (s.climbValid) {
            tembedTft.printf("Alt %.0fm  %+.1fm/s", s.alt, (double)s.climb);
        }
        else {
            tembedTft.printf("Alt %.0fm  V:---", s.alt);
        }
    }
}

static void drawRssiBar(const DisplaySnapshot &s)
{
    const int16_t x = 10;
    const int16_t y = 150;
    const int16_t w = 300;
    const int16_t h = 12;

    tembedTft.drawRect(x, y, w, h, ST77XX_WHITE);
    tembedTft.fillRect(x + 1, y + 1, w - 2, h - 2, ST77XX_BLACK);

    if (isfinite(s.rssi) && s.rssi > -127.5f) {
        float level = (s.rssi + 120.0f) / 80.0f;
        if (level < 0.0f) level = 0.0f;
        if (level > 1.0f) level = 1.0f;
        int16_t fill = (int16_t)lround(level * (w - 2));
        if (fill > 0) {
            tembedTft.fillRect(x + 1, y + 1, fill, h - 2, ST77XX_CYAN);
        }
    }

    if (isfinite(s.rssiMax) && s.rssiMax > -127.5f) {
        float maxLevel = (s.rssiMax + 120.0f) / 80.0f;
        if (maxLevel < 0.0f) maxLevel = 0.0f;
        if (maxLevel > 1.0f) maxLevel = 1.0f;
        int16_t marker = x + 1 + (int16_t)lround(maxLevel * (w - 3));
        tembedTft.drawFastVLine(marker, y - 2, h + 4, ST77XX_YELLOW);
    }
}

static void drawChaseScreen(const DisplaySnapshot &s)
{
    const double distance = distanceMeters(s.ownLat, s.ownLon, s.lat, s.lon);
    const double bearing = bearingDegrees(s.ownLat, s.ownLon, s.lat, s.lon);
    const bool relativeToCourse = s.ownSpeedValid && s.ownCourseValid &&
                                  isfinite(s.ownSpeedKmh) && s.ownSpeedKmh >= 2.0f;
    const double arrowBearing = relativeToCourse
        ? normalize360(bearing - s.ownCourseDeg)
        : bearing;

    tembedTft.fillScreen(ST77XX_BLACK);
    tembedTft.setTextWrap(false);

    tembedTft.setTextSize(2);
    tembedTft.setTextColor(ST77XX_CYAN);
    tembedTft.setCursor(8, 5);
    tembedTft.print("CHASE");

    tembedTft.setTextColor(ST77XX_WHITE);
    tembedTft.setCursor(78, 5);
    tembedTft.print(s.sondeId);

    tembedTft.setTextSize(1);
    tembedTft.setTextColor(ST77XX_YELLOW);
    tembedTft.setCursor(260, 6);
    tembedTft.printf("GPS:%lu", (unsigned long)s.ownSatellites);

    tembedTft.setTextColor(ST77XX_WHITE);
    tembedTft.setCursor(8, 27);
    if (s.freqHz >= 100000000UL) {
        tembedTft.printf("%.3f MHz", (double)s.freqHz / 1000000.0);
    }
    if (isfinite(s.rssi) && s.rssi > -127.5f) {
        tembedTft.setCursor(82, 27);
        tembedTft.printf("RSSI %.0f dBm", (double)s.rssi);
    }

    tembedTft.setTextSize(1);
    tembedTft.setTextColor(ST77XX_YELLOW);
    tembedTft.setCursor(22, 43);
    tembedTft.print(relativeToCourse ? "REL KURS" : "N OBEN");

    drawArrow(62, 99, 42, arrowBearing, ST77XX_GREEN);

    tembedTft.setTextColor(ST77XX_WHITE);
    tembedTft.setTextSize(3);
    tembedTft.setCursor(122, 45);
    if (distance < 1000.0) {
        tembedTft.printf("%.0f m", distance);
    }
    else if (distance < 10000.0) {
        tembedTft.printf("%.2f km", distance / 1000.0);
    }
    else {
        tembedTft.printf("%.1f km", distance / 1000.0);
    }

    tembedTft.setTextSize(2);
    tembedTft.setTextColor(ST77XX_CYAN);
    tembedTft.setCursor(122, 80);
    tembedTft.printf("%03.0f deg %s", bearing, cardinal16(bearing));

    tembedTft.setTextSize(1);
    tembedTft.setTextColor(ST77XX_WHITE);
    tembedTft.setCursor(122, 106);
    if (relativeToCourse) {
        double turn = normalize360(bearing - s.ownCourseDeg);
        if (turn > 180.0) turn -= 360.0;
        tembedTft.printf("Kurs %.0f deg  Turn %+.0f deg", (double)s.ownCourseDeg, turn);
    }
    else {
        tembedTft.print("Pfeil: Norden ist oben");
    }

    tembedTft.setCursor(122, 121);
    if (s.altitudeValid) {
        if (s.climbValid) {
            tembedTft.printf("Sonde %.0fm  %+.1fm/s", s.alt, (double)s.climb);
        }
        else {
            tembedTft.printf("Sonde %.0fm", s.alt);
        }
    }

    tembedTft.setCursor(10, 137);
    tembedTft.setTextColor(ST77XX_YELLOW);
    if (isfinite(s.rssi) && s.rssi > -127.5f) {
        if (isfinite(s.rssiMax) && s.rssiMax > -127.5f) {
            tembedTft.printf("Signal %.0f dBm   Max %.0f dBm", (double)s.rssi, (double)s.rssiMax);
        }
        else {
            tembedTft.printf("Signal %.0f dBm", (double)s.rssi);
        }
    }

    drawRssiBar(s);
}

static void drawDisplay(const DisplaySnapshot &s)
{
    const bool chase = !s.scannerActive && s.sondeValid && s.positionValid && s.ownGpsValid;

    if (chase) {
        drawChaseScreen(s);
    }
    else {
        if (lastRenderedChase) {
            drawNormalBase();
        }
        drawNormalRows(s);
    }

    lastRenderedChase = chase;
}

} // namespace

extern "C" void TEMBED_displaySetBtState(bool connected)
{
    portENTER_CRITICAL(&displayStateMux);
    liveBtConnected = connected;
    liveDirty = true;
    portEXIT_CRITICAL(&displayStateMux);
}

extern "C" void TEMBED_displaySetFrequencyHz(uint32_t freqHz)
{
    portENTER_CRITICAL(&displayStateMux);

    if (liveSondeValid && liveFreqHz != 0 && freqHz != 0) {
        uint32_t delta = (liveFreqHz > freqHz) ? (liveFreqHz - freqHz) : (freqHz - liveFreqHz);
        if (delta > 1000U) {
            clearSondeStateLocked();
        }
    }

    liveFreqHz = freqHz;
    if (!liveScannerActive) {
        liveDirty = true;
    }

    portEXIT_CRITICAL(&displayStateMux);
}

extern "C" void TEMBED_displaySetRssi(float rssi)
{
    if (!isfinite(rssi)) {
        return;
    }

    portENTER_CRITICAL(&displayStateMux);
    liveRssi = rssi;
    if (!liveScannerActive && rssi > liveRssiMax) {
        liveRssiMax = rssi;
    }
    if (!liveScannerActive) {
        liveDirty = true;
    }
    portEXIT_CRITICAL(&displayStateMux);
}

extern "C" void TEMBED_displayClearSonde()
{
    portENTER_CRITICAL(&displayStateMux);
    clearSondeStateLocked();
    liveDirty = true;
    portEXIT_CRITICAL(&displayStateMux);
}

extern "C" void TEMBED_displaySetSondeData(const char *id, double lat, double lon, double alt, uint32_t frameCounter)
{
    const uint32_t now = millis();
    const bool hasId = (id != nullptr) && (id[0] != 0);
    const bool positionValid = isfinite(lat) && isfinite(lon) &&
                               lat >= -90.0 && lat <= 90.0 &&
                               lon >= -180.0 && lon <= 180.0 &&
                               !(fabs(lat) < 0.000001 && fabs(lon) < 0.000001);
    const bool altitudeValid = isfinite(alt) && alt > -1000.0 && alt < 100000.0;

    char newId[16] = {0};
    if (hasId) {
        strncpy(newId, id, sizeof(newId) - 1);
    }

    portENTER_CRITICAL(&displayStateMux);

    bool sameSonde = previousSondeSampleValid && hasId &&
                     (strncmp(previousSondeId, newId, sizeof(previousSondeId)) == 0);
    bool newFrame = !previousSondeSampleValid || frameCounter != previousFrame;

    if (!sameSonde && hasId) {
        liveRssiMax = liveRssi;
    }

    if (sameSonde && newFrame && altitudeValid && previousSampleMs != 0) {
        uint32_t dtMs = now - previousSampleMs;
        if (dtMs >= 400U && dtMs <= 10000U) {
            float instantClimb = (float)((alt - previousAltitude) * 1000.0 / (double)dtMs);
            if (isfinite(instantClimb) && fabsf(instantClimb) <= 100.0f) {
                liveClimb = liveClimbValid
                    ? (0.65f * liveClimb + 0.35f * instantClimb)
                    : instantClimb;
                liveClimbValid = true;
            }
            else {
                liveClimbValid = false;
            }
        }
        else {
            liveClimbValid = false;
        }
    }
    else if (!sameSonde) {
        liveClimbValid = false;
        liveClimb = 0.0f;
    }

    liveSondeValid = hasId;
    livePositionValid = positionValid;
    liveAltitudeValid = altitudeValid;
    strncpy(liveSondeId, newId, sizeof(liveSondeId) - 1);
    liveSondeId[sizeof(liveSondeId) - 1] = 0;
    liveLat = lat;
    liveLon = lon;
    liveAlt = alt;

    if (hasId && newFrame && altitudeValid) {
        strncpy(previousSondeId, newId, sizeof(previousSondeId) - 1);
        previousSondeId[sizeof(previousSondeId) - 1] = 0;
        previousAltitude = alt;
        previousFrame = frameCounter;
        previousSampleMs = now;
        previousSondeSampleValid = true;
    }

    if (!liveScannerActive) {
        liveDirty = true;
    }

    portEXIT_CRITICAL(&displayStateMux);
}

extern "C" void TEMBED_displaySetOwnGps(bool valid,
                                         double lat,
                                         double lon,
                                         double alt,
                                         float speedKmh,
                                         float courseDeg,
                                         uint32_t satellites,
                                         uint32_t ageMs)
{
    const bool positionValid = valid && isfinite(lat) && isfinite(lon) &&
                               lat >= -90.0 && lat <= 90.0 &&
                               lon >= -180.0 && lon <= 180.0 &&
                               !(fabs(lat) < 0.000001 && fabs(lon) < 0.000001);

    portENTER_CRITICAL(&displayStateMux);
    liveOwnGpsValid = positionValid;
    liveOwnLat = positionValid ? lat : 0.0;
    liveOwnLon = positionValid ? lon : 0.0;
    liveOwnAlt = isfinite(alt) ? alt : NAN;
    liveOwnSpeedValid = isfinite(speedKmh) && speedKmh >= 0.0f;
    liveOwnSpeedKmh = liveOwnSpeedValid ? speedKmh : NAN;
    liveOwnCourseValid = isfinite(courseDeg) && courseDeg >= 0.0f && courseDeg < 360.0f;
    liveOwnCourseDeg = liveOwnCourseValid ? courseDeg : NAN;
    liveOwnSatellites = satellites;
    liveOwnGpsAgeMs = ageMs;
    liveDirty = true;
    portEXIT_CRITICAL(&displayStateMux);
}

extern "C" void TEMBED_displaySetScanner(bool active)
{
    portENTER_CRITICAL(&displayStateMux);
    liveScannerActive = active;
    if (active) {
        clearSondeStateLocked();
    }
    liveDirty = true;
    portEXIT_CRITICAL(&displayStateMux);
}

extern "C" void TEMBED_displaySetup()
{
    pinMode(TEMBED_CC1101_CS, OUTPUT);
    digitalWrite(TEMBED_CC1101_CS, HIGH);
    pinMode(TEMBED_SD_CS, OUTPUT);
    digitalWrite(TEMBED_SD_CS, HIGH);
    pinMode(TEMBED_TFT_CS, OUTPUT);
    digitalWrite(TEMBED_TFT_CS, HIGH);

    pinMode(TEMBED_TFT_BL, OUTPUT);
    digitalWrite(TEMBED_TFT_BL, HIGH);

    tembedTft.init(170, 320);
    tembedTft.setRotation(1);
    tembedTft.setTextWrap(false);

    drawNormalBase();

    displayReady = true;
    lastDrawMs = millis();

    DisplaySnapshot s = getSnapshot();
    drawDisplay(s);

    portENTER_CRITICAL(&displayStateMux);
    liveDirty = false;
    portEXIT_CRITICAL(&displayStateMux);
}

extern "C" void TEMBED_displayService()
{
    if (!displayReady) {
        return;
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - lastDrawMs) < 500U) {
        return;
    }

    bool shouldDraw = false;
    portENTER_CRITICAL(&displayStateMux);
    if (liveDirty) {
        liveDirty = false;
        shouldDraw = true;
    }
    portEXIT_CRITICAL(&displayStateMux);

    if (!shouldDraw) {
        return;
    }

    DisplaySnapshot s = getSnapshot();
    lastDrawMs = now;
    drawDisplay(s);
}

#else

extern "C" void TEMBED_displaySetup() {}
extern "C" void TEMBED_displayService() {}
extern "C" void TEMBED_displaySetBtState(bool) {}
extern "C" void TEMBED_displaySetFrequencyHz(uint32_t) {}
extern "C" void TEMBED_displaySetRssi(float) {}
extern "C" void TEMBED_displaySetScanner(bool) {}
extern "C" void TEMBED_displaySetSondeData(const char *, double, double, double, uint32_t) {}
extern "C" void TEMBED_displaySetOwnGps(bool, double, double, double, float, float, uint32_t, uint32_t) {}
extern "C" void TEMBED_displayClearSonde() {}

#endif
