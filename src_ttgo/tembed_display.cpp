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

static Adafruit_ST7789 tembedTft(&SPI, TEMBED_TFT_CS, TEMBED_TFT_DC, -1);

struct DisplaySnapshot {
    uint32_t freqHz;
    float rssi;
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
};

static portMUX_TYPE displayStateMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t liveFreqHz = 0;
static float liveRssi = -128.0f;
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

static bool previousSondeSampleValid = false;
static char previousSondeId[16] = {0};
static double previousAltitude = 0.0;
static uint32_t previousFrame = 0;
static uint32_t previousSampleMs = 0;

static bool displayReady = false;
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
    portEXIT_CRITICAL(&displayStateMux);

    return s;
}

static void clearRow(int16_t y, int16_t h = 24)
{
    tembedTft.fillRect(0, y, 320, h, ST77XX_BLACK);
}

static void drawLiveRows(const DisplaySnapshot &s)
{
    char line[40];

    clearRow(42);
    tembedTft.setTextSize(2);
    tembedTft.setTextColor(s.btConnected ? ST77XX_GREEN : ST77XX_YELLOW);
    tembedTft.setCursor(10, 45);
    snprintf(line, sizeof(line), "CC1101 OK  iRa:%s", s.btConnected ? "ON" : "--");
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
        tembedTft.print("GPS: waiting");
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

    bool sameSonde = previousSondeSampleValid && hasId && (strncmp(previousSondeId, newId, sizeof(previousSondeId)) == 0);
    bool newFrame = !previousSondeSampleValid || frameCounter != previousFrame;

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
    tembedTft.fillScreen(ST77XX_BLACK);
    tembedTft.setTextWrap(false);

    tembedTft.setTextColor(ST77XX_CYAN);
    tembedTft.setTextSize(3);
    tembedTft.setCursor(10, 8);
    tembedTft.print("Ra-TEmbed");

    displayReady = true;
    lastDrawMs = millis();

    DisplaySnapshot s = getSnapshot();
    drawLiveRows(s);

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
    drawLiveRows(s);
}

#else

extern "C" void TEMBED_displaySetup() {}
extern "C" void TEMBED_displayService() {}
extern "C" void TEMBED_displaySetBtState(bool) {}
extern "C" void TEMBED_displaySetFrequencyHz(uint32_t) {}
extern "C" void TEMBED_displaySetRssi(float) {}
extern "C" void TEMBED_displaySetScanner(bool) {}
extern "C" void TEMBED_displaySetSondeData(const char *, double, double, double, uint32_t) {}
extern "C" void TEMBED_displayClearSonde() {}

#endif
