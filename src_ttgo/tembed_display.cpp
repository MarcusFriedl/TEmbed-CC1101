#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>

#ifdef TEMBED_CC1101

namespace {

static constexpr int TEMBED_CC1101_CS = 12;
static constexpr int TEMBED_SD_CS      = 13;
static constexpr int TEMBED_TFT_CS     = 41;
static constexpr int TEMBED_TFT_DC     = 16;
static constexpr int TEMBED_TFT_BL     = 21;

// TFT and CC1101 share the already-running hardware SPI bus.
// The ESP32 SPI implementation serializes beginTransaction()/endTransaction(),
// so live TFT updates can coexist with RadioLib without changing GPIO 9/11.
static Adafruit_ST7789 tembedTft(&SPI, TEMBED_TFT_CS, TEMBED_TFT_DC, -1);

static volatile uint32_t liveFreqHz = 0;
static volatile float liveRssi = -128.0f;
static volatile bool liveBtConnected = false;
static volatile bool liveScannerActive = false;
static volatile bool liveDirty = true;

static bool displayReady = false;
static uint32_t lastDrawMs = 0;

static void clearRow(int16_t y, int16_t h = 26)
{
    tembedTft.fillRect(0, y, 320, h, ST77XX_BLACK);
}

static void drawLiveRows()
{
    const uint32_t freqHz = liveFreqHz;
    const float rssi = liveRssi;
    const bool btConnected = liveBtConnected;
    const bool scannerActive = liveScannerActive;

    char line[32];

    clearRow(48);
    tembedTft.setTextSize(2);
    tembedTft.setTextColor(btConnected ? ST77XX_GREEN : ST77XX_YELLOW);
    tembedTft.setCursor(12, 52);
    snprintf(line, sizeof(line), "CC1101 OK  iRa:%s", btConnected ? "ON" : "--");
    tembedTft.print(line);

    clearRow(78);
    tembedTft.setTextColor(ST77XX_WHITE);
    tembedTft.setCursor(12, 82);
    if (scannerActive) {
        tembedTft.print("400-406 MHz scan");
    }
    else if (freqHz >= 100000000UL) {
        tembedTft.printf("%.3f MHz", (double)freqHz / 1000000.0);
    }
    else {
        tembedTft.print("Frequency: --");
    }

    clearRow(108);
    tembedTft.setCursor(12, 112);
    if (scannerActive) {
        tembedTft.print("RSSI: spectrum");
    }
    else if (isfinite(rssi) && rssi > -127.5f) {
        tembedTft.printf("RSSI: %.0f dBm", (double)rssi);
    }
    else {
        tembedTft.print("RSSI: --- dBm");
    }

    clearRow(138, 32);
    tembedTft.setTextColor(ST77XX_CYAN);
    tembedTft.setCursor(12, 142);
    tembedTft.print(scannerActive ? "Scanner active" : "RS41 / RS92");
}

} // namespace

extern "C" void TEMBED_displaySetBtState(bool connected)
{
    liveBtConnected = connected;
    liveDirty = true;
}

extern "C" void TEMBED_displaySetFrequencyHz(uint32_t freqHz)
{
    liveFreqHz = freqHz;
    if (!liveScannerActive) {
        liveDirty = true;
    }
}

extern "C" void TEMBED_displaySetRssi(float rssi)
{
    if (!isfinite(rssi)) {
        return;
    }

    liveRssi = rssi;
    if (!liveScannerActive) {
        liveDirty = true;
    }
}

extern "C" void TEMBED_displaySetScanner(bool active)
{
    liveScannerActive = active;
    liveDirty = true;
}

extern "C" void TEMBED_displaySetup()
{
    // Keep all other devices deselected during initial TFT setup. Scanner and
    // decoder tasks have not been started yet at this point.
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
    tembedTft.setCursor(12, 10);
    tembedTft.print("Ra-TEmbed");

    displayReady = true;
    liveDirty = false;
    lastDrawMs = millis();
    drawLiveRows();
}

extern "C" void TEMBED_displayService()
{
    if (!displayReady || !liveDirty) {
        return;
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - lastDrawMs) < 500U) {
        return;
    }

    // Clear before drawing. If another task changes state while the TFT is
    // being refreshed it will set liveDirty again and cause another pass.
    liveDirty = false;
    lastDrawMs = now;
    drawLiveRows();
}

#else

extern "C" void TEMBED_displaySetup() {}
extern "C" void TEMBED_displayService() {}
extern "C" void TEMBED_displaySetBtState(bool) {}
extern "C" void TEMBED_displaySetFrequencyHz(uint32_t) {}
extern "C" void TEMBED_displaySetRssi(float) {}
extern "C" void TEMBED_displaySetScanner(bool) {}

#endif
