#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>

#ifdef TEMBED_CC1101

namespace {

static constexpr int TEMBED_CC1101_CS = 12;
static constexpr int TEMBED_SD_CS      = 13;
static constexpr int TEMBED_TFT_CS     = 41;
static constexpr int TEMBED_TFT_DC     = 16;
static constexpr int TEMBED_TFT_BL     = 21;

// Share the already-running hardware SPI bus with the CC1101.
// ttgo_setup() has already called SPI.begin(11, 10, 9) before this function
// runs. Adafruit's hardware-SPI constructor therefore reuses that bus instead
// of changing GPIO 9/11 into software-SPI GPIOs.
static Adafruit_ST7789 tembedTft(&SPI, TEMBED_TFT_CS, TEMBED_TFT_DC, -1);

} // namespace

extern "C" void TEMBED_displaySetup()
{
    // Keep every other device on the shared bus deselected while the TFT is
    // initialised. No scanner/decoder tasks exist yet at this point.
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
    tembedTft.setCursor(12, 12);
    tembedTft.print("Ra-TEmbed");

    tembedTft.setTextColor(ST77XX_GREEN);
    tembedTft.setTextSize(2);
    tembedTft.setCursor(12, 55);
    tembedTft.print("CC1101 OK");

    tembedTft.setTextColor(ST77XX_WHITE);
    tembedTft.setCursor(12, 85);
    tembedTft.print("400-406 MHz");

    tembedTft.setCursor(12, 115);
    tembedTft.print("RS41 / RS92");

    tembedTft.setTextColor(ST77XX_YELLOW);
    tembedTft.setCursor(12, 145);
    tembedTft.print("iRa ready");
}

#else

extern "C" void TEMBED_displaySetup() {}

#endif
