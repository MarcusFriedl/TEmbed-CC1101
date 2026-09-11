#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#ifdef TEMBED_CC1101

// Fast spectrum scanner for the T-Embed CC1101.
//
// Important design point:
// - RadioLib still owns all SPI/CS handling. This avoids the unreliable raw
//   SPI experiment used in the first fast scanner.
// - Direct Mode is configured only once when spectrum mode starts.
// - Per scan bin we only do IDLE -> FREQ2/1/0 -> RX -> RSSI.
//   We therefore avoid calling receiveDirect() hundreds of times per sweep.

extern uint8_t PIN_DIO1;
void onDIO1Edge();

namespace {

static constexpr int CC1101_CS   = 12;
static constexpr int CC1101_GDO0 = 3;
static constexpr int CC1101_GDO2 = 38;

static SPISettings scanSpiSettings(2000000, MSBFIRST, SPI_MODE0);

// RadioLib deliberately exposes the low-level CC1101 SPI helpers as protected.
// This tiny subclass lets the scanner use those proven helpers without
// duplicating RadioLib's chip-select / SPI-ready handling.
class FastScanCC1101 : public CC1101 {
public:
    using CC1101::CC1101;

    int16_t fastTuneAndRx(float freqMHz)
    {
        if (!(((freqMHz >= 300.0f) && (freqMHz <= 348.0f)) ||
              ((freqMHz >= 387.0f) && (freqMHz <= 464.0f)) ||
              ((freqMHz >= 779.0f) && (freqMHz <= 928.0f)))) {
            return RADIOLIB_ERR_INVALID_FREQUENCY;
        }

        // Same frequency formula and register order as RadioLib::setFrequency,
        // but without re-running setOutputPower(), which is irrelevant for RX.
        SPIsendCommand(RADIOLIB_CC1101_CMD_IDLE);

        const uint32_t frf = (uint32_t)((freqMHz * 65536.0f) / 26.0f);
        SPIwriteRegister(RADIOLIB_CC1101_REG_FREQ2, (uint8_t)((frf >> 16) & 0xFF));
        SPIwriteRegister(RADIOLIB_CC1101_REG_FREQ1, (uint8_t)((frf >> 8) & 0xFF));
        SPIwriteRegister(RADIOLIB_CC1101_REG_FREQ0, (uint8_t)(frf & 0xFF));

        // Direct Mode configuration survives the IDLE transition. We only need
        // to put the already-configured receiver back into RX.
        SPIsendCommand(RADIOLIB_CC1101_CMD_RX);

        return RADIOLIB_ERR_NONE;
    }
};

static FastScanCC1101 scanCc1101 = new Module(
    CC1101_CS,
    CC1101_GDO0,
    RADIOLIB_NC,
    CC1101_GDO2,
    SPI,
    scanSpiSettings
);

static bool fastScanActive = false;
static bool scanRadioReady = false;

static bool initScanRadio(uint32_t freqHz)
{
    const float freqMHz = (float)freqHz / 1000000.0f;

    // Configure the physical CC1101 once using the same known-good RS41 radio
    // parameters as the normal decoder path.
    int16_t state = scanCc1101.begin(
        freqMHz,
        4.8,    // kbit/s
        2.4,    // kHz deviation
        58.0,   // kHz RX bandwidth
        10,     // dBm (irrelevant for reception)
        16
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 scanner init failed: %d\n", state);
        return false;
    }

    // Configure synchronous Direct Mode exactly once. During the sweep GDO0's
    // interrupt is detached, so the 4.8-kHz bit clock cannot load SyncDet.
    state = scanCc1101.receiveDirect();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 scanner initial receiveDirect failed: %d\n", state);
        return false;
    }

    scanRadioReady = true;
    return true;
}

} // namespace

extern "C" void TEMBED_CC1101_fastScanEnd()
{
    if (!fastScanActive) {
        return;
    }

    // Re-enable the normal synchronous bit-clock ISR. The following normal
    // LilyGo::SX1278_setRadioFrequencyHz(..., false) call restores the proven
    // decoder configuration through the original RadioLib object.
    attachInterrupt(PIN_DIO1, onDIO1Edge, RISING);
    fastScanActive = false;
    scanRadioReady = false;
}

extern "C" float TEMBED_CC1101_fastScanRssi(uint32_t freqHz)
{
    if (!fastScanActive) {
        // Spectrum mode does not decode sonde bits. Pausing GDO0 keeps the
        // high-priority SyncDet task from processing noise during the sweep.
        detachInterrupt(PIN_DIO1);
        fastScanActive = true;
    }

    if (!scanRadioReady && !initScanRadio(freqHz)) {
        return -128.0f;
    }

    const float freqMHz = (float)freqHz / 1000000.0f;

    int16_t state = scanCc1101.fastTuneAndRx(freqMHz);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 fast tune %.4f failed: %d\n", freqMHz, state);
        return -128.0f;
    }

    // Allow synthesizer + AGC/RSSI to settle. This is intentionally shorter
    // than the old full RadioLib per-bin setup, while still leaving enough
    // time for a meaningful RSSI sample.
    delayMicroseconds(1200);

    return scanCc1101.getRSSI();
}

#endif // TEMBED_CC1101
