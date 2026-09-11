#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

#ifdef TEMBED_CC1101

// Spectrum scanner for the T-Embed CC1101.
// Important: all actual radio access stays inside RadioLib.  We no longer
// write CC1101 registers or strobes ourselves here.

extern uint8_t PIN_DIO1;
void onDIO1Edge();

namespace {

static constexpr int CC1101_CS   = 12;
static constexpr int CC1101_GDO0 = 3;
static constexpr int CC1101_GDO2 = 38;

static SPISettings scanSpiSettings(2000000, MSBFIRST, SPI_MODE0);
static CC1101 scanCc1101 = new Module(
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

    // Configure the same proven RS41 receive parameters as the normal radio
    // path, but on a scanner-only RadioLib wrapper.  Both wrappers address the
    // same physical CC1101; the normal path restores its configuration after
    // spectrum mode ends.
    int16_t state = scanCc1101.begin(
        freqMHz,
        4.8,    // kbit/s
        2.4,    // kHz deviation
        58.0,   // kHz RX bandwidth
        10,     // dBm (irrelevant for RX)
        16
    );

    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 scanner init failed: %d\n", state);
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

    // Re-enable the normal synchronous bit-clock ISR.  The next normal
    // LilyGo::SX1278_setRadioFrequencyHz(..., false) call restores the decoder
    // configuration through the original RadioLib object.
    attachInterrupt(PIN_DIO1, onDIO1Edge, RISING);
    fastScanActive = false;
    scanRadioReady = false;
}

extern "C" float TEMBED_CC1101_fastScanRssi(uint32_t freqHz)
{
    if (!fastScanActive) {
        // Spectrum mode does not need the 4.8-kHz sonde bit clock.  Keeping it
        // detached prevents the high-priority SyncDet task from processing
        // noise while the scanner is sweeping.
        detachInterrupt(PIN_DIO1);
        fastScanActive = true;
    }

    if (!scanRadioReady && !initScanRadio(freqHz)) {
        return -128.0f;
    }

    const float freqMHz = (float)freqHz / 1000000.0f;

    // RadioLib's CC1101::setFrequency() already issues the IDLE strobe before
    // updating FREQ2/1/0.  We deliberately do NOT call standby() beforehand:
    // standby() additionally polls MARCSTATE and was the expensive part of the
    // old per-bin path.
    int16_t state = scanCc1101.setFrequency(freqMHz);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 scanner setFrequency %.4f failed: %d\n", freqMHz, state);
        return -128.0f;
    }

    // Start synchronous Direct Mode again on the new frequency.  This keeps
    // CC1101 state transitions, frequency calibration and status-register
    // handling entirely inside RadioLib instead of duplicating them here.
    state = scanCc1101.receiveDirect();
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("CC1101 scanner receiveDirect failed: %d\n", state);
        return -128.0f;
    }

    // Same settling order of magnitude as the previously working RadioLib
    // scanner, but without the extra blocking standby() call.
    delayMicroseconds(1200);

    return scanCc1101.getRSSI();
}

#endif // TEMBED_CC1101
