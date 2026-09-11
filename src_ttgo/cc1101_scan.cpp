#include <Arduino.h>
#include <SPI.h>

#ifdef TEMBED_CC1101

// Fast spectrum-scan path for the T-Embed CC1101.
// The normal decoder path continues to use RadioLib in lilygo.cpp.
// For a spectrum bin we only need: IDLE -> frequency registers -> RX.
// RSSI itself is deliberately read through RadioLib again, so we use the
// already proven CC1101 status-register handling from the normal receiver.

extern uint8_t PIN_DIO1;
void onDIO1Edge();
extern "C" float TEMBED_CC1101_readRadioLibRssi();

namespace {

static constexpr int CC1101_CS   = 12;
static constexpr int CC1101_MISO = 10;

static constexpr uint8_t CC1101_REG_FREQ2 = 0x0D;
static constexpr uint8_t CC1101_CMD_SRX    = 0x34;
static constexpr uint8_t CC1101_CMD_SIDLE  = 0x36;
static constexpr uint8_t CC1101_WRITE_BURST = 0x40;

static SPISettings scanSpiSettings(2000000, MSBFIRST, SPI_MODE0);
static bool fastScanActive = false;

static bool selectCc1101()
{
    digitalWrite(CC1101_CS, LOW);

    // SO/MISO stays high until the CC1101 is ready for SPI access.
    // In normal operation this is practically immediate, but keep a short
    // timeout so a radio problem cannot stall the scanner forever.
    uint32_t started = micros();
    while (digitalRead(CC1101_MISO) == HIGH) {
        if ((uint32_t)(micros() - started) > 1000U) {
            digitalWrite(CC1101_CS, HIGH);
            return false;
        }
    }

    return true;
}

static void deselectCc1101()
{
    digitalWrite(CC1101_CS, HIGH);
}

static bool sendStrobe(uint8_t command)
{
    if (!selectCc1101()) {
        return false;
    }

    SPI.transfer(command);
    deselectCc1101();
    return true;
}

static bool writeFrequencyRegisters(uint32_t freqHz)
{
    // CC1101: FREQ = f_carrier * 2^16 / f_xosc, f_xosc = 26 MHz.
    const uint32_t frf = (uint32_t)(((uint64_t)freqHz << 16) / 26000000ULL);

    if (!selectCc1101()) {
        return false;
    }

    SPI.transfer(CC1101_REG_FREQ2 | CC1101_WRITE_BURST);
    SPI.transfer((uint8_t)((frf >> 16) & 0xFF));
    SPI.transfer((uint8_t)((frf >> 8) & 0xFF));
    SPI.transfer((uint8_t)(frf & 0xFF));
    deselectCc1101();

    return true;
}

} // namespace

extern "C" void TEMBED_CC1101_fastScanEnd()
{
    if (!fastScanActive) {
        return;
    }

    // Restore the normal bit-clock ISR. The following normal RadioLib retune
    // restores the proven synchronous Direct Mode configuration.
    attachInterrupt(PIN_DIO1, onDIO1Edge, RISING);
    fastScanActive = false;
}

extern "C" float TEMBED_CC1101_fastScanRssi(uint32_t freqHz)
{
    if (!fastScanActive) {
        // While sweeping there is no useful sonde bitstream. Pausing GDO0 keeps
        // the high-priority SyncDet task from consuming a 4.8-kHz noise stream.
        detachInterrupt(PIN_DIO1);
        fastScanActive = true;
    }

    bool ok;

    SPI.beginTransaction(scanSpiSettings);

    // Deliberately much smaller than RadioLib's old per-bin path:
    // no standby polling, no repeated Direct-Mode register setup, no PA update.
    ok = sendStrobe(CC1101_CMD_SIDLE);
    if (ok) {
        ok = writeFrequencyRegisters(freqHz);
    }
    if (ok) {
        ok = sendStrobe(CC1101_CMD_SRX);
    }

    SPI.endTransaction();

    if (!ok) {
        return -128.0f;
    }

    // Allow AGC/RSSI to settle, while retaining the fast sweep.
    delayMicroseconds(1500);

    // Important: Do not duplicate CC1101 RSSI register handling here.
    // RadioLib already reads and converts the live RSSI register correctly in
    // Direct Mode. This also keeps scanner and normal S-meter calibration equal.
    return TEMBED_CC1101_readRadioLibRssi();
}

#endif // TEMBED_CC1101
