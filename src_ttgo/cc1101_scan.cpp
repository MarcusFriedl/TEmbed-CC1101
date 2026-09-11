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

static constexpr int CC1101_CS = 12;

static constexpr uint8_t CC1101_REG_FREQ2 = 0x0D;
static constexpr uint8_t CC1101_CMD_SRX    = 0x34;
static constexpr uint8_t CC1101_CMD_SIDLE  = 0x36;
static constexpr uint8_t CC1101_WRITE_BURST = 0x40;

static SPISettings scanSpiSettings(2000000, MSBFIRST, SPI_MODE0);
static bool fastScanActive = false;

// The CC1101 is already awake for the whole spectrum scan.  The previous
// version polled SO/MISO with digitalRead() after every CS assertion.  On the
// ESP32-S3 SPI matrix that check proved unreliable and caused every bin to
// fall back to the -128 dBm error value.  A short CS setup time is sufficient
// here because we never put the radio into SLEEP/POWER-DOWN during scanning.
static inline void selectCc1101()
{
    digitalWrite(CC1101_CS, LOW);
    delayMicroseconds(2);
}

static inline void deselectCc1101()
{
    digitalWrite(CC1101_CS, HIGH);
}

static void sendStrobe(uint8_t command)
{
    selectCc1101();
    SPI.transfer(command);
    deselectCc1101();
}

static void writeFrequencyRegisters(uint32_t freqHz)
{
    // CC1101: FREQ = f_carrier * 2^16 / f_xosc, f_xosc = 26 MHz.
    const uint32_t frf = (uint32_t)(((uint64_t)freqHz << 16) / 26000000ULL);

    selectCc1101();
    SPI.transfer(CC1101_REG_FREQ2 | CC1101_WRITE_BURST);
    SPI.transfer((uint8_t)((frf >> 16) & 0xFF));
    SPI.transfer((uint8_t)((frf >> 8) & 0xFF));
    SPI.transfer((uint8_t)(frf & 0xFF));
    deselectCc1101();
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

    SPI.beginTransaction(scanSpiSettings);

    // Leave RX before changing FREQx.  Unlike the old RadioLib standby() call
    // this is deliberately non-blocking; a small fixed settling time is enough
    // for the already-running CC1101 and avoids hundreds of MARCSTATE polls.
    sendStrobe(CC1101_CMD_SIDLE);
    delayMicroseconds(40);

    writeFrequencyRegisters(freqHz);
    sendStrobe(CC1101_CMD_SRX);

    SPI.endTransaction();

    // RSSI/AGC needs a little time after the new RX frequency is active.
    // 2 ms is still fast enough for the 10-kHz sweep but gives the CC1101
    // enough time for a meaningful live RSSI sample.
    delayMicroseconds(2000);

    return TEMBED_CC1101_readRadioLibRssi();
}

#endif // TEMBED_CC1101
