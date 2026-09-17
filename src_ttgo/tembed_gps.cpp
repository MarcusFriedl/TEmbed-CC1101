#include <Arduino.h>

#ifdef TEMBED_CC1101

#include <TinyGPSPlus.h>

namespace {

// T-Embed CC1101 Plus UART used by the M5Stack GPS/BDS Unit v1.1 (AT6668).
// Wiring used by the existing Grove -> Qwiic connection:
//   T-Embed TX GPIO43 -> GPS RX
//   T-Embed RX GPIO44 <- GPS TX
static constexpr int TEMBED_GPS_TX = 43;
static constexpr int TEMBED_GPS_RX = 44;
static constexpr uint32_t TEMBED_GPS_BAUD = 115200;
static constexpr uint32_t TEMBED_GPS_FIX_TIMEOUT_MS = 5000;
static constexpr uint32_t TEMBED_GPS_PUBLISH_MS = 500;

HardwareSerial tembedGpsSerial(1);
TinyGPSPlus tembedGps;
uint32_t lastPublishMs = 0;

extern "C" void TEMBED_displaySetOwnGps(bool valid,
                                         double lat,
                                         double lon,
                                         double alt,
                                         float speedKmh,
                                         float courseDeg,
                                         uint32_t satellites,
                                         uint32_t ageMs);

static void publishGpsState()
{
    const uint32_t age = tembedGps.location.isValid()
        ? tembedGps.location.age()
        : UINT32_MAX;

    const bool valid = tembedGps.location.isValid() &&
                       age <= TEMBED_GPS_FIX_TIMEOUT_MS;

    const double lat = valid ? tembedGps.location.lat() : 0.0;
    const double lon = valid ? tembedGps.location.lng() : 0.0;
    const double alt = tembedGps.altitude.isValid() ? tembedGps.altitude.meters() : NAN;
    const float speedKmh = tembedGps.speed.isValid() ? tembedGps.speed.kmph() : NAN;
    const float courseDeg = tembedGps.course.isValid() ? tembedGps.course.deg() : NAN;
    const uint32_t satellites = tembedGps.satellites.isValid()
        ? tembedGps.satellites.value()
        : 0U;

    TEMBED_displaySetOwnGps(valid,
                            lat,
                            lon,
                            alt,
                            speedKmh,
                            courseDeg,
                            satellites,
                            age);
}

} // namespace

extern "C" void TEMBED_gpsSetup()
{
    tembedGpsSerial.begin(TEMBED_GPS_BAUD,
                          SERIAL_8N1,
                          TEMBED_GPS_RX,
                          TEMBED_GPS_TX);

    lastPublishMs = millis();
    publishGpsState();
}

extern "C" void TEMBED_gpsService()
{
    while (tembedGpsSerial.available() > 0) {
        tembedGps.encode((char)tembedGpsSerial.read());
    }

    const uint32_t now = millis();
    if ((uint32_t)(now - lastPublishMs) >= TEMBED_GPS_PUBLISH_MS) {
        lastPublishMs = now;
        publishGpsState();
    }
}

#else

extern "C" void TEMBED_gpsSetup() {}
extern "C" void TEMBED_gpsService() {}

#endif
