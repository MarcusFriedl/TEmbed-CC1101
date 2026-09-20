# T-Embed Geocache Firmware

Eigenständige Geocaching-Firmware für den **LILYGO T-Embed CC1101** mit dem bereits verwendeten **M5Stack GPS/BDS Unit v1.1 (AT6668)**.

## Hardware

Die Firmware verwendet die Pinbelegung, die im bestehenden Ra-TEmbed-Projekt bereits funktioniert:

- GPS UART: RX GPIO 44 / TX GPIO 43, 115200 Baud
- TFT ST7789: CS 41, DC 16, Backlight 21
- Gemeinsamer SPI-Bus: SCK 11, MISO 10, MOSI 9
- Encoder: A 4, B 5, Taste 0
- Seitentaste: GPIO 6
- Peripherie-Power: GPIO 15

## Bedienung

- **Encoder drehen:** nächsten/vorherigen gespeicherten Cache wählen
- **Encoder kurz drücken:** zwischen Navigation, GPS-Daten und Hilfe wechseln
- **Encoder ca. 1,2 s halten:** WLAN-Konfiguration ein/aus
- **Seitentaste kurz:** vorherige Ansicht
- **Seitentaste ca. 2,5 s halten:** zurück zum Launcher

## Cache-Koordinaten vom iPhone eingeben

1. Encoder ca. 1,2 s halten.
2. Mit dem iPhone WLAN **TEmbed-Geocache** wählen.
3. Passwort: **geocache1**
4. Safari: **http://192.168.4.1**
5. Cachename, Breitengrad und Längengrad eingeben.

Bis zu 12 Ziele werden dauerhaft im Flash gespeichert.

## Navigation

Die Firmware zeigt Entfernung und Peilung zum Ziel. Wenn du dich mit mindestens ca. 2 km/h bewegst und das GPS einen Kurs liefert, wird der Pfeil relativ zu deiner Bewegungsrichtung dargestellt. Im Stand gibt es ohne Magnetkompass keine echte Geräteausrichtung; deshalb wird dann die Peilung mit **N oben** angezeigt.

## Build

GitHub Actions baut bei Änderungen auf dem Branch **geocache-firmware** automatisch:

- Artifact: **TEmbed-Geocache**
- Datei darin: **TEmbed-Geocache.bin**

Die BIN kann anschließend wie deine anderen Firmwares über den Launcher verwendet werden.
