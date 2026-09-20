# T-Embed Geocache Online

Standalone-Geocaching-Firmware für den LILYGO T-Embed CC1101 mit M5Stack GPS/BDS Unit.

## Idee

Die Firmware braucht keine Handy-App zur Bedienung:

1. GPS-Fix holen.
2. Mit einem bereits gespeicherten WLAN verbinden oder per WPS koppeln.
3. Die nächsten verfügbaren Caches von Opencaching.de laden.
4. Cache am T-Embed auswählen.
5. Direkt zum Cache navigieren.
6. Die zuletzt geladenen Caches bleiben für Offline-Nutzung gespeichert.

## Bedienung

- **Encoder drehen:** Cache in der Liste auswählen.
- **Encoder kurz drücken:** Ansichten wechseln: Liste → Navigation → Details → GPS → Online-Info.
- **Encoder ca. 1,3 s halten:**
  - WLAN verbunden: Umgebungssuche neu laden.
  - Kein WLAN: WPS starten. Danach am Router die WPS-Taste drücken.
- **Seitentaste kurz:** vorherige Ansicht.
- **Seitentaste ca. 2,5 s halten:** zurück zum Launcher.

## Online-Suche

- Quelle: **Opencaching.de / OKAPI**
- Suchradius: **25 km**
- Maximal: **20 Caches**
- Nur verfügbare Caches
- Geladen werden Name, OC-Code, Position, Typ, Größe, D/T und Hint.
- Die Liste wird im Flash gespeichert und steht danach offline zur Verfügung.

## OKAPI Consumer Key

Die verwendeten Suchmethoden benötigen nur **OKAPI Level 1** und damit den Consumer Key. Der Consumer Secret wird für diese Firmware nicht benötigt.

Aus Sicherheitsgründen enthält das öffentliche Repository **keinen persönlichen Consumer Key**. GitHub Actions kompiliert deshalb mit einem eindeutigen Platzhalter. Eine personalisierte BIN kann anschließend mit `tools/patch_okapi_key.py` gepatcht werden; das Skript berechnet dabei auch ESP32-Checksumme und angehängten SHA-256-Image-Hash neu.

Der rohe GitHub-Actions-Artifact ist deshalb ein Template-Build. Für die tatsächliche Online-Suche muss der Platzhalter durch einen gültigen Consumer Key ersetzt werden.

## WLAN

Beim Start versucht die Firmware ein bereits im ESP32 gespeichertes WLAN zu verwenden.

Ist keines vorhanden:

1. Encoder ca. 1,3 s halten.
2. Auf dem Display erscheint WPS.
3. WPS-Taste am Router drücken.
4. Nach erfolgreicher Verbindung werden die Zugangsdaten vom ESP32 gespeichert.

## Hardware

M5Stack GPS/BDS Unit:
- RX: GPIO 44
- TX: GPIO 43
- Baud: 115200

Display/SPI/PWR entsprechen der bereits funktionierenden T-Embed-Firmware im Repository.

## GitHub Build

Workflow:

`.github/workflows/geocache-online-build.yml`

Artifact:

`TEmbed-Geocache-Online.bin`
