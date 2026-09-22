#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <TFT_eSPI.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_NeoPixel.h>
#include <esp_sleep.h>

// Fox Ultimate for LILYGO T-Embed CC1101 / CC1101 Plus
// H4M/Mayhem Fox Hunt: tune AM to the configured frequency.

static constexpr int PIN_PWR_EN   = 15;
static constexpr int PIN_SD_CS    = 13;
static constexpr int PIN_RADIO_CS = 12;
static constexpr int PIN_GDO0     = 3;
static constexpr int PIN_GDO2     = 38;
static constexpr int PIN_RF_SW1   = 47;
static constexpr int PIN_RF_SW0   = 48;
static constexpr int PIN_ENC_A    = 4;
static constexpr int PIN_ENC_B    = 5;
static constexpr int PIN_ENC_KEY  = 0;
static constexpr int PIN_SIDE_KEY = 6;
static constexpr int PIN_RGB      = 14;
static constexpr int RGB_COUNT    = 8;
static constexpr int PIN_GPS_RX   = 44;
static constexpr int PIN_GPS_TX   = 43;
static constexpr int PIN_TFT_BL   = 21;
static constexpr uint32_t GPS_BAUD = 115200;
static constexpr uint32_t GPS_FIX_MAX_AGE_MS = 6000;
static constexpr uint32_t SIDE_FOUND_MS = 1200;
static constexpr uint32_t SIDE_LAUNCHER_MS = 2500;
static constexpr int SPI_SCK      = 11;
static constexpr int SPI_MISO     = 10;
static constexpr int SPI_MOSI     = 9;

static constexpr float FREQ_MIN = 433.050f;
static constexpr float FREQ_MAX = 434.790f;
static constexpr float FREQ_DEFAULT = 433.920f;
static constexpr char FW_VERSION[] = "1.1.0";

TFT_eSPI tft;
SPIClass radioSPI(HSPI);
CC1101 radio = new Module(PIN_RADIO_CS, PIN_GDO0, RADIOLIB_NC, PIN_GDO2, radioSPI);
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
Adafruit_NeoPixel pixels(RGB_COUNT, PIN_RGB, NEO_GRB + NEO_KHZ800);
Preferences prefs;
WebServer server(80);

enum BeaconMode : uint8_t { MODE_PULSE=0, MODE_MORSE=1, MODE_HYBRID=2 };

struct Settings {
  float freqMHz = FREQ_DEFAULT;
  int8_t powerDbm = 7;
  BeaconMode mode = MODE_HYBRID;
  uint32_t intervalMs = 2500;
  uint16_t pulseMs = 180;
  uint16_t toneHz = 700;
  uint8_t wpm = 18;
  uint8_t messageEvery = 10;
  bool gpsEnabled = true;
  bool dataBeacon = false;
  bool includeGpsInData = false;
  bool ledsEnabled = true;
  bool stealthEnabled = true;
  uint16_t stealthAfterSec = 15;
  bool autoStart = false;
  String foxId = "FOX 1";
  String message = "FIND ME";
} cfg;

bool hunting=false, foundMarked=false, displaySleeping=false, webMode=false, radioReady=false;
bool inMenu=false, gpsScreen=false, gpsPortStarted=false, encWasDown=false, sideWasDown=false;
uint32_t huntStartMs=0, nextTxMs=0, lastUserInputMs=0, beaconCount=0;
uint32_t bestTimeSec=0, lastFoundSec=0, totalHunts=0, totalFound=0;
uint32_t encPressStart=0, sidePressStart=0;
int lastEncA=HIGH, menuIndex=0;

static uint16_t clampU16(int v,int lo,int hi){ return (uint16_t)max(lo,min(hi,v)); }
static float clampFreq(float f){ return max(FREQ_MIN,min(FREQ_MAX,f)); }

String modeName(){
  if(cfg.mode==MODE_MORSE) return "MORSE";
  if(cfg.mode==MODE_HYBRID) return "HYBRID";
  return "PULSE";
}

String formatTime(uint32_t sec){
  char b[20];
  snprintf(b,sizeof(b),"%02lu:%02lu:%02lu",
    (unsigned long)(sec/3600),(unsigned long)((sec%3600)/60),(unsigned long)(sec%60));
  return String(b);
}

uint32_t elapsedSec(){ return hunting ? (millis()-huntStartMs)/1000UL : lastFoundSec; }

void allPixels(uint8_t r,uint8_t g,uint8_t b){
  if(!cfg.ledsEnabled || displaySleeping){ pixels.clear(); pixels.show(); return; }
  for(int i=0;i<RGB_COUNT;i++) pixels.setPixelColor(i,pixels.Color(r,g,b));
  pixels.show();
}
void txFlash(bool on){
  if(!cfg.ledsEnabled || displaySleeping) return;
  if(on) allPixels(100,0,0);
  else if(hunting) allPixels(0,25,0);
  else allPixels(0,0,15);
}

void wakeDisplay(){
  displaySleeping=false;
  digitalWrite(PIN_TFT_BL,HIGH);
  lastUserInputMs=millis();
}
void maybeSleepDisplay(){
  if(!cfg.stealthEnabled || !hunting || displaySleeping || webMode) return;
  if(millis()-lastUserInputMs > cfg.stealthAfterSec*1000UL){
    displaySleeping=true;
    digitalWrite(PIN_TFT_BL,LOW);
    pixels.clear(); pixels.show();
  }
}

void select433Antenna(){
  pinMode(PIN_RF_SW1,OUTPUT); pinMode(PIN_RF_SW0,OUTPUT);
  digitalWrite(PIN_RF_SW1,HIGH);
  digitalWrite(PIN_RF_SW0,HIGH);
}

int8_t validPower(int v){
  static const int8_t p[]={-30,-20,-15,-10,0,5,7,10};
  int best=p[0],d=999;
  for(int8_t x:p){ int nd=abs(v-x); if(nd<d){d=nd;best=x;} }
  return best;
}

void loadSettings(){
  prefs.begin("foxultimate",true);
  cfg.freqMHz=clampFreq(prefs.getFloat("freq",FREQ_DEFAULT));
  cfg.powerDbm=validPower(prefs.getInt("pwr",7));
  cfg.mode=(BeaconMode)constrain((int)prefs.getUChar("mode",MODE_HYBRID),0,2);
  cfg.intervalMs=constrain(prefs.getUInt("intv",2500),500U,120000U);
  cfg.pulseMs=clampU16(prefs.getUShort("pulse",180),40,5000);
  cfg.toneHz=clampU16(prefs.getUShort("tone",700),300,1800);
  cfg.wpm=constrain((int)prefs.getUChar("wpm",18),5,40);
  cfg.messageEvery=constrain((int)prefs.getUChar("msgN",10),1,50);
  cfg.gpsEnabled=prefs.getBool("gps",true);
  cfg.dataBeacon=prefs.getBool("data",false);
  cfg.includeGpsInData=prefs.getBool("gpsTx",false);
  cfg.ledsEnabled=prefs.getBool("leds",true);
  cfg.stealthEnabled=prefs.getBool("stealth",true);
  cfg.stealthAfterSec=constrain((int)prefs.getUShort("stSec",15),5,600);
  cfg.autoStart=prefs.getBool("auto",false);
  cfg.foxId=prefs.getString("foxid","FOX 1");
  cfg.message=prefs.getString("msg","FIND ME");
  bestTimeSec=prefs.getUInt("best",0);
  lastFoundSec=prefs.getUInt("last",0);
  totalHunts=prefs.getUInt("hunts",0);
  totalFound=prefs.getUInt("found",0);
  prefs.end();
}

void saveSettings(){
  prefs.begin("foxultimate",false);
  prefs.putFloat("freq",cfg.freqMHz); prefs.putInt("pwr",cfg.powerDbm); prefs.putUChar("mode",(uint8_t)cfg.mode);
  prefs.putUInt("intv",cfg.intervalMs); prefs.putUShort("pulse",cfg.pulseMs); prefs.putUShort("tone",cfg.toneHz);
  prefs.putUChar("wpm",cfg.wpm); prefs.putUChar("msgN",cfg.messageEvery);
  prefs.putBool("gps",cfg.gpsEnabled); prefs.putBool("data",cfg.dataBeacon); prefs.putBool("gpsTx",cfg.includeGpsInData);
  prefs.putBool("leds",cfg.ledsEnabled); prefs.putBool("stealth",cfg.stealthEnabled); prefs.putUShort("stSec",cfg.stealthAfterSec);
  prefs.putBool("auto",cfg.autoStart); prefs.putString("foxid",cfg.foxId.substring(0,31)); prefs.putString("msg",cfg.message.substring(0,79));
  prefs.end();
}

void saveStats(){
  prefs.begin("foxultimate",false);
  prefs.putUInt("best",bestTimeSec); prefs.putUInt("last",lastFoundSec);
  prefs.putUInt("hunts",totalHunts); prefs.putUInt("found",totalFound);
  prefs.end();
}

void appendLog(const String &event,uint32_t sec=0){
  File f=LittleFS.open("/huntlog.csv",FILE_APPEND);
  if(!f) return;
  if(f.size()==0) f.println("event,uptime_ms,elapsed_s,fox_id,lat,lon,gps_valid,message");
  bool fix=gps.location.isValid() && gps.location.age()<5000;
  String lat=fix?String(gps.location.lat(),6):"";
  String lon=fix?String(gps.location.lng(),6):"";
  String id=cfg.foxId, msg=cfg.message; id.replace(",",";"); msg.replace(",",";");
  f.printf("%s,%lu,%lu,%s,%s,%s,%d,%s\n",event.c_str(),(unsigned long)millis(),(unsigned long)sec,
    id.c_str(),lat.c_str(),lon.c_str(),fix?1:0,msg.c_str());
  f.close();
}

bool initRadio(){
  digitalWrite(PIN_RADIO_CS,HIGH);
  select433Antenna();
  radioSPI.begin(SPI_SCK,SPI_MISO,SPI_MOSI,PIN_RADIO_CS);
  int16_t state=radio.begin(cfg.freqMHz,4.8,5.0,58.0,cfg.powerDbm,16);
  if(state!=RADIOLIB_ERR_NONE){ Serial.printf("CC1101 begin error %d\n",state); return false; }
  radio.setDataShaping(0);
  state=radio.setOOK(true);
  if(state!=RADIOLIB_ERR_NONE){ Serial.printf("CC1101 OOK error %d\n",state); return false; }
  pinMode(PIN_GDO0,OUTPUT); digitalWrite(PIN_GDO0,LOW);
  return true;
}

void reconfigureRadio(){
  if(!radioReady) return;
  radio.standby(); radio.packetMode();
  radio.setFrequency(cfg.freqMHz); radio.setOutputPower(cfg.powerDbm);
  radio.setBitRate(4.8); radio.setFrequencyDeviation(5.0); radio.setRxBandwidth(58.0);
  radio.setOOK(true); pinMode(PIN_GDO0,OUTPUT); digitalWrite(PIN_GDO0,LOW);
}

void ensureGpsPort(){
  if(gpsPortStarted) return;
  gpsSerial.begin(GPS_BAUD,SERIAL_8N1,PIN_GPS_RX,PIN_GPS_TX);
  gpsPortStarted=true;
}
void serviceGps(){
  if(!cfg.gpsEnabled) return;
  ensureGpsPort();
  while(gpsSerial.available()) gps.encode((char)gpsSerial.read());
}
bool gpsFixValid(){
  return cfg.gpsEnabled && gps.location.isValid() && gps.location.age()<=GPS_FIX_MAX_AGE_MS;
}
String gpsStatus(){
  if(!cfg.gpsEnabled) return "GPS: AUS";
  if(gps.charsProcessed()<10){
    if(millis()<5000) return "GPS: warte auf Daten";
    return "GPS: KEINE DATEN";
  }
  uint32_t sats=gps.satellites.isValid()?gps.satellites.value():0;
  if(gpsFixValid()) return "GPS: FIX | Sat "+String(sats);
  return "GPS: sucht | Sat "+String(sats);
}

void amToneFor(uint16_t hz,uint32_t durationMs){
  hz=max((uint16_t)100,hz);
  uint32_t halfUs=500000UL/hz;
  uint32_t endUs=micros()+durationMs*1000UL;
  while((int32_t)(micros()-endUs)<0){
    digitalWrite(PIN_GDO0,HIGH); delayMicroseconds(halfUs);
    digitalWrite(PIN_GDO0,LOW); delayMicroseconds(halfUs);
    serviceGps();
  }
  digitalWrite(PIN_GDO0,LOW);
}

const char* morseFor(char c){
  c=toupper((unsigned char)c);
  switch(c){
    case 'A':return ".-";case 'B':return "-...";case 'C':return "-.-.";case 'D':return "-..";case 'E':return ".";
    case 'F':return "..-.";case 'G':return "--.";case 'H':return "....";case 'I':return "..";case 'J':return ".---";
    case 'K':return "-.-";case 'L':return ".-..";case 'M':return "--";case 'N':return "-.";case 'O':return "---";
    case 'P':return ".--.";case 'Q':return "--.-";case 'R':return ".-.";case 'S':return "...";case 'T':return "-";
    case 'U':return "..-";case 'V':return "...-";case 'W':return ".--";case 'X':return "-..-";case 'Y':return "-.--";
    case 'Z':return "--..";case '0':return "-----";case '1':return ".----";case '2':return "..---";case '3':return "...--";
    case '4':return "....-";case '5':return ".....";case '6':return "-....";case '7':return "--...";case '8':return "---..";
    case '9':return "----.";case '.':return ".-.-.-";case '-':return "-....-";case '/':return "-..-.";
    default:return nullptr;
  }
}

uint32_t sendMorseAM(const String &text){
  uint32_t start=millis();
  uint16_t dot=1200U/max((uint8_t)5,cfg.wpm);
  for(size_t ci=0;ci<text.length();ci++){
    char c=text[ci];
    if(c==' '){ delay(dot*7); continue; }
    const char *m=morseFor(c); if(!m) continue;
    for(size_t i=0;m[i];i++){
      amToneFor(cfg.toneHz,m[i]=='.'?dot:dot*3);
      if(m[i+1]) delay(dot);
    }
    if(ci+1<text.length() && text[ci+1]!=' ') delay(dot*3);
  }
  return millis()-start;
}

String dataPayload(){
  String p="FX1|"+cfg.foxId+"|t="+String(elapsedSec())+"|n="+String(beaconCount)+"|m="+cfg.message;
  bool fix=gps.location.isValid() && gps.location.age()<5000;
  p+="|gps="+String(fix?1:0);
  if(cfg.includeGpsInData && fix) p+="|lat="+String(gps.location.lat(),6)+"|lon="+String(gps.location.lng(),6);
  if(p.length()>220) p.remove(220);
  return p;
}

uint32_t sendDataPacket(){
  uint32_t start=millis();
  radio.standby(); radio.packetMode(); radio.setOOK(false);
  radio.setFrequency(cfg.freqMHz); radio.setBitRate(4.8); radio.setFrequencyDeviation(5.0);
  radio.setRxBandwidth(58.0); radio.setOutputPower(cfg.powerDbm);
  String payload=dataPayload();
  radio.transmit(payload);
  radio.standby(); radio.setOOK(true);
  pinMode(PIN_GDO0,OUTPUT); digitalWrite(PIN_GDO0,LOW);
  return millis()-start;
}

uint32_t transmitBeacon(){
  if(!radioReady || webMode) return 0;
  uint32_t started=millis();
  txFlash(true);
  radio.standby(); radio.packetMode(); radio.setFrequency(cfg.freqMHz); radio.setOutputPower(cfg.powerDbm); radio.setOOK(true);
  pinMode(PIN_GDO0,OUTPUT); digitalWrite(PIN_GDO0,LOW);
  int16_t st=radio.transmitDirectAsync();
  if(st!=RADIOLIB_ERR_NONE){ radio.standby(); txFlash(false); return 0; }

  if(cfg.mode==MODE_PULSE) amToneFor(cfg.toneHz,cfg.pulseMs);
  else if(cfg.mode==MODE_MORSE) sendMorseAM(cfg.foxId);
  else { amToneFor(cfg.toneHz,cfg.pulseMs); delay(100); sendMorseAM(cfg.foxId); }

  if(cfg.messageEvery && ((beaconCount+1)%cfg.messageEvery)==0){
    delay(180); sendMorseAM(cfg.message);
  }

  digitalWrite(PIN_GDO0,LOW); radio.standby();
  if(cfg.dataBeacon && cfg.messageEvery && ((beaconCount+1)%cfg.messageEvery)==0){
    delay(80); sendDataPacket();
  }
  txFlash(false); beaconCount++;
  return millis()-started;
}

// Hard safety guard: total burst time must not exceed 10% of the overall cycle.
uint32_t guardedCycleMs(uint32_t txEnvelopeMs){
  if(!txEnvelopeMs) return cfg.intervalMs;
  uint32_t guardMs=txEnvelopeMs*10U;
  return cfg.intervalMs>guardMs?cfg.intervalMs:guardMs;
}

void returnToLauncher(){
  hunting=false;
  webMode=false;
  if(radioReady) radio.standby();
  digitalWrite(PIN_GDO0,LOW);
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wakeDisplay();
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_YELLOW,TFT_BLACK);
  tft.drawString("Zurueck zum Launcher...",160,78,4);
  pixels.clear(); pixels.show();
  delay(180);
  esp_sleep_enable_timer_wakeup(1000000ULL);
  delay(50);
  esp_deep_sleep_start();
}

void startHunt(){
  if(webMode) return;
  hunting=true; foundMarked=false; beaconCount=0; huntStartMs=millis(); nextTxMs=millis()+300;
  lastUserInputMs=millis(); totalHunts++; saveStats(); appendLog("START",0); allPixels(0,25,0);
}
void stopHunt(){
  uint32_t sec=hunting?(millis()-huntStartMs)/1000UL:lastFoundSec;
  hunting=false; if(radioReady) radio.standby(); digitalWrite(PIN_GDO0,LOW);
  appendLog("STOP",sec); allPixels(0,0,15); wakeDisplay();
}
void markFound(){
  if(!hunting) return;
  lastFoundSec=(millis()-huntStartMs)/1000UL; hunting=false; foundMarked=true; totalFound++;
  if(bestTimeSec==0 || lastFoundSec<bestTimeSec) bestTimeSec=lastFoundSec;
  if(radioReady) radio.standby(); digitalWrite(PIN_GDO0,LOW);
  saveStats(); appendLog("FOUND",lastFoundSec); wakeDisplay();
  for(int i=0;i<3;i++){ allPixels(0,100,0); delay(100); pixels.clear(); pixels.show(); delay(70); }
}

void drawHeader(const String &title,uint16_t color=TFT_ORANGE){
  tft.fillScreen(TFT_BLACK); tft.fillRect(0,0,320,24,TFT_DARKGREY);
  tft.setTextDatum(ML_DATUM); tft.setTextColor(color,TFT_DARKGREY); tft.drawString(title,8,12,2);
  tft.setTextDatum(MR_DATUM); tft.setTextColor(TFT_LIGHTGREY,TFT_DARKGREY); tft.drawString("v"+String(FW_VERSION),312,12,2);
}

void updateMainDynamic(){
  if(displaySleeping || inMenu || gpsScreen || foundMarked) return;

  // Only repaint changing rectangles. Full-screen redraws caused visible flicker.
  tft.fillRect(6,28,308,55,TFT_BLACK);
  tft.setTextDatum(TL_DATUM); tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.drawString(formatTime(elapsedSec()),10,34,6);

  tft.fillRect(6,130,308,31,TFT_BLACK);
  tft.setTextDatum(TL_DATUM); tft.setTextColor(gpsFixValid()?TFT_GREEN:(cfg.gpsEnabled?TFT_YELLOW:TFT_DARKGREY),TFT_BLACK);
  tft.drawString(gpsStatus(),10,133,2);
  if(gpsFixValid()){
    tft.setTextColor(TFT_LIGHTGREY,TFT_BLACK);
    tft.drawString(String(gps.location.lat(),5)+"  "+String(gps.location.lng(),5),10,151,1);
  } else if(cfg.gpsEnabled && gps.charsProcessed()>0){
    tft.setTextColor(TFT_DARKGREY,TFT_BLACK);
    tft.drawString("NMEA-Zeichen: "+String(gps.charsProcessed()),10,151,1);
  }

  tft.setTextDatum(TR_DATUM); tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.drawString(hunting?("TX #"+String(beaconCount)):("Best "+(bestTimeSec?formatTime(bestTimeSec):"--")),310,133,2);
}

void drawMain(){
  if(displaySleeping) return;
  drawHeader(hunting?"FUCHS AKTIV":(foundMarked?"FUCHS GEFUNDEN":"FUCHS BEREIT"),hunting?TFT_GREEN:TFT_ORANGE);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(TFT_CYAN,TFT_BLACK); tft.drawString(String(cfg.freqMHz,3)+" MHz",10,86,4);
  tft.setTextColor(TFT_WHITE,TFT_BLACK); tft.drawString("Modus: "+modeName()+"   Leistung: "+String(cfg.powerDbm)+" dBm",10,116,2);
  tft.setTextDatum(BR_DATUM); tft.setTextColor(TFT_DARKGREY,TFT_BLACK);
  tft.drawString("SIDE kurz Start | 1.2s FOUND | 2.5s Launcher | Encoder Menue",315,166,1);
  updateMainDynamic();
}

const char* menuLabels[]={
  "Sendemodus",
  "Sendefrequenz",
  "Sendeleistung",
  "Abstand zwischen Signalen",
  "Dauer kurzer Signalton",
  "Morse-Geschwindigkeit",
  "Tonhoehe",
  "Hinweis senden alle N Signale",
  "GPS-Empfaenger",
  "FoxLink-Zusatzdaten senden",
  "GPS-Position im FoxLink",
  "LEDs",
  "Display automatisch aus",
  "Display aus nach",
  "Jagd beim Einschalten starten",
  "Web-Konfiguration oeffnen",
  "GPS-Status anzeigen",
  "Jagdstatistik loeschen",
  "Zurueck zum Launcher",
  "Menue schliessen"
};
static constexpr int MENU_COUNT=sizeof(menuLabels)/sizeof(menuLabels[0]);

String menuValue(int i){
  switch(i){
    case 0:return modeName(); case 1:return String(cfg.freqMHz,3)+" MHz"; case 2:return String(cfg.powerDbm)+" dBm";
    case 3:return String(cfg.intervalMs)+" ms"; case 4:return String(cfg.pulseMs)+" ms"; case 5:return String(cfg.wpm)+" WPM";
    case 6:return String(cfg.toneHz)+" Hz"; case 7:return "jedes "+String(cfg.messageEvery)+".";
    case 8:return cfg.gpsEnabled?"AN":"AUS"; case 9:return cfg.dataBeacon?"AN":"AUS"; case 10:return cfg.includeGpsInData?"AN":"AUS";
    case 11:return cfg.ledsEnabled?"AN":"AUS"; case 12:return cfg.stealthEnabled?"AN":"AUS"; case 13:return String(cfg.stealthAfterSec)+" s";
    case 14:return cfg.autoStart?"AN":"AUS"; case 15:return "OEFFNEN"; case 16:return gpsFixValid()?"FIX":"ANZEIGEN";
    case 17:return "DRUECKEN"; case 18:return "DRUECKEN"; default:return "";
  }
}

void drawMenu(){
  if(displaySleeping) return;
  drawHeader("FUCHS-EINSTELLUNGEN",TFT_CYAN);
  int first=max(0,min(menuIndex-3,MENU_COUNT-7));
  for(int row=0;row<7;row++){
    int i=first+row; if(i>=MENU_COUNT) break; int y=28+row*20; bool sel=i==menuIndex;
    if(sel) tft.fillRoundRect(4,y,312,18,4,TFT_DARKGREY);
    tft.setTextDatum(ML_DATUM); tft.setTextColor(sel?TFT_YELLOW:TFT_WHITE,sel?TFT_DARKGREY:TFT_BLACK); tft.drawString(menuLabels[i],10,y+9,2);
    tft.setTextDatum(MR_DATUM); tft.setTextColor(TFT_CYAN,sel?TFT_DARKGREY:TFT_BLACK); tft.drawString(menuValue(i),310,y+9,2);
  }
}
void drawFound(){
  if(displaySleeping) return;
  drawHeader("GEFUNDEN!",TFT_GREEN); tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_GREEN,TFT_BLACK); tft.drawString(formatTime(lastFoundSec),160,70,7);
  tft.setTextColor(TFT_WHITE,TFT_BLACK); tft.drawString("Bestzeit: "+(bestTimeSec?formatTime(bestTimeSec):"--"),160,125,4);
  tft.setTextColor(TFT_LIGHTGREY,TFT_BLACK); tft.drawString(gpsStatus(),160,153,2);
}
void updateGpsScreen(){
  if(displaySleeping || !gpsScreen) return;
  tft.fillRect(0,25,320,145,TFT_BLACK);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(gpsFixValid()?TFT_GREEN:TFT_YELLOW,TFT_BLACK);
  tft.drawString(gpsStatus(),10,32,4);
  tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.drawString("NMEA-Zeichen: "+String(gps.charsProcessed()),10,66,2);
  tft.drawString("Satelliten: "+String(gps.satellites.isValid()?gps.satellites.value():0),10,86,2);
  if(gps.hdop.isValid()) tft.drawString("HDOP: "+String(gps.hdop.hdop(),1),170,86,2);
  if(gpsFixValid()){
    tft.drawString("Breite: "+String(gps.location.lat(),6),10,108,2);
    tft.drawString("Laenge: "+String(gps.location.lng(),6),10,128,2);
    if(gps.altitude.isValid()) tft.drawString("Hoehe: "+String(gps.altitude.meters(),0)+" m",10,148,2);
  } else {
    tft.setTextColor(TFT_LIGHTGREY,TFT_BLACK);
    tft.drawString(gps.charsProcessed()<10?"Keine seriellen GPS-Daten":"Daten da - warte auf Satellitenfix",10,112,2);
    tft.drawString("GPS-Modul braucht freie Sicht nach draussen.",10,136,1);
  }
  tft.setTextDatum(BR_DATUM); tft.setTextColor(TFT_DARKGREY,TFT_BLACK);
  tft.drawString("Encoder = zurueck | SIDE 2.5s = Launcher",315,166,1);
}
void drawGpsScreen(){
  if(displaySleeping) return;
  drawHeader("GPS-STATUS",TFT_CYAN);
  updateGpsScreen();
}
void redraw(){
  if(displaySleeping)return;
  if(gpsScreen) drawGpsScreen();
  else if(foundMarked&&!inMenu)drawFound();
  else if(inMenu)drawMenu();
  else drawMain();
}

String htmlEscape(String s){ s.replace("&","&amp;");s.replace("<","&lt;");s.replace(">","&gt;");s.replace("\"","&quot;");return s; }

String webPage(){
  String h=R"HTML(<!doctype html><html><head><meta name=viewport content='width=device-width,initial-scale=1'><style>
body{font-family:system-ui;background:#111;color:#eee;max-width:720px;margin:auto;padding:18px}h1{color:#ff9d00}.card{background:#191919;padding:15px;border-radius:12px;margin:12px 0}label{display:block;margin-top:12px;color:#aaa}input,select{width:100%;box-sizing:border-box;padding:10px;background:#222;color:#fff;border:1px solid #555;border-radius:8px}button{margin-top:18px;padding:12px 18px;border:0;border-radius:10px;background:#ff9d00;font-weight:bold}a{color:#6cf}small{color:#888}</style></head><body>)HTML";
  h+="<h1>Fox Ultimate</h1><div class=card><b>H4M:</b> AM "+String(cfg.freqMHz,3)+" MHz<br><b>ID:</b> "+htmlEscape(cfg.foxId)+"<br><b>"+gpsStatus()+"</b></div>";
  h+="<form method=post action=/save>";
  h+="<label>Fox ID / Morse name</label><input name=foxid maxlength=31 value=\""+htmlEscape(cfg.foxId)+"\">";
  h+="<label>Message / clue</label><input name=msg maxlength=79 value=\""+htmlEscape(cfg.message)+"\">";
  h+="<label>Frequency MHz</label><input name=freq type=number step=.001 min=433.050 max=434.790 value=\""+String(cfg.freqMHz,3)+"\">";
  h+="<label>Power dBm</label><select name=pwr>";
  const int powers[]={-30,-20,-15,-10,0,5,7,10}; for(int p:powers) h+="<option"+String(p==cfg.powerDbm?" selected":"")+">"+String(p)+"</option>"; h+="</select>";
  h+="<label>Mode</label><select name=mode><option value=0"+String(cfg.mode==MODE_PULSE?" selected":"")+">Pulse</option><option value=1"+String(cfg.mode==MODE_MORSE?" selected":"")+">Morse</option><option value=2"+String(cfg.mode==MODE_HYBRID?" selected":"")+">Hybrid</option></select>";
  h+="<label>Interval ms</label><input name=intv type=number min=500 max=120000 value=\""+String(cfg.intervalMs)+"\">";
  h+="<label>Pulse ms</label><input name=pulse type=number min=40 max=5000 value=\""+String(cfg.pulseMs)+"\">";
  h+="<label>Morse WPM</label><input name=wpm type=number min=5 max=40 value=\""+String(cfg.wpm)+"\">";
  h+="<label>AM tone Hz</label><input name=tone type=number min=300 max=1800 value=\""+String(cfg.toneHz)+"\">";
  h+="<label>Send message every N beacons</label><input name=msgN type=number min=1 max=50 value=\""+String(cfg.messageEvery)+"\">";
  auto cb=[&](const char*n,bool v,const char*l){ h+="<label><input style='width:auto' type=checkbox name="+String(n)+(v?" checked":"")+"> "+String(l)+"</label>"; };
  cb("gps",cfg.gpsEnabled,"GPS enabled"); cb("data",cfg.dataBeacon,"Structured data beacon"); cb("gpsTx",cfg.includeGpsInData,"Include GPS coordinates in data (reveals fox!)");
  cb("leds",cfg.ledsEnabled,"LEDs"); cb("stealth",cfg.stealthEnabled,"Stealth screen-off"); cb("auto",cfg.autoStart,"Auto-start hunt after boot");
  h+="<label>Stealth delay seconds</label><input name=stSec type=number min=5 max=600 value=\""+String(cfg.stealthAfterSec)+"\">";
  h+="<p><small>Automatic 10% duty guard is always active and may lengthen the interval after long Morse messages.</small></p><button>Save</button></form>";
  h+="<p><a href=/log>Hunt log</a> | <a href=/exit>Exit web config</a></p></body></html>";
  return h;
}

void handleSave(){
  cfg.foxId=server.arg("foxid").substring(0,31); if(!cfg.foxId.length()) cfg.foxId="FOX 1";
  cfg.message=server.arg("msg").substring(0,79); if(!cfg.message.length()) cfg.message="FIND ME";
  cfg.freqMHz=clampFreq(server.arg("freq").toFloat()); cfg.powerDbm=validPower(server.arg("pwr").toInt());
  cfg.mode=(BeaconMode)constrain(server.arg("mode").toInt(),0,2);
  cfg.intervalMs=constrain((uint32_t)server.arg("intv").toInt(),500U,120000U);
  cfg.pulseMs=clampU16(server.arg("pulse").toInt(),40,5000); cfg.wpm=constrain(server.arg("wpm").toInt(),5,40);
  cfg.toneHz=clampU16(server.arg("tone").toInt(),300,1800); cfg.messageEvery=constrain(server.arg("msgN").toInt(),1,50);
  cfg.gpsEnabled=server.hasArg("gps"); cfg.dataBeacon=server.hasArg("data"); cfg.includeGpsInData=server.hasArg("gpsTx");
  cfg.ledsEnabled=server.hasArg("leds"); cfg.stealthEnabled=server.hasArg("stealth"); cfg.autoStart=server.hasArg("auto");
  cfg.stealthAfterSec=constrain(server.arg("stSec").toInt(),5,600);
  saveSettings(); reconfigureRadio();
  server.send(200,"text/html","<html><body><h2>Saved</h2><p><a href='/'>Back</a></p></body></html>");
}

void exitWebMode(){
  server.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); webMode=false; inMenu=false; wakeDisplay(); reconfigureRadio(); redraw();
}
void startWebMode(){
  if(hunting) stopHunt();
  webMode=true; inMenu=false; wakeDisplay(); if(radioReady)radio.standby();
  WiFi.mode(WIFI_AP); WiFi.softAP("FoxUltimate","foxhunt42");
  server.on("/",HTTP_GET,[]{server.send(200,"text/html",webPage());});
  server.on("/save",HTTP_POST,handleSave);
  server.on("/log",HTTP_GET,[]{ if(!LittleFS.exists("/huntlog.csv")){server.send(200,"text/plain","No log yet\n");return;} File f=LittleFS.open("/huntlog.csv","r"); server.streamFile(f,"text/csv"); f.close(); });
  server.on("/exit",HTTP_GET,[]{ server.send(200,"text/html","<h2>Closed</h2>"); delay(100); exitWebMode(); });
  server.begin();
  drawHeader("WEB CONFIG",TFT_MAGENTA); tft.setTextDatum(MC_DATUM); tft.setTextColor(TFT_WHITE,TFT_BLACK);
  tft.drawString("Wi-Fi: FoxUltimate",160,62,4); tft.drawString("Password: foxhunt42",160,94,2);
  tft.setTextColor(TFT_CYAN,TFT_BLACK); tft.drawString("192.168.4.1",160,122,4);
  tft.setTextColor(TFT_LIGHTGREY,TFT_BLACK); tft.drawString("SIDE = exit",160,154,2); allPixels(40,0,40);
}

void cyclePower(){ static const int8_t p[]={-30,-20,-15,-10,0,5,7,10}; int idx=0; for(int i=0;i<8;i++)if(p[i]==cfg.powerDbm)idx=i; cfg.powerDbm=p[(idx+1)%8]; }

void activateMenuItem(){
  switch(menuIndex){
    case 0:cfg.mode=(BeaconMode)((cfg.mode+1)%3);break;
    case 1:cfg.freqMHz+=.025f;if(cfg.freqMHz>FREQ_MAX)cfg.freqMHz=FREQ_MIN;break;
    case 2:cyclePower();break;
    case 3:cfg.intervalMs+=500;if(cfg.intervalMs>30000)cfg.intervalMs=1000;break;
    case 4:cfg.pulseMs+=50;if(cfg.pulseMs>1000)cfg.pulseMs=100;break;
    case 5:cfg.wpm+=2;if(cfg.wpm>30)cfg.wpm=8;break;
    case 6:cfg.toneHz+=100;if(cfg.toneHz>1500)cfg.toneHz=400;break;
    case 7:cfg.messageEvery++;if(cfg.messageEvery>20)cfg.messageEvery=1;break;
    case 8:
      cfg.gpsEnabled=!cfg.gpsEnabled;
      if(cfg.gpsEnabled) ensureGpsPort();
      break;
    case 9:cfg.dataBeacon=!cfg.dataBeacon;break;
    case 10:cfg.includeGpsInData=!cfg.includeGpsInData;break;
    case 11:cfg.ledsEnabled=!cfg.ledsEnabled;break;
    case 12:cfg.stealthEnabled=!cfg.stealthEnabled;break;
    case 13:cfg.stealthAfterSec+=5;if(cfg.stealthAfterSec>60)cfg.stealthAfterSec=5;break;
    case 14:cfg.autoStart=!cfg.autoStart;break;
    case 15:saveSettings();startWebMode();return;
    case 16:
      saveSettings(); inMenu=false; gpsScreen=true; ensureGpsPort(); drawGpsScreen(); return;
    case 17:
      bestTimeSec=lastFoundSec=totalHunts=totalFound=0;saveStats();LittleFS.remove("/huntlog.csv");break;
    case 18:
      saveSettings(); returnToLauncher(); return;
    case 19:
      inMenu=false;saveSettings();reconfigureRadio();redraw();return;
  }
  saveSettings(); reconfigureRadio(); redraw();
}

void encoderStep(int delta){ wakeDisplay(); if(!inMenu)return; menuIndex=(menuIndex+delta+MENU_COUNT)%MENU_COUNT; drawMenu(); }

void pollInputs(){
  int a=digitalRead(PIN_ENC_A);
  if(a!=lastEncA && a==LOW) encoderStep(digitalRead(PIN_ENC_B)!=a?1:-1);
  lastEncA=a;

  bool encDown=digitalRead(PIN_ENC_KEY)==LOW;
  if(encDown&&!encWasDown){encPressStart=millis();encWasDown=true;wakeDisplay();}
  if(!encDown&&encWasDown){
    uint32_t held=millis()-encPressStart;encWasDown=false;
    if(gpsScreen){gpsScreen=false;redraw();return;}
    if(held>1000){cfg.stealthEnabled=!cfg.stealthEnabled;saveSettings();}
    else{if(!inMenu){inMenu=true;foundMarked=false;}else activateMenuItem();}
    redraw();
  }

  bool sideDown=digitalRead(PIN_SIDE_KEY)==LOW;
  if(sideDown&&!sideWasDown){sidePressStart=millis();sideWasDown=true;wakeDisplay();}
  if(!sideDown&&sideWasDown){
    uint32_t held=millis()-sidePressStart;sideWasDown=false;
    if(held>=SIDE_LAUNCHER_MS){returnToLauncher();return;}
    if(webMode){exitWebMode();return;}
    if(gpsScreen){gpsScreen=false;redraw();return;}
    if(held>=SIDE_FOUND_MS){markFound();inMenu=false;}
    else{if(hunting)stopHunt();else startHunt();inMenu=false;foundMarked=false;}
    redraw();
  }
}

void setup(){
  Serial.begin(115200);delay(100);
  pinMode(PIN_PWR_EN,OUTPUT);digitalWrite(PIN_PWR_EN,HIGH);
  pinMode(PIN_SD_CS,OUTPUT);digitalWrite(PIN_SD_CS,HIGH);
  pinMode(PIN_RADIO_CS,OUTPUT);digitalWrite(PIN_RADIO_CS,HIGH);
  pinMode(PIN_ENC_A,INPUT_PULLUP);pinMode(PIN_ENC_B,INPUT_PULLUP);pinMode(PIN_ENC_KEY,INPUT_PULLUP);pinMode(PIN_SIDE_KEY,INPUT_PULLUP);
  pinMode(PIN_TFT_BL,OUTPUT);digitalWrite(PIN_TFT_BL,HIGH);lastEncA=digitalRead(PIN_ENC_A);

  pixels.begin();pixels.clear();pixels.show();loadSettings();LittleFS.begin(true);
  tft.begin();tft.setRotation(3);tft.setSwapBytes(true);tft.fillScreen(TFT_BLACK);
  drawHeader("FOX ULTIMATE",TFT_ORANGE);tft.setTextDatum(MC_DATUM);tft.setTextColor(TFT_WHITE,TFT_BLACK);tft.drawString("Booting...",160,82,4);

  ensureGpsPort();
  radioReady=initRadio();
  if(!radioReady){tft.setTextColor(TFT_RED,TFT_BLACK);tft.drawString("CC1101 ERROR",160,120,4);}
  delay(400);lastUserInputMs=millis();redraw();allPixels(0,0,15);
  if(cfg.autoStart&&radioReady)startHunt();
}

void loop(){
  serviceGps();pollInputs();
  if(webMode){server.handleClient();delay(2);return;}
  if(hunting&&radioReady&&(int32_t)(millis()-nextTxMs)>=0){
    uint32_t txMs=transmitBeacon();
    nextTxMs=millis()+guardedCycleMs(txMs);
    if(!displaySleeping)updateMainDynamic();
  }
  maybeSleepDisplay();
  static uint32_t lastDraw=0;
  if(!displaySleeping&&!inMenu&&millis()-lastDraw>1000){
    lastDraw=millis();
    if(gpsScreen) updateGpsScreen();
    else if(foundMarked) { /* static result screen - no full redraw needed */ }
    else updateMainDynamic();
  }
  delay(2);
}
