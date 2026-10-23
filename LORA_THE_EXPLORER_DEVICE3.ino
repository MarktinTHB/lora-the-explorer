// Current Version: v2.3.1.1
// Fixed ping replies and messages, compass label/circle overlap, and the device list format.
// Ping: "Sending..." blocks repeat taps, TIMEOUT message removed, receiver ACKs before redrawing, broadcast pings now get staggered ACKs, and sender/receiver show "You pinged Lora #N (X ms)" / "Lora #N pinged you (X ms)"; compass radius 80 -> 76 px with letters kept at the same 93 px orbit and a tight letter erase box; rows now read "LORA #N | D meters away! | DIR".

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <XPT2046_Touchscreen.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <math.h>
#include <Preferences.h>

// Arduino entry points
void setup();
void loop();

// =====================================================
// DEVICE
// =====================================================

#define DEVICE_ID 3

// =====================================================
// TFT SPI
// =====================================================

#define TFT_SCK   18
#define TFT_MISO  19
#define TFT_MOSI  23
#define TFT_CS    5
#define TFT_DC    2
#define TFT_RST   4

// =====================================================
// XPT2046 TOUCH
// =====================================================

#define TOUCH_CS  21
XPT2046_Touchscreen touch(TOUCH_CS);

// =====================================================
// RFM95W / LORA
// =====================================================

#define LORA_CS    27
#define LORA_RST   32
#define LORA_DIO0  26
#define LORA_FREQ  915E6

// =====================================================
// NEO-6M GPS
// VCC -> 5V, GND -> GND, RX -> GPIO16, TX -> GPIO17
// =====================================================

#define GPS_RX 16
#define GPS_TX 17
#define GPS_BAUD 9600

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

// =====================================================
// MAGNETOMETER
// Supports QMC5883L (0x0D) and HMC5883L (0x1E)
// SDA -> GPIO13, SCL -> GPIO22 (moved off GPIO12: strapping pin, breaks flash boot)
// =====================================================

#define MAG_SDA 13
#define MAG_SCL 22
#define QMC_ADDR 0x0D
#define HMC_ADDR 0x1E

// Calibration supplied by the user.
const float MAG_X_OFFSET = (-1811.0f + 1475.0f) / 2.0f;
const float MAG_Y_OFFSET = (-2355.0f + 585.0f) / 2.0f;
const float MAG_DECLINATION = 0.0f;

// Active compass calibration: X/Y offsets (hard iron) and X/Y scales (soft
// iron). Replaced by this board's own calibration if one was saved.
float magCal[4] = {MAG_X_OFFSET, MAG_Y_OFFSET, 1.0f, 1.0f};

enum MagnetometerType {
  MAG_NONE,
  MAG_QMC5883L,
  MAG_HMC5883L
};

MagnetometerType magType = MAG_NONE;
bool magnetometerPresent = false;

int16_t magX = 0;
int16_t magY = 0;
int16_t magZ = 0;
float heading = 0.0f;
String directionShort = "N";
String directionLong = "North";

// =====================================================
// COLORS
// =====================================================

#define BLACK       0x0000
#define WHITE       0xFFFF
#define RED         0xF800
#define GREEN       0x07E0
#define BLUE        0x001F
#define YELLOW      0xFFE0
#define GRAY        0x8410
#define DARKGRAY    0x4208
#define LIGHTGREEN  0x7ECA
#define LIGHTBLUE   0xE0C3
#define LOCATION_GREEN 0xFF47

// =====================================================
// SCREEN
// =====================================================

#define SCREEN_W 240
#define SCREEN_H 320

// =====================================================
// UI SCREENS
// =====================================================

enum Screen {
  MAIN_MENU,
  PEER_LOCATION_SCREEN
};

Screen currentScreen = MAIN_MENU;

// =====================================================
// GPS LOCAL DATA
// =====================================================

double latitude = 0.0;
double longitude = 0.0;
bool gpsHasFix = false;

int gpsHour = 0;
int gpsMinute = 0;
int gpsSecond = 0;
int gpsDay = 0;
int gpsMonth = 0;
int gpsYear = 0;

// =====================================================
// REMOTE CLOCK FROM LORA
// Used only when local GPS clock is unavailable.
// =====================================================

bool remoteTimeValid = false;
int remoteHour = 0;
int remoteMinute = 0;
int remoteSecond = 0;
int remoteDay = 0;
int remoteMonth = 0;
int remoteYear = 0;
unsigned long remoteSyncMillis = 0;

// =====================================================
// REMOTE DEVICES
// =====================================================

struct RemoteDevice {
  bool online = false;
  bool asleep = false;
  bool gpsValid = false;
  bool headingValid = false;
  double lat = 0.0;
  double lon = 0.0;
  float remoteHeading = 0.0f;
  unsigned long lastSeen = 0;
  unsigned long locationUpdatedAt = 0;
  uint8_t heartbeatCount = 0;
};

RemoteDevice remote[4];

#define HEARTBEAT_INTERVAL 3000UL
// An awake device is considered offline after 10 seconds of no
// heartbeat/ping communication (about 3 missed heartbeats).
#define DEVICE_TIMEOUT 10000UL
// Sleeping devices report every 5 minutes. Allow two missed reports before
// considering the sleeping device truly offline.
#define SLEEP_LOCATION_INTERVAL 300000UL
#define SLEEP_DEVICE_TIMEOUT 720000UL
#define ONLINE_CONFIRM_PACKETS 2
#define PEER_CIRCLE_CENTER_X 120
#define PEER_CIRCLE_CENTER_Y 124
// Reduced from 80 so the rotating N/E/S/W letters (kept on the same 93 px
// orbit, see PEER_CARDINAL_ORBIT) never touch the circle outline.
#define PEER_CIRCLE_RADIUS 76
#define PEER_CARDINAL_ORBIT (PEER_CIRCLE_RADIUS + 17)
#define PEER_MAX_DISPLAY_METERS 100.0f
unsigned long nextHeartbeat = 0;

// =====================================================
// SLEEP MODE
// =====================================================
// Disabled by default. While enabled, normal heartbeats stop and a
// compact sleep/location report is sent immediately and every 5 minutes.
bool sleepMode = false;
bool radioSleeping = false;
unsigned long nextSleepLocationReport = 0;

// When THIS device enters Sleep Mode, keep the last known remote statuses
// frozen on its screen instead of turning them OFFLINE while the radio sleeps.
bool remoteStatusFrozen = false;
bool resumeStatusSync = false;
unsigned long resumeStatusSyncUntil = 0;
bool cachedRemoteOnline[4] = {false, false, false, false};
bool cachedRemoteAsleep[4] = {false, false, false, false};

// =====================================================
// PING
// =====================================================

String pingMessage = "";
unsigned long pingMessageTime = 0;
#define PING_MESSAGE_TIME 5000UL
// How long "Sending..." stays up waiting for the reply. There is no TIMEOUT
// message; this only stops a lost reply from locking the ping button.
#define PING_REPLY_WINDOW 3000UL
// Broadcast replies arrive in per-device slots (ID x 250 ms); this window
// collects all of them before "You pinged everyone" is shown.
#define PING_BROADCAST_WINDOW 1200UL
#define PING_BROADCAST_SLOT 250UL
uint16_t pingSequence = 0;
bool pingPending = false;
int pendingPingTarget = 0;
bool pendingPingBroadcast = false;
uint16_t pendingPingSequence = 0;
unsigned long pendingPingStartedAt = 0;
bool broadcastAckSeen[4] = {false, false, false, false};
uint8_t broadcastAckCount = 0;
// Broadcast pings are acknowledged later, in this device's own time slot,
// so the replies from two receivers do not collide on air.
struct ScheduledPingAck {
  bool pending = false;
  uint16_t sequence = 0;
  unsigned long receivedAt = 0;
  unsigned long dueAt = 0;
};
ScheduledPingAck scheduledPingAck[4];
bool receivedPingSequenceValid[4][4] = {};
uint16_t receivedPingSequences[4][4] = {};
uint8_t receivedPingSequenceNext[4] = {0, 0, 0, 0};
unsigned long lastDistanceLog[4] = {0, 0, 0, 0};

// Optional LoRa airtime diagnostic; this is not measured ping latency.
float estimateLoRaToAMs(uint8_t payloadBytes) {
  const float bandwidth = 125000.0f;
  const int sf = 7;
  const int cr = 1;       // 4/5
  const int ih = 0;       // explicit header
  const int de = 0;       // SF7
  const int crc = 1;
  const int preamble = 8;

  const float symbolTimeMs = ((float)(1UL << sf) / bandwidth) * 1000.0f;
  const int numerator = 8 * (int)payloadBytes - 4 * sf + 28 + 16 * crc - 20 * ih;
  const int denominator = 4 * (sf - 2 * de);

  int payloadSymbols = 0;
  if (numerator > 0) {
    const int ceilSymbols = (numerator + denominator - 1) / denominator;
    payloadSymbols = ceilSymbols * (4 + cr);
  }
  payloadSymbols += 8;

  return (preamble + 4.25f + payloadSymbols) * symbolTimeMs;
}

// =====================================================
// TOUCH
// 9-point calibration previously supplied by the user.
// Targets are treated as a 3x3 screen grid.
// =====================================================

// These coefficients are an affine fit of the previously provided 9-point
// raw readings to screen coordinates approximately at:
// (20,20), (120,20), (220,20),
// (20,160), (120,160), (220,160),
// (20,300), (120,300), (220,300).
const float TOUCH_SX_A = -0.00139430734f;
const float TOUCH_SX_B =  0.0732204386f;
const float TOUCH_SX_C = -28.8373901f;
const float TOUCH_SY_A = -0.0971957788f;
const float TOUCH_SY_B = -0.00210196664f;
const float TOUCH_SY_C = 368.197823f;

// Active touch calibration: screenX = c[0]*rawX + c[1]*rawY + c[2],
// screenY = c[3]*rawX + c[4]*rawY + c[5]. Starts from the defaults above and
// is replaced by this board's own calibration if one was saved.
float touchCal[6] = {
  TOUCH_SX_A, TOUCH_SX_B, TOUCH_SX_C,
  TOUCH_SY_A, TOUCH_SY_B, TOUCH_SY_C
};


// 13x13 status indicator bitmap from the supplied UI.
static const unsigned char PROGMEM image_activeDev3_bits[] = {
  0x1f,0xc0,0x3f,0xe0,0x70,0x70,0xef,0xb8,0xdf,0xd8,0xdf,0xd8,
  0xdf,0xd8,0xdf,0xd8,0xdf,0xd8,0xef,0xb8,0x70,0x70,0x3f,0xe0,0x1f,0xc0
};

// =====================================================
// FORWARD DECLARATIONS
// =====================================================

class WorkingILI9341;

void drawMainMenu();
void updateMainMenuDynamic(bool force);
void drawPeerLocation();
void updatePeerLocationDynamic(bool force);
void processTouch();
void loadMagCalibration();
void runMagCalibration();
void loadTouchCalibration();
void runTouchCalibration();

void updateGPS();
void updateCompass();
void detectMagnetometer();
void configureQMC5883L();
void configureHMC5883L();
bool readMagnetometer();
float calculateHeading();
void updateDirectionStrings();

void sendHeartbeat();
void requestStatusSync();
void sendStateResponse(int targetId);
void sendSleepLocationReport();
void setSleepMode(bool enabled);
void sendPing(int targetId);
void updatePing();
void acknowledgePing(int requesterId, int targetId, uint16_t sequence,
                     unsigned long receivedAt);
void receiveLoRa();
void handlePacketTest(const String &packet);
void runPacketTest();
void updateDeviceStatus();

void getDisplayClock(int &hour, int &minute, int &second, int &day, int &month, int &year, bool &valid);
void advanceRemoteClock();
void addSecondsToDate(int secondsToAdd, int &hour, int &minute, int &second, int &day, int &month, int &year);
bool isLeapYear(int year);
int daysInMonth(int month, int year);

float bearingToPeer(double peerLat, double peerLon);
float distanceToPeer(double peerLat, double peerLon);
float normalizeBearing(float degrees);
bool validCoordinates(double lat, double lon);
bool localLocationValid();
bool remoteLocationFresh(int deviceId);
void updateRemoteLocation(int deviceId, bool gpsValid, double lat, double lon,
                          bool headingValid, float remoteHeading);
void logPeerMeasurement(int deviceId, float bearing, float distance);
int relativeBearingDegrees(double peerLat, double peerLon);
bool parsePingSequence(const String &text, uint16_t &sequence);
void drawPeerDot(int deviceId, double peerLat, double peerLon, uint16_t color);

String shortDirection(float degrees);
String longDirection(float degrees);
bool cardinalActive(const char *cardinal, float degrees);
uint16_t deviceColor(int id);
void drawCenteredText(const String &text, int y, uint16_t fg, uint16_t bg, uint8_t size);
void drawCardinalLetter(const char *letter, int x, int y, bool active);
void clearPeerDynamicObjects();
void drawPeerDynamicObjects();
void drawPeerLegendLine(int deviceId, int y);

// =====================================================
// CUSTOM ILI9341
// =====================================================

class WorkingILI9341 : public Adafruit_GFX {
public:
  WorkingILI9341() : Adafruit_GFX(SCREEN_W, SCREEN_H) {}

  void beginDisplay() {
    pinMode(TFT_CS, OUTPUT);
    pinMode(TFT_DC, OUTPUT);
    pinMode(TFT_RST, OUTPUT);

    digitalWrite(TFT_CS, HIGH);
    digitalWrite(TFT_DC, HIGH);
    digitalWrite(TFT_RST, HIGH);

    delay(5);
    digitalWrite(TFT_RST, LOW);
    delay(15);
    digitalWrite(TFT_RST, HIGH);
    delay(15);

    initController();
  }

  // Read the ILI9341 power mode register (0x0A). A healthy, awake display
  // returns 0x9C (booster on, sleep out, display on). If the controller was
  // reset by a power dip it falls back to sleep mode (e.g. 0x08).
  uint8_t readPowerMode() {
    SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    selectTFT();
    digitalWrite(TFT_DC, LOW);  SPI.transfer(0xD9);
    digitalWrite(TFT_DC, HIGH); SPI.transfer(0x10);
    digitalWrite(TFT_DC, LOW);  SPI.transfer(0x0A);
    digitalWrite(TFT_DC, HIGH);
    uint8_t value = SPI.transfer(0x00);
    deselectTFT();
    SPI.endTransaction();
    return value;
  }

  void fillScreen(uint16_t color) override {
    fillRect(0, 0, SCREEN_W, SCREEN_H, color);
  }

  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    if (w <= 0 || h <= 0) return;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > SCREEN_W) w = SCREEN_W - x;
    if (y + h > SCREEN_H) h = SCREEN_H - y;
    if (w <= 0 || h <= 0) return;

    selectTFT();
    setAddressWindow(x, y, x + w - 1, y + h - 1);
    digitalWrite(TFT_DC, HIGH);

    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    uint32_t count = (uint32_t)w * (uint32_t)h;
    for (uint32_t i = 0; i < count; ++i) {
      SPI.transfer(hi);
      SPI.transfer(lo);
    }
    deselectTFT();
  }

protected:
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || x >= SCREEN_W || y < 0 || y >= SCREEN_H) return;

    selectTFT();
    setAddressWindow(x, y, x, y);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(color >> 8);
    SPI.transfer(color & 0xFF);
    deselectTFT();
  }

private:
  void selectTFT() {
    digitalWrite(TOUCH_CS, HIGH);
    digitalWrite(LORA_CS, HIGH);
    digitalWrite(TFT_CS, LOW);
  }

  void deselectTFT() {
    digitalWrite(TFT_CS, HIGH);
  }

  void setAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    digitalWrite(TFT_DC, LOW);
    SPI.transfer(0x2A);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(x0 >> 8);
    SPI.transfer(x0 & 0xFF);
    SPI.transfer(x1 >> 8);
    SPI.transfer(x1 & 0xFF);

    digitalWrite(TFT_DC, LOW);
    SPI.transfer(0x2B);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(y0 >> 8);
    SPI.transfer(y0 & 0xFF);
    SPI.transfer(y1 >> 8);
    SPI.transfer(y1 & 0xFF);

    digitalWrite(TFT_DC, LOW);
    SPI.transfer(0x2C);
  }

  void initController() {
    digitalWrite(TFT_CS, LOW);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xCB);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x39); SPI.transfer(0x2C); SPI.transfer(0x00); SPI.transfer(0x34); SPI.transfer(0x02);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xCF);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x00); SPI.transfer(0xC1); SPI.transfer(0x30);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xE8);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x85); SPI.transfer(0x00); SPI.transfer(0x78);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xEA);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x00); SPI.transfer(0x00);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xED);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x64); SPI.transfer(0x03); SPI.transfer(0x12); SPI.transfer(0x81);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xF7);
    digitalWrite(TFT_DC, HIGH); SPI.transfer(0x20);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xC0);
    digitalWrite(TFT_DC, HIGH); SPI.transfer(0x23);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xC1);
    digitalWrite(TFT_DC, HIGH); SPI.transfer(0x10);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xC5);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x3E); SPI.transfer(0x28);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xC7);
    digitalWrite(TFT_DC, HIGH); SPI.transfer(0x86);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0x36);
    digitalWrite(TFT_DC, HIGH); SPI.transfer(0x48);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0x3A);
    digitalWrite(TFT_DC, HIGH); SPI.transfer(0x55);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xB1);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x00); SPI.transfer(0x18);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0xB6);
    digitalWrite(TFT_DC, HIGH);
    SPI.transfer(0x08); SPI.transfer(0x82); SPI.transfer(0x27);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0x11);
    delay(120);

    digitalWrite(TFT_DC, LOW); SPI.transfer(0x29);
    delay(20);

    digitalWrite(TFT_CS, HIGH);
  }
};

WorkingILI9341 tft;

// =====================================================
// UI CACHE
// =====================================================

String lastMainTime = "";
String lastMainDate = "";
String lastMainPing = "";
String lastMainSleepMessage = "";
bool lastMainOnline[4] = {false, false, false, false};
bool lastMainAsleep[4] = {false, false, false, false};

String lastPeerLocationText = "";
String lastPeerHeadingText = "";
String lastPeerDistance2 = "";
String lastPeerDistance3 = "";
String lastPeerDistanceDisplay[4] = {"", "", "", ""};
float lastPeerHeading = -999.0f;
String lastPeerDirection = "";
bool lastPeerGpsFix = false;
bool lastPeerGpsFresh[4] = {false, false, false, false};
bool lastPeerMagFound = false;
float lastPeer1Lat = 0.0f;
float lastPeer1Lon = 0.0f;
bool lastPeer1Online = false;
bool lastPeer1Asleep = false;
bool lastPeer1Gps = false;
float lastPeer2Lat = 0.0f;
float lastPeer2Lon = 0.0f;
float lastPeer3Lat = 0.0f;
float lastPeer3Lon = 0.0f;
bool lastPeer2Online = false;
bool lastPeer3Online = false;
bool lastPeer2Asleep = false;
bool lastPeer3Asleep = false;
bool lastPeer2Gps = false;
bool lastPeer3Gps = false;
double lastPeerOwnLat = 0.0;
double lastPeerOwnLon = 0.0;

// Dynamic objects inside the peer circle. The outer circle is NEVER redrawn.
bool peerVisualInitialized = false;
struct PeerDotCache {
  bool valid = false;
  int x = 0;
  int y = 0;
  uint16_t color = BLACK;
};
PeerDotCache lastPeerDot[4];
struct CardinalLabelCache {
  bool valid = false;
  int x = 0;
  int y = 0;
};
CardinalLabelCache lastCardinalLabel[4];

unsigned long lastMainDynamic = 0;
unsigned long lastPeerDynamic = 0;
#define MAIN_DYNAMIC_INTERVAL 50UL
#define PEER_DYNAMIC_INTERVAL 100UL

// Force a full redraw of the current screen's dynamic areas every 10 seconds
// so device ONLINE/OFFLINE status is always up to date.
unsigned long lastScreenRefresh = 0;
#define SCREEN_REFRESH_INTERVAL 10000UL

// Power mode read right after init. The health check is only trusted when
// this baseline is valid (TFT MISO wired and readable).
uint8_t tftHealthyMode = 0;

// =====================================================
// SETUP
// =====================================================



// =====================================================
// LOOP
// =====================================================



// =====================================================
// GPS
// =====================================================

void updateGPS() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid() && gps.location.age() <= 5000UL &&
      validCoordinates(gps.location.lat(), gps.location.lng())) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gpsHasFix = true;
  } else {
    gpsHasFix = false;
  }

  if (gps.time.isValid()) {
    gpsHour = gps.time.hour();
    gpsMinute = gps.time.minute();
    gpsSecond = gps.time.second();
  }

  if (gps.date.isValid()) {
    gpsDay = gps.date.day();
    gpsMonth = gps.date.month();
    gpsYear = gps.date.year();
  }
}

bool validCoordinates(double lat, double lon) {
  return isfinite(lat) && isfinite(lon) && lat >= -90.0 && lat <= 90.0 &&
         lon >= -180.0 && lon <= 180.0 && !(lat == 0.0 && lon == 0.0);
}

bool localLocationValid() {
  return gpsHasFix && gps.location.isValid() && gps.location.age() <= 5000UL &&
         validCoordinates(latitude, longitude);
}

bool remoteLocationFresh(int deviceId) {
  if (deviceId < 1 || deviceId > 3 || deviceId == DEVICE_ID ||
      !remote[deviceId].online || !remote[deviceId].gpsValid ||
      remote[deviceId].locationUpdatedAt == 0) return false;
  unsigned long maxAge = remote[deviceId].asleep ? 360000UL : 15000UL;
  return millis() - remote[deviceId].locationUpdatedAt <= maxAge;
}

void updateRemoteLocation(int deviceId, bool hasFix, double lat, double lon,
                          bool hasHeading, float peerHeading) {
  bool coordinatesOk = hasFix && validCoordinates(lat, lon);
  remote[deviceId].gpsValid = coordinatesOk;
  if (coordinatesOk) {
    remote[deviceId].lat = lat;
    remote[deviceId].lon = lon;
    remote[deviceId].locationUpdatedAt = millis();
  }
  remote[deviceId].headingValid = hasHeading && isfinite(peerHeading) &&
                                  peerHeading >= 0.0f && peerHeading < 360.0f;
  if (remote[deviceId].headingValid) remote[deviceId].remoteHeading = peerHeading;
}

float normalizeBearing(float degrees) {
  while (degrees < 0.0f) degrees += 360.0f;
  while (degrees >= 360.0f) degrees -= 360.0f;
  return degrees;
}

void logPeerMeasurement(int deviceId, float bearing, float distance) {
  unsigned long now = millis();
  if (now - lastDistanceLog[deviceId] < 5000UL) return;
  lastDistanceLog[deviceId] = now;
  Serial.printf("GPS DIST local=%d %.6f,%.6f fix=%d peer=%d %.6f,%.6f fix=%d age_ms=%lu bearing_deg=%.1f distance_m=%.1f\n",
                DEVICE_ID, latitude, longitude, localLocationValid(), deviceId,
                remote[deviceId].lat, remote[deviceId].lon,
                remoteLocationFresh(deviceId),
                now - remote[deviceId].locationUpdatedAt, bearing, distance);
}

int relativeBearingDegrees(double peerLat, double peerLon) {
  int degrees = (int)roundf(normalizeBearing(bearingToPeer(peerLat, peerLon) - heading));
  return degrees >= 360 ? 0 : degrees;
}

bool parsePingSequence(const String &text, uint16_t &sequence) {
  if (text.length() == 0 || text.length() > 5) return false;
  uint32_t value = 0;
  for (unsigned int i = 0; i < text.length(); ++i) {
    char digit = text.charAt(i);
    if (digit < '0' || digit > '9') return false;
    value = value * 10UL + (uint32_t)(digit - '0');
    if (value > 65535UL) return false;
  }
  if (value == 0) return false;
  sequence = (uint16_t)value;
  return true;
}

// =====================================================
// MAGNETOMETER AUTO-DETECTION
// =====================================================

bool probeI2C(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void printI2CScan() {
  static bool scanned = false;
  if (scanned) return;
  scanned = true;

  Serial.println("I2C scan:");
  bool found = false;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  Found I2C device at 0x%02X\n", addr);
      found = true;
    }
  }
  if (!found) Serial.println("  No I2C devices found.");
}

// The I2C driver can freeze if SDA/SCL are stuck LOW (magnetometer not
// powered or miswired), so only start I2C once both idle lines read HIGH.
bool i2cStarted = false;

bool startI2CIfLinesOk() {
  if (i2cStarted) return true;

  static int lastSda = -1, lastScl = -1;
  pinMode(MAG_SDA, INPUT);
  pinMode(MAG_SCL, INPUT);
  int sda = digitalRead(MAG_SDA);
  int scl = digitalRead(MAG_SCL);

  if (sda != lastSda || scl != lastScl) {
    Serial.printf("I2C lines: SDA(G%d)=%d SCL(G%d)=%d%s\n",
                  MAG_SDA, sda, MAG_SCL, scl,
                  (sda && scl) ? "" : "  -> LOW, check magnetometer wiring/power");
    lastSda = sda;
    lastScl = scl;
  }
  if (!sda || !scl) return false;

  Wire.begin(MAG_SDA, MAG_SCL);
  Wire.setClock(400000);
  Wire.setTimeOut(20);
  i2cStarted = true;
  return true;
}

void detectMagnetometer() {
  magType = MAG_NONE;
  magnetometerPresent = false;

  if (!startI2CIfLinesOk()) return;

  if (probeI2C(QMC_ADDR)) {
    magType = MAG_QMC5883L;
    configureQMC5883L();

    // Verify that data registers actually answer.
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(QMC_ADDR, 6) == 6) {
      while (Wire.available()) Wire.read();
      magnetometerPresent = true;
      Serial.println("Magnetometer: QMC5883L @ 0x0D");
      return;
    }
  }

  if (probeI2C(HMC_ADDR)) {
    magType = MAG_HMC5883L;
    configureHMC5883L();

    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x03);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(HMC_ADDR, 6) == 6) {
      while (Wire.available()) Wire.read();
      magnetometerPresent = true;
      Serial.println("Magnetometer: HMC5883L @ 0x1E");
      return;
    }
  }

  Serial.println("Magnetometer: NOT FOUND");
  printI2CScan();
}

void configureQMC5883L() {
  // Matches the user's known-good QMC5883L calibration sketch.
  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x0B);
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.beginTransmission(QMC_ADDR);
  Wire.write(0x09);
  Wire.write(0x1D);
  Wire.endTransmission();

  delay(10);
}

void configureHMC5883L() {
  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x00);
  Wire.write(0x70);
  Wire.endTransmission();

  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x01);
  Wire.write(0x20);
  Wire.endTransmission();

  Wire.beginTransmission(HMC_ADDR);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission();

  delay(10);
}

bool readMagnetometer() {
  if (!magnetometerPresent) return false;

  if (magType == MAG_QMC5883L) {
    Wire.beginTransmission(QMC_ADDR);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(QMC_ADDR, 6) != 6) return false;

    magX = (int16_t)(Wire.read() | (Wire.read() << 8));
    magY = (int16_t)(Wire.read() | (Wire.read() << 8));
    magZ = (int16_t)(Wire.read() | (Wire.read() << 8));
    return true;
  }

  if (magType == MAG_HMC5883L) {
    Wire.beginTransmission(HMC_ADDR);
    Wire.write(0x03);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom(HMC_ADDR, 6) != 6) return false;

    magX = (int16_t)((Wire.read() << 8) | Wire.read());
    magZ = (int16_t)((Wire.read() << 8) | Wire.read());
    magY = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
  }

  return false;
}

void updateCompass() {
  static unsigned long lastRead = 0;
  static unsigned long lastProbe = 0;
  static uint8_t readFailures = 0;

  if (millis() - lastRead < 100UL) return;
  lastRead = millis();

  if (!magnetometerPresent) {
    if (millis() - lastProbe >= 2000UL) {
      lastProbe = millis();
      detectMagnetometer();
    }
    return;
  }

  if (!readMagnetometer()) {
    if (++readFailures >= 3) {
      magnetometerPresent = false;
      readFailures = 0;
      Serial.println("Magnetometer read lost; retrying detection.");
    }
    return;
  }

  readFailures = 0;
  heading = calculateHeading();
  directionShort = shortDirection(heading);
  directionLong = longDirection(heading);
}

float calculateHeading() {
  float calibratedX = ((float)magX - magCal[0]) * magCal[2];
  float calibratedY = ((float)magY - magCal[1]) * magCal[3];

  float angle = atan2(calibratedX, calibratedY) * 180.0f / PI;
  angle += MAG_DECLINATION;

  while (angle < 0.0f) angle += 360.0f;
  while (angle >= 360.0f) angle -= 360.0f;

  return angle;
}

String shortDirection(float degrees) {
  if (degrees >= 337.5f || degrees < 22.5f) return "N";
  if (degrees < 67.5f) return "NE";
  if (degrees < 112.5f) return "E";
  if (degrees < 157.5f) return "SE";
  if (degrees < 202.5f) return "S";
  if (degrees < 247.5f) return "SW";
  if (degrees < 292.5f) return "W";
  return "NW";
}

String longDirection(float degrees) {
  if (degrees >= 337.5f || degrees < 22.5f) return "North";
  if (degrees < 67.5f) return "Northeast";
  if (degrees < 112.5f) return "East";
  if (degrees < 157.5f) return "Southeast";
  if (degrees < 202.5f) return "South";
  if (degrees < 247.5f) return "Southwest";
  if (degrees < 292.5f) return "West";
  return "Northwest";
}

// For the 4-letter compass UI:
// NNE/NE -> N+E, ESE/SE -> E+S, etc.
// Pure N/E/S/W stays as a single active letter.
bool cardinalActive(const char *cardinal, float degrees) {
  if (degrees >= 337.5f || degrees < 22.5f) return strcmp(cardinal, "N") == 0;
  if (degrees < 67.5f) return strcmp(cardinal, "N") == 0 || strcmp(cardinal, "E") == 0;
  if (degrees < 112.5f) return strcmp(cardinal, "E") == 0;
  if (degrees < 157.5f) return strcmp(cardinal, "E") == 0 || strcmp(cardinal, "S") == 0;
  if (degrees < 202.5f) return strcmp(cardinal, "S") == 0;
  if (degrees < 247.5f) return strcmp(cardinal, "S") == 0 || strcmp(cardinal, "W") == 0;
  if (degrees < 292.5f) return strcmp(cardinal, "W") == 0;
  return strcmp(cardinal, "W") == 0 || strcmp(cardinal, "N") == 0;
}

uint16_t deviceColor(int id) {
  if (id == 1) return RED;
  if (id == 2) return GREEN;
  if (id == 3) return BLUE;
  return WHITE;
}

void drawCenteredText(const String &text, int y, uint16_t fg, uint16_t bg, uint8_t size) {
  int16_t x1, y1;
  uint16_t w, h;
  tft.setTextSize(size);
  // The caller already clears the update rectangle. Do not give each
  // character its own background box; that is what caused the square artifact.
  tft.setTextColor(fg);
  tft.getTextBounds(text, 0, y, &x1, &y1, &w, &h);
  int x = (SCREEN_W - (int)w) / 2;
  tft.setCursor(x, y);
  tft.print(text);
}

// =====================================================
// CLOCK HELPERS
// =====================================================

// =====================================================
// CLOCK HELPERS
// =====================================================

bool isLeapYear(int year) {
  return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

int daysInMonth(int month, int year) {
  if (month == 2) return isLeapYear(year) ? 29 : 28;
  if (month == 4 || month == 6 || month == 9 || month == 11) return 30;
  return 31;
}

void addSecondsToDate(int secondsToAdd, int &hour, int &minute, int &second, int &day, int &month, int &year) {
  long total = (long)hour * 3600L + (long)minute * 60L + (long)second + secondsToAdd;
  while (total >= 86400L) {
    total -= 86400L;
    day++;
    if (day > daysInMonth(month, year)) {
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
        year++;
      }
    }
  }

  hour = total / 3600L;
  minute = (total % 3600L) / 60L;
  second = total % 60L;
}

void advanceRemoteClock() {
  if (!remoteTimeValid) return;

  unsigned long elapsed = (millis() - remoteSyncMillis) / 1000UL;
  if (elapsed == 0) return;

  remoteSyncMillis += elapsed * 1000UL;
  addSecondsToDate((int)elapsed, remoteHour, remoteMinute, remoteSecond,
                   remoteDay, remoteMonth, remoteYear);
}

void getDisplayClock(int &hour, int &minute, int &second, int &day, int &month, int &year, bool &valid) {
  if (gps.time.isValid()) {
    int utcHour = gps.time.hour();
    minute = gps.time.minute();
    second = gps.time.second();
    day = gpsDay;
    month = gpsMonth;
    year = gpsYear;

    hour = utcHour + 8;
    if (hour >= 24) {
      hour -= 24;
      if (day > 0 && month > 0 && year > 0) {
        day++;
        if (day > daysInMonth(month, year)) {
          day = 1;
          month++;
          if (month > 12) { month = 1; year++; }
        }
      }
    }
    valid = true;
    return;
  }

  advanceRemoteClock();
  if (remoteTimeValid) {
    hour = remoteHour;
    minute = remoteMinute;
    second = remoteSecond;
    day = remoteDay;
    month = remoteMonth;
    year = remoteYear;
    valid = true;
    return;
  }

  valid = false;
  hour = minute = second = day = month = year = 0;
}

// =====================================================
// LORA HEARTBEAT
// =====================================================

void sendHeartbeat() {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);
  radioSleeping = false;
  LoRa.idle();

  String packet;
  packet.reserve(100);
  packet = "HB,";
  packet += String(DEVICE_ID);
  packet += ",";
  packet += String(localLocationValid() ? 1 : 0);
  packet += ",";
  packet += String(latitude, 6);
  packet += ",";
  packet += String(longitude, 6);
  packet += ",";

  if (gps.time.isValid() && gps.date.isValid()) {
    int h = gps.time.hour() + 8;
    int d = gpsDay;
    int m = gpsMonth;
    int y = gpsYear;

    if (h >= 24) {
      h -= 24;
      d++;
      if (d > daysInMonth(m, y)) {
        d = 1;
        m++;
        if (m > 12) {
          m = 1;
          y++;
        }
      }
    }

    packet += String(h) + ",";
    packet += String(gps.time.minute()) + ",";
    packet += String(gps.time.second()) + ",";
    packet += String(d) + ",";
    packet += String(m) + ",";
    packet += String(y);
  } else {
    packet += "-1,-1,-1,-1,-1,-1";
  }
  packet += ",";
  packet += String(magnetometerPresent ? 1 : 0);
  packet += ",";
  packet += magnetometerPresent ? String(heading, 1) : String(-1);

  // Responsive build: send ONE heartbeat, then immediately return to RX.
  // The longer interval leaves substantially more time for incoming packets.
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

  LoRa.receive();
}

// =====================================================
// STATUS SYNCHRONIZATION
// =====================================================
// A wake-up device asks all awake peers for their current state.
// Sleeping peers will not answer (their radio is sleeping), so their last
// known state remains on the display until they next report.
void requestStatusSync() {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  radioSleeping = false;
  LoRa.idle();

  String packet = "SYNC," + String(DEVICE_ID);

  // Responsive build: one SYNC packet only.
  // Repeated SYNC bursts were occupying the half-duplex radio and
  // delaying incoming Ping packets.
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

  LoRa.receive();
}

void sendStateResponse(int targetId) {
  if (sleepMode || radioSleeping) return;

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  String packet = "STATE," + String(DEVICE_ID) + "," + String(targetId) + ",";
  packet += String(sleepMode ? 1 : 0);
  packet += ",";
  packet += String(localLocationValid() ? 1 : 0);
  packet += ",";
  packet += String(latitude, 6);
  packet += ",";
  packet += String(longitude, 6);
  packet += ",";
  packet += String(magnetometerPresent ? 1 : 0);
  packet += ",";
  packet += magnetometerPresent ? String(heading, 1) : String(-1);

  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

  LoRa.receive();
}

// =====================================================
// SEND PING
// =====================================================

// =====================================================
// SEND PING
// =====================================================

void sendPing(int targetId) {
  if (targetId < 0 || targetId > 3 || targetId == DEVICE_ID) return;
  // A ping is already on its way: keep "Sending..." on screen and ignore the
  // extra tap so a double/triple tap cannot send more than one ping.
  if (pingPending) return;

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  radioSleeping = false;
  LoRa.idle();

  uint16_t sequence = ++pingSequence;
  if (sequence == 0) sequence = ++pingSequence;
  String packet = "P," + String(DEVICE_ID) + "," + String(targetId) + "," + String(sequence);

  pingPending = true;
  pendingPingTarget = targetId;
  pendingPingBroadcast = (targetId == 0);
  pendingPingSequence = sequence;
  broadcastAckCount = 0;
  for (int id = 1; id <= 3; ++id) broadcastAckSeen[id] = false;
  pingMessage = "Sending...";
  pingMessageTime = millis();
  if (currentScreen == MAIN_MENU) {
    updateMainMenuDynamic(true);
  }

  // One radio transmission per user action. Targeted and broadcast pings
  // both expect ACKs; latency is measured from this moment.
  pendingPingStartedAt = millis();
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

  if (sleepMode) {
    LoRa.sleep();
    radioSleeping = true;
  } else {
    LoRa.receive();
  }
}

void updatePing() {
  // Receiver side: send broadcast ACKs whose time slot has arrived.
  for (int id = 1; id <= 3; ++id) {
    if (!scheduledPingAck[id].pending ||
        (long)(millis() - scheduledPingAck[id].dueAt) < 0) continue;
    scheduledPingAck[id].pending = false;
    acknowledgePing(id, 0, scheduledPingAck[id].sequence,
                    scheduledPingAck[id].receivedAt);
  }

  // Sender side: finish the broadcast after its collection window, or
  // release the "Sending..." lock if a targeted reply never arrived.
  if (!pingPending) return;
  unsigned long elapsed = millis() - pendingPingStartedAt;
  if (pendingPingBroadcast) {
    if (elapsed < PING_BROADCAST_WINDOW) return;
    pingMessage = broadcastAckCount
      ? "You pinged everyone (" + String(elapsed) + " ms)"
      : String("No device replied");
  } else {
    if (elapsed < PING_REPLY_WINDOW) return;
    pingMessage = "Lora #" + String(pendingPingTarget) + " did not reply";
  }
  pingPending = false;
  pendingPingBroadcast = false;
  pingMessageTime = millis();
  if (currentScreen == MAIN_MENU) updateMainMenuDynamic(true);
}

void sendPingAck(int requesterId, int targetId, uint16_t sequence) {
  if (sleepMode || radioSleeping) return;
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print("PA,");
  LoRa.print(DEVICE_ID);
  LoRa.print(",");
  LoRa.print(requesterId);
  LoRa.print(",");
  LoRa.print(targetId);
  LoRa.print(",");
  LoRa.print(sequence);
  LoRa.endPacket();
  LoRa.receive();
}

// Send the ACK first, then show the notification. The receiver's figure is
// measured on its own clock: ping received -> ACK fully transmitted
// (LoRa.endPacket() returns only after transmission completes). For a
// broadcast it includes this device's reply slot delay.
void acknowledgePing(int requesterId, int targetId, uint16_t sequence,
                     unsigned long receivedAt) {
  sendPingAck(requesterId, targetId, sequence);
  unsigned long responseMs = millis() - receivedAt;
  pingMessage = "Lora #" + String(requesterId) + " pinged you (" +
                String(responseMs) + " ms)";
  pingMessageTime = millis();
  if (currentScreen == MAIN_MENU) updateMainMenuDynamic(true);
}

// =====================================================
// LORA RECEIVE
// =====================================================

void receiveLoRa() {
  if (radioSleeping) return;

  int packetSize = LoRa.parsePacket();
  if (!packetSize) return;

  String packet = "";
  while (LoRa.available()) packet += (char)LoRa.read();
  unsigned long packetReceivedAt = millis();

  Serial.print("LoRa RX: ");
  Serial.println(packet);

  // PACKET TEST: T,<sender>,<seq> and replies TR,<responder>,<to>,<seq>,<rssi>,<snr>
  if (packet.startsWith("T,") || packet.startsWith("TR,")) {
    handlePacketTest(packet);
    LoRa.receive();
    return;
  }

  // ---------------------------------------------------
  // NORMAL HEARTBEAT
  // HB,id,gpsFix,lat,lon,h,m,s,day,month,year
  // ---------------------------------------------------
  if (packet.startsWith("HB,")) {
    String f[12];
    int startIndex = 3;
    int count = 0;

    while (count < 12) {
      int comma = packet.indexOf(',', startIndex);
      if (comma < 0) {
        f[count++] = packet.substring(startIndex);
        break;
      }
      f[count++] = packet.substring(startIndex, comma);
      startIndex = comma + 1;
    }

    if (count >= 10) {
      int sender = f[0].toInt();

      if (sender >= 1 && sender <= 3 && sender != DEVICE_ID) {
        // One valid heartbeat is enough to mark the peer ONLINE immediately.
        remote[sender].lastSeen = millis();
        remote[sender].asleep = false;
        remote[sender].online = true;
        remote[sender].heartbeatCount = ONLINE_CONFIRM_PACKETS;
        cachedRemoteOnline[sender] = true;
        cachedRemoteAsleep[sender] = false;

        updateRemoteLocation(sender, f[1].toInt() == 1, f[2].toDouble(),
                 f[3].toDouble(), count >= 12 && f[10].toInt() == 1,
                 count >= 12 ? f[11].toFloat() : -1.0f);

        int h = f[4].toInt();
        int mi = f[5].toInt();
        int sec = f[6].toInt();
        int d = f[7].toInt();
        int mo = f[8].toInt();
        int y = f[9].toInt();

        if (!gps.time.isValid() &&
            h >= 0 && mi >= 0 && sec >= 0 &&
            d > 0 && mo > 0 && y > 0) {
          remoteTimeValid = true;
          remoteHour = h;
          remoteMinute = mi;
          remoteSecond = sec;
          remoteDay = d;
          remoteMonth = mo;
          remoteYear = y;
          remoteSyncMillis = millis();
        }
      }
    }

    LoRa.receive();
    return;
  }

  // ---------------------------------------------------
  // SLEEP LOCATION REPORT
  // SLP,id,1,gpsFix,lat,lon
  // ---------------------------------------------------
  if (packet.startsWith("SLP,")) {
    String f[7];
    int startIndex = 4;
    int count = 0;

    while (count < 7) {
      int comma = packet.indexOf(',', startIndex);
      if (comma < 0) {
        f[count++] = packet.substring(startIndex);
        break;
      }
      f[count++] = packet.substring(startIndex, comma);
      startIndex = comma + 1;
    }

    if (count >= 5) {
      int sender = f[0].toInt();
      int enabled = f[1].toInt();

      if (sender >= 1 && sender <= 3 && sender != DEVICE_ID) {
        remote[sender].lastSeen = millis();
        remote[sender].asleep = (enabled == 1);
        remote[sender].online = true;
        remote[sender].heartbeatCount = ONLINE_CONFIRM_PACKETS;
        cachedRemoteOnline[sender] = true;
        cachedRemoteAsleep[sender] = (enabled == 1);
        updateRemoteLocation(sender, f[2].toInt() == 1, f[3].toDouble(),
                 f[4].toDouble(), count >= 7 && f[5].toInt() == 1,
                 count >= 7 ? f[6].toFloat() : -1.0f);
      }
    }

    LoRa.receive();
    return;
  }

  // ---------------------------------------------------
  // STATUS SYNC REQUEST
  // SYNC,requesterId
  // ---------------------------------------------------
  if (packet.startsWith("SYNC,")) {
    int requester = packet.substring(5).toInt();

    if (requester >= 1 && requester <= 3 && requester != DEVICE_ID) {
      // Awake devices answer immediately. Sleeping devices never reach here
      // because their radio is intentionally asleep.
      sendStateResponse(requester);
    }

    LoRa.receive();
    return;
  }

  // ---------------------------------------------------
  // STATUS RESPONSE
  // STATE,sender,target,asleep,gpsValid,lat,lon
  // ---------------------------------------------------
  if (packet.startsWith("STATE,")) {
    String f[8];
    int startIndex = 6;
    int count = 0;

    while (count < 8) {
      int comma = packet.indexOf(',', startIndex);
      if (comma < 0) {
        f[count++] = packet.substring(startIndex);
        break;
      }
      f[count++] = packet.substring(startIndex, comma);
      startIndex = comma + 1;
    }

    if (count >= 6) {
      int sender = f[0].toInt();
      int target = f[1].toInt();

      if (target == DEVICE_ID &&
          sender >= 1 && sender <= 3 &&
          sender != DEVICE_ID) {

        bool asleep = (f[2].toInt() == 1);

        remote[sender].lastSeen = millis();
        remote[sender].asleep = asleep;
        remote[sender].online = true;
        remote[sender].heartbeatCount = ONLINE_CONFIRM_PACKETS;
        updateRemoteLocation(sender, f[3].toInt() == 1, f[4].toDouble(),
                 f[5].toDouble(), count >= 8 && f[6].toInt() == 1,
                 count >= 8 ? f[7].toFloat() : -1.0f);

        cachedRemoteOnline[sender] = true;
        cachedRemoteAsleep[sender] = asleep;
      }
    }

    LoRa.receive();
    return;
  }

  // ---------------------------------------------------
  // P,sender,target,sequence and PA,responder,requester,target,sequence
  // ---------------------------------------------------
  if (packet.startsWith("PA,")) {
    int first = packet.indexOf(',', 3);
    int second = first < 0 ? -1 : packet.indexOf(',', first + 1);
    int third = second < 0 ? -1 : packet.indexOf(',', second + 1);
    if (first > 3 && second > first && third > second &&
      packet.indexOf(',', third + 1) < 0) {
      int responder = packet.substring(3, first).toInt();
      int requester = packet.substring(first + 1, second).toInt();
      int target = packet.substring(second + 1, third).toInt();
      uint16_t sequence = 0;
      bool sequenceValid = parsePingSequence(packet.substring(third + 1), sequence);
      if (sequenceValid && pingPending &&
          millis() - pendingPingStartedAt <
            (pendingPingBroadcast ? PING_BROADCAST_WINDOW : PING_REPLY_WINDOW) &&
          requester == DEVICE_ID && target == pendingPingTarget &&
          responder >= 1 && responder <= 3 && responder != DEVICE_ID &&
          sequence == pendingPingSequence) {
        unsigned long rtt = millis() - pendingPingStartedAt;
        if (pendingPingBroadcast) {
          // Collect every responder once; the message is shown when the
          // broadcast window closes (see updatePing()).
          if (!broadcastAckSeen[responder]) {
            broadcastAckSeen[responder] = true;
            ++broadcastAckCount;
            Serial.printf("PING BROADCAST ACK responder=%d seq=%u rtt_ms=%lu\n",
                          responder, sequence, rtt);
          }
        } else if (responder == pendingPingTarget) {
          pingPending = false;
          pingMessage = "You pinged Lora #" + String(responder) + " (" +
                        String(rtt) + " ms)";
          pingMessageTime = millis();
          Serial.printf("PING RTT target=%d seq=%u rtt_ms=%lu\n", responder, sequence, rtt);
          if (currentScreen == MAIN_MENU) updateMainMenuDynamic(true);
        }
      }
    }
    LoRa.receive();
    return;
  }

  if (packet.startsWith("P,")) {
    int first = packet.indexOf(',', 2);
    int second = first < 0 ? -1 : packet.indexOf(',', first + 1);
    if (first > 2 && second > first && packet.indexOf(',', second + 1) < 0) {
      int sender = packet.substring(2, first).toInt();
      int target = packet.substring(first + 1, second).toInt();
      uint16_t sequence = 0;
      bool sequenceValid = parsePingSequence(packet.substring(second + 1), sequence);
      bool addressedToMe = target == 0 || target == DEVICE_ID;
      bool duplicate = false;
      if (sender >= 1 && sender <= 3) {
        for (int i = 0; i < 4; ++i) {
          if (receivedPingSequenceValid[sender][i] &&
              receivedPingSequences[sender][i] == sequence) duplicate = true;
        }
      }
      if (sender >= 1 && sender <= 3 && sender != DEVICE_ID &&
          (target == 0 || (target >= 1 && target <= 3 && target != sender)) &&
          addressedToMe && sequenceValid && !duplicate) {
        uint8_t slot = receivedPingSequenceNext[sender];
        receivedPingSequenceValid[sender][slot] = true;
        receivedPingSequences[sender][slot] = sequence;
        receivedPingSequenceNext[sender] = (slot + 1) % 4;
        remote[sender].lastSeen = millis();
        remote[sender].online = true;
        remote[sender].asleep = false;
        cachedRemoteOnline[sender] = true;
        cachedRemoteAsleep[sender] = false;
        if (target == 0) {
          // Broadcast: reply in this device's own slot to avoid collisions.
          scheduledPingAck[sender].pending = true;
          scheduledPingAck[sender].sequence = sequence;
          scheduledPingAck[sender].receivedAt = packetReceivedAt;
          scheduledPingAck[sender].dueAt =
            packetReceivedAt + (unsigned long)DEVICE_ID * PING_BROADCAST_SLOT;
        } else {
          // Targeted: ACK immediately, before any screen redraw.
          acknowledgePing(sender, target, sequence, packetReceivedAt);
        }
      }
    }
  }
  LoRa.receive();
}

void updateDeviceStatus() {
  unsigned long now = millis();

  // Our own state is always known locally.
  remote[DEVICE_ID].online = true;
  remote[DEVICE_ID].asleep = sleepMode;

  // While this device is sleeping, preserve the exact last-known remote
  // statuses. Do NOT let the local timeout logic turn them OFFLINE.
  if (remoteStatusFrozen) {
    for (int i = 1; i <= 3; ++i) {
      if (i == DEVICE_ID) continue;
      remote[i].online = cachedRemoteOnline[i];
      remote[i].asleep = cachedRemoteAsleep[i];
    }
    return;
  }

  // After waking, allow a short synchronization grace period. During this
  // window, the display keeps its previous state until fresh heartbeats or
  // sleep reports arrive, preventing an ONLINE/OFFLINE flicker on wake-up.
  if (resumeStatusSync) {
    if ((long)(now - resumeStatusSyncUntil) < 0) {
      return;
    }
    resumeStatusSync = false;
  }

  for (int i = 1; i <= 3; ++i) {
    if (i == DEVICE_ID) continue;

    if (remote[i].lastSeen == 0) {
      remote[i].online = false;
      continue;
    }

    unsigned long timeout =
      remote[i].asleep ? SLEEP_DEVICE_TIMEOUT : DEVICE_TIMEOUT;

    if (now - remote[i].lastSeen > timeout) {
      remote[i].online = false;
      remote[i].asleep = false;
    }
  }
}


// =====================================================
// SLEEP MODE CONTROL
// =====================================================

void sendSleepLocationReport() {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  radioSleeping = false;
  LoRa.idle();

  LoRa.beginPacket();
  LoRa.print("SLP,");
  LoRa.print(DEVICE_ID);
  LoRa.print(",1,");
  LoRa.print(localLocationValid() ? 1 : 0);
  LoRa.print(",");
  LoRa.print(latitude, 6);
  LoRa.print(",");
  LoRa.print(longitude, 6);
  LoRa.print(",");
  LoRa.print(magnetometerPresent ? 1 : 0);
  LoRa.print(",");
  if (magnetometerPresent) LoRa.print(heading, 1);
  else LoRa.print(-1);
  LoRa.endPacket();

  // The communication radio now sleeps until the next 5-minute report.
  LoRa.sleep();
  radioSleeping = true;
}

void setSleepMode(bool enabled) {

  if (enabled == sleepMode) return;

  if (enabled) {
    // Capture the last known state of the OTHER devices BEFORE the radio
    // is put to sleep. Their status must remain frozen on our display.
    for (int i = 1; i <= 3; ++i) {
      if (i == DEVICE_ID) continue;
      cachedRemoteOnline[i] = remote[i].online;
      cachedRemoteAsleep[i] = remote[i].asleep;
    }

    remoteStatusFrozen = true;
    resumeStatusSync = false;
    sleepMode = true;
    remote[DEVICE_ID].online = true;
    remote[DEVICE_ID].asleep = true;

    // Immediately announce sleep mode and save current location.
    sendSleepLocationReport();
    nextSleepLocationReport = millis() + SLEEP_LOCATION_INTERVAL;
    nextHeartbeat = millis() + HEARTBEAT_INTERVAL;

    Serial.println("SLEEP MODE ENABLED - REMOTE STATUS FROZEN");

  } else {
    // Wake the radio first, then immediately announce that this device is
    // awake. Keep the old remote display state for a short grace period while
    // fresh heartbeats/sleep reports synchronize the statuses again.
    sleepMode = false;
    radioSleeping = false;
    LoRa.idle();

    remote[DEVICE_ID].online = true;
    remote[DEVICE_ID].asleep = false;

    remoteStatusFrozen = false;
    resumeStatusSync = true;
    resumeStatusSyncUntil = millis() + 8000UL;

    // Send one wake heartbeat immediately, then request current peer state.
    // Avoiding the old duplicate heartbeat + delay makes the touchscreen and
    // radio responsive again as soon as Sleep Mode is disabled.
    sendHeartbeat();

    // Ask all currently awake peers for their current state.
    // This resynchronizes ONLINE/ASLEEP status without forcing our
    // display to flash OFFLINE while waiting for their answer.
    requestStatusSync();

    nextHeartbeat = millis() + HEARTBEAT_INTERVAL;

    Serial.println("SLEEP MODE DISABLED - RESYNCING STATUSES");
  }
}



// =====================================================
// MAIN MENU
// New Lopaka UI supplied by the user.
// =====================================================

void drawMainMenu() {
  currentScreen = MAIN_MENU;
  tft.fillScreen(BLACK);

  tft.setTextSize(2);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(19, 11);
  tft.print("LORA THE EXPLORER");

  // Static white action rectangles.
  tft.fillRect(22, 136, 196, 48, WHITE);
  tft.setTextColor(BLACK, WHITE);
  tft.setTextSize(2);
  tft.setCursor(37, 153);
  tft.print("CHECK LOCATION");

  tft.fillRect(22, 191, 196, 48, WHITE);
  tft.setCursor(60, 206);
  tft.print("SLEEP MODE");

  // Status icons keep device identity colors at all times.
  // ONLINE/OFFLINE is shown only by the text.
  for (int i = 1; i <= 3; i++) {
    int iconX = (i == 1) ? 34 : (i == 2 ? 114 : 191);
    int textX = (i == 1) ? 23 : (i == 2 ? 103 : 178);
    tft.drawBitmap(iconX, 258, image_activeDev3_bits, 13, 13,
                   deviceColor(i));
    tft.setTextSize(1);
    if (!remote[i].online) {
      tft.setTextColor(GRAY, BLACK);
      tft.setCursor(textX, 280);
      tft.print("OFFLINE");
    } else if (remote[i].asleep) {
      tft.setTextColor(YELLOW, BLACK);
      tft.setCursor(textX, 280);
      tft.print("ASLEEP");
    } else {
      tft.setTextColor(GREEN, BLACK);
      tft.setCursor(textX, 280);
      tft.print("ONLINE");
    }
  }

  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(20, 295);  tft.print("LORA #1");
  tft.setCursor(100, 295); tft.print("LORA #2");
  tft.setCursor(177, 295); tft.print("LORA #3");

  lastMainTime = "";
  lastMainDate = "";
  lastMainPing = "";
  lastMainSleepMessage = "";
  for (int i = 1; i <= 3; i++) {
    lastMainOnline[i] = !remote[i].online;
    lastMainAsleep[i] = !remote[i].asleep;
  }

  updateMainMenuDynamic(true);
}

void drawPingArea(const String &message) {
  tft.fillRect(0, 100, 240, 22, BLACK);
  if (message.length()) {
    tft.setTextSize(1);
    tft.setTextColor(LIGHTGREEN, BLACK);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(message.c_str(), 0, 112, &x1, &y1, &w, &h);
    int x = (SCREEN_W - (int)w) / 2;
    if (x < 0) x = 0;
    tft.setCursor(x, 112);
    tft.print(message);
  }
}

void drawSleepModeArea(const String &message) {
  tft.fillRect(0, 122, 240, 13, BLACK);
  if (message.length()) {
    tft.setTextSize(1);
    tft.setTextColor(LIGHTGREEN, BLACK);
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(message.c_str(), 0, 124, &x1, &y1, &w, &h);
    int x = (SCREEN_W - (int)w) / 2;
    if (x < 0) x = 0;
    tft.setCursor(x, 124);
    tft.print(message);
  }
}

void updateMainMenuDynamic(bool force) {
  if (currentScreen != MAIN_MENU) return;

  int h, mi, s, d, mo, y;
  bool clockValid;
  getDisplayClock(h, mi, s, d, mo, y, clockValid);

  // TIME: centered exactly using text metrics. Explicit black background
  // prevents the character background blocks/squares.
  char timeBuf[12];
  if (clockValid) sprintf(timeBuf, "%02d:%02d:%02d", h, mi, s);
  else sprintf(timeBuf, "--:--:--");

  String timeText = timeBuf;
  if (force || timeText != lastMainTime) {
    tft.fillRect(25, 43, 190, 31, BLACK);
    drawCenteredText(timeText, 49, YELLOW, BLACK, 3);
    lastMainTime = timeText;
  }

  // DATE: long readable format, e.g. September 30, 2026.
  // If no GPS/remote date is available yet, show Loading... instead of blank.
  String dateText = "Loading...";
  if (d > 0 && mo > 0 && y > 0) {
    static const char *months[] = {
      "", "January", "February", "March", "April", "May", "June",
      "July", "August", "September", "October", "November", "December"
    };
    if (mo >= 1 && mo <= 12) {
      dateText = String(months[mo]) + " " + String(d) + ", " + String(y);
    }
  }

  if (force || dateText != lastMainDate) {
    tft.fillRect(35, 84, 170, 20, BLACK);
    if (dateText.length()) {
      drawCenteredText(dateText, 89, YELLOW, BLACK, 1);
    }
    lastMainDate = dateText;
  }

  String showPing = "";
  if (pingMessage.length() && millis() - pingMessageTime < PING_MESSAGE_TIME) {
    showPing = pingMessage;
  }

  if (force || showPing != lastMainPing) {
    drawPingArea(showPing);
    lastMainPing = showPing;
  }

  String sleepText = sleepMode ? "SLEEP MODE IS ENABLED!" : "";
  if (force || sleepText != lastMainSleepMessage) {
    drawSleepModeArea(sleepText);
    lastMainSleepMessage = sleepText;
  }

  for (int i = 1; i <= 3; i++) {
    if (force ||
        lastMainOnline[i] != remote[i].online ||
        lastMainAsleep[i] != remote[i].asleep) {

      int iconX = (i == 1) ? 34 : (i == 2 ? 114 : 191);
      int textX = (i == 1) ? 23 : (i == 2 ? 103 : 178);

      // Identity circles never change color. They are the legend.
      tft.fillRect(iconX - 2, 255, 18, 18, BLACK);
      tft.drawBitmap(iconX, 258, image_activeDev3_bits, 13, 13,
                     deviceColor(i));

      tft.fillRect(textX, 277, 58, 13, BLACK);
      tft.setTextSize(1);

      if (!remote[i].online) {
        tft.setTextColor(GRAY, BLACK);
        tft.setCursor(textX, 280);
        tft.print("OFFLINE");
      } else if (remote[i].asleep) {
        tft.setTextColor(YELLOW, BLACK);
        tft.setCursor(textX, 280);
        tft.print("ASLEEP");
      } else {
        tft.setTextColor(GREEN, BLACK);
        tft.setCursor(textX, 280);
        tft.print("ONLINE");
      }

      lastMainOnline[i] = remote[i].online;
      lastMainAsleep[i] = remote[i].asleep;
    }
  }
}

// =====================================================
// PEER LOCATION SCREEN
// =====================================================

void drawBackArrow() {
  // Visible arrow.
  tft.fillTriangle(10, 18, 23, 10, 23, 26, WHITE);
  tft.fillRect(21, 16, 14, 5, WHITE);
}

void drawPeerStatic() {
  // Stable static compass and labels. The circle is drawn only once.
  tft.setTextSize(2);
  tft.setTextColor(WHITE, BLACK);

  tft.drawCircle(PEER_CIRCLE_CENTER_X, PEER_CIRCLE_CENTER_Y,
                 PEER_CIRCLE_RADIUS, WHITE);

  tft.drawLine(11, 248, 231, 248, WHITE);

  // Page title: aligned with the back arrow and kept clear of the N marker.
  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);
  tft.setCursor(42, 12);
  tft.print("LOCATION");

  // Show the two remote devices from this device's point of view.
  int peerA = 1;
  int peerB = 2;
  if (DEVICE_ID == 1) { peerA = 2; peerB = 3; }
  else if (DEVICE_ID == 2) { peerA = 1; peerB = 3; }

  drawPeerLegendLine(peerA, 291);
  drawPeerLegendLine(peerB, 303);
}

void drawPeerLegendLine(int deviceId, int y) {
  uint16_t color = deviceColor(deviceId);
  bool positionValid = localLocationValid() && remoteLocationFresh(deviceId);
  String distanceText = positionValid
    ? String((int)roundf(distanceToPeer(remote[deviceId].lat, remote[deviceId].lon)))
    : String("N/A");
  // Same relative bearing the radar dot uses, shown as N/NE/E/...
  String directionText = positionValid && magnetometerPresent
    ? shortDirection((float)relativeBearingDegrees(remote[deviceId].lat, remote[deviceId].lon))
    : String("--");
  String line = "LORA #" + String(deviceId) + " | " + distanceText +
                " meters away! | " + directionText;

  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);

  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(line.c_str(), 0, y, &x1, &y1, &w, &h);

  uint16_t statusColor = !remote[deviceId].online ? GRAY
    : (remote[deviceId].asleep ? YELLOW : WHITE);
  tft.setTextColor(statusColor, BLACK);

  const int bulletRadius = 3;
  const int gap = 5;
  int totalWidth = (bulletRadius * 2) + gap + (int)w;
  int startX = (SCREEN_W - totalWidth) / 2;

  if (startX < 2) startX = 2;

  int bulletX = startX + bulletRadius;
  int bulletY = y + 4;
  int textX = startX + (bulletRadius * 2) + gap;

  tft.fillCircle(bulletX, bulletY, bulletRadius, color);
  tft.setCursor(textX, y);
  tft.print(line);
}

void drawPeerLocation() {
  currentScreen = PEER_LOCATION_SCREEN;
  tft.fillScreen(BLACK);

  peerVisualInitialized = false;
  for (int i = 1; i <= 3; i++) lastPeerDot[i].valid = false;

  lastPeerLocationText = "";
  lastPeerHeadingText = "";
  lastPeerDistance2 = "";
  lastPeerDistance3 = "";
  for (int i = 0; i < 4; ++i) lastPeerDistanceDisplay[i] = "";
  lastPeerHeading = -999.0f;
  lastPeerDirection = "";
  lastPeerGpsFix = !localLocationValid();
  lastPeerMagFound = !magnetometerPresent;
  lastPeer1Lat = remote[1].lat;
  lastPeer1Lon = remote[1].lon;
  lastPeer1Online = remote[1].online;
  lastPeer1Asleep = remote[1].asleep;
  lastPeer1Gps = remote[1].gpsValid;
  lastPeer2Lat = remote[2].lat;
  lastPeer2Lon = remote[2].lon;
  lastPeer3Lat = remote[3].lat;
  lastPeer3Lon = remote[3].lon;
  lastPeer2Online = remote[2].online;
  lastPeer3Online = remote[3].online;
  lastPeer2Asleep = remote[2].asleep;
  lastPeer3Asleep = remote[3].asleep;
  lastPeer2Gps = remote[2].gpsValid;
  lastPeer3Gps = remote[3].gpsValid;
  lastPeerOwnLat = latitude;
  lastPeerOwnLon = longitude;

  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);
  drawBackArrow();

  // LOCATION is drawn once in drawPeerStatic(), aligned with the arrow.
  drawPeerStatic();
  updatePeerLocationDynamic(true);
}

void drawCardinalLetter(const char *letter, int x, int y, bool active) {
  // Erase only the letter's own 12x16 cell (+1 px). The old 24x20 box
  // reached up to 16 px right of the letter and cut into the circle.
  tft.fillRect(x - 1, y - 1, 12, 16, BLACK);
  tft.setTextSize(2);
  tft.setTextColor(active ? RED : WHITE, BLACK);
  tft.setCursor(x, y);
  tft.print(letter);
}

void updatePeerCardinals(bool force) {
  if (!force) return;
  const char *labels[4] = {"N", "E", "S", "W"};
  const float bearings[4] = {0.0f, 90.0f, 180.0f, 270.0f};
  float localHeading = magnetometerPresent ? heading : 0.0f;
  for (int i = 0; i < 4; ++i) {
    if (lastCardinalLabel[i].valid) {
      tft.fillRect(lastCardinalLabel[i].x - 1, lastCardinalLabel[i].y - 1,
                   12, 16, BLACK);
    }
  }
  tft.drawCircle(PEER_CIRCLE_CENTER_X, PEER_CIRCLE_CENTER_Y,
                 PEER_CIRCLE_RADIUS, WHITE);
  for (int i = 0; i < 4; ++i) {
    float relative = normalizeBearing(bearings[i] - localHeading) * PI / 180.0f;
    int centerX = PEER_CIRCLE_CENTER_X + (int)roundf(sinf(relative) * PEER_CARDINAL_ORBIT);
    int centerY = PEER_CIRCLE_CENTER_Y - (int)roundf(cosf(relative) * PEER_CARDINAL_ORBIT);
    int x = centerX - 6;
    int y = centerY - 8;
    drawCardinalLetter(labels[i], x, y,
                       magnetometerPresent && cardinalActive(labels[i], heading));
    lastCardinalLabel[i].x = x;
    lastCardinalLabel[i].y = y;
    lastCardinalLabel[i].valid = true;
  }
}

void erasePeerDot(int id) {
  if (!lastPeerDot[id].valid) return;
  tft.fillCircle(lastPeerDot[id].x, lastPeerDot[id].y, 4, BLACK);
  lastPeerDot[id].valid = false;
}

void drawPeerDot(int deviceId, double peerLat, double peerLon, uint16_t color) {
  if (!localLocationValid() || !magnetometerPresent ||
      !validCoordinates(peerLat, peerLon)) return;

  float distance = distanceToPeer(peerLat, peerLon);
  float geographicBearing = normalizeBearing(bearingToPeer(peerLat, peerLon));
  float bearing = normalizeBearing(geographicBearing - heading);
  logPeerMeasurement(deviceId, geographicBearing, distance);

  // Fixed 100 m display scale. Closer points sit closer to the center;
  // farther points are capped at the inside of the ring.
  float radius = (distance / PEER_MAX_DISPLAY_METERS) * (PEER_CIRCLE_RADIUS - 8.0f);
  if (radius < 7.0f) radius = 7.0f;
  if (radius > PEER_CIRCLE_RADIUS - 8.0f) radius = PEER_CIRCLE_RADIUS - 8.0f;

  float rad = bearing * PI / 180.0f;
  int x = PEER_CIRCLE_CENTER_X + (int)roundf(sinf(rad) * radius);
  int y = PEER_CIRCLE_CENTER_Y - (int)roundf(cosf(rad) * radius);

  tft.fillCircle(x, y, 3, color);

  lastPeerDot[deviceId].valid = true;
  lastPeerDot[deviceId].x = x;
  lastPeerDot[deviceId].y = y;
  lastPeerDot[deviceId].color = color;
}

void clearPeerDynamicObjects() {
  // Erase only the old moving peer dots.
  // The outer compass circle, cardinal letters, and center point are preserved.
  for (int i = 1; i <= 3; i++) {
    erasePeerDot(i);
  }
}

void redrawPeerDynamicObjects() {
  // No animation and no repaint of the whole circle.
  // Old dots disappear and new dots are written at their latest coordinates immediately.
  clearPeerDynamicObjects();

  // Own device: fixed center point, using its device identity color.
  // Redraw it after erasing peer dots so a nearby peer can never erase the center.
  tft.fillCircle(PEER_CIRCLE_CENTER_X, PEER_CIRCLE_CENTER_Y,
                 3, deviceColor(DEVICE_ID));

  // Remote peers.
  for (int i = 1; i <= 3; i++) {
    if (i == DEVICE_ID) continue;
    if (remoteLocationFresh(i) && localLocationValid() && magnetometerPresent) {
      drawPeerDot(i, remote[i].lat, remote[i].lon, deviceColor(i));
    }
  }

  peerVisualInitialized = true;
}

void updatePeerDynamicText(const String &locationText, bool locationOk,
                           const String &headingText, bool headingOk) {
  // Fixed-position clear-and-redraw, matching the TIME update style.
  // No horizontal recentering means changing digits cannot slide left/right.
  if (locationText != lastPeerLocationText) {
    // Clear only the coordinate row, then center the complete value.
    tft.fillRect(2, 269, SCREEN_W - 4, 11, BLACK);
    tft.setTextSize(1);
    tft.setTextColor(locationOk ? LOCATION_GREEN : GRAY, BLACK);

    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(locationText.c_str(), 0, 273, &x1, &y1, &w, &h);
    int x = (SCREEN_W - (int)w) / 2;
    if (x < 0) x = 0;
    tft.setCursor(x, 273);
    tft.print(locationText);
    lastPeerLocationText = locationText;
  }

  if (headingText != lastPeerHeadingText) {
    // Clear only the magnetometer row, then center the complete value.
    tft.fillRect(2, 254, SCREEN_W - 4, 11, BLACK);
    tft.setTextSize(1);
    tft.setTextColor(headingOk ? WHITE : GRAY, BLACK);

    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(headingText.c_str(), 0, 258, &x1, &y1, &w, &h);
    int x = (SCREEN_W - (int)w) / 2;
    if (x < 0) x = 0;
    tft.setCursor(x, 258);
    tft.print(headingText);
    lastPeerHeadingText = headingText;
  }

  // Draw the two correct remote-device rows for this device's POV.
  int peerA = 1;
  int peerB = 2;
  if (DEVICE_ID == 1) { peerA = 2; peerB = 3; }
  else if (DEVICE_ID == 2) { peerA = 1; peerB = 3; }

  int ids[2] = {peerA, peerB};
  int ys[2] = {291, 303};

  for (int k = 0; k < 2; ++k) {
    int id = ids[k];
    int y = ys[k];

    String directionText = localLocationValid() && remoteLocationFresh(id) && magnetometerPresent
      ? shortDirection((float)relativeBearingDegrees(remote[id].lat, remote[id].lon))
      : String("--");
    String shown = localLocationValid() && remoteLocationFresh(id)
      ? String((int)roundf(distanceToPeer(remote[id].lat, remote[id].lon)))
      : String("N/A");
    String line = "LORA #" + String(id) + " | " + shown +
                  " meters away! | " + directionText;

    // Status is no longer printed in the row, but it still sets the text
    // color (gray offline / yellow asleep), so keep it in the redraw key.
    String displayKey = line + (remote[id].online ? "|ONLINE" : "|OFFLINE") +
                        (remote[id].asleep ? "|ASLEEP" : "|AWAKE");

    if (displayKey != lastPeerDistanceDisplay[id]) {
      // Clear the complete row, including the bullet, then redraw it centered.
      tft.fillRect(2, y - 1, SCREEN_W - 4, 11, BLACK);
      drawPeerLegendLine(id, y);
      lastPeerDistanceDisplay[id] = displayKey;
    }
  }
}
void updatePeerLocationDynamic(bool force) {
  if (currentScreen != PEER_LOCATION_SCREEN) return;

  String locationText;
  bool locationOk = localLocationValid();
  if (locationOk) {
    char buf[40];
    snprintf(buf, sizeof(buf), "%.6f, %.6f", latitude, longitude);
    locationText = buf;
  } else {
    locationText = "NO GPS FIX FOUND!";
  }

  String headingText;
  bool headingOk = magnetometerPresent;
  if (magnetometerPresent) {
    char buf[50];
    snprintf(buf, sizeof(buf), "%d%c, %s", (int)roundf(heading), 247, directionLong.c_str());
    headingText = buf;
  } else {
    headingText = "NO MAGNETOMETER FOUND!";
  }

  // Distance values are rendered in updatePeerDynamicText() according to this
  // device's POV: #1 -> #2/#3, #2 -> #1/#3.

  bool compassChanged = force ||
    (magnetometerPresent != lastPeerMagFound) ||
    (!magnetometerPresent ? false :
      fabsf(heading - lastPeerHeading) >= 1.0f) ||
    directionShort != lastPeerDirection;

  bool peerGpsChanged = force ||
    localLocationValid() != lastPeerGpsFix ||
    remoteLocationFresh(1) != lastPeerGpsFresh[1] ||
    remoteLocationFresh(2) != lastPeerGpsFresh[2] ||
    remoteLocationFresh(3) != lastPeerGpsFresh[3] ||
    (gpsHasFix && (fabs(latitude - lastPeerOwnLat) > 0.0000001 || fabs(longitude - lastPeerOwnLon) > 0.0000001)) ||
    remote[1].online != lastPeer1Online || remote[1].asleep != lastPeer1Asleep || remote[1].gpsValid != lastPeer1Gps ||
    fabs(remote[1].lat - lastPeer1Lat) > 0.0000001 ||
    fabs(remote[1].lon - lastPeer1Lon) > 0.0000001 ||
    remote[2].online != lastPeer2Online || remote[2].asleep != lastPeer2Asleep || remote[2].gpsValid != lastPeer2Gps ||
    remote[3].online != lastPeer3Online || remote[3].asleep != lastPeer3Asleep || remote[3].gpsValid != lastPeer3Gps ||
    fabs(remote[2].lat - lastPeer2Lat) > 0.0000001 ||
    fabs(remote[2].lon - lastPeer2Lon) > 0.0000001 ||
    fabs(remote[3].lat - lastPeer3Lat) > 0.0000001 ||
    fabs(remote[3].lon - lastPeer3Lon) > 0.0000001;

  // Redraw ONLY inside the circle when something in the compass changes.
  if (force || compassChanged || peerGpsChanged) {
    updatePeerCardinals(force || compassChanged);
    redrawPeerDynamicObjects();

    lastPeerHeading = heading;
    lastPeerDirection = directionShort;
    lastPeerGpsFix = localLocationValid();
    lastPeerMagFound = magnetometerPresent;
    lastPeer1Lat = remote[1].lat;
    lastPeer1Lon = remote[1].lon;
    lastPeer1Online = remote[1].online;
    lastPeer1Asleep = remote[1].asleep;
    lastPeer1Gps = remote[1].gpsValid;
    lastPeer2Lat = remote[2].lat;
    lastPeer2Lon = remote[2].lon;
    lastPeer3Lat = remote[3].lat;
    lastPeer3Lon = remote[3].lon;
    lastPeer2Online = remote[2].online;
    lastPeer3Online = remote[3].online;
    lastPeer2Asleep = remote[2].asleep;
    lastPeer3Asleep = remote[3].asleep;
    lastPeer2Gps = remote[2].gpsValid;
    lastPeer3Gps = remote[3].gpsValid;
    lastPeerOwnLat = latitude;
    lastPeerOwnLon = longitude;
    for (int id = 1; id <= 3; ++id) lastPeerGpsFresh[id] = remoteLocationFresh(id);
  }

  updatePeerDynamicText(locationText, locationOk, headingText, headingOk);
}

// =====================================================
// DISTANCE / BEARING

// =====================================================
// DISTANCE / BEARING
// =====================================================

float distanceToPeer(double peerLat, double peerLon) {
  if (!localLocationValid() || !validCoordinates(peerLat, peerLon)) return -1.0f;
  return (float)TinyGPSPlus::distanceBetween(latitude, longitude, peerLat, peerLon);
}

float bearingToPeer(double peerLat, double peerLon) {
  if (!localLocationValid() || !validCoordinates(peerLat, peerLon)) return -1.0f;
  return (float)TinyGPSPlus::courseTo(latitude, longitude, peerLat, peerLon);
}

// =====================================================
// TOUCH
// Main-menu hitboxes exactly match the two white rectangles.
// Peer Location gets a larger square hitbox around the visible arrow.
// =====================================================

void processTouch() {
  static bool wasTouched = false;
  static unsigned long lastAcceptedTouch = 0;
  static unsigned long compassHoldStart = 0;
  bool nowTouched = touch.touched();

  if (!nowTouched) {
    wasTouched = false;
    compassHoldStart = 0;
    return;
  }
  if (wasTouched) {
    // Holding the compass circle for 3 s starts compass calibration.
    if (compassHoldStart && millis() - compassHoldStart >= 3000UL) {
      compassHoldStart = 0;
      runMagCalibration();
      drawPeerLocation();
    }
    return;
  }
  wasTouched = true;

  digitalWrite(TFT_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(TOUCH_CS, LOW);

  TS_Point p = touch.getPoint();
  digitalWrite(TOUCH_CS, HIGH);

  // Ignore noise readings (0 or full-scale) instead of treating them as taps.
  if (p.x <= 50 || p.y <= 50 || p.x >= 4045 || p.y >= 4045) {
    return;
  }
  if (lastAcceptedTouch && millis() - lastAcceptedTouch < 180UL) return;
  lastAcceptedTouch = millis();

  float sx = touchCal[0] * (float)p.x + touchCal[1] * (float)p.y + touchCal[2];
  float sy = touchCal[3] * (float)p.x + touchCal[4] * (float)p.y + touchCal[5];

  int screenX = constrain((int)lroundf(sx), 0, 239);
  int screenY = constrain((int)lroundf(sy), 0, 319);

  Serial.printf("TOUCH raw X=%d raw Y=%d -> screen X=%d Y=%d\n",
                p.x, p.y, screenX, screenY);

  if (currentScreen == MAIN_MENU) {
    // Double-tap the TIME/DATE area to run the packet test.
    static unsigned long lastTimeTap = 0;
    if (screenX >= 20 && screenX < 220 &&
        screenY >= 35 && screenY < 110) {
      if (lastTimeTap && millis() - lastTimeTap <= 500UL) {
        lastTimeTap = 0;
        Serial.println("BUTTON: PACKET TEST (DOUBLE TAP TIME)");
        runPacketTest();
        drawMainMenu();
      } else {
        lastTimeTap = millis();
      }
      return;
    }

    // EXACT white rectangle: CHECK LOCATION
    if (screenX >= 22 && screenX < 218 &&
        screenY >= 136 && screenY < 184) {
      Serial.println("BUTTON: CHECK LOCATION");
      drawPeerLocation();
      return;
    }

    // EXACT white rectangle: SLEEP MODE
    if (screenX >= 22 && screenX < 218 &&
        screenY >= 191 && screenY < 239) {
      Serial.println("BUTTON: SLEEP MODE");
      setSleepMode(!sleepMode);
      updateMainMenuDynamic(true);
      return;
    }

    // PING TEST: only the device's own identity circle is clickable.
    int ownIconCenterX = (DEVICE_ID == 1) ? 40 : (DEVICE_ID == 2 ? 120 : 198);
    if (screenX >= ownIconCenterX - 14 &&
        screenX <= ownIconCenterX + 14 &&
        screenY >= 248 && screenY <= 280) {
      Serial.println("BUTTON: PING ALL (OWN DEVICE ICON)");
      sendPing(0);
      updateMainMenuDynamic(true);
      return;
    }

    // Specific remote device circles: ping ONLY that device.
    int targetCenters[4] = {0, 40, 120, 198};
    for (int targetId = 1; targetId <= 3; ++targetId) {
      if (targetId == DEVICE_ID) continue;

      int cx = targetCenters[targetId];
      if (screenX >= cx - 14 && screenX <= cx + 14 &&
          screenY >= 248 && screenY <= 280) {
        Serial.print("BUTTON: PING LORA #");
        Serial.println(targetId);
        sendPing(targetId);
        updateMainMenuDynamic(true);
        return;
      }
    }
  } else if (currentScreen == PEER_LOCATION_SCREEN) {
    // Larger square hitbox surrounding the arrow only.
    if (screenX >= 0 && screenX <= 42 &&
        screenY >= 0 && screenY <= 42) {
      Serial.println("BUTTON: BACK ARROW");
      drawMainMenu();
      return;
    }

    int dx = screenX - PEER_CIRCLE_CENTER_X;
    int dy = screenY - PEER_CIRCLE_CENTER_Y;
    if (magnetometerPresent &&
        dx * dx + dy * dy <= PEER_CIRCLE_RADIUS * PEER_CIRCLE_RADIUS) {
      compassHoldStart = millis();
    }
  }
}

// =====================================================
// ARDUINO ENTRY POINTS
// Kept at the end of the sketch for reliable Arduino IDE linking.
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("========================================");
  Serial.print("LORA THE EXPLORER - DEVICE #");
  Serial.println(DEVICE_ID);
  Serial.println("========================================");

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI);

  pinMode(TFT_CS, OUTPUT);
  pinMode(TOUCH_CS, OUTPUT);
  pinMode(LORA_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);

  tft.beginDisplay();
  tft.setRotation(0);
  tft.fillScreen(BLACK);
  tftHealthyMode = tft.readPowerMode();
  Serial.printf("TFT power mode: 0x%02X%s\n", tftHealthyMode,
                (tftHealthyMode == 0x00 || tftHealthyMode == 0xFF)
                  ? " (cannot read display, health check disabled)" : "");

  touch.begin();
  loadTouchCalibration();
  loadMagCalibration();
  if (touch.touched()) runTouchCalibration();

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);

  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    tft.fillScreen(BLACK);
    tft.setTextColor(RED);
    tft.setTextSize(2);
    tft.setCursor(25, 140);
    tft.print("LORA ERROR");
    while (true) delay(1000);
  }

  LoRa.setTxPower(17);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);

  // Start in continuous receive mode immediately. This prevents a device
  // that boots later from missing the other device's first heartbeat or ping.
  LoRa.receive();

  // Magnetometer starts after LoRa so an I2C problem can't stop the radio.
  detectMagnetometer();

  randomSeed((uint32_t)micros() ^ (DEVICE_ID * 7919UL));
  pingSequence = (uint16_t)random(1, 65536);
  // Default: Sleep Mode is OFF and remote status cache mirrors the
  // currently known states.
  sleepMode = false;
  radioSleeping = false;
  remoteStatusFrozen = false;
  resumeStatusSync = false;
  for (int i = 1; i <= 3; ++i) {
    cachedRemoteOnline[i] = remote[i].online;
    cachedRemoteAsleep[i] = remote[i].asleep;
  }
  remote[DEVICE_ID].asleep = false;
  nextHeartbeat = millis() + 500UL + (DEVICE_ID * 650UL);
  nextSleepLocationReport = millis() + SLEEP_LOCATION_INTERVAL;

  drawMainMenu();
  Serial.println("SYSTEM READY");
}

// =====================================================
// TOUCH CALIBRATION
// Hold a finger on the screen while powering on to start it. Tap the center
// of each cross; the result is saved in flash for this board only.
// =====================================================

void loadTouchCalibration() {
  Preferences prefs;
  prefs.begin("touch", true);
  if (prefs.getBytesLength("cal") == sizeof(touchCal)) {
    prefs.getBytes("cal", touchCal, sizeof(touchCal));
    Serial.println("Touch calibration: saved (this board)");
  } else {
    Serial.println("Touch calibration: default");
  }
  prefs.end();
}

// Wait for one tap and return the averaged raw position.
TS_Point readCalibrationTap() {
  while (true) {
    while (!touch.touched()) delay(5);

    long sumX = 0, sumY = 0;
    int samples = 0;
    while (touch.touched()) {
      TS_Point p = touch.getPoint();
      if (p.x > 0 && p.y > 0 && p.x < 4095 && p.y < 4095) {
        sumX += p.x;
        sumY += p.y;
        samples++;
      }
      delay(10);
    }
    delay(300);
    if (samples >= 3) return TS_Point(sumX / samples, sumY / samples, 0);
  }
}

// Least-squares fit of out = a*x + b*y + c over n points.
void fitTouchAxis(const float *x, const float *y, const float *out, int n,
                  float &a, float &b, float &c) {
  double sxx = 0, sxy = 0, sx = 0, syy = 0, sy = 0;
  double sxo = 0, syo = 0, so = 0;
  for (int i = 0; i < n; i++) {
    sxx += x[i] * x[i]; sxy += x[i] * y[i]; sx += x[i];
    syy += y[i] * y[i]; sy += y[i];
    sxo += x[i] * out[i]; syo += y[i] * out[i]; so += out[i];
  }
  double m[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, (double)n}};
  double v[3] = {sxo, syo, so};
  double det =
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
    m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
    m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
  double r[3];
  for (int col = 0; col < 3; col++) {
    double t[3][3];
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) t[i][j] = (j == col) ? v[i] : m[i][j];
    r[col] = (t[0][0] * (t[1][1] * t[2][2] - t[1][2] * t[2][1]) -
              t[0][1] * (t[1][0] * t[2][2] - t[1][2] * t[2][0]) +
              t[0][2] * (t[1][0] * t[2][1] - t[1][1] * t[2][0])) / det;
  }
  a = r[0]; b = r[1]; c = r[2];
}

void runTouchCalibration() {
  const int n = 5;
  const int targetX[n] = {30, 210, 30, 210, 120};
  const int targetY[n] = {30, 30, 290, 290, 160};
  float rawX[n], rawY[n], outX[n], outY[n];

  Serial.println("TOUCH CALIBRATION: release the screen, then tap each cross");

  tft.fillScreen(BLACK);
  tft.setTextSize(2);
  tft.setTextColor(WHITE, BLACK);
  drawCenteredText("TOUCH CALIBRATION", 120, WHITE, BLACK, 2);
  drawCenteredText("Release the screen", 150, YELLOW, BLACK, 1);
  while (touch.touched()) delay(10);
  delay(500);

  for (int i = 0; i < n; i++) {
    tft.fillScreen(BLACK);
    drawCenteredText("Tap the center of the cross", 140, WHITE, BLACK, 1);
    drawCenteredText(String(i + 1) + " / " + String(n), 155, YELLOW, BLACK, 1);
    tft.drawLine(targetX[i] - 12, targetY[i], targetX[i] + 12, targetY[i], RED);
    tft.drawLine(targetX[i], targetY[i] - 12, targetX[i], targetY[i] + 12, RED);
    tft.drawCircle(targetX[i], targetY[i], 5, RED);

    TS_Point p = readCalibrationTap();
    rawX[i] = p.x;
    rawY[i] = p.y;
    outX[i] = targetX[i];
    outY[i] = targetY[i];
    Serial.printf("  point %d: raw X=%d Y=%d -> target %d,%d\n",
                  i + 1, p.x, p.y, targetX[i], targetY[i]);
  }

  fitTouchAxis(rawX, rawY, outX, n, touchCal[0], touchCal[1], touchCal[2]);
  fitTouchAxis(rawX, rawY, outY, n, touchCal[3], touchCal[4], touchCal[5]);

  Preferences prefs;
  prefs.begin("touch", false);
  prefs.putBytes("cal", touchCal, sizeof(touchCal));
  prefs.end();

  Serial.printf("TOUCH CALIBRATION SAVED: %.8f %.8f %.4f | %.8f %.8f %.4f\n",
                touchCal[0], touchCal[1], touchCal[2],
                touchCal[3], touchCal[4], touchCal[5]);

  tft.fillScreen(BLACK);
  drawCenteredText("CALIBRATION SAVED", 150, GREEN, BLACK, 2);
  delay(1200);
}

// =====================================================
// COMPASS CALIBRATION
// On the CHECK LOCATION screen, hold the compass circle for 3 seconds.
// Keep the device flat and rotate it slowly through 2-3 full circles.
// The result is saved in flash for this board only.
// =====================================================

#define MAG_CAL_TIME 20000UL
#define MAG_CAL_MIN_SPAN 200

void loadMagCalibration() {
  Preferences prefs;
  prefs.begin("mag", true);
  if (prefs.getBytesLength("cal") == sizeof(magCal)) {
    prefs.getBytes("cal", magCal, sizeof(magCal));
    Serial.println("Compass calibration: saved (this board)");
  } else {
    Serial.println("Compass calibration: default");
  }
  prefs.end();
}

void runMagCalibration() {
  if (!magnetometerPresent) return;

  Serial.println("COMPASS CALIBRATION: rotate the device flat through full circles");

  tft.fillScreen(BLACK);
  drawCenteredText("COMPASS CALIBRATION", 70, WHITE, BLACK, 2);
  drawCenteredText("Keep the device FLAT", 110, YELLOW, BLACK, 1);
  drawCenteredText("Rotate it slowly in full circles", 125, YELLOW, BLACK, 1);
  drawCenteredText("(2-3 turns) until the timer ends", 140, YELLOW, BLACK, 1);

  int16_t minX = 32767, maxX = -32768, minY = 32767, maxY = -32768;
  unsigned long start = millis();
  unsigned long lastDraw = 0;
  unsigned long lastBeat = millis();

  while (millis() - start < MAG_CAL_TIME) {
    if (readMagnetometer()) {
      if (magX < minX) minX = magX;
      if (magX > maxX) maxX = magX;
      if (magY < minY) minY = magY;
      if (magY > maxY) maxY = magY;
    }

    // Keep this device ONLINE for the others during calibration.
    receiveLoRa();
    if (millis() - lastBeat >= HEARTBEAT_INTERVAL) {
      lastBeat = millis();
      sendHeartbeat();
    }

    if (millis() - lastDraw >= 500UL) {
      lastDraw = millis();
      int secondsLeft = (int)((MAG_CAL_TIME - (millis() - start) + 999) / 1000);
      int spanX = maxX > minX ? maxX - minX : 0;
      int spanY = maxY > minY ? maxY - minY : 0;
      tft.fillRect(0, 175, SCREEN_W, 50, BLACK);
      drawCenteredText(String(secondsLeft) + " s", 180, GREEN, BLACK, 2);
      drawCenteredText("X range " + String(spanX) + "   Y range " + String(spanY),
                       210, WHITE, BLACK, 1);
    }
    delay(50);
  }

  int spanX = maxX - minX;
  int spanY = maxY - minY;
  Serial.printf("  X: %d..%d (range %d)  Y: %d..%d (range %d)\n",
                minX, maxX, spanX, minY, maxY, spanY);

  tft.fillScreen(BLACK);
  if (spanX < MAG_CAL_MIN_SPAN || spanY < MAG_CAL_MIN_SPAN) {
    Serial.println("COMPASS CALIBRATION FAILED: not enough rotation, old values kept");
    drawCenteredText("CALIBRATION FAILED", 140, RED, BLACK, 2);
    drawCenteredText("Rotate in FULL circles. Try again.", 170, WHITE, BLACK, 1);
    delay(2500);
    return;
  }

  float avgSpan = (spanX + spanY) / 2.0f;
  magCal[0] = (maxX + minX) / 2.0f;
  magCal[1] = (maxY + minY) / 2.0f;
  magCal[2] = avgSpan / spanX;
  magCal[3] = avgSpan / spanY;

  Preferences prefs;
  prefs.begin("mag", false);
  prefs.putBytes("cal", magCal, sizeof(magCal));
  prefs.end();

  Serial.printf("COMPASS CALIBRATION SAVED: offset X=%.1f Y=%.1f  scale X=%.3f Y=%.3f\n",
                magCal[0], magCal[1], magCal[2], magCal[3]);
  drawCenteredText("CALIBRATION SAVED", 150, GREEN, BLACK, 2);
  delay(1500);
}

// =====================================================
// PACKET TEST
// Double-tap the TIME on the main menu. This device sends TEST_PACKET_COUNT
// test packets; every awake peer echoes each one back in its own time slot
// (so peers never collide). The result shows, per peer, how many packets
// made the round trip, the round-trip time, and signal strength both ways.
// =====================================================

#define TEST_PACKET_COUNT 20
#define TEST_PACKET_INTERVAL 300UL
#define TEST_REPLY_SLOT 70UL

// One entry per test packet per peer.
struct PacketSample {
  bool received;
  uint16_t rttMs;
  int16_t rssiHere;    // RSSI of the reply, measured on this device
  float snrHere;
  int16_t rssiThere;   // RSSI of our packet, measured on the peer
  float snrThere;
};

bool packetTestRunning = false;
int packetTestId = 0;
// Set while another device's test packets are arriving, so this device
// doesn't start its own test at the same time (the packets would collide).
unsigned long peerTestActiveUntil = 0;
int peerTestDevice = 0;
unsigned long packetTestSentAt[TEST_PACKET_COUNT];
PacketSample packetSamples[4][TEST_PACKET_COUNT];

// Each device answers in its own slot so replies from #2 and #3 don't collide.
unsigned long packetTestReplyDelay(int id) {
  return 10UL + (unsigned long)(id - 1) * TEST_REPLY_SLOT;
}

void sendRawLoRa(const String &packet) {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);
  LoRa.idle();
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();
  LoRa.receive();
}

void handlePacketTest(const String &packet) {
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (packet.startsWith("T,")) {
    // T,<sender>,<seq> -> echo it back.
    int comma = packet.indexOf(',', 2);
    if (comma < 0) return;
    int sender = packet.substring(2, comma).toInt();
    int seq = packet.substring(comma + 1).toInt();
    if (sender < 1 || sender > 3 || sender == DEVICE_ID || sleepMode) return;

    peerTestActiveUntil = millis() + 2000UL;
    peerTestDevice = sender;

    // Busy with our own test: don't transmit over it.
    if (packetTestRunning) return;

    remote[sender].lastSeen = millis();
    remote[sender].online = true;
    cachedRemoteOnline[sender] = true;

    delay(packetTestReplyDelay(DEVICE_ID));
    sendRawLoRa("TR," + String(DEVICE_ID) + "," + String(sender) + "," +
                String(seq) + "," + String(rssi) + "," + String(snr, 1));
    return;
  }

  // TR,<responder>,<to>,<seq>,<rssi>,<snr>
  String f[5];
  int startIndex = 3;
  int count = 0;
  while (count < 5) {
    int comma = packet.indexOf(',', startIndex);
    if (comma < 0) {
      f[count++] = packet.substring(startIndex);
      break;
    }
    f[count++] = packet.substring(startIndex, comma);
    startIndex = comma + 1;
  }
  if (count < 5) return;

  int responder = f[0].toInt();
  int to = f[1].toInt();
  int seq = f[2].toInt();
  if (!packetTestRunning || to != DEVICE_ID) return;
  if (responder < 1 || responder > 3 || responder == DEVICE_ID) return;
  if (seq < 0 || seq >= TEST_PACKET_COUNT || packetTestSentAt[seq] == 0) return;

  remote[responder].lastSeen = millis();
  remote[responder].online = true;
  cachedRemoteOnline[responder] = true;

  PacketSample &s = packetSamples[responder][seq];
  if (s.received) return;   // duplicate

  // A late reply (arriving after the next packet was sent) still counts,
  // timed against its own packet.
  unsigned long rtt = millis() - packetTestSentAt[seq];
  unsigned long slot = packetTestReplyDelay(responder);
  rtt = rtt > slot ? rtt - slot : 0;

  s.received = true;
  s.rttMs = (uint16_t)min(rtt, 65535UL);
  s.rssiHere = rssi;
  s.snrHere = snr;
  s.rssiThere = f[3].toInt();
  s.snrThere = f[4].toFloat();
}

// Keep receiving and sending heartbeats while a blocking screen is shown,
// so the other devices don't mark this one OFFLINE.
void serviceRadio() {
  receiveLoRa();
  if (!sleepMode && (long)(millis() - nextHeartbeat) >= 0) {
    sendHeartbeat();
    nextHeartbeat = millis() + HEARTBEAT_INTERVAL;
  }
}

// GPS distance to a peer in meters, or -1 when either side has no GPS fix.
float packetTestDistance(int id) {
  if (!localLocationValid() || !remoteLocationFresh(id)) return -1.0f;
  return distanceToPeer(remote[id].lat, remote[id].lon);
}

String csvDistance(float meters) {
  return meters < 0 ? String("") : String(meters, 1);
}

// Serial CSV output (saved on the PC by tools/packet_logger.py):
// CSV,device,test,seq,peer,status,rtt_ms,rssi_here,snr_here,rssi_there,snr_there,distance_m,own_lat,own_lon,peer_lat,peer_lon
// SUM,device,test,peer,received,sent,success_pct,avg_rtt_ms,avg_rssi_here,avg_snr_here,avg_rssi_there,avg_snr_there,distance_m
void printPacketTestCsv() {
  for (int id = 1; id <= 3; id++) {
    if (id == DEVICE_ID) continue;
    String dist = csvDistance(packetTestDistance(id));
    String ownPos = gpsHasFix ? String(latitude, 6) + "," + String(longitude, 6) : String(",");
    String peerPos = remote[id].gpsValid
      ? String(remote[id].lat, 6) + "," + String(remote[id].lon, 6) : String(",");

    for (int seq = 0; seq < TEST_PACKET_COUNT; seq++) {
      PacketSample &s = packetSamples[id][seq];
      String line = "CSV," + String(DEVICE_ID) + "," + String(packetTestId) + "," +
                    String(seq) + "," + String(id) + ",";
      if (s.received) {
        line += "OK," + String(s.rttMs) + "," + String(s.rssiHere) + "," +
                String(s.snrHere, 1) + "," + String(s.rssiThere) + "," +
                String(s.snrThere, 1) + ",";
      } else {
        line += "LOST,,,,,,";
      }
      line += dist + "," + ownPos + "," + peerPos;
      Serial.println(line);
    }
  }
}

void runPacketTest() {
  if (sleepMode) {
    pingMessage = "Turn off SLEEP MODE to test";
    pingMessageTime = millis();
    return;
  }

  if ((long)(peerTestActiveUntil - millis()) > 0) {
    Serial.printf("PACKET TEST BLOCKED: LORA #%d is testing\n", peerTestDevice);
    pingMessage = "LORA #" + String(peerTestDevice) + " is testing, please wait";
    pingMessageTime = millis();
    return;
  }

  // Listen for a random moment first: if another device just started its
  // test, its packets arrive here and this test is cancelled.
  unsigned long listenStart = millis();
  unsigned long listenFor = 300UL + (unsigned long)random(400);
  while (millis() - listenStart < listenFor) {
    receiveLoRa();
    delay(1);
  }
  if ((long)(peerTestActiveUntil - millis()) > 0) {
    Serial.printf("PACKET TEST BLOCKED: LORA #%d is testing\n", peerTestDevice);
    pingMessage = "LORA #" + String(peerTestDevice) + " is testing, please wait";
    pingMessageTime = millis();
    return;
  }

  packetTestId++;
  memset(packetSamples, 0, sizeof(packetSamples));
  memset(packetTestSentAt, 0, sizeof(packetTestSentAt));

  tft.fillScreen(BLACK);
  drawCenteredText("PACKET TEST", 20, WHITE, BLACK, 2);
  drawCenteredText("Sending " + String(TEST_PACKET_COUNT) + " packets...", 50, YELLOW, BLACK, 1);

  Serial.printf("PACKET TEST START device=%d test=%d\n", DEVICE_ID, packetTestId);
  packetTestRunning = true;

  for (int seq = 0; seq < TEST_PACKET_COUNT; seq++) {
    packetTestSentAt[seq] = millis();
    sendRawLoRa("T," + String(DEVICE_ID) + "," + String(seq));

    while (millis() - packetTestSentAt[seq] < TEST_PACKET_INTERVAL) {
      receiveLoRa();
      delay(1);
    }

    tft.fillRect(0, 80, SCREEN_W, 40, BLACK);
    drawCenteredText(String(seq + 1) + " / " + String(TEST_PACKET_COUNT), 85, GREEN, BLACK, 2);
  }

  // Short grace period for the last replies.
  unsigned long graceStart = millis();
  while (millis() - graceStart < TEST_PACKET_INTERVAL) {
    receiveLoRa();
    delay(1);
  }

  packetTestRunning = false;
  nextHeartbeat = millis() + HEARTBEAT_INTERVAL;

  printPacketTestCsv();

  // ---- Results ----
  tft.fillScreen(BLACK);
  drawCenteredText("PACKET TEST RESULT", 15, WHITE, BLACK, 2);

  int y = 45;
  bool anyReply = false;
  for (int id = 1; id <= 3; id++) {
    if (id == DEVICE_ID) continue;

    int received = 0;
    unsigned long rttSum = 0;
    long rssiHereSum = 0, rssiThereSum = 0;
    float snrHereSum = 0, snrThereSum = 0;
    for (int seq = 0; seq < TEST_PACKET_COUNT; seq++) {
      PacketSample &s = packetSamples[id][seq];
      if (!s.received) continue;
      received++;
      rttSum += s.rttMs;
      rssiHereSum += s.rssiHere;
      snrHereSum += s.snrHere;
      rssiThereSum += s.rssiThere;
      snrThereSum += s.snrThere;
    }
    int pct = (received * 100) / TEST_PACKET_COUNT;
    float distance = packetTestDistance(id);

    uint16_t color = pct >= 90 ? GREEN : (pct >= 50 ? YELLOW : RED);
    if (received == 0) color = GRAY;

    drawCenteredText("LORA #" + String(id) + ": " + String(received) + "/" +
                     String(TEST_PACKET_COUNT) + " (" + String(pct) + "%)",
                     y, color, BLACK, 2);

    String distText = distance < 0 ? String("Distance: no GPS fix")
                                   : "Distance: " + String((int)roundf(distance)) + " m (GPS)";

    String sum = "SUM," + String(DEVICE_ID) + "," + String(packetTestId) + "," +
                 String(id) + "," + String(received) + "," + String(TEST_PACKET_COUNT) + "," +
                 String(pct) + ",";

    if (received > 0) {
      anyReply = true;
      drawCenteredText("Avg round trip: " + String(rttSum / received) + " ms",
                       y + 25, WHITE, BLACK, 1);
      drawCenteredText("Signal here:  " + String(rssiHereSum / received) + " dBm, SNR " +
                       String(snrHereSum / received, 1) + " dB",
                       y + 40, WHITE, BLACK, 1);
      drawCenteredText("Signal there: " + String(rssiThereSum / received) + " dBm, SNR " +
                       String(snrThereSum / received, 1) + " dB",
                       y + 55, WHITE, BLACK, 1);
      sum += String(rttSum / received) + "," + String(rssiHereSum / received) + "," +
             String(snrHereSum / received, 1) + "," + String(rssiThereSum / received) + "," +
             String(snrThereSum / received, 1) + ",";
    } else {
      drawCenteredText("No reply (OFF, asleep, or out of range)", y + 25, GRAY, BLACK, 1);
      sum += ",,,,,";
    }
    drawCenteredText(distText, y + 70, distance < 0 ? GRAY : LOCATION_GREEN, BLACK, 1);
    sum += csvDistance(distance);
    Serial.println(sum);

    y += 100;
  }

  if (anyReply) {
    drawCenteredText("RSSI: -30 great ... -120 very weak", 255, GRAY, BLACK, 1);
  }
  drawCenteredText("Tap anywhere to go back", 295, YELLOW, BLACK, 1);
  Serial.println("PACKET TEST END");

  // Wait for the finger to lift, then for a new tap (or 30 s).
  unsigned long shownAt = millis();
  while (touch.touched() && millis() - shownAt < 30000UL) { serviceRadio(); delay(5); }
  while (!touch.touched() && millis() - shownAt < 30000UL) { serviceRadio(); delay(5); }
  while (touch.touched() && millis() - shownAt < 31000UL) { serviceRadio(); delay(5); }
}

// If the display controller lost its setup (power dip, glitch on the SPI
// lines), re-initialize it and redraw the current screen so it never stays
// frozen.
void checkDisplayHealth() {
  uint8_t mode = tft.readPowerMode();

  Serial.printf("[status] TFT=0x%02X  #1=%s #2=%s #3=%s\n", mode,
                remote[1].online ? (remote[1].asleep ? "ASLEEP" : "ONLINE") : "OFFLINE",
                remote[2].online ? (remote[2].asleep ? "ASLEEP" : "ONLINE") : "OFFLINE",
                remote[3].online ? (remote[3].asleep ? "ASLEEP" : "ONLINE") : "OFFLINE");

  if (tftHealthyMode == 0x00 || tftHealthyMode == 0xFF) return;
  if (mode == tftHealthyMode) return;

  Serial.printf("TFT not responding (0x%02X, expected 0x%02X) -> re-initializing\n",
                mode, tftHealthyMode);
  tft.beginDisplay();
  tft.setRotation(0);
  if (currentScreen == MAIN_MENU) drawMainMenu();
  else drawPeerLocation();
}

void loop() {
  updateGPS();
  updateCompass();
  receiveLoRa();
  updatePing();
  updateDeviceStatus();
  processTouch();

  if (sleepMode) {
    if ((long)(millis() - nextSleepLocationReport) >= 0) {
      sendSleepLocationReport();
      nextSleepLocationReport = millis() + SLEEP_LOCATION_INTERVAL;
    }
  } else {
    if ((long)(millis() - nextHeartbeat) >= 0) {
      sendHeartbeat();
      nextHeartbeat = millis() + HEARTBEAT_INTERVAL;
    }
  }

  bool forceRefresh = false;
  if (millis() - lastScreenRefresh >= SCREEN_REFRESH_INTERVAL) {
    lastScreenRefresh = millis();
    forceRefresh = true;
    checkDisplayHealth();
  }

  if (currentScreen == MAIN_MENU) {
    if (forceRefresh || millis() - lastMainDynamic >= MAIN_DYNAMIC_INTERVAL) {
      lastMainDynamic = millis();
      updateMainMenuDynamic(forceRefresh);
    }
  } else {
    if (forceRefresh || millis() - lastPeerDynamic >= PEER_DYNAMIC_INTERVAL) {
      lastPeerDynamic = millis();
      updatePeerLocationDynamic(forceRefresh);
    }
  }

  delay(2);
}
