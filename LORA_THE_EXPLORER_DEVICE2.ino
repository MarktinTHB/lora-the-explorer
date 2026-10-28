// Current Version: v2.2.5.1
// Fixed devices showing OFFLINE during and right after a packet test and flickering OFFLINE at long range, and packet tests reading 0/20 when only the replies were lost.
// No device is timed out while a packet test runs (heartbeats are held then) or for 10 s after; awake devices go OFFLINE after 26 s (5 heartbeats) instead of 16 s; the device answering a test now shows its own count ("Test from LORA #1: got 17/20") for 60 s, and the tester says when no reply came back; the radio's settings are checked every 10 s and restored if a supply dip reset the chip.

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

#define DEVICE_ID 2

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
// Radio settings, shared by all three devices (a device with different
// settings cannot hear the others at all).
// v2.x.5.0 range extension: SF7 -> SF10 (+9 dB receiver sensitivity,
// datasheet -123 -> -132 dBm) and 17 -> 20 dBm (+3 dB, RFM95W maximum).
// SF10 is the highest spreading factor at which a heartbeat (~0.66 s on
// air) still arrives within the thesis latency limit of 1000 ms; SF11/SF12
// would exceed it. Packets take ~6x longer on air, so the timings below
// (heartbeat slots, sleep window, ping and packet test slots) are scaled.
#define LORA_SF        10
#define LORA_BW_HZ     125E3
#define LORA_TX_DBM    20

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
// [4] = raw sensor angle (degrees) of the direction the device faced when
//       calibration started; that direction becomes 0 degrees.
// [5] = +1 or -1 so that the heading increases as the device turns clockwise,
//       whichever way the module is mounted.
// Defaults (0, +1) reproduce the previous behaviour until a calibration runs.
#define MAG_CAL_ZERO 4
#define MAG_CAL_DIR  5
float magCal[6] = {MAG_X_OFFSET, MAG_Y_OFFSET, 1.0f, 1.0f, 0.0f, 1.0f};

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
// Automatic proximity sleep (manual sleep stays YELLOW).
#define AQUA        0x07FF

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

// Position smoothing: while the device stands still, latitude/longitude are
// the average of the last GPS_SMOOTH_SAMPLES fixes (1 per second), which
// removes second-to-second jumps. Moving faster than GPS_SMOOTH_MAX_SPEED_MPS
// narrows the average to the last GPS_SMOOTH_MOVING_SAMPLES fixes (so a
// walking user is not shown lagging behind) instead of dropping it: the old
// restart showed one raw, jumpy fix whenever the GPS speed read above 1 m/s,
// which NEO-6M speed noise does even when standing still. Back to the full
// average below GPS_SMOOTH_STILL_SPEED_MPS. A single fix farther than
// GPS_SMOOTH_JUMP_M from the average is a GPS glitch and is ignored; only
// GPS_SMOOTH_JUMP_CONFIRM such fixes in a row restart the average.
#define GPS_SMOOTH_SAMPLES        10
#define GPS_SMOOTH_MOVING_SAMPLES 3
#define GPS_SMOOTH_MAX_SPEED_MPS  1.0f
#define GPS_SMOOTH_STILL_SPEED_MPS 0.5f
#define GPS_SMOOTH_JUMP_M         15.0
#define GPS_SMOOTH_JUMP_CONFIRM   3
double gpsSmoothLat[GPS_SMOOTH_SAMPLES];
double gpsSmoothLon[GPS_SMOOTH_SAMPLES];
int gpsSmoothCount = 0;
int gpsSmoothNext = 0;
bool gpsSmoothMoving = false;
int gpsSmoothFarFixes = 0;

// Distance calibration ("zeroing"): with the devices side by side, the GPS
// offset to each peer (which should be 0 m) is measured and subtracted from
// later distances and directions, and the GPS wander seen during the 30 s
// becomes a noise floor: readings within it are pulled toward 0 m, fading
// out by DIST_ZERO_TAPER_M, so devices together read ~0 m and the distance
// rises as they separate. Saved in flash; it stays until recalibrated.
#define DIST_CAL_TIME_MS            30000UL
#define DIST_CAL_MIN_OWN_SAMPLES    10
#define DIST_CAL_MIN_PEER_SAMPLES   3
// An offset larger than this means the devices were not together (or a fix
// was bad), so it is refused instead of applied.
#define DIST_CAL_MAX_OFFSET_M       20.0f
// Noise floor = 2 x the RMS wander measured during calibration, limited to
// this range (a 30 s sample underestimates slower GPS drift, hence the minimum).
#define DIST_NOISE_FLOOR_MIN_M      5.0f
#define DIST_NOISE_FLOOR_MAX_M      8.0f
// Above this distance readings are not adjusted at all.
#define DIST_ZERO_TAPER_M           30.0f
// Below this shown distance the bearing is GPS noise, so "--" is shown.
#define PEER_BEARING_MIN_DISPLAY_M  2.0f
// Fastest the shown distance to a peer may change, in m/s (brisk walking
// apart). A GPS jump is shown as a short ramp instead of an instant leap.
#define DIST_DISPLAY_MAX_RATE_MPS   2.0f
struct DistanceCalibration {
  bool valid = false;
  float north = 0.0f;   // metres; peer's apparent offset when side by side
  float east = 0.0f;
  float noiseFloor = 0.0f;   // metres
};
DistanceCalibration distCal[4];

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
  // Asleep because of automatic proximity sleep (SLP code 2), as opposed to
  // the manual 5-minute sleep mode (SLP code 1).
  bool proximitySleep = false;
  // Last time a valid compass heading was received from this device.
  unsigned long headingUpdatedAt = 0;
  // This device's own proximity reading, shared in its heartbeats:
  // 0 = group apart, 1 = group together (under 10 m), 2 = unknown (no GPS),
  // -1 = not reported (older firmware). See evaluateGroupProximity().
  int8_t proxVote = -1;
  unsigned long proxVoteAt = 0;
};

RemoteDevice remote[4];

// Heartbeats: every 5 s. With GPS time each device sends in its own slot of
// a shared 5 s cycle (ID 1 at 0.2 s, ID 2 at 1.0 s, ID 3 at 1.8 s - see
// heartbeatDue()), so at SF10 (~0.66 s per heartbeat) they never collide;
// the same slots fall inside the proximity-sleep listening window. Without
// GPS time it falls back to a timer.
#define HEARTBEAT_INTERVAL 5000UL
#define HEARTBEAT_CYCLE_MS 5000UL
// A slot that the loop reaches later than this is skipped (it would run
// into the next device's slot); the next cycle is 5 s later.
#define HEARTBEAT_SLOT_LATE_MS 100UL
// Random 0-1000 ms added to each timer heartbeat (no GPS time). With an
// exact period, two devices that fall into step keep transmitting at the
// same moment on every cycle: neither hears the other (half-duplex) and the
// third hears a collision, so both drop to OFFLINE.
#define HEARTBEAT_JITTER_MS 1000UL
// An awake device is considered offline after 26 seconds of no
// heartbeat/ping communication (5 missed heartbeats). Near the edge of range
// a few heartbeats in a row can be lost; 3 (16 s) made devices flicker
// OFFLINE there.
#define DEVICE_TIMEOUT 26000UL
// While a packet test runs, heartbeats are held on purpose, so no device is
// timed out until this time (the test plus two heartbeat cycles after it).
unsigned long statusTimeoutHoldUntil = 0;
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
// Extra sleep reports sent after entering Sleep Mode. A single report that
// collided left the peers showing this device OFFLINE for 5 minutes.
#define SLEEP_ANNOUNCE_REPEATS 2
#define SLEEP_ANNOUNCE_GAP_MS 1500UL
int sleepAnnounceRepeats = 0;
uint32_t lastSleepReportCycle = 0xFFFFFFFFUL;

// When THIS device enters Sleep Mode, keep the last known remote statuses
// frozen on its screen instead of turning them OFFLINE while the radio sleeps.
bool remoteStatusFrozen = false;
bool resumeStatusSync = false;
unsigned long resumeStatusSyncUntil = 0;
bool cachedRemoteOnline[4] = {false, false, false, false};
bool cachedRemoteAsleep[4] = {false, false, false, false};

// =====================================================
// AUTOMATIC PROXIMITY SLEEP
// =====================================================
// Rules:
//  - The group is "together" when every pairwise distance (as shown on
//    screen) is BELOW PROX_THRESHOLD_M - all three pairs, or the remaining
//    pair when exactly one device is confirmed offline (that device is left
//    awake for recovery) - AND every other device in the group reports in
//    its heartbeat that it sees the group together too. Each device measures
//    with its own GPS and calibration, so without this agreement one device
//    could read 9 m and sleep while another read 11 m and stayed awake.
//  - Enter: together continuously for PROX_ENTRY_CONFIRM_MS (5 s).
//  - Exit: apart continuously for PROX_EXIT_OUT_OF_RANGE_MS (5 s); a device
//    that wakes reports "apart", so the rest of the group wakes with it.
//    Any opposite reading restarts the 5 s.
//  - Missing/stale GPS pauses the decision (PROX_UNKNOWN); it never counts
//    as being apart and never changes the current sleep state.
// Note: the thesis text specifies 50 m; this build uses 10 m as requested.
#define PROX_THRESHOLD_M           10.0f
#define PROX_ENTRY_CONFIRM_MS      5000UL
#define PROX_EXIT_OUT_OF_RANGE_MS  5000UL
// Checked every second (the GPS updates once per second).
#define PROX_EVAL_INTERVAL_MS      1000UL
// Peer coordinates older than this are not used for a proximity decision.
#define PROX_LOCATION_MAX_AGE_MS   25000UL
// After waking, wait this long (fresh GPS + status) before a new decision.
#define PROX_RECOVERY_HOLD_MS      20000UL
// A peer counts as confirmed offline only after this much silence.
#define PROX_OFFLINE_CONFIRM_MS    60000UL
// Online timeout for peers heard only in proximity-sleep windows.
#define PROX_PEER_TIMEOUT          45000UL
// While asleep the radio is on only in a 2 s window every 10 s, aligned to
// GPS UTC time so all devices listen at the same moment. Each device sends
// its report in its own slot inside that window.
#define PROX_CYCLE_MS              10000UL
// (v2.x.5.0: window 2 s -> 3 s and slots 0.4 s -> 0.8 s apart, because a
// report takes ~0.5 s and a heartbeat ~0.66 s on air at SF10.)
#define PROX_WINDOW_MS             3000UL
#define PROX_SLOT_BASE_MS          200UL
#define PROX_SLOT_SPACING_MS       800UL
// Without GPS time the radio stays on and reports on this local timer. It
// is shorter than the 2 s listening window, so sleeping peers (which only
// listen in that window) still hear at least one report every cycle; at
// 10 s they mostly missed it and showed this device OFFLINE. (2.4 s = the
// 3 s window minus one ~0.5 s report at SF10.)
#define PROX_UNSYNCED_REPORT_MS    2400UL
// The entry report is repeated once after this delay, so a peer that missed
// the first one doesn't show this device OFFLINE until the next window.
#define PROX_ENTRY_REPEAT_MS       1000UL

enum ProximityState {
  PROX_NORMAL_OPERATION,
  PROX_CONFIRMATION,
  PROX_SLEEP,
  PROX_OUT_OF_RANGE_MONITORING,
  PROX_WAKE_AND_RECOVERY,
  PROX_UNKNOWN
};

enum ProximityReading {
  PROX_READING_IN_RANGE,
  PROX_READING_OUT_OF_RANGE,
  PROX_READING_UNKNOWN
};

ProximityState proxState = PROX_NORMAL_OPERATION;
ProximityState proxStateBeforeUnknown = PROX_NORMAL_OPERATION;
bool proxSleepActive = false;
// Developer setting (hold the title on the main menu for 3 s): automatic
// proximity sleep on/off. Saved in flash. When off, this device stays awake
// and reports "apart", so the rest of the group stays awake too.
bool proxSleepEnabled = true;
unsigned long proxEntryRepeatAt = 0;
unsigned long proxLastEvalAt = 0;
// Time accumulated in the current confirmation / out-of-range period.
unsigned long proxAccumMs = 0;
// proxChainValid is false after an unknown reading, so the gap is not counted.
bool proxChainValid = false;
unsigned long proxChainAt = 0;
unsigned long proxRecoveryUntil = 0;
int proxOfflineDevice = 0;
ProximityReading proxLastReading = PROX_READING_UNKNOWN;
// This device's own distance reading before the other devices' votes are
// applied. This is what it reports as its vote: reporting the combined
// result instead would let two devices that each start from "apart" keep
// each other apart forever.
ProximityReading proxOwnGeometryReading = PROX_READING_UNKNOWN;
unsigned long proxLastLogAt = 0;
uint32_t proxLastReportCycle = 0xFFFFFFFFUL;
uint32_t proxLastSlotHeartbeatCycle = 0xFFFFFFFFUL;
unsigned long proxLastUnsyncedReport = 0;

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
// (SF10: an ACK takes ~0.29 s on air, so slots are 400 ms apart.)
#define PING_BROADCAST_WINDOW 2200UL
#define PING_BROADCAST_SLOT 400UL
uint16_t pingSequence = 0;
bool pingPending = false;
int pendingPingTarget = 0;
bool pendingPingBroadcast = false;
uint16_t pendingPingSequence = 0;
unsigned long pendingPingStartedAt = 0;
// Shown on the main menu of a device that answered a packet test: how many
// of the tester's packets it received (the one-way count, measured here).
String peerTestResultMessage = "";
unsigned long peerTestResultAt = 0;
#define PEER_TEST_RESULT_SHOW_MS 60000UL
bool broadcastAckSeen[4] = {false, false, false, false};
uint8_t broadcastAckCount = 0;
// Round trip per broadcast responder, with its reply slot taken out.
unsigned long broadcastAckRtt[4] = {0, 0, 0, 0};
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
  const float bandwidth = (float)LORA_BW_HZ;
  const int sf = LORA_SF;
  const int cr = 1;       // 4/5
  const int ih = 0;       // explicit header
  const int crc = 1;
  const int preamble = 8;

  const float symbolTimeMs = ((float)(1UL << sf) / bandwidth) * 1000.0f;
  // Low data rate optimisation is on when a symbol lasts 16 ms or more.
  const int de = symbolTimeMs >= 16.0f ? 1 : 0;
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
void serviceRadio();
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
void configureLoRaRadio();
void checkRadioHealth();
bool heartbeatDue();
void heartbeatSent();
void requestStatusSync();
void sendStateResponse(int targetId);
void sendSleepLocationReport();
void setSleepMode(bool enabled);
void sendPing(int targetId);
void updatePing();
void acknowledgePing(int requesterId, int targetId, uint16_t sequence,
                     unsigned long receivedAt);
void receiveLoRa();
void handlePacketTest(const String &packet, unsigned long receivedAt);
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
// headingFlag: 1 = valid heading in packet, 0 = sender reports no
// magnetometer, -1 = packet carried no heading field (keep the last value).
void updateRemoteLocation(int deviceId, bool gpsValid, double lat, double lon,
                          int headingFlag, float remoteHeading);
unsigned long remoteDataMaxAge(int deviceId);
bool remoteHeadingCurrent(int deviceId);

void updateProximity();
void serviceProximitySleep();
void enterProximitySleep();
void exitProximitySleep();
void sendProximitySleepReport();
ProximityReading evaluateGroupProximity(int &offlineDevice, float distances[3]);
ProximityReading proxPeerVote(int deviceId);
int proxOwnVote();
bool peerConfirmedOffline(int deviceId);
bool peerLocationFreshForProximity(int deviceId);
bool anyPeerInProximitySleep();
bool gpsUtcMillisOfDay(uint32_t &msOfDay);
const char *proximityStateName(ProximityState state);
void logPeerMeasurement(int deviceId, float bearing, float distance);
int relativeBearingDegrees(double peerLat, double peerLon);

void addSmoothedFix(double lat, double lon);
void offsetMeters(double fromLat, double fromLon, double toLat, double toLon,
                  float &north, float &east);
bool distCalActive(int deviceId);
bool peerOffsetMeters(int deviceId, float &north, float &east);
float peerDistance(int deviceId);
float peerBearing(int deviceId);
int peerRelativeBearingDegrees(int deviceId);
float peerPairDistance(int idA, int idB);
float applyZeroFloor(float distance, float noiseFloor);
float peerDisplayDistance(int deviceId);
float peerPairDisplayDistance(int idA, int idB);
String peerRowText(int deviceId);
void loadDistanceCalibration();
void loadDeveloperSettings();
void runDeveloperScreen();
bool sleepReportSlotReady();
void saveDistanceCalibration();
void runDistanceCalibration();
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
bool lastMainProximitySleep[4] = {false, false, false, false};

String lastPeerLocationText = "";
String lastPeerHeadingText = "";
String lastPeerCalText = "";
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
  // Short line from the dot showing which way that peer is facing.
  bool tickValid = false;
  int tickX = 0;
  int tickY = 0;
};
PeerDotCache lastPeerDot[4];
// Remote heading last drawn as a tick, in 5-degree steps (-1 = none), so a
// peer turning redraws its tick without redrawing on every tiny change.
int lastPeerRemoteHeadingStep[4] = {-1, -1, -1, -1};
// Facing tick length from the dot centre. Dots sit at most
// PEER_CIRCLE_RADIUS - 8 from the centre, so 6 px keeps the tick inside
// the circle outline.
#define PEER_HEADING_TICK_LEN 6.0f
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

  // GPS diagnostics every 5 s, to tell wiring, data loss and sky view apart:
  //  chars not increasing     -> no data: check GPS power and TX/RX wiring
  //  bad_csum increasing      -> data is arriving damaged (lost bytes/noise)
  //  sats 0-3, no fix         -> data is fine; the module cannot see enough
  //                              satellites (indoors, antenna, cold start)
  static unsigned long lastGpsStat = 0;
  if (millis() - lastGpsStat >= 5000UL) {
    lastGpsStat = millis();
    Serial.printf("GPS STAT chars=%lu ok_sentences=%lu bad_csum=%lu fix_sentences=%lu sats=%lu hdop=%.1f fix=%d loc_age_ms=%ld\n",
                  (unsigned long)gps.charsProcessed(),
                  (unsigned long)gps.passedChecksum(),
                  (unsigned long)gps.failedChecksum(),
                  (unsigned long)gps.sentencesWithFix(),
                  gps.satellites.isValid() ? (unsigned long)gps.satellites.value() : 0UL,
                  gps.hdop.isValid() ? gps.hdop.hdop() : 99.9,
                  gps.location.isValid() ? 1 : 0,
                  gps.location.isValid() ? (long)gps.location.age() : -1L);
  }

  bool fixFresh = gps.location.isValid() && gps.location.age() <= 5000UL;
  // isUpdated() must be read before lat()/lng(), which clear it.
  bool newFix = fixFresh && gps.location.isUpdated();
  double rawLat = fixFresh ? gps.location.lat() : 0.0;
  double rawLon = fixFresh ? gps.location.lng() : 0.0;
  if (fixFresh && validCoordinates(rawLat, rawLon)) {
    if (newFix || !gpsHasFix) addSmoothedFix(rawLat, rawLon);
    gpsHasFix = true;
  } else {
    gpsHasFix = false;
    gpsSmoothCount = 0;     // restart averaging after a lost fix
    gpsSmoothNext = 0;
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

// How old a peer's location/heading may be before it is shown as stale.
// Awake peers send every 3 s, proximity-sleeping peers every 10 s, and
// manually sleeping peers every 5 minutes.
unsigned long remoteDataMaxAge(int deviceId) {
  if (!remote[deviceId].asleep) return 16000UL;
  return remote[deviceId].proximitySleep ? 30000UL : 360000UL;
}

bool remoteLocationFresh(int deviceId) {
  if (deviceId < 1 || deviceId > 3 || deviceId == DEVICE_ID ||
      !remote[deviceId].online || !remote[deviceId].gpsValid ||
      remote[deviceId].locationUpdatedAt == 0) return false;
  return millis() - remote[deviceId].locationUpdatedAt <= remoteDataMaxAge(deviceId);
}

// A remote heading is shown only while it is current; the last value is kept
// (with its timestamp) but never presented as current once it is stale.
bool remoteHeadingCurrent(int deviceId) {
  if (deviceId < 1 || deviceId > 3 || deviceId == DEVICE_ID ||
      !remote[deviceId].online || !remote[deviceId].headingValid ||
      remote[deviceId].headingUpdatedAt == 0) return false;
  return millis() - remote[deviceId].headingUpdatedAt <= remoteDataMaxAge(deviceId);
}

void updateRemoteLocation(int deviceId, bool hasFix, double lat, double lon,
                          int headingFlag, float peerHeading) {
  bool coordinatesOk = hasFix && validCoordinates(lat, lon);
  remote[deviceId].gpsValid = coordinatesOk;
  if (coordinatesOk) {
    remote[deviceId].lat = lat;
    remote[deviceId].lon = lon;
    remote[deviceId].locationUpdatedAt = millis();
  }

  if (headingFlag == 1) {
    // String(heading, 1) prints a heading of 359.95..359.99 as "360.0";
    // previously that value was rejected and the heading showed as "--".
    if (peerHeading >= 360.0f && peerHeading < 360.5f) peerHeading = 0.0f;
    if (isfinite(peerHeading) && peerHeading >= 0.0f && peerHeading < 360.0f) {
      remote[deviceId].headingValid = true;
      remote[deviceId].remoteHeading = peerHeading;
      remote[deviceId].headingUpdatedAt = millis();
    }
    // An unparsable value keeps the previous valid heading.
  } else if (headingFlag == 0) {
    // The peer explicitly reports that it has no working magnetometer.
    remote[deviceId].headingValid = false;
  }
  // headingFlag == -1: no heading field in this packet; keep the last value.
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
  // bearing_deg = geographic bearing from this device to the peer;
  // rel_deg = bearing relative to where this device is facing (radar angle);
  // peer_hdg = the direction the peer itself faces (-1 = not current).
  // distance_m / bearing_deg have the calibration offset applied when
  // dist_cal=1; shown_m is the zeroed value on screen; raw_m is the
  // uncorrected GPS distance for comparison.
  Serial.printf("GPS DIST local=%d %.6f,%.6f fix=%d hdg=%.1f peer=%d %.6f,%.6f fix=%d age_ms=%lu bearing_deg=%.1f rel_deg=%.1f distance_m=%.1f peer_hdg=%.1f dist_cal=%d shown_m=%.1f raw_m=%.1f\n",
                DEVICE_ID, latitude, longitude, localLocationValid(), heading, deviceId,
                remote[deviceId].lat, remote[deviceId].lon,
                remoteLocationFresh(deviceId),
                now - remote[deviceId].locationUpdatedAt, bearing,
                normalizeBearing(bearing - heading), distance,
                remoteHeadingCurrent(deviceId) ? remote[deviceId].remoteHeading : -1.0f,
                distCalActive(deviceId) ? 1 : 0,
                peerDisplayDistance(deviceId),
                distanceToPeer(remote[deviceId].lat, remote[deviceId].lon));
}

int relativeBearingDegrees(double peerLat, double peerLon) {
  int degrees = (int)roundf(normalizeBearing(bearingToPeer(peerLat, peerLon) - heading));
  return degrees >= 360 ? 0 : degrees;
}

// ---------------------------------------------------------------------------
// Position smoothing (see GPS_SMOOTH_*)
// ---------------------------------------------------------------------------
void addSmoothedFix(double lat, double lon) {
  // Moving/still with hysteresis, so speed noise around 1 m/s doesn't flip it.
  if (gps.speed.isValid()) {
    float speed = gps.speed.mps();
    if (speed > GPS_SMOOTH_MAX_SPEED_MPS) gpsSmoothMoving = true;
    else if (speed < GPS_SMOOTH_STILL_SPEED_MPS) gpsSmoothMoving = false;
  }
  if (gpsSmoothCount == 0) gpsSmoothFarFixes = 0;
  if (gpsSmoothCount > 0 &&
      TinyGPSPlus::distanceBetween(latitude, longitude, lat, lon) > GPS_SMOOTH_JUMP_M) {
    if (++gpsSmoothFarFixes < GPS_SMOOTH_JUMP_CONFIRM) return;   // glitch: ignore
    gpsSmoothCount = 0;     // the jump persisted: it is real, start over here
    gpsSmoothNext = 0;
  }
  gpsSmoothFarFixes = 0;
  gpsSmoothLat[gpsSmoothNext] = lat;
  gpsSmoothLon[gpsSmoothNext] = lon;
  gpsSmoothNext = (gpsSmoothNext + 1) % GPS_SMOOTH_SAMPLES;
  if (gpsSmoothCount < GPS_SMOOTH_SAMPLES) ++gpsSmoothCount;

  // Average the newest fixes: all of them when still, the last few when moving.
  int window = gpsSmoothCount;
  if (gpsSmoothMoving && window > GPS_SMOOTH_MOVING_SAMPLES) window = GPS_SMOOTH_MOVING_SAMPLES;
  double sumLat = 0.0, sumLon = 0.0;
  for (int k = 0; k < window; ++k) {
    int i = (gpsSmoothNext - 1 - k + GPS_SMOOTH_SAMPLES) % GPS_SMOOTH_SAMPLES;
    sumLat += gpsSmoothLat[i];
    sumLon += gpsSmoothLon[i];
  }
  latitude = sumLat / window;
  longitude = sumLon / window;
}

// ---------------------------------------------------------------------------
// Peer geometry with distance calibration applied
// ---------------------------------------------------------------------------
// North/east metres from one coordinate to another, using the WGS84
// ellipsoid's local radii (the shape GPS coordinates are defined on). A
// single-radius sphere (as in TinyGPS++) makes north-south distances ~0.5%
// too long at Philippine latitudes. The local flat approximation is accurate
// to well under 0.1% for the distances this system works at (a few km).
void offsetMeters(double fromLat, double fromLon, double toLat, double toLon,
                  float &north, float &east) {
  const double a = 6378137.0;              // WGS84 semi-major axis (m)
  const double e2 = 6.69437999014e-3;      // WGS84 first eccentricity squared
  const double degToRad = PI / 180.0;
  double midLat = (fromLat + toLat) * 0.5 * degToRad;
  double s = sin(midLat);
  double w = sqrt(1.0 - e2 * s * s);
  double meridianRadius = a * (1.0 - e2) / (w * w * w);   // north-south
  double primeVerticalRadius = a / w;                      // east-west
  north = (float)((toLat - fromLat) * degToRad * meridianRadius);
  east = (float)((toLon - fromLon) * degToRad * primeVerticalRadius * cos(midLat));
}

bool distCalActive(int deviceId) {
  return deviceId >= 1 && deviceId <= 3 && distCal[deviceId].valid;
}

// Pull readings within the noise floor toward 0, fading out linearly so
// distances at or beyond DIST_ZERO_TAPER_M are unchanged. Continuous and
// always increasing, so moving apart always shows a growing distance.
float applyZeroFloor(float distance, float noiseFloor) {
  if (distance < 0.0f || noiseFloor <= 0.0f) return distance;
  float fade = 1.0f - distance / DIST_ZERO_TAPER_M;
  if (fade < 0.0f) fade = 0.0f;
  float shown = distance - noiseFloor * fade;
  return shown < 0.0f ? 0.0f : shown;
}

void loadDistanceCalibration() {
  Preferences prefs;
  prefs.begin("dist", true);
  if (prefs.getBytesLength("cal") == sizeof(distCal)) {
    prefs.getBytes("cal", distCal, sizeof(distCal));
    for (int id = 1; id <= 3; ++id) {
      // Older builds accepted offsets up to 60 m. An offset that large comes
      // from a bad GPS fix during calibration and made devices 50-60 m apart
      // read as close together, so it is dropped (recalibrate that peer).
      float savedOffset = sqrtf(distCal[id].north * distCal[id].north +
                                distCal[id].east * distCal[id].east);
      if (distCal[id].valid && !(savedOffset <= DIST_CAL_MAX_OFFSET_M)) {
        Serial.printf("Distance calibration LORA #%d: saved offset %.1f m is over %.0f m - ignored, recalibrate\n",
                      id, savedOffset, DIST_CAL_MAX_OFFSET_M);
        distCal[id].valid = false;
        continue;
      }
      if (distCal[id].valid) {
        Serial.printf("Distance calibration LORA #%d: offset N=%.1f E=%.1f floor=%.1f m (saved)\n",
                      id, distCal[id].north, distCal[id].east, distCal[id].noiseFloor);
      }
    }
  } else {
    Serial.println("Distance calibration: none saved");
  }
  prefs.end();
}

void saveDistanceCalibration() {
  Preferences prefs;
  prefs.begin("dist", false);
  prefs.putBytes("cal", distCal, sizeof(distCal));
  prefs.end();
}

// Vector from this device to a peer, minus that peer's calibration offset.
bool peerOffsetMeters(int deviceId, float &north, float &east) {
  if (!localLocationValid() || !validCoordinates(remote[deviceId].lat, remote[deviceId].lon)) {
    return false;
  }
  offsetMeters(latitude, longitude, remote[deviceId].lat, remote[deviceId].lon, north, east);
  if (distCalActive(deviceId)) {
    north -= distCal[deviceId].north;
    east -= distCal[deviceId].east;
  }
  return true;
}

float peerDistance(int deviceId) {
  float n, e;
  if (!peerOffsetMeters(deviceId, n, e)) return -1.0f;
  return sqrtf(n * n + e * e);
}

// Geographic bearing to a peer: 0 = north, clockwise.
float peerBearing(int deviceId) {
  float n, e;
  if (!peerOffsetMeters(deviceId, n, e)) return -1.0f;
  return normalizeBearing(atan2f(e, n) * 180.0f / PI);
}

int peerRelativeBearingDegrees(int deviceId) {
  int degrees = (int)roundf(normalizeBearing(peerBearing(deviceId) - heading));
  return degrees >= 360 ? 0 : degrees;
}

// Distance between two peers, both corrected relative to this device.
float peerPairDistance(int idA, int idB) {
  float nA, eA, nB, eB;
  if (!peerOffsetMeters(idA, nA, eA) || !peerOffsetMeters(idB, nB, eB)) return -1.0f;
  float n = nA - nB, e = eA - eB;
  return sqrtf(n * n + e * e);
}

// Distance as shown on screen and used for proximity sleep: calibrated and
// zeroed. Without a calibration this is the plain GPS distance.
// Changes are limited to DIST_DISPLAY_MAX_RATE_MPS, so one jumpy GPS fix
// (either device) cannot make the distance leap; real walking is followed.
float peerDisplayDistance(int deviceId) {
  static float shownDistance[4] = {-1.0f, -1.0f, -1.0f, -1.0f};
  static unsigned long shownAt[4] = {0, 0, 0, 0};
  float d = peerDistance(deviceId);
  if (d >= 0.0f && distCalActive(deviceId)) d = applyZeroFloor(d, distCal[deviceId].noiseFloor);
  if (deviceId < 1 || deviceId > 3) return d;

  unsigned long now = millis();
  if (d < 0.0f || shownDistance[deviceId] < 0.0f) {
    shownDistance[deviceId] = d;      // no position (or first reading): no ramp
  } else {
    float maxStep = DIST_DISPLAY_MAX_RATE_MPS * (float)(now - shownAt[deviceId]) / 1000.0f;
    float step = d - shownDistance[deviceId];
    if (step > maxStep) step = maxStep;
    if (step < -maxStep) step = -maxStep;
    shownDistance[deviceId] += step;
  }
  shownAt[deviceId] = now;
  return shownDistance[deviceId];
}

// Pair distance carries both peers' GPS wander, so its floor combines both.
float peerPairDisplayDistance(int idA, int idB) {
  float d = peerPairDistance(idA, idB);
  if (!distCalActive(idA) || !distCalActive(idB)) return d;
  float fa = distCal[idA].noiseFloor, fb = distCal[idB].noiseFloor;
  return applyZeroFloor(d, sqrtf(fa * fa + fb * fb));
}

// One device-list row: "LORA #2 | 45m away | 315°". The bearing is the
// compass bearing from this device to the peer (0 = north, clockwise), not
// relative to where this device faces; the radar dot shows the relative view.
String peerRowText(int deviceId) {
  bool positionValid = localLocationValid() && remoteLocationFresh(deviceId);
  float shown = positionValid ? peerDisplayDistance(deviceId) : -1.0f;
  String distanceText = positionValid ? String((int)roundf(shown)) : String("--");
  String bearingText = (positionValid && shown >= PEER_BEARING_MIN_DISPLAY_M)
    ? String(((int)roundf(peerBearing(deviceId))) % 360) : String("--");
  // (char)247 renders as the degree sign in the built-in font, as in the
  // heading line on this screen.
  return "LORA #" + String(deviceId) + " | " + distanceText + "m away | " +
         bearingText + String((char)247);
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

// Raw angle of a magnetometer sample after hard/soft-iron correction. This is
// NOT yet a compass heading: its zero and direction depend on how the module
// is mounted, which calibration measures (magCal[MAG_CAL_ZERO/DIR]).
float rawMagAngle(float x, float y) {
  float calibratedX = (x - magCal[0]) * magCal[2];
  float calibratedY = (y - magCal[1]) * magCal[3];
  return atan2(calibratedX, calibratedY) * 180.0f / PI;
}

float calculateHeading() {
  // 0 = the direction faced at the start of calibration; increases clockwise.
  float angle = magCal[MAG_CAL_DIR] *
                (rawMagAngle((float)magX, (float)magY) - magCal[MAG_CAL_ZERO]);
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
  // Proximity vote (added at the end; older firmware ignores it).
  packet += ",";
  packet += String(proxOwnVote());

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
  // 0 = awake, 1 = manual sleep, 2 = automatic proximity sleep.
  packet += String(sleepMode ? 1 : (proxSleepActive ? 2 : 0));
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
  for (int id = 1; id <= 3; ++id) {
    broadcastAckSeen[id] = false;
    broadcastAckRtt[id] = 0;
  }
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

  // Listen for the reply even in sleep mode (sleeping the radio here made
  // every ping from a sleeping device end in "did not reply"). updatePing()
  // puts the radio back to sleep once the ping is finished.
  radioSleeping = false;
  LoRa.receive();
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
  if (!pingPending) {
    // A ping sent during manual sleep kept the radio listening; sleep again.
    if (sleepMode && !radioSleeping) {
      digitalWrite(TFT_CS, HIGH);
      digitalWrite(TOUCH_CS, HIGH);
      LoRa.sleep();
      radioSleeping = true;
    }
    return;
  }
  unsigned long elapsed = millis() - pendingPingStartedAt;
  if (pendingPingBroadcast) {
    if (elapsed < PING_BROADCAST_WINDOW) return;
    // Each responder's own round trip, e.g. "Pinged all: #2 95ms, #3 102ms".
    // (The old figure was the 1.2 s collection window, not a latency.)
    if (broadcastAckCount) {
      pingMessage = "Pinged all:";
      bool first = true;
      for (int id = 1; id <= 3; ++id) {
        if (id == DEVICE_ID) continue;
        pingMessage += first ? " #" : ", #";
        first = false;
        pingMessage += String(id) + " ";
        pingMessage += broadcastAckSeen[id] ? String(broadcastAckRtt[id]) + "ms"
                                            : String("no reply");
      }
    } else {
      pingMessage = "No device replied";
    }
  } else {
    if (elapsed < PING_REPLY_WINDOW) return;
    // A sleeping peer's radio is off (manual) or only listens 2 s in 10 s
    // (proximity), so say why instead of a bare "did not reply".
    if (remote[pendingPingTarget].online && remote[pendingPingTarget].asleep) {
      pingMessage = "Lora #" + String(pendingPingTarget) + " is asleep, no reply";
    } else {
      pingMessage = "Lora #" + String(pendingPingTarget) + " did not reply";
    }
    Serial.printf("PING NO ACK target=%d seq=%u after %lu ms\n",
                  pendingPingTarget, pendingPingSequence, elapsed);
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
// NOT the ping latency: it is this device's reply time, from the moment its
// radio finished receiving the ping to the moment the reply finished
// transmitting (LoRa.endPacket() returns only after transmission completes),
// so it is mostly the reply's own airtime (~40 ms at SF7). The latency is the
// round trip shown on the sender. For a broadcast it includes this device's
// reply slot delay.
void acknowledgePing(int requesterId, int targetId, uint16_t sequence,
                     unsigned long receivedAt) {
  sendPingAck(requesterId, targetId, sequence);
  unsigned long responseMs = millis() - receivedAt;
  pingMessage = "Lora #" + String(requesterId) + " pinged you (replied in " +
                String(responseMs) + " ms)";
  pingMessageTime = millis();
  if (currentScreen == MAIN_MENU) updateMainMenuDynamic(true);
}

// =====================================================
// LORA RECEIVE
// =====================================================

// Packet detection without leaving continuous receive.
// LoRa.receive() puts the radio in continuous RX with DIO0 mapped to RxDone.
// Calling LoRa.parsePacket() while no packet is waiting switches the radio to
// SINGLE RX instead: that restarts reception (losing a packet in flight) and
// times out after ~100 ms into standby, leaving the radio deaf while the loop
// is busy redrawing the screen. That caused missed heartbeats (peers shown
// OFFLINE) and missed ping ACKs ("did not reply"). So parsePacket() is only
// called once DIO0 reports a complete packet.
unsigned long lastDio0PacketAt = 0;
unsigned long lastFallbackPollAt = 0;

// Time the radio raised DIO0 (packet fully received), taken in an interrupt
// so ping timing does not include however long the main loop took to notice
// the packet (screen redraws, GPS, etc.). The ISR only stores a timestamp;
// all SPI work stays in receiveLoRa().
volatile unsigned long loraDio0RiseAt = 0;
void IRAM_ATTR onLoRaDio0Rise() {
  loraDio0RiseAt = millis();
}
bool dio0WarningShown = false;
#define LORA_DIO0_FALLBACK_AFTER_MS 15000UL
#define LORA_FALLBACK_POLL_MS       1000UL

bool loraPacketWaiting() {
  if (digitalRead(LORA_DIO0) == HIGH) {
    lastDio0PacketAt = millis();
    return true;
  }
  // Safety net in case DIO0 is not connected on a board: after 15 s without
  // a DIO0 packet, check the radio directly once per second. With DIO0 wired
  // and peers transmitting, this never runs.
  if (millis() - lastDio0PacketAt >= LORA_DIO0_FALLBACK_AFTER_MS &&
      millis() - lastFallbackPollAt >= LORA_FALLBACK_POLL_MS) {
    lastFallbackPollAt = millis();
    return true;
  }
  return false;
}

void receiveLoRa() {
  if (radioSleeping) return;
  if (!loraPacketWaiting()) return;

  bool viaDio0 = (digitalRead(LORA_DIO0) == HIGH);
  int packetSize = LoRa.parsePacket();
  if (!packetSize) {
    // No packet (fallback poll) or CRC error: return to continuous RX, since
    // parsePacket() may have switched the radio to single RX.
    LoRa.receive();
    return;
  }
  if (!viaDio0 && !dio0WarningShown) {
    dio0WarningShown = true;
    Serial.println("WARNING: packet found by fallback poll - check the LoRa DIO0 wire (GPIO26)");
  }

  String packet = "";
  while (LoRa.available()) packet += (char)LoRa.read();
  // Reception time for ping timing: the DIO0 interrupt time when this packet
  // was signalled on DIO0 (and the stamp is recent), otherwise now.
  unsigned long packetReceivedAt = millis();
  unsigned long dio0At = loraDio0RiseAt;
  if (viaDio0 && dio0At != 0 && (long)(packetReceivedAt - dio0At) >= 0 &&
      packetReceivedAt - dio0At < 2000UL) {
    packetReceivedAt = dio0At;
  }

  Serial.print("LoRa RX: ");
  Serial.println(packet);

  // PACKET TEST: T,<sender>,<seq> and replies TR,<responder>,<to>,<seq>,<rssi>,<snr>
  if (packet.startsWith("T,") || packet.startsWith("TR,")) {
    handlePacketTest(packet, packetReceivedAt);
    LoRa.receive();
    return;
  }

  // ---------------------------------------------------
  // NORMAL HEARTBEAT
  // HB,id,gpsFix,lat,lon,h,m,s,day,month,year
  // ---------------------------------------------------
  if (packet.startsWith("HB,")) {
    String f[13];
    int startIndex = 3;
    int count = 0;

    while (count < 13) {
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
        remote[sender].proximitySleep = false;
        remote[sender].online = true;
        remote[sender].heartbeatCount = ONLINE_CONFIRM_PACKETS;
        cachedRemoteOnline[sender] = true;
        cachedRemoteAsleep[sender] = false;

        updateRemoteLocation(sender, f[1].toInt() == 1, f[2].toDouble(),
                 f[3].toDouble(), count >= 12 ? f[10].toInt() : -1,
                 count >= 12 ? f[11].toFloat() : -1.0f);

        // Proximity vote: 0 apart, 1 together, 2 unknown (missing = older firmware).
        int vote = count >= 13 ? f[12].toInt() : -1;
        remote[sender].proxVote = (vote >= 0 && vote <= 2) ? (int8_t)vote : (int8_t)-1;
        remote[sender].proxVoteAt = millis();

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
  // SLP,id,mode,gpsFix,lat,lon,magPresent,heading
  // mode: 1 = manual sleep (every 5 min), 2 = proximity sleep (every 10 s)
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
        bool sleeping = (enabled == 1 || enabled == 2);
        remote[sender].lastSeen = millis();
        remote[sender].asleep = sleeping;
        remote[sender].proximitySleep = (enabled == 2);
        remote[sender].online = true;
        remote[sender].heartbeatCount = ONLINE_CONFIRM_PACKETS;
        cachedRemoteOnline[sender] = true;
        cachedRemoteAsleep[sender] = sleeping;
        // A device in proximity sleep sees the group together; one in manual
        // sleep is not measuring.
        remote[sender].proxVote = (enabled == 2) ? 1 : 2;
        remote[sender].proxVoteAt = millis();
        updateRemoteLocation(sender, f[2].toInt() == 1, f[3].toDouble(),
                 f[4].toDouble(), count >= 7 ? f[5].toInt() : -1,
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

        int sleepCode = f[2].toInt();
        bool asleep = (sleepCode == 1 || sleepCode == 2);

        remote[sender].lastSeen = millis();
        remote[sender].asleep = asleep;
        remote[sender].proximitySleep = (sleepCode == 2);
        remote[sender].online = true;
        remote[sender].heartbeatCount = ONLINE_CONFIRM_PACKETS;
        if (sleepCode == 2) {
          remote[sender].proxVote = 1;
          remote[sender].proxVoteAt = millis();
        }
        updateRemoteLocation(sender, f[3].toInt() == 1, f[4].toDouble(),
                 f[5].toDouble(), count >= 8 ? f[6].toInt() : -1,
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
      // Any ACK proves the responder is reachable right now.
      if (responder >= 1 && responder <= 3 && responder != DEVICE_ID) {
        remote[responder].lastSeen = millis();
        remote[responder].online = true;
        cachedRemoteOnline[responder] = true;
      }
      if (sequenceValid && pingPending &&
          millis() - pendingPingStartedAt <
            (pendingPingBroadcast ? PING_BROADCAST_WINDOW : PING_REPLY_WINDOW) &&
          requester == DEVICE_ID && target == pendingPingTarget &&
          responder >= 1 && responder <= 3 && responder != DEVICE_ID &&
          sequence == pendingPingSequence) {
        // Round trip: ping transmission started -> reply fully received by
        // the radio (interrupt time), so loop/redraw delays on this device
        // are not counted.
        unsigned long rtt = packetReceivedAt - pendingPingStartedAt;
        unsigned long noticedAfter = millis() - packetReceivedAt;
        if (pendingPingBroadcast) {
          // Collect every responder once; the message is shown when the
          // broadcast window closes (see updatePing()).
          if (!broadcastAckSeen[responder]) {
            broadcastAckSeen[responder] = true;
            ++broadcastAckCount;
            // The responder waited its slot (ID x 250 ms) on purpose.
            unsigned long slot = (unsigned long)responder * PING_BROADCAST_SLOT;
            broadcastAckRtt[responder] = rtt > slot ? rtt - slot : 0;
            Serial.printf("PING BROADCAST ACK responder=%d seq=%u rtt_ms=%lu (minus slot %lu ms) loop_delay_ms=%lu\n",
                          responder, sequence, broadcastAckRtt[responder], slot, noticedAfter);
          }
        } else if (responder == pendingPingTarget) {
          pingPending = false;
          pingMessage = "You pinged Lora #" + String(responder) + " (" +
                        String(rtt) + " ms)";
          pingMessageTime = millis();
          Serial.printf("PING RTT target=%d seq=%u rtt_ms=%lu loop_delay_ms=%lu\n",
                        responder, sequence, rtt, noticedAfter);
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
  remote[DEVICE_ID].asleep = sleepMode || proxSleepActive;
  // Own status icon on the main menu: aqua for proximity, yellow for manual.
  remote[DEVICE_ID].proximitySleep = proxSleepActive && !sleepMode;

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

    unsigned long timeout = remote[i].asleep
      ? (remote[i].proximitySleep ? PROX_PEER_TIMEOUT : SLEEP_DEVICE_TIMEOUT)
      : DEVICE_TIMEOUT;
    // While THIS device is in proximity sleep it only listens 3 s out of
    // every 10 s, so awake peers are heard less often. Do not mistake that
    // for them going offline.
    if (proxSleepActive && timeout < PROX_PEER_TIMEOUT) timeout = PROX_PEER_TIMEOUT;
    // Heartbeats are held during a packet test: keep every status as it is.
    if ((long)(statusTimeoutHoldUntil - now) > 0) continue;

    if (now - remote[i].lastSeen > timeout) {
      remote[i].online = false;
      remote[i].asleep = false;
      remote[i].proximitySleep = false;
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
    // Manual sleep takes over from automatic proximity sleep (if active);
    // updateProximity() resets the proximity state machine.
    proxSleepActive = false;
    remote[DEVICE_ID].online = true;
    remote[DEVICE_ID].asleep = true;

    // Immediately announce sleep mode and save current location, then repeat
    // the report twice (see sleepReportSlotReady()) so a lost report cannot
    // leave this device shown OFFLINE.
    sendSleepLocationReport();
    sleepAnnounceRepeats = SLEEP_ANNOUNCE_REPEATS;
    lastSleepReportCycle = 0xFFFFFFFFUL;
    nextSleepLocationReport = millis() + SLEEP_ANNOUNCE_GAP_MS;
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
// AUTOMATIC PROXIMITY SLEEP
// =====================================================

const char *proximityStateName(ProximityState state) {
  switch (state) {
    case PROX_NORMAL_OPERATION:        return "NORMAL_OPERATION";
    case PROX_CONFIRMATION:            return "PROXIMITY_CONFIRMATION";
    case PROX_SLEEP:                   return "PROXIMITY_SLEEP";
    case PROX_OUT_OF_RANGE_MONITORING: return "OUT_OF_RANGE_MONITORING";
    case PROX_WAKE_AND_RECOVERY:       return "WAKE_AND_RECOVERY";
    case PROX_UNKNOWN:                 return "PROXIMITY_UNKNOWN";
  }
  return "?";
}

// UTC time of day in ms from the GPS, used only to line up the sleep
// listening windows of the three devices. Accuracy is limited by NMEA
// sentence latency (typically well under 1 s), which the 2 s window absorbs.
bool gpsUtcMillisOfDay(uint32_t &msOfDay) {
  if (!gps.time.isValid() || gps.time.age() > 2000UL) return false;
  uint32_t t = ((uint32_t)gps.time.hour() * 3600UL +
                (uint32_t)gps.time.minute() * 60UL +
                (uint32_t)gps.time.second()) * 1000UL +
               (uint32_t)gps.time.centisecond() * 10UL + gps.time.age();
  msOfDay = t % 86400000UL;
  return true;
}

uint32_t proxSlotOffset() {
  return PROX_SLOT_BASE_MS + (uint32_t)(DEVICE_ID - 1) * PROX_SLOT_SPACING_MS;
}

// Is a regular heartbeat due now? nextHeartbeat is a "not before" time
// (holds during pings, packet tests and after waking still apply). With GPS
// time: once per 5 s cycle, in this device's slot. Without: on the timer.
uint32_t lastHeartbeatCycle = 0xFFFFFFFFUL;
bool heartbeatDue() {
  if ((long)(millis() - nextHeartbeat) < 0) return false;
  uint32_t utcMs = 0;
  if (!gpsUtcMillisOfDay(utcMs)) return true;
  uint32_t cycle = utcMs / HEARTBEAT_CYCLE_MS;
  uint32_t pos = utcMs % HEARTBEAT_CYCLE_MS;
  if (cycle == lastHeartbeatCycle) return false;
  return pos >= proxSlotOffset() && pos < proxSlotOffset() + HEARTBEAT_SLOT_LATE_MS;
}

void heartbeatSent() {
  uint32_t utcMs = 0;
  if (gpsUtcMillisOfDay(utcMs)) {
    lastHeartbeatCycle = utcMs / HEARTBEAT_CYCLE_MS;
    nextHeartbeat = millis() + 1000UL;
  } else {
    nextHeartbeat = millis() + HEARTBEAT_INTERVAL + (unsigned long)random(HEARTBEAT_JITTER_MS + 1);
  }
}

// Offline means silent for a long time - never just asleep. A peer in either
// sleep mode keeps reporting, so it stays online.
bool peerConfirmedOffline(int deviceId) {
  if (remote[deviceId].online) return false;
  if (remote[deviceId].lastSeen == 0) return millis() >= PROX_OFFLINE_CONFIRM_MS;
  return millis() - remote[deviceId].lastSeen >= PROX_OFFLINE_CONFIRM_MS;
}

bool peerLocationFreshForProximity(int deviceId) {
  return remote[deviceId].online && remote[deviceId].gpsValid &&
         remote[deviceId].locationUpdatedAt != 0 &&
         millis() - remote[deviceId].locationUpdatedAt <= PROX_LOCATION_MAX_AGE_MS;
}

bool anyPeerInProximitySleep() {
  for (int i = 1; i <= 3; ++i) {
    if (i == DEVICE_ID) continue;
    if (remote[i].online && remote[i].asleep && remote[i].proximitySleep) return true;
  }
  return false;
}

// What a peer last reported seeing. A peer that sees the group apart makes
// the group apart for everyone; a peer that cannot tell (no GPS) pauses the
// decision. Older firmware without the vote is taken as agreeing.
ProximityReading proxPeerVote(int deviceId) {
  const RemoteDevice &r = remote[deviceId];
  if (r.proxVote < 0) return PROX_READING_IN_RANGE;
  if (millis() - r.proxVoteAt > PROX_LOCATION_MAX_AGE_MS) return PROX_READING_UNKNOWN;
  if (r.proxVote == 1) return PROX_READING_IN_RANGE;
  if (r.proxVote == 0) return PROX_READING_OUT_OF_RANGE;
  return PROX_READING_UNKNOWN;
}

// The vote this device puts in its heartbeat: its own latest reading.
int proxOwnVote() {
  if (proxOwnGeometryReading == PROX_READING_IN_RANGE) return 1;
  if (proxOwnGeometryReading == PROX_READING_OUT_OF_RANGE) return 0;
  return 2;
}

// distances[0] = this device <-> peer A, [1] = this device <-> peer B,
// [2] = peer A <-> peer B (-1 when not computed).
ProximityReading evaluateGroupProximity(int &offlineDevice, float distances[3]) {
  offlineDevice = 0;
  distances[0] = distances[1] = distances[2] = -1.0f;

  proxOwnGeometryReading = PROX_READING_UNKNOWN;

  // No local fix: the decision is paused, never treated as "apart".
  if (!localLocationValid()) return PROX_READING_UNKNOWN;

  int peers[2];
  int n = 0;
  for (int i = 1; i <= 3; ++i) if (i != DEVICE_ID) peers[n++] = i;

  bool offline[2] = {peerConfirmedOffline(peers[0]), peerConfirmedOffline(peers[1])};
  bool fresh[2] = {peerLocationFreshForProximity(peers[0]),
                   peerLocationFreshForProximity(peers[1])};

  // Both others confirmed offline: there is no group to be "together" with.
  // This never starts proximity sleep and lets a sleeping device recover
  // through the normal 5 s exit rule.
  if (offline[0] && offline[1]) {
    proxOwnGeometryReading = PROX_READING_OUT_OF_RANGE;
    return PROX_READING_OUT_OF_RANGE;
  }

  // Exactly one device confirmed offline: it is left out of the group (and
  // stays awake for recovery); the other two decide on their own distance.
  if (offline[0] || offline[1]) {
    int k = offline[0] ? 1 : 0;
    offlineDevice = peers[offline[0] ? 0 : 1];
    if (!fresh[k]) return PROX_READING_UNKNOWN;
    distances[k] = peerDisplayDistance(peers[k]);
    if (distances[k] >= PROX_THRESHOLD_M) {
      proxOwnGeometryReading = PROX_READING_OUT_OF_RANGE;
      return PROX_READING_OUT_OF_RANGE;
    }
    proxOwnGeometryReading = PROX_READING_IN_RANGE;
    return proxPeerVote(peers[k]);
  }

  // Normal case: all three pairwise distances must be within the threshold.
  if (!fresh[0] || !fresh[1]) return PROX_READING_UNKNOWN;
  // Calibration-corrected distances (same values as the screen shows).
  distances[0] = peerDisplayDistance(peers[0]);
  distances[1] = peerDisplayDistance(peers[1]);
  distances[2] = peerPairDisplayDistance(peers[0], peers[1]);
  bool together = distances[0] < PROX_THRESHOLD_M &&
                  distances[1] < PROX_THRESHOLD_M &&
                  distances[2] < PROX_THRESHOLD_M;
  proxOwnGeometryReading = together ? PROX_READING_IN_RANGE : PROX_READING_OUT_OF_RANGE;
  if (!together) return PROX_READING_OUT_OF_RANGE;
  // Together by this device's measurement; the others must agree.
  ProximityReading a = proxPeerVote(peers[0]);
  ProximityReading b = proxPeerVote(peers[1]);
  if (a == PROX_READING_OUT_OF_RANGE || b == PROX_READING_OUT_OF_RANGE) {
    return PROX_READING_OUT_OF_RANGE;
  }
  if (a == PROX_READING_UNKNOWN || b == PROX_READING_UNKNOWN) return PROX_READING_UNKNOWN;
  return PROX_READING_IN_RANGE;
}

// Start a new timed period (confirmation or out-of-range) from this moment.
void setProxState(ProximityState next, unsigned long now) {
  if (next != proxState) {
    Serial.printf("PROX STATE %s -> %s\n", proximityStateName(proxState),
                  proximityStateName(next));
  }
  proxState = next;
  proxAccumMs = 0;
  proxChainValid = true;
  proxChainAt = now;
}

// Add the time since the previous valid reading. After an unknown reading
// the chain is broken, so the unknown gap itself is never counted.
void proxAccumulate(unsigned long now) {
  if (proxChainValid) proxAccumMs += now - proxChainAt;
  proxChainValid = true;
  proxChainAt = now;
}

void sendProximitySleepReport() {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);

  radioSleeping = false;
  LoRa.idle();

  // Same layout as the manual-sleep SLP report, with mode 2.
  LoRa.beginPacket();
  LoRa.print("SLP,");
  LoRa.print(DEVICE_ID);
  LoRa.print(",2,");
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

  // Keep listening for the rest of the window; serviceProximitySleep()
  // puts the radio to sleep when the window closes.
  LoRa.receive();
}

void enterProximitySleep() {
  proxSleepActive = true;
  remote[DEVICE_ID].asleep = true;
  proxLastReportCycle = 0xFFFFFFFFUL;
  proxLastUnsyncedReport = millis();
  // Announce immediately so awake peers know this silence is intentional,
  // and once more shortly after in case that report collided.
  sendProximitySleepReport();
  proxEntryRepeatAt = millis() + PROX_ENTRY_REPEAT_MS;
  if (proxEntryRepeatAt == 0) proxEntryRepeatAt = 1;
  if (proxOfflineDevice) {
    Serial.printf("PROXIMITY SLEEP ENABLED (LORA #%d offline, left awake for recovery)\n",
                  proxOfflineDevice);
  } else {
    Serial.println("PROXIMITY SLEEP ENABLED (group within threshold)");
  }
}

void exitProximitySleep() {
  proxSleepActive = false;
  proxEntryRepeatAt = 0;
  radioSleeping = false;
  LoRa.idle();
  remote[DEVICE_ID].asleep = false;

  // Same wake-up path as leaving manual sleep: announce, then resync.
  sendHeartbeat();
  requestStatusSync();
  nextHeartbeat = millis() + HEARTBEAT_INTERVAL;
  Serial.println("PROXIMITY SLEEP DISABLED - 5 s CONTINUOUSLY OUT OF RANGE");
}

void updateProximity() {
  unsigned long now = millis();
  if (now - proxLastEvalAt < PROX_EVAL_INTERVAL_MS) return;
  proxLastEvalAt = now;

  // Manual sleep has priority; the proximity machine stands aside.
  if (sleepMode) {
    proxSleepActive = false;
    if (proxState != PROX_NORMAL_OPERATION) setProxState(PROX_NORMAL_OPERATION, now);
    return;
  }

  // Turned off on the developer screen: stay awake and vote "apart".
  if (!proxSleepEnabled) {
    if (proxSleepActive) {
      Serial.println("PROXIMITY SLEEP turned off on the developer screen");
      exitProximitySleep();
    }
    if (proxState != PROX_NORMAL_OPERATION) setProxState(PROX_NORMAL_OPERATION, now);
    proxOwnGeometryReading = PROX_READING_OUT_OF_RANGE;
    proxLastReading = PROX_READING_OUT_OF_RANGE;
    return;
  }

  float d[3];
  int offlineDevice = 0;
  ProximityReading reading = evaluateGroupProximity(offlineDevice, d);
  proxOfflineDevice = offlineDevice;

  if (reading != proxLastReading || now - proxLastLogAt >= 10000UL) {
    proxLastLogAt = now;
    int peerA = DEVICE_ID == 1 ? 2 : 1;
    int peerB = DEVICE_ID == 3 ? 2 : 3;
    Serial.printf("PROX CHECK reading=%s state=%s d_a=%.1f d_b=%.1f d_ab=%.1f votes #%d=%d #%d=%d offline=%d accum_ms=%lu\n",
                  reading == PROX_READING_IN_RANGE ? "IN_RANGE"
                    : (reading == PROX_READING_OUT_OF_RANGE ? "OUT_OF_RANGE" : "UNKNOWN"),
                  proximityStateName(proxState), d[0], d[1], d[2],
                  peerA, remote[peerA].proxVote, peerB, remote[peerB].proxVote,
                  offlineDevice, proxAccumMs);
  }
  proxLastReading = reading;

  // Missing/stale GPS: pause. Keep the sleep state and the accumulated time,
  // but do not count the unknown period.
  if (reading == PROX_READING_UNKNOWN) {
    if (proxState != PROX_UNKNOWN) {
      Serial.printf("PROX STATE %s -> PROXIMITY_UNKNOWN (paused)\n",
                    proximityStateName(proxState));
      proxStateBeforeUnknown = proxState;
      proxState = PROX_UNKNOWN;
    }
    proxChainValid = false;
    return;
  }
  if (proxState == PROX_UNKNOWN) {
    Serial.printf("PROX STATE PROXIMITY_UNKNOWN -> %s (resumed)\n",
                  proximityStateName(proxStateBeforeUnknown));
    proxState = proxStateBeforeUnknown;
  }

  bool inRange = (reading == PROX_READING_IN_RANGE);

  switch (proxState) {
    case PROX_NORMAL_OPERATION:
    case PROX_CONFIRMATION:
      // Must stay together for 5 s in a row; one apart reading cancels it.
      if (!inRange) {
        if (proxState != PROX_NORMAL_OPERATION) setProxState(PROX_NORMAL_OPERATION, now);
        break;
      }
      if (proxState == PROX_NORMAL_OPERATION) {
        setProxState(PROX_CONFIRMATION, now);
        // A peer already asleep has confirmed the 5 s for the group (it
        // needed this device's "together" vote to do so); join it now
        // instead of starting a new 5 s, so the group sleeps as one.
        if (!anyPeerInProximitySleep()) break;
        proxAccumMs = PROX_ENTRY_CONFIRM_MS;
      } else {
        proxAccumulate(now);
        if (anyPeerInProximitySleep()) proxAccumMs = PROX_ENTRY_CONFIRM_MS;
      }
      if (proxAccumMs >= PROX_ENTRY_CONFIRM_MS) {
        enterProximitySleep();
        setProxState(PROX_SLEEP, now);
      }
      break;

    case PROX_SLEEP:
      // First valid out-of-range reading starts the 5 s continuous timer.
      if (!inRange) setProxState(PROX_OUT_OF_RANGE_MONITORING, now);
      break;

    case PROX_OUT_OF_RANGE_MONITORING:
      if (inRange) {
        // Back together before 5 s: cancel the timer, stay asleep.
        setProxState(PROX_SLEEP, now);
        break;
      }
      proxAccumulate(now);
      if (proxAccumMs >= PROX_EXIT_OUT_OF_RANGE_MS) {
        exitProximitySleep();
        setProxState(PROX_WAKE_AND_RECOVERY, now);
        proxRecoveryUntil = now + PROX_RECOVERY_HOLD_MS;
      }
      break;

    case PROX_WAKE_AND_RECOVERY:
      if ((long)(now - proxRecoveryUntil) >= 0) setProxState(PROX_NORMAL_OPERATION, now);
      break;

    case PROX_UNKNOWN:
      break;
  }
}

// A due Sleep Mode report waits for this device's slot in the shared
// proximity listening window (when GPS time is known), so devices in
// proximity sleep - which only listen in that window - hear it too. At most
// one per window; without GPS time it is sent right away.
bool sleepReportSlotReady() {
  uint32_t utcMs = 0;
  if (!gpsUtcMillisOfDay(utcMs)) return true;
  uint32_t cycle = utcMs / PROX_CYCLE_MS;
  uint32_t pos = utcMs % PROX_CYCLE_MS;
  if (pos < proxSlotOffset() || pos >= proxSlotOffset() + HEARTBEAT_SLOT_LATE_MS) return false;
  if (cycle == lastSleepReportCycle) return false;
  lastSleepReportCycle = cycle;
  return true;
}

// Radio duty cycle for proximity sleep, called every loop.
void serviceProximitySleep() {
  if (sleepMode) return;   // manual sleep manages the radio itself

  uint32_t utcMs = 0;
  bool synced = gpsUtcMillisOfDay(utcMs);
  uint32_t cycle = utcMs / PROX_CYCLE_MS;
  uint32_t pos = utcMs % PROX_CYCLE_MS;

  if (proxSleepActive) {
    if (proxEntryRepeatAt && (long)(millis() - proxEntryRepeatAt) >= 0) {
      proxEntryRepeatAt = 0;
      sendProximitySleepReport();   // leaves the radio on; sleeps again below
    }
    static bool unsyncedLogged = false;
    if (!synced != unsyncedLogged) {
      unsyncedLogged = !synced;
      Serial.println(synced ? "PROX SLEEP: GPS time OK - using the shared 2 s window"
                            : "PROX SLEEP: no GPS time - radio stays on, report every 1.8 s");
    }
    if (!synced) {
      // Without GPS time the devices cannot share a listening window, so the
      // radio stays on (no saving, but no peer is missed) and reports on a
      // local timer.
      if (radioSleeping) {
        radioSleeping = false;
        LoRa.idle();
        LoRa.receive();
      }
      if (millis() - proxLastUnsyncedReport >= PROX_UNSYNCED_REPORT_MS) {
        proxLastUnsyncedReport = millis();
        sendProximitySleepReport();
      }
      return;
    }

    if (pos < PROX_WINDOW_MS) {
      if (radioSleeping) {
        radioSleeping = false;
        LoRa.idle();
        LoRa.receive();
      }
      if (cycle != proxLastReportCycle && pos >= proxSlotOffset()) {
        proxLastReportCycle = cycle;
        sendProximitySleepReport();
      }
    } else if (!radioSleeping && !pingPending) {
      // (While a ping this device sent is waiting for its reply, the radio
      // stays on; sleeping it here made the reply arrive at a deaf radio.)
      digitalWrite(TFT_CS, HIGH);
      digitalWrite(TOUCH_CS, HIGH);
      LoRa.sleep();
      radioSleeping = true;
    }
    return;
  }

  // An awake device's regular heartbeats are sent in its slot of the 5 s
  // cycle (heartbeatDue()), which falls inside every 10 s listening window,
  // so sleeping peers hear it without an extra heartbeat here (an extra one
  // in the same slot would now run into the next device's slot).
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
      tft.setTextColor(remote[i].proximitySleep ? AQUA : YELLOW, BLACK);
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
  } else if (peerTestResultMessage.length() &&
             millis() - peerTestResultAt < PEER_TEST_RESULT_SHOW_MS) {
    showPing = peerTestResultMessage;
  }

  if (force || showPing != lastMainPing) {
    drawPingArea(showPing);
    lastMainPing = showPing;
  }

  String sleepText = sleepMode ? "SLEEP MODE IS ENABLED!"
                   : (proxSleepActive ? "AUTO SLEEP: GROUP IS NEARBY"
                   : (!proxSleepEnabled ? "AUTO SLEEP OFF (DEVELOPER)" : ""));
  if (force || sleepText != lastMainSleepMessage) {
    drawSleepModeArea(sleepText);
    lastMainSleepMessage = sleepText;
  }

  for (int i = 1; i <= 3; i++) {
    if (force ||
        lastMainOnline[i] != remote[i].online ||
        lastMainAsleep[i] != remote[i].asleep ||
        lastMainProximitySleep[i] != remote[i].proximitySleep) {

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
        // Yellow = manual sleep, aqua = automatic proximity sleep.
        tft.setTextColor(remote[i].proximitySleep ? AQUA : YELLOW, BLACK);
        tft.setCursor(textX, 280);
        tft.print("ASLEEP");
      } else {
        tft.setTextColor(GREEN, BLACK);
        tft.setCursor(textX, 280);
        tft.print("ONLINE");
      }

      lastMainOnline[i] = remote[i].online;
      lastMainAsleep[i] = remote[i].asleep;
      lastMainProximitySleep[i] = remote[i].proximitySleep;
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
  String line = peerRowText(deviceId);

  tft.setTextSize(1);
  tft.setTextColor(WHITE, BLACK);

  int16_t x1, y1;
  uint16_t w, h;
  tft.getTextBounds(line.c_str(), 0, y, &x1, &y1, &w, &h);

  // Gray = offline, yellow = manual sleep, aqua = automatic proximity sleep.
  uint16_t statusColor = !remote[deviceId].online ? GRAY
    : (remote[deviceId].asleep ? (remote[deviceId].proximitySleep ? AQUA : YELLOW) : WHITE);
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
  lastPeerCalText = "";
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
  if (lastPeerDot[id].tickValid) {
    tft.drawLine(lastPeerDot[id].x, lastPeerDot[id].y,
                 lastPeerDot[id].tickX, lastPeerDot[id].tickY, BLACK);
    lastPeerDot[id].tickValid = false;
  }
  tft.fillCircle(lastPeerDot[id].x, lastPeerDot[id].y, 4, BLACK);
  lastPeerDot[id].valid = false;
}

void drawPeerDot(int deviceId, double peerLat, double peerLon, uint16_t color) {
  if (!localLocationValid() || !magnetometerPresent ||
      !validCoordinates(peerLat, peerLon)) return;

  // Calibration-corrected, so the dot agrees with the row text.
  float distance = peerDistance(deviceId);
  float geographicBearing = peerBearing(deviceId);
  float bearing = normalizeBearing(geographicBearing - heading);
  logPeerMeasurement(deviceId, geographicBearing, distance);

  // Fixed 100 m display scale. Closer points sit closer to the center;
  // farther points are capped at the inside of the ring. Uses the zeroed
  // distance shown in the row, so the dot and the text agree.
  float shownDistance = peerDisplayDistance(deviceId);
  float radius = (shownDistance / PEER_MAX_DISPLAY_METERS) * (PEER_CIRCLE_RADIUS - 8.0f);
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

  // Facing tick: the peer's own compass heading, rotated by this device's
  // heading like everything else on the radar. It only shows which way the
  // peer faces; the dot's position comes from GPS bearing and distance.
  lastPeerDot[deviceId].tickValid = false;
  if (remoteHeadingCurrent(deviceId)) {
    float facing = normalizeBearing(remote[deviceId].remoteHeading - heading) * PI / 180.0f;
    int tx = x + (int)roundf(sinf(facing) * PEER_HEADING_TICK_LEN);
    int ty = y - (int)roundf(cosf(facing) * PEER_HEADING_TICK_LEN);
    tft.drawLine(x, y, tx, ty, color);
    lastPeerDot[deviceId].tickValid = true;
    lastPeerDot[deviceId].tickX = tx;
    lastPeerDot[deviceId].tickY = ty;
  }
}

int remoteHeadingStep(int deviceId) {
  if (!remoteHeadingCurrent(deviceId)) return -1;
  return ((int)roundf(remote[deviceId].remoteHeading / 5.0f)) % 72;
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

  // A facing tick that points toward the centre can reach the centre dot;
  // draw the centre dot again on top.
  tft.fillCircle(PEER_CIRCLE_CENTER_X, PEER_CIRCLE_CENTER_Y,
                 3, deviceColor(DEVICE_ID));

  peerVisualInitialized = true;
}

void updatePeerDynamicText(const String &locationText, bool locationOk,
                           const String &headingText, bool headingOk) {
  // "DIST CAL" top right while a distance calibration is applied (blank when
  // none). This corner stays clear of the rotating N/E/S/W letters.
  bool anyCal = false;
  for (int id = 1; id <= 3; ++id) {
    if (id != DEVICE_ID && distCalActive(id)) anyCal = true;
  }
  String calText = anyCal ? String("DIST CAL") : String("");
  if (calText != lastPeerCalText) {
    tft.fillRect(150, 8, 88, 12, BLACK);
    if (calText.length()) {
      tft.setTextSize(1);
      tft.setTextColor(LIGHTGREEN, BLACK);
      tft.setCursor(236 - (int)calText.length() * 6, 10);
      tft.print(calText);
    }
    lastPeerCalText = calText;
  }

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

    String line = peerRowText(id);

    // Status is no longer printed in the row, but it still sets the text
    // color (gray offline / yellow asleep), so keep it in the redraw key.
    String displayKey = line + (remote[id].online ? "|ONLINE" : "|OFFLINE") +
                        (remote[id].asleep ? (remote[id].proximitySleep ? "|PROXSLEEP" : "|ASLEEP")
                                           : "|AWAKE");

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
    remoteHeadingStep(1) != lastPeerRemoteHeadingStep[1] ||
    remoteHeadingStep(2) != lastPeerRemoteHeadingStep[2] ||
    remoteHeadingStep(3) != lastPeerRemoteHeadingStep[3] ||
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
    for (int id = 1; id <= 3; ++id) lastPeerRemoteHeadingStep[id] = remoteHeadingStep(id);
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
  static unsigned long distanceHoldStart = 0;
  static unsigned long titleHoldStart = 0;
  bool nowTouched = touch.touched();

  if (!nowTouched) {
    wasTouched = false;
    compassHoldStart = 0;
    distanceHoldStart = 0;
    titleHoldStart = 0;
    return;
  }
  if (wasTouched) {
    // Holding the title on the main menu for 3 s opens the developer screen.
    if (titleHoldStart && millis() - titleHoldStart >= 3000UL) {
      titleHoldStart = 0;
      runDeveloperScreen();
      drawMainMenu();
    }
    // Holding the compass circle for 3 s starts compass calibration.
    if (compassHoldStart && millis() - compassHoldStart >= 3000UL) {
      compassHoldStart = 0;
      runMagCalibration();
      drawPeerLocation();
    }
    // Holding the device list for 3 s starts distance calibration.
    if (distanceHoldStart && millis() - distanceHoldStart >= 3000UL) {
      distanceHoldStart = 0;
      runDistanceCalibration();
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
    // Title bar "LORA THE EXPLORER": hold 3 s for the developer screen.
    if (screenY < 35) {
      titleHoldStart = millis();
      return;
    }

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
      sendPing(0);   // sendPing() already redrew "Sending..."
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
        sendPing(targetId);   // sendPing() already redrew "Sending..."
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

    // Device list rows (y 291-310) start the distance-calibration hold.
    if (screenY >= 285) {
      distanceHoldStart = millis();
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
  // DIO0 signals "packet received" in continuous RX (see loraPacketWaiting()).
  pinMode(LORA_DIO0, INPUT);
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
  loadDistanceCalibration();
  loadDeveloperSettings();
  if (touch.touched()) runTouchCalibration();

  // The default 256-byte UART buffer holds only ~0.5 s of NMEA data (the
  // NEO-6M sends ~500 bytes/s); a longer screen redraw overflowed it and the
  // damaged sentences were discarded. 2048 bytes holds ~4 s. Must be set
  // before begin().
  gpsSerial.setRxBufferSize(2048);
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
  // Timestamp only (see onLoRaDio0Rise); reception itself is still polled.
  attachInterrupt(digitalPinToInterrupt(LORA_DIO0), onLoRaDio0Rise, RISING);

  configureLoRaRadio();

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
// The direction the device faces when calibration starts becomes 0 degrees
// (face it toward true north for correct radar directions). Keep the device
// flat and rotate it CLOCKWISE through 2-3 full turns for 20 seconds.
// The result is saved in flash for this board only.
// =====================================================

#define MAG_CAL_TIME 20000UL
#define MAG_CAL_MIN_SPAN 200
// Samples kept for the zero/direction analysis (one per ~50 ms loop).
#define MAG_CAL_MAX_SAMPLES 480
// The first samples (~250 ms, before real rotation starts) define 0 degrees.
#define MAG_CAL_ZERO_SAMPLES 5
// At least one full turn is needed to measure the direction reliably.
#define MAG_CAL_MIN_TURNS 1.0f
// Time to point the device north after the 3 s hold, before sampling starts.
#define MAG_CAL_READY_MS 5000UL

void loadMagCalibration() {
  Preferences prefs;
  prefs.begin("mag", true);
  size_t len = prefs.getBytesLength("cal");
  if (len == sizeof(magCal)) {
    prefs.getBytes("cal", magCal, sizeof(magCal));
    Serial.printf("Compass calibration: saved (this board) zero=%.1f dir=%+.0f\n",
                  magCal[MAG_CAL_ZERO], magCal[MAG_CAL_DIR]);
  } else if (len == 4 * sizeof(float)) {
    // Calibration saved by an older build: keep its offsets/scales, but it has
    // no 0-degree reference or direction yet.
    prefs.getBytes("cal", magCal, 4 * sizeof(float));
    Serial.println("Compass calibration: old format (offsets only) - recalibrate to set facing");
  } else {
    Serial.println("Compass calibration: default");
  }
  prefs.end();
  if (magCal[MAG_CAL_DIR] != 1.0f && magCal[MAG_CAL_DIR] != -1.0f) {
    magCal[MAG_CAL_DIR] = 1.0f;
  }
}

// Wrap an angle difference into -180..+180 degrees.
float wrapAngleDelta(float d) {
  while (d > 180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

void runMagCalibration() {
  if (!magnetometerPresent) return;

  Serial.println("COMPASS CALIBRATION: face NORTH, start facing = 0 deg, rotate CLOCKWISE 2-3 turns");

  // The start facing becomes 0 degrees, and peers are placed on the radar
  // by their GPS (true-north) bearing minus this heading. If the start
  // facing is not north, every dot, the N/E/S/W letters and the bearing
  // numbers are turned by that error. So the user points the device NORTH
  // first; this countdown gives time to do that after the 3 s hold.
  tft.fillScreen(BLACK);
  drawCenteredText("COMPASS CALIBRATION", 55, WHITE, BLACK, 2);
  drawCenteredText("Point it NORTH", 95, YELLOW, BLACK, 2);
  drawCenteredText("(use a phone compass), keep it FLAT", 125, YELLOW, BLACK, 1);
  unsigned long readyStart = millis();
  int lastCount = -1;
  while (millis() - readyStart < MAG_CAL_READY_MS) {
    serviceRadio();   // stay ONLINE for the others while waiting
    int count = (int)((MAG_CAL_READY_MS - (millis() - readyStart) + 999UL) / 1000UL);
    if (count != lastCount) {
      lastCount = count;
      tft.fillRect(0, 175, SCREEN_W, 20, BLACK);
      drawCenteredText("Starting in " + String(count) + " s", 180, GREEN, BLACK, 2);
    }
    delay(10);
  }

  tft.fillScreen(BLACK);
  drawCenteredText("COMPASS CALIBRATION", 55, WHITE, BLACK, 2);
  drawCenteredText("North is now 0 degrees", 90, YELLOW, BLACK, 1);
  drawCenteredText("Keep the device FLAT", 110, YELLOW, BLACK, 1);
  drawCenteredText("Rotate it slowly CLOCKWISE", 125, YELLOW, BLACK, 1);
  drawCenteredText("(2-3 turns) until the timer ends", 140, YELLOW, BLACK, 1);

  // Raw samples are kept so that, once the offsets are known, the start
  // facing (0 degrees) and the turning direction can be worked out.
  static int16_t sampleX[MAG_CAL_MAX_SAMPLES];
  static int16_t sampleY[MAG_CAL_MAX_SAMPLES];
  int sampleCount = 0;

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
      if (sampleCount < MAG_CAL_MAX_SAMPLES) {
        sampleX[sampleCount] = magX;
        sampleY[sampleCount] = magY;
        ++sampleCount;
      }
    }

    // Keep this device ONLINE for the others during calibration.
    receiveLoRa();
    // Keep reading the GPS so its data is not lost during the 20 s.
    updateGPS();
    if (heartbeatDue()) {
      lastBeat = millis();
      sendHeartbeat();
      heartbeatSent();
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

  // Work on a copy so a failed check leaves the old calibration untouched.
  float previous[6];
  memcpy(previous, magCal, sizeof(magCal));

  float avgSpan = (spanX + spanY) / 2.0f;
  magCal[0] = (maxX + minX) / 2.0f;
  magCal[1] = (maxY + minY) / 2.0f;
  magCal[2] = avgSpan / spanX;
  magCal[3] = avgSpan / spanY;

  // ---- 0-degree reference: average facing of the first samples ----
  int zeroCount = sampleCount < MAG_CAL_ZERO_SAMPLES ? sampleCount : MAG_CAL_ZERO_SAMPLES;
  float sumSin = 0.0f, sumCos = 0.0f;
  for (int i = 0; i < zeroCount; ++i) {
    float a = rawMagAngle(sampleX[i], sampleY[i]) * PI / 180.0f;
    sumSin += sinf(a);
    sumCos += cosf(a);
  }
  float zeroAngle = atan2f(sumSin, sumCos) * 180.0f / PI;

  // ---- Direction and number of turns: follow the angle through the run ----
  float totalTurn = 0.0f;     // signed, degrees
  int forwardSteps = 0, backwardSteps = 0;
  float prev = rawMagAngle(sampleX[0], sampleY[0]);
  for (int i = 1; i < sampleCount; ++i) {
    float a = rawMagAngle(sampleX[i], sampleY[i]);
    float d = wrapAngleDelta(a - prev);
    prev = a;
    totalTurn += d;
    if (d > 0.5f) ++forwardSteps;
    else if (d < -0.5f) ++backwardSteps;
  }
  float turns = fabsf(totalTurn) / 360.0f;
  // The user turns CLOCKWISE, so whichever sign the raw angle moved in is
  // "increasing heading" for this board.
  float direction = totalTurn >= 0.0f ? 1.0f : -1.0f;
  int movingSteps = forwardSteps + backwardSteps;
  float evenness = movingSteps ? (float)(direction > 0 ? forwardSteps : backwardSteps) / movingSteps : 0.0f;

  Serial.printf("  samples=%d zero_raw=%.1f total_turn=%.1f deg (%.2f turns) dir=%+.0f even=%.0f%%\n",
                sampleCount, zeroAngle, totalTurn, turns, direction, evenness * 100.0f);

  if (turns < MAG_CAL_MIN_TURNS) {
    memcpy(magCal, previous, sizeof(magCal));
    Serial.println("COMPASS CALIBRATION FAILED: less than one full turn, old values kept");
    drawCenteredText("CALIBRATION FAILED", 130, RED, BLACK, 2);
    drawCenteredText("Turned " + String(turns, 1) + " times - need 2-3 turns.", 160, WHITE, BLACK, 1);
    drawCenteredText("Rotate CLOCKWISE in full circles.", 175, WHITE, BLACK, 1);
    delay(3000);
    return;
  }

  magCal[MAG_CAL_ZERO] = zeroAngle;
  magCal[MAG_CAL_DIR] = direction;

  Preferences prefs;
  prefs.begin("mag", false);
  prefs.putBytes("cal", magCal, sizeof(magCal));
  prefs.end();

  Serial.printf("COMPASS CALIBRATION SAVED: offset X=%.1f Y=%.1f  scale X=%.3f Y=%.3f  zero=%.1f dir=%+.0f\n",
                magCal[0], magCal[1], magCal[2], magCal[3],
                magCal[MAG_CAL_ZERO], magCal[MAG_CAL_DIR]);
  drawCenteredText("CALIBRATION SAVED", 120, GREEN, BLACK, 2);
  drawCenteredText("Turns measured: " + String(turns, 1), 150, WHITE, BLACK, 1);
  // Discrepancy hints: under 2 turns, or many steps in the wrong direction,
  // means the result may be less accurate.
  if (turns < 2.0f) {
    drawCenteredText("Under 2 turns: consider repeating", 165, YELLOW, BLACK, 1);
  }
  if (evenness < 0.8f) {
    drawCenteredText("Uneven rotation: consider repeating", 180, YELLOW, BLACK, 1);
  }
  delay(2500);
}

// =====================================================
// PACKET TEST
// Double-tap the TIME on the main menu. This device sends TEST_PACKET_COUNT
// test packets; every awake peer echoes each one back in its own time slot
// (so peers never collide), and peers hold their heartbeats until it ends.
// The result shows, per peer, how many test packets it received (one way),
// how many replies came back, the round-trip time, and signal both ways.
// =====================================================

#define TEST_PACKET_COUNT 20
// SF10: a test packet takes ~0.25 s and a reply ~0.37 s on air. The two
// responders reply one after the other (slots by rank, not ID), then the
// next test packet follows: 0.25 + 0.01 + 2 x 0.42 s + margin = 1.15 s.
// 20 packets take ~23 s.
#define TEST_PACKET_INTERVAL 1150UL
#define TEST_REPLY_SLOT 420UL

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
// Test packets received from the device that is testing (one-way count,
// sent back in every reply so the tester can show the delivery rate).
int peerTestRxCount = 0;
int peerTestLastSeq = -1;
unsigned long packetTestSentAt[TEST_PACKET_COUNT];
PacketSample packetSamples[4][TEST_PACKET_COUNT];
// Highest one-way count each peer reported in this test (-1 = not reported).
int packetTestDelivered[4];

// Each device answers in its own slot so replies from #2 and #3 don't collide.
// The two responders take slots 1 and 2 in ID order (the tester's own ID
// is skipped), so no slot is wasted.
unsigned long packetTestReplyDelay(int id, int testerId) {
  int rank = (testerId >= 1 && testerId < id) ? id - 1 : id;   // 1 or 2
  return 10UL + (unsigned long)(rank - 1) * TEST_REPLY_SLOT;
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

// receivedAt = when the radio finished receiving this packet (DIO0
// interrupt time, see receiveLoRa()), not when the loop got to it.
void handlePacketTest(const String &packet, unsigned long receivedAt) {
  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  if (packet.startsWith("T,")) {
    // T,<sender>,<seq> -> echo it back.
    int comma = packet.indexOf(',', 2);
    if (comma < 0) return;
    int sender = packet.substring(2, comma).toInt();
    int seq = packet.substring(comma + 1).toInt();
    if (sender < 1 || sender > 3 || sender == DEVICE_ID || sleepMode) return;

    // A new test (other sender, earlier one finished, or sequence restarted)
    // restarts the one-way count.
    if (sender != peerTestDevice || (long)(peerTestActiveUntil - millis()) <= 0 ||
        seq <= peerTestLastSeq) {
      peerTestRxCount = 0;
    }
    ++peerTestRxCount;
    peerTestLastSeq = seq;

    peerTestActiveUntil = millis() + 2UL * TEST_PACKET_INTERVAL + 500UL;
    peerTestDevice = sender;
    statusTimeoutHoldUntil = peerTestActiveUntil + 2UL * HEARTBEAT_INTERVAL;

    // Hold this device's heartbeat until the test is over. While a heartbeat
    // is on air this radio cannot hear the next test packet, and it collides
    // with the other replies (a timing model puts this at ~3 of every 20).
    // Staggered by device ID so the held heartbeats don't all go out at once.
    unsigned long resumeAt = peerTestActiveUntil + (unsigned long)(DEVICE_ID - 1) * 250UL +
                             (unsigned long)random(HEARTBEAT_JITTER_MS + 1);
    if ((long)(nextHeartbeat - resumeAt) < 0) nextHeartbeat = resumeAt;

    // Busy with our own test: don't transmit over it.
    if (packetTestRunning) return;

    remote[sender].lastSeen = millis();
    remote[sender].online = true;
    cachedRemoteOnline[sender] = true;

    // Reply in this device's slot, counted from when the test packet
    // ARRIVED. Counting from when the loop read it (after a screen redraw,
    // say) made late replies run into the next test packet, which this
    // radio then could not hear. A reply that can no longer finish before
    // the next test packet is skipped: the following reply still reports
    // the received count, so the delivery result is unaffected.
    String reply = "TR," + String(DEVICE_ID) + "," + String(sender) + "," +
                   String(seq) + "," + String(rssi) + "," + String(snr, 1) + "," +
                   String(peerTestRxCount);
    unsigned long replyAt = receivedAt + packetTestReplyDelay(DEVICE_ID, sender);
    // Next test packet starts TEST_PACKET_INTERVAL after this one started,
    // i.e. (interval - this packet's airtime) after it arrived.
    unsigned long nextPacketAt = receivedAt + TEST_PACKET_INTERVAL -
      (unsigned long)estimateLoRaToAMs((uint8_t)packet.length());
    unsigned long replyAirtime = (unsigned long)estimateLoRaToAMs((uint8_t)reply.length()) + 10UL;
    unsigned long now = millis();
    if ((long)(now - replyAt) < 0) {
      delay(replyAt - now);
    } else if ((long)(now + replyAirtime - nextPacketAt) > 0) {
      Serial.printf("TEST REPLY SKIPPED seq=%d (%lu ms late)\n", seq, now - replyAt);
      return;
    }
    sendRawLoRa(reply);
    return;
  }

  // TR,<responder>,<to>,<seq>,<rssi>,<snr>,<received>
  // (<received> = test packets the responder got so far; older firmware omits it)
  String f[6];
  int startIndex = 3;
  int count = 0;
  while (count < 6) {
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
  if (responder < 1 || responder > 3 || responder == DEVICE_ID) return;

  // Any valid reply proves the responder is awake, even one addressed to
  // another device: heartbeats are held during a test, so this keeps the
  // two responders from marking each other OFFLINE.
  remote[responder].lastSeen = millis();
  remote[responder].online = true;
  cachedRemoteOnline[responder] = true;

  if (!packetTestRunning || to != DEVICE_ID) return;
  if (seq < 0 || seq >= TEST_PACKET_COUNT || packetTestSentAt[seq] == 0) return;

  if (count >= 6) {
    int delivered = f[5].toInt();
    if (delivered >= 1 && delivered <= TEST_PACKET_COUNT &&
        delivered > packetTestDelivered[responder]) {
      packetTestDelivered[responder] = delivered;
    }
  }

  PacketSample &s = packetSamples[responder][seq];
  if (s.received) return;   // duplicate

  // A late reply (arriving after the next packet was sent) still counts,
  // timed against its own packet, up to when the radio received the reply.
  unsigned long rtt = receivedAt - packetTestSentAt[seq];
  unsigned long slot = packetTestReplyDelay(responder, DEVICE_ID);
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
  updateGPS();
  if (!sleepMode && !proxSleepActive && heartbeatDue()) {
    sendHeartbeat();
    heartbeatSent();
  }
}

// =====================================================
// DISTANCE CALIBRATION
// On the CHECK LOCATION screen, hold the device list (bottom rows) for 3 s.
// With all devices side by side, each GPS reports a slightly different
// position; that apparent offset to each peer (which should be 0 m) is
// averaged over 30 s and then subtracted from later distances/directions.
// Each device calibrates its own view, so run it on every device (they can
// run at the same time). Saved in flash; it stays until recalibrated.
// =====================================================

void runDistanceCalibration() {
  if (sleepMode) {
    tft.fillScreen(BLACK);
    drawCenteredText("DISTANCE CALIBRATION", 130, WHITE, BLACK, 2);
    drawCenteredText("Turn off SLEEP MODE first", 165, YELLOW, BLACK, 1);
    delay(2500);
    return;
  }
  // Proximity sleep only listens 2 s out of 10 s; wake fully so every peer
  // report is heard. The proximity machine re-evaluates afterwards.
  if (proxSleepActive) {
    Serial.println("Waking from proximity sleep for distance calibration");
    exitProximitySleep();
    setProxState(PROX_WAKE_AND_RECOVERY, millis());
    proxRecoveryUntil = millis() + PROX_RECOVERY_HOLD_MS;
  }

  Serial.println("DISTANCE CALIBRATION: devices side by side for 30 s");
  tft.fillScreen(BLACK);
  drawCenteredText("DISTANCE CALIBRATION", 40, WHITE, BLACK, 2);
  drawCenteredText("Place ALL devices side by side", 80, YELLOW, BLACK, 1);
  drawCenteredText("Keep them still, GPS facing up", 95, YELLOW, BLACK, 1);
  drawCenteredText("Run this on every device", 110, YELLOW, BLACK, 1);

  int ownCount = 0;   // seconds with a valid own fix
  double vecN[4] = {0.0, 0.0, 0.0, 0.0};
  double vecE[4] = {0.0, 0.0, 0.0, 0.0};
  double vecN2[4] = {0.0, 0.0, 0.0, 0.0};
  double vecE2[4] = {0.0, 0.0, 0.0, 0.0};
  int peerCount[4] = {0, 0, 0, 0};
  unsigned long lastPeerUpdate[4];
  for (int id = 1; id <= 3; ++id) lastPeerUpdate[id] = remote[id].locationUpdatedAt;

  unsigned long start = millis();
  unsigned long lastOwnSample = 0;
  unsigned long lastDraw = 0;
  while (millis() - start < DIST_CAL_TIME_MS) {
    serviceRadio();   // receive peer packets, read GPS, keep heartbeats going

    // Own fix: counted once per second while valid.
    if (localLocationValid() && millis() - lastOwnSample >= 1000UL) {
      lastOwnSample = millis();
      ++ownCount;
    }
    // Peer offset vectors: one sample per newly received valid location,
    // measured from this device's position at that moment. Their mean is
    // the offset to remove; their spread is the GPS wander (noise floor).
    for (int id = 1; id <= 3; ++id) {
      if (id == DEVICE_ID) continue;
      if (remote[id].gpsValid && remote[id].locationUpdatedAt != lastPeerUpdate[id]) {
        lastPeerUpdate[id] = remote[id].locationUpdatedAt;
        if (!localLocationValid()) continue;
        float n, e;
        offsetMeters(latitude, longitude, remote[id].lat, remote[id].lon, n, e);
        vecN[id] += n;
        vecE[id] += e;
        vecN2[id] += (double)n * n;
        vecE2[id] += (double)e * e;
        ++peerCount[id];
      }
    }

    if (millis() - lastDraw >= 500UL) {
      lastDraw = millis();
      int secondsLeft = (int)((DIST_CAL_TIME_MS - (millis() - start) + 999UL) / 1000UL);
      String peersText = "";
      for (int id = 1; id <= 3; ++id) {
        if (id == DEVICE_ID) continue;
        if (peersText.length()) peersText += "   ";
        peersText += "LORA #" + String(id) + ": " + String(peerCount[id]);
      }
      tft.fillRect(0, 140, SCREEN_W, 80, BLACK);
      drawCenteredText(String(secondsLeft) + " s", 145, GREEN, BLACK, 2);
      drawCenteredText("Own GPS fixes: " + String(ownCount), 180, WHITE, BLACK, 1);
      drawCenteredText(peersText, 195, WHITE, BLACK, 1);
    }
    delay(5);
  }

  tft.fillScreen(BLACK);
  drawCenteredText("DISTANCE CALIBRATION", 40, WHITE, BLACK, 2);

  if (ownCount < DIST_CAL_MIN_OWN_SAMPLES) {
    Serial.printf("DISTANCE CALIBRATION FAILED: only %d own GPS fixes\n", ownCount);
    drawCenteredText("No GPS fix on this device", 120, RED, BLACK, 1);
    drawCenteredText("Go outside and try again", 140, WHITE, BLACK, 1);
    delay(3000);
    return;
  }

  int y = 90;
  bool anySaved = false;
  for (int id = 1; id <= 3; ++id) {
    if (id == DEVICE_ID) continue;
    String line = "LORA #" + String(id) + ": ";
    uint16_t color;
    if (peerCount[id] < DIST_CAL_MIN_PEER_SAMPLES) {
      line += "no location data";
      color = GRAY;
      Serial.printf("  LORA #%d: only %d location reports, not calibrated\n", id, peerCount[id]);
    } else {
      float north = (float)(vecN[id] / peerCount[id]);
      float east = (float)(vecE[id] / peerCount[id]);
      float varN = (float)(vecN2[id] / peerCount[id]) - north * north;
      float varE = (float)(vecE2[id] / peerCount[id]) - east * east;
      float wander = sqrtf(fmaxf(0.0f, varN + varE));   // RMS radius, metres
      float floorM = constrain(2.0f * wander, DIST_NOISE_FLOOR_MIN_M, DIST_NOISE_FLOOR_MAX_M);
      float offset = sqrtf(north * north + east * east);
      if (offset > DIST_CAL_MAX_OFFSET_M) {
        line += String(offset, 0) + " m apart - together?";
        color = RED;
        Serial.printf("  LORA #%d: offset %.1f m too large, refused\n", id, offset);
      } else {
        distCal[id].valid = true;
        distCal[id].north = north;
        distCal[id].east = east;
        distCal[id].noiseFloor = floorM;
        anySaved = true;
        line += "zeroed (" + String(offset, 1) + " m offset)";
        color = GREEN;
        Serial.printf("  LORA #%d: offset north=%.1f east=%.1f (%.1f m), wander %.1f m -> floor %.1f m, from %d own / %d peer samples, saved\n",
                      id, north, east, offset, wander, floorM, ownCount, peerCount[id]);
      }
    }
    drawCenteredText(line, y, color, BLACK, 1);
    y += 18;
  }

  if (anySaved) {
    saveDistanceCalibration();
    drawCenteredText("Saved - stays until you recalibrate", y + 12, LIGHTGREEN, BLACK, 1);
  } else {
    drawCenteredText("Nothing saved - previous values kept", y + 12, YELLOW, BLACK, 1);
  }
  delay(4000);
}

// GPS distance to a peer in meters, or -1 when either side has no GPS fix.
// The same calibrated value the CHECK LOCATION rows show.
float packetTestDistance(int id) {
  if (!localLocationValid() || !remoteLocationFresh(id)) return -1.0f;
  return peerDisplayDistance(id);
}

// Plain GPS distance with no calibration applied, for comparison.
float packetTestRawDistance(int id) {
  if (!localLocationValid() || !remoteLocationFresh(id)) return -1.0f;
  float n, e;
  offsetMeters(latitude, longitude, remote[id].lat, remote[id].lon, n, e);
  return sqrtf(n * n + e * e);
}

// Small clock at the top center of the result screen (same text size as the
// DIST CAL label); redrawn only when the time changes.
void drawPacketTestClock(bool force) {
  static String lastText = "";
  int h, mi, s, d, mo, y;
  bool valid;
  getDisplayClock(h, mi, s, d, mo, y, valid);
  char buf[12];
  if (valid) sprintf(buf, "%02d:%02d:%02d", h, mi, s);
  else sprintf(buf, "--:--:--");
  String text = buf;
  if (!force && text == lastText) return;
  lastText = text;
  tft.fillRect(80, 2, 80, 10, BLACK);
  drawCenteredText(text, 3, YELLOW, BLACK, 1);
}

// While the result screen waits for a tap: radio, GPS, heartbeats, clock.
void packetTestResultIdle() {
  serviceRadio();
  drawPacketTestClock(false);
  delay(5);
}

String csvDistance(float meters) {
  return meters < 0 ? String("") : String(meters, 1);
}

// Serial CSV output (saved on the PC by tools/packet_logger.py):
// CSV,device,test,seq,peer,status,rtt_ms,rssi_here,snr_here,rssi_there,snr_there,distance_m,own_lat,own_lon,peer_lat,peer_lon
// SUM,device,test,peer,received,sent,success_pct,avg_rtt_ms,avg_rssi_here,avg_snr_here,avg_rssi_there,avg_snr_there,distance_m,delivered,delivered_pct,raw_distance_m
// received/success_pct = round trips (packet there AND reply back);
// delivered/delivered_pct = packets the peer received (one way, the thesis
// packet success rate; blank if the peer runs older firmware).
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
  if (sleepMode || proxSleepActive) {
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
    updateGPS();
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
  for (int id = 0; id <= 3; id++) packetTestDelivered[id] = -1;

  tft.fillScreen(BLACK);
  drawCenteredText("PACKET TEST", 20, WHITE, BLACK, 2);
  drawCenteredText("Sending " + String(TEST_PACKET_COUNT) + " packets...", 50, YELLOW, BLACK, 1);

  Serial.printf("PACKET TEST START device=%d test=%d\n", DEVICE_ID, packetTestId);
  packetTestRunning = true;

  // Packets go out on a fixed 300 ms schedule. The counter is redrawn right
  // after each packet is sent (replies wait in the radio meanwhile), so the
  // ~40 ms redraw no longer stretches the gap between packets.
  unsigned long testStart = millis();
  for (int seq = 0; seq < TEST_PACKET_COUNT; seq++) {
    unsigned long dueAt = testStart + (unsigned long)seq * TEST_PACKET_INTERVAL;
    while ((long)(millis() - dueAt) < 0) {
      receiveLoRa();
      updateGPS();
      delay(1);
    }
    packetTestSentAt[seq] = millis();
    sendRawLoRa("T," + String(DEVICE_ID) + "," + String(seq));

    tft.fillRect(0, 80, SCREEN_W, 40, BLACK);
    drawCenteredText(String(seq + 1) + " / " + String(TEST_PACKET_COUNT), 85, GREEN, BLACK, 2);
  }
  // Let the last packet's reply slots pass before the grace period.
  while ((long)(millis() - (testStart + (unsigned long)TEST_PACKET_COUNT * TEST_PACKET_INTERVAL)) < 0) {
    receiveLoRa();
    updateGPS();
    delay(1);
  }

  // Short grace period for the last replies.
  unsigned long graceStart = millis();
  while (millis() - graceStart < TEST_PACKET_INTERVAL) {
    receiveLoRa();
    updateGPS();
    delay(1);
  }

  packetTestRunning = false;
  nextHeartbeat = millis() + HEARTBEAT_INTERVAL;
  // The peers held their heartbeats during the test (and this device's loop
  // was not running): give them two cycles to be heard before any timeout.
  statusTimeoutHoldUntil = millis() + 2UL * HEARTBEAT_INTERVAL;

  printPacketTestCsv();

  // ---- Results ----
  tft.fillScreen(BLACK);
  drawPacketTestClock(true);
  drawCenteredText("PACKET TEST RESULT", 15, WHITE, BLACK, 2);
  drawCenteredText("Packets received by each device", 34, GRAY, BLACK, 1);

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
    float rawDistance = packetTestRawDistance(id);

    // Headline = packets this peer received (one way), as in the thesis
    // packet success rate. Every reply proves its packet arrived, so it is
    // never below the round-trip count. Older peers don't report it: then
    // the round-trip count is shown.
    int delivered = packetTestDelivered[id];
    bool oneWay = delivered >= 0;
    if (oneWay && delivered < received) delivered = received;
    int shownCount = oneWay ? delivered : received;
    int shownPct = (shownCount * 100) / TEST_PACKET_COUNT;

    uint16_t color = shownPct >= 90 ? GREEN : (shownPct >= 50 ? YELLOW : RED);
    if (received == 0) color = GRAY;

    drawCenteredText("LORA #" + String(id) + ": " + String(shownCount) + "/" +
                     String(TEST_PACKET_COUNT) + " (" + String(shownPct) + "%)",
                     y, color, BLACK, 2);

    String distText;
    if (distance < 0) {
      distText = "Distance: no GPS fix";
    } else if (distCalActive(id)) {
      distText = "Distance: " + String((int)roundf(distance)) + " m (GPS, raw " +
                 String((int)roundf(rawDistance)) + " m)";
    } else {
      distText = "Distance: " + String((int)roundf(distance)) + " m (GPS, not calibrated)";
    }

    String sum = "SUM," + String(DEVICE_ID) + "," + String(packetTestId) + "," +
                 String(id) + "," + String(received) + "," + String(TEST_PACKET_COUNT) + "," +
                 String(pct) + ",";

    if (received > 0) {
      anyReply = true;
      String rttText = oneWay
        ? "Replies back: " + String(received) + "/" + String(TEST_PACKET_COUNT) +
          ", avg " + String(rttSum / received) + " ms"
        : "Round trip only, avg " + String(rttSum / received) + " ms";
      drawCenteredText(rttText, y + 25, WHITE, BLACK, 1);
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
      // No reply came back, so this device cannot know how many of its
      // packets arrived; the peer counts them itself and shows the result.
      drawCenteredText("No replies came back to this device", y + 25, GRAY, BLACK, 1);
      drawCenteredText("Packets received: see LORA #" + String(id) + "'s screen", y + 40, YELLOW, BLACK, 1);
      sum += ",,,,,";
    }
    drawCenteredText(distText, y + 70, distance < 0 ? GRAY : LOCATION_GREEN, BLACK, 1);
    sum += csvDistance(distance) + ",";
    sum += oneWay ? String(delivered) + "," + String(shownPct) + "," : String(",,");
    sum += csvDistance(rawDistance);
    Serial.println(sum);

    y += 100;
  }

  if (anyReply) {
    drawCenteredText("RSSI: -30 great ... -130 at the limit", 255, GRAY, BLACK, 1);
  }
  drawCenteredText("Tap anywhere to go back", 295, YELLOW, BLACK, 1);
  Serial.println("PACKET TEST END");

  // Wait for the finger to lift, then for a new tap (or 30 s).
  unsigned long shownAt = millis();
  while (touch.touched() && millis() - shownAt < 30000UL) packetTestResultIdle();
  while (!touch.touched() && millis() - shownAt < 30000UL) packetTestResultIdle();
  while (touch.touched() && millis() - shownAt < 31000UL) packetTestResultIdle();
}

// =====================================================
// DEVELOPER SCREEN
// Hold the title "LORA THE EXPLORER" on the main menu for 3 s. Turns
// automatic proximity sleep on/off (saved in flash, per device). While it
// is off this device reports "apart" in its heartbeats, so no device in the
// group goes into proximity sleep - handy while testing distances.
// =====================================================

void loadDeveloperSettings() {
  Preferences prefs;
  prefs.begin("dev", true);
  proxSleepEnabled = prefs.getBool("proxOn", true);
  prefs.end();
  Serial.printf("Developer: automatic proximity sleep %s\n", proxSleepEnabled ? "ON" : "OFF");
}

void saveDeveloperSettings() {
  Preferences prefs;
  prefs.begin("dev", false);
  prefs.putBool("proxOn", proxSleepEnabled);
  prefs.end();
}

// One tap on a blocking screen, in screen coordinates (same mapping as
// processTouch()). Returns false when nothing new was tapped.
bool readScreenTap(int &screenX, int &screenY) {
  if (!touch.touched()) return false;
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(LORA_CS, HIGH);
  digitalWrite(TOUCH_CS, LOW);
  TS_Point p = touch.getPoint();
  digitalWrite(TOUCH_CS, HIGH);
  // Wait for release so one tap counts once.
  unsigned long pressedAt = millis();
  while (touch.touched() && millis() - pressedAt < 3000UL) { serviceRadio(); delay(5); }
  if (p.x <= 50 || p.y <= 50 || p.x >= 4045 || p.y >= 4045) return false;
  float sx = touchCal[0] * (float)p.x + touchCal[1] * (float)p.y + touchCal[2];
  float sy = touchCal[3] * (float)p.x + touchCal[4] * (float)p.y + touchCal[5];
  screenX = constrain((int)lroundf(sx), 0, 239);
  screenY = constrain((int)lroundf(sy), 0, 319);
  return true;
}

void drawDeveloperToggle() {
  tft.fillRect(22, 110, 196, 48, proxSleepEnabled ? GREEN : GRAY);
  drawCenteredText(proxSleepEnabled ? "AUTO SLEEP: ON" : "AUTO SLEEP: OFF", 127, BLACK,
                   proxSleepEnabled ? GREEN : GRAY, 2);
  tft.fillRect(0, 165, SCREEN_W, 30, BLACK);
  drawCenteredText(proxSleepEnabled ? "Group sleeps when all are < 10 m"
                                    : "Off: the whole group stays awake",
                   170, WHITE, BLACK, 1);
  drawCenteredText("(saved on this device)", 183, GRAY, BLACK, 1);
}

void runDeveloperScreen() {
  Serial.println("DEVELOPER SCREEN");
  tft.fillScreen(BLACK);
  drawCenteredText("DEVELOPER", 20, YELLOW, BLACK, 2);
  drawCenteredText("Proximity sleep (10 m)", 85, WHITE, BLACK, 1);
  drawDeveloperToggle();
  tft.fillRect(22, 240, 196, 48, WHITE);
  drawCenteredText("BACK", 257, BLACK, WHITE, 2);

  // Wait for the finger from the 3 s hold to lift first.
  while (touch.touched()) { serviceRadio(); delay(5); }

  unsigned long lastActivity = millis();
  while (millis() - lastActivity < 30000UL) {
    serviceRadio();
    int x, y;
    if (readScreenTap(x, y)) {
      lastActivity = millis();
      if (x >= 22 && x < 218 && y >= 110 && y < 158) {
        proxSleepEnabled = !proxSleepEnabled;
        saveDeveloperSettings();
        Serial.printf("Developer: automatic proximity sleep %s\n", proxSleepEnabled ? "ON" : "OFF");
        drawDeveloperToggle();
      } else if (x >= 22 && x < 218 && y >= 240 && y < 288) {
        break;
      }
    }
    delay(5);
  }
}

// =====================================================
// LORA RADIO SETTINGS AND HEALTH CHECK
// =====================================================

void configureLoRaRadio() {
  LoRa.setTxPower(LORA_TX_DBM);
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW_HZ);
  LoRa.setCodingRate4(5);
  // Payload CRC: near the edge of range a packet can arrive with damaged
  // bytes; without a CRC it was accepted and could carry wrong coordinates.
  // Damaged packets are now dropped by the radio (parsePacket() returns 0).
  LoRa.enableCrc();
}

uint8_t readLoRaRegister(uint8_t reg) {
  digitalWrite(TFT_CS, HIGH);
  digitalWrite(TOUCH_CS, HIGH);
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(LORA_CS, LOW);
  SPI.transfer(reg & 0x7F);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(LORA_CS, HIGH);
  SPI.endTransaction();
  return value;
}

// A supply dip during a 20 dBm transmission (~120 mA) can reset the radio
// chip. It then comes back on its power-on defaults (433 MHz-band frequency,
// SF7): it can no longer hear or be heard, and the device shows the others -
// and the others show it - OFFLINE until it is restarted. Every 10 s the
// key registers are checked and the radio is set up again if they changed.
void checkRadioHealth() {
  uint8_t version = readLoRaRegister(0x42);            // RegVersion: 0x12
  uint8_t opMode = readLoRaRegister(0x01);             // bit 7 = LoRa mode
  uint8_t sf = readLoRaRegister(0x1E) >> 4;            // RegModemConfig2
  uint8_t frfMsb = readLoRaRegister(0x06);             // RegFrfMsb
  uint8_t expectedFrfMsb = (uint8_t)((((uint64_t)LORA_FREQ << 19) / 32000000ULL) >> 16);
  if (version == 0x12 && (opMode & 0x80) && sf == LORA_SF && frfMsb == expectedFrfMsb) return;

  Serial.printf("LORA RADIO LOST ITS SETTINGS (version=0x%02X opmode=0x%02X sf=%u frf=0x%02X) -> re-initializing\n",
                version, opMode, sf, frfMsb);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LORA RADIO NOT RESPONDING - check its 3.3 V/VIN supply and wiring");
    return;
  }
  configureLoRaRadio();
  if (radioSleeping) LoRa.sleep();
  else LoRa.receive();
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
  // Touch is ignored while another device's packet test runs here (a tap
  // could open a blocking screen or send a ping over the test).
  if ((long)(peerTestActiveUntil - millis()) <= 0) processTouch();
  updateProximity();
  serviceProximitySleep();

  if (sleepMode) {
    if ((long)(millis() - nextSleepLocationReport) >= 0 && sleepReportSlotReady()) {
      sendSleepLocationReport();
      if (sleepAnnounceRepeats > 0) {
        --sleepAnnounceRepeats;
        nextSleepLocationReport = millis() + SLEEP_ANNOUNCE_GAP_MS;
      } else {
        nextSleepLocationReport = millis() + SLEEP_LOCATION_INTERVAL;
      }
    }
  } else if (!proxSleepActive) {
    // In proximity sleep, reports are sent by serviceProximitySleep().
    // Held while a ping waits for its reply (at most PING_REPLY_WINDOW): a
    // heartbeat on air would make this radio deaf to the reply.
    if (!pingPending && heartbeatDue()) {
      sendHeartbeat();
      heartbeatSent();
    }
  }

  // While another device's packet test is running here, the screen is not
  // redrawn: a redraw keeps the loop from reading test packets and sending
  // replies on time. Everything is redrawn once the test is over.
  static bool screenPausedForTest = false;
  if ((long)(peerTestActiveUntil - millis()) > 0) {
    if (!screenPausedForTest) Serial.println("PACKET TEST from another device: screen paused");
    screenPausedForTest = true;
    delay(1);
    return;
  }
  bool forceRefresh = false;
  if (screenPausedForTest) {
    screenPausedForTest = false;
    forceRefresh = true;
    lastScreenRefresh = millis();
    Serial.println("PACKET TEST over: screen resumed");
    // One-way result, counted here: the tester's own screen only knows
    // about packets whose reply made it back.
    if (peerTestRxCount > 0 && peerTestDevice >= 1 && peerTestDevice <= 3) {
      int pct = (peerTestRxCount * 100) / TEST_PACKET_COUNT;
      peerTestResultMessage = "Test from LORA #" + String(peerTestDevice) + ": got " +
                              String(peerTestRxCount) + "/" + String(TEST_PACKET_COUNT) +
                              " (" + String(pct) + "%)";
      peerTestResultAt = millis();
      Serial.printf("PACKET TEST RX from=%d received=%d sent=%d pct=%d\n",
                    peerTestDevice, peerTestRxCount, TEST_PACKET_COUNT, pct);
    }
  }
  if (millis() - lastScreenRefresh >= SCREEN_REFRESH_INTERVAL) {
    lastScreenRefresh = millis();
    forceRefresh = true;
    checkDisplayHealth();
    checkRadioHealth();
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
