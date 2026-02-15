/*
  (C) James Theimer 2026 Poor Man's Throttle
  ESP32 BLE Heavy-Train Throttle Controller (FINAL SPEC)

  LED behavior (GPIO2) — per your request:
  - Default / disconnected: LED blinks continuously
  - Connected: LED stays solid ON
  - Disconnected: LED returns to blinking
  - RX/TX while connected (solid ON): LED briefly turns OFF (a quick dip), then returns solid ON

  Target: ESP32-WROOM-32 (Arduino framework)
  Motor driver: IBT-2 / BTS7960 (RPWM/LPWM)
  BLE: Custom service with RX (Write / WriteNR) + TX (Notify)
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <type_traits>

// ------------------------- Firmware ID -------------------------
static const char* FW_NAME    = "GScaleThrottle";
static const char* FW_VERSION = "1.0.0";

// ------------------------- Strict no-float guard (compile-time) -------------------------
#ifndef DISABLE_STRICT_NO_FLOAT_GUARD
  #define float  __FORBIDDEN_FLOAT_TYPE_USE_INTEGER_FIXED_POINT__
  #define double __FORBIDDEN_DOUBLE_TYPE_USE_INTEGER_FIXED_POINT__
#endif

// ------------------------- State enums (MUST be before use) -------------------------
enum class Direction : uint8_t { STOP = 0, FWD = 1, REV = 2 };
enum class RampKind  : uint8_t { NONE = 0, MOMENTUM, BRAKE, QUICKSTOP };
enum class PendingStage : uint8_t { NONE = 0, WAIT_DIR_DELAY };

// ------------------------- Pins & PWM -------------------------
static const int PIN_RPWM = 25;   // Forward PWM -> RPWM
static const int PIN_LPWM = 26;   // Reverse PWM -> LPWM

static const uint32_t PWM_FREQ_HZ  = 20000;   // ~20kHz
static const uint8_t  PWM_RES_BITS = 10;      // 8–10 bits allowed
static const uint8_t  PWM_CH_R     = 0;
static const uint8_t  PWM_CH_L     = 1;
static const uint32_t PWM_MAX_DUTY = (1UL << PWM_RES_BITS) - 1;

// ------------------------- Fixed-point easing -------------------------
static const int32_t P_SCALE = 1000; // mandatory

// ------------------------- Timing constants -------------------------
static const uint32_t FULL_MOMENTUM_ACCEL_MS = 10000; // full-scale 100 step
static const uint32_t FULL_MOMENTUM_DECEL_MS = 6000;
static const uint32_t FULL_BRAKE_MS          = 4000;
static const uint32_t FULL_QUICKSTOP_MS      = 1500;
static const uint32_t DIR_CHANGE_DELAY_MS    = 250;
static const uint32_t BLE_GRACE_MS           = 10000;

// ------------------------- BLE UUIDs (custom) -------------------------
static const char* SERVICE_UUID = "9b2b7b30-5f3d-4a51-9bd6-1e8cde2c9a10";
static const char* RX_UUID      = "9b2b7b31-5f3d-4a51-9bd6-1e8cde2c9a10"; // Write / WriteNR
static const char* TX_UUID      = "9b2b7b32-5f3d-4a51-9bd6-1e8cde2c9a10"; // Notify

// ------------------------- Motion/BLE state -------------------------
static volatile bool debugMode = false;

static int32_t appliedThrottle = 0;      // 0..100
static int32_t targetThrottle  = 0;      // 0..100
static Direction currentDirection = Direction::STOP;
static Direction targetDirection  = Direction::STOP;

// Ramp state
static bool rampActive = false;
static uint32_t rampStartMs = 0;
static uint32_t rampDurationMs = 0;
static int32_t rampStartThrottle = 0;
static int32_t rampTargetThrottle = 0;
static Direction rampDirection = Direction::STOP;
static RampKind rampKind = RampKind::NONE;

// Start assist config + state
static int32_t cfgMinStart = 0;
static int32_t cfgKickThrottle = 0;
static int32_t cfgKickMs = 0;

static bool kickActive = false;
static uint32_t kickEndMs = 0;
static int32_t kickHoldThrottle = 0;
static Direction kickDirection = Direction::STOP;

// Reverse sequencing state
static bool reversePending = false;
static PendingStage pendingStage = PendingStage::NONE;
static uint32_t pendingStageUntilMs = 0;
static int32_t pendingFinalTargetThrottle = 0;
static Direction pendingFinalDirection = Direction::STOP;
static bool pendingFinalIsInstant = false;
static bool pendingFinalIsMomentum = false;

// BLE state
static bool bleConnected = false;
static bool graceActive = false;
static uint32_t disconnectMs = 0;
static bool forcedStopLatched = false;

static NimBLECharacteristic* pTxChar = nullptr;
static NimBLEServer* pServerGlobal = nullptr;

// MTU-aware chunking state
static uint16_t g_peerMtu = 23;  // default

// ------------------------- Status LED (GPIO2) -------------------------
// Your requested behavior:
// - Disconnected: blink continuously
// - Connected: solid ON
// - RX/TX while connected: briefly OFF (dip), then back ON
static const int LED_PIN = 2;

// Blink timing while disconnected
static const uint32_t LED_BLINK_ON_MS  = 500;
static const uint32_t LED_BLINK_OFF_MS = 500;

// Dip timing (brief OFF) on activity while connected
static const uint16_t LED_DIP_MS_RX = 35;
static const uint16_t LED_DIP_MS_TX = 20;

static bool     ledIsOn = false;
static uint32_t ledNextToggleMs = 0;

// Activity dip state
static bool     ledDipActive = false;
static uint32_t ledDipUntilMs = 0;

static inline void ledWrite(bool on) {
  digitalWrite(LED_PIN, on ? HIGH : LOW);
  ledIsOn = on;
}

static inline void ledInit() {
  pinMode(LED_PIN, OUTPUT);
  ledWrite(false);
  ledNextToggleMs = millis();
  ledDipActive = false;
  ledDipUntilMs = 0;
}

// Call on RX while connected: briefly force LED OFF
static inline void ledDipRx() {
  if (!bleConnected) return;         // dips are only meaningful in solid-on mode
  ledDipActive = true;
  ledDipUntilMs = millis() + (uint32_t)LED_DIP_MS_RX;
  ledWrite(false);
}

// Call on TX while connected: briefly force LED OFF
static inline void ledDipTx() {
  if (!bleConnected) return;
  ledDipActive = true;
  ledDipUntilMs = millis() + (uint32_t)LED_DIP_MS_TX;
  ledWrite(false);
}

// Non-blocking LED service; call often from loop()
static inline void ledService() {
  uint32_t now = millis();

  // 1) If connected: base state is SOLID ON, except during a dip
  if (bleConnected) {
    if (ledDipActive) {
      if ((int32_t)(now - ledDipUntilMs) >= 0) {
        ledDipActive = false;
        ledWrite(true); // back to solid ON
      }
    } else {
      // Ensure solid ON
      if (!ledIsOn) ledWrite(true);
    }
    return;
  }

  // 2) If disconnected: blinking
  ledDipActive = false; // no dips in blink mode

  if ((int32_t)(now - ledNextToggleMs) < 0) return;

  if (ledIsOn) {
    ledWrite(false);
    ledNextToggleMs = now + LED_BLINK_OFF_MS;
  } else {
    ledWrite(true);
    ledNextToggleMs = now + LED_BLINK_ON_MS;
  }
}

// ------------------------- Helpers -------------------------
static inline int32_t clampI32(int32_t v, int32_t lo, int32_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static inline bool isDigitStr(const String& s) {
  if (s.length() == 0) return false;
  for (size_t i = 0; i < s.length(); i++) {
    if (!isDigit((unsigned char)s[i])) return false;
  }
  return true;
}

static uint32_t scaledDurationMs(uint32_t fullScaleMs, int32_t deltaThrottleAbs) {
  deltaThrottleAbs = clampI32(deltaThrottleAbs, 0, 100);
  uint32_t dur = (uint32_t)((uint64_t)fullScaleMs * (uint64_t)deltaThrottleAbs / 100ULL);
  if (deltaThrottleAbs > 0 && dur == 0) dur = 1;
  return dur;
}

static void debugPrintln(const String& s) {
  if (debugMode) Serial.println(s);
}

static uint32_t throttleToDuty(int32_t thr) {
  thr = clampI32(thr, 0, 100);
  return (uint32_t)((uint64_t)thr * (uint64_t)PWM_MAX_DUTY / 100ULL);
}

static void applyPwmOutputs(Direction dir, int32_t thr) {
  uint32_t duty = throttleToDuty(thr);
  switch (dir) {
    case Direction::FWD:
      ledcWrite(PWM_CH_R, duty);
      ledcWrite(PWM_CH_L, 0);
      break;
    case Direction::REV:
      ledcWrite(PWM_CH_R, 0);
      ledcWrite(PWM_CH_L, duty);
      break;
    case Direction::STOP:
    default:
      ledcWrite(PWM_CH_R, 0);
      ledcWrite(PWM_CH_L, 0);
      break;
  }
}

static void stopMotorNow(const char* reason) {
  appliedThrottle = 0;
  targetThrottle = 0;
  currentDirection = Direction::STOP;
  targetDirection  = Direction::STOP;

  rampActive = false;
  rampKind = RampKind::NONE;

  kickActive = false;

  reversePending = false;
  pendingStage = PendingStage::NONE;

  applyPwmOutputs(Direction::STOP, 0);

  if (debugMode) {
    Serial.print("[STOP] ");
    Serial.println(reason);
  }
}

// ------------------------- MTU-aware BLE notify (chunked) -------------------------
static size_t getNotifyPayloadLimit() {
  const size_t fallback = 20;
  if (!bleConnected) return fallback;

  uint16_t peerMtu = g_peerMtu;
  if (peerMtu < 23) return fallback;

  size_t payload = (size_t)peerMtu - 3;
  if (payload < 20) payload = 20;
  return payload;
}

static void bleNotifyChunked(const String& text) {
  if (!pTxChar || !bleConnected) return;

  const size_t limit = getNotifyPayloadLimit();
  const size_t n = text.length();

  size_t i = 0;
  while (i < n) {
    size_t take = n - i;
    if (take > limit) take = limit;

    String part = text.substring((unsigned int)i, (unsigned int)(i + take));
    pTxChar->setValue(part.c_str());

    // ✅ Your new behavior: TX briefly turns LED OFF (dip) while connected/solid
    ledDipTx();

    pTxChar->notify();
    i += take;
  }
}

static void sendACK(const String& original)  { bleNotifyChunked("ACK:" + original); }
static void sendERR(const String& original)  { bleNotifyChunked("ERR:" + original); }
static void sendMENU(const String& text)     { bleNotifyChunked("MENU:" + text); }

// ------------------------- Ramp engine (fixed-point smoothstep) -------------------------
static int32_t smoothstepEasedThrottle(int32_t startThr, int32_t targetThr, uint32_t elapsedMs, uint32_t durationMs) {
  if (durationMs == 0) return targetThr;

  int32_t p = (int32_t)((uint64_t)elapsedMs * (uint64_t)P_SCALE / (uint64_t)durationMs);
  p = clampI32(p, 0, P_SCALE);

  int64_t p64 = (int64_t)p;
  int64_t p2  = p64 * p64;
  int64_t p3  = p2  * p64;

  int64_t term1 = (3LL * p2) / (int64_t)P_SCALE;
  int64_t term2 = (2LL * p3) / ((int64_t)P_SCALE * (int64_t)P_SCALE);
  int32_t e = (int32_t)(term1 - term2); // 0..P_SCALE

  int32_t delta = (targetThr - startThr);
  int32_t out = startThr + (int32_t)(((int64_t)delta * (int64_t)e) / (int64_t)P_SCALE);
  return clampI32(out, 0, 100);
}

static void cancelAllMotionActivities() {
  rampActive = false;
  kickActive = false;
  reversePending = false;
  pendingStage = PendingStage::NONE;
}

static void startRamp(Direction dirDuringRamp, int32_t startThr, int32_t endThr, uint32_t durationMs, RampKind kind) {
  cancelAllMotionActivities();

  startThr = clampI32(startThr, 0, 100);
  endThr   = clampI32(endThr, 0, 100);

  rampKind = kind;
  rampDirection = dirDuringRamp;

  targetDirection = (endThr == 0) ? Direction::STOP : dirDuringRamp;
  targetThrottle  = endThr;

  if (durationMs == 0 || startThr == endThr) {
    appliedThrottle = endThr;
    currentDirection = (endThr == 0) ? Direction::STOP : dirDuringRamp;
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  rampActive = true;
  rampStartMs = millis();
  rampDurationMs = durationMs;
  rampStartThrottle = startThr;
  rampTargetThrottle = endThr;

  currentDirection = (startThr == 0 && endThr == 0) ? Direction::STOP : dirDuringRamp;
}

// ------------------------- Start assist (MINSTART + KICK) -------------------------
static int32_t effectiveStartTargetThrottle(int32_t requestedThr, bool startingFromStop) {
  requestedThr = clampI32(requestedThr, 0, 100);
  if (!startingFromStop) return requestedThr;
  if (cfgMinStart > 0 && requestedThr > 0) return max(requestedThr, cfgMinStart);
  return requestedThr;
}

static bool shouldKickOnStart(bool startingFromStop, int32_t finalTargetThr) {
  if (!startingFromStop) return false;
  if (finalTargetThr <= 0) return false;
  if (cfgKickThrottle <= 0 || cfgKickMs <= 0) return false;
  return true;
}

static void beginKick(Direction dir, int32_t finalTargetThr, bool afterKickIsInstant, bool afterKickIsMomentum) {
  int32_t kickThr = max(cfgKickThrottle, cfgMinStart);
  kickThr = clampI32(kickThr, 0, 100);

  kickActive = true;
  kickDirection = dir;
  kickHoldThrottle = kickThr;
  kickEndMs = millis() + (uint32_t)cfgKickMs;

  appliedThrottle = kickHoldThrottle;
  currentDirection = dir;
  applyPwmOutputs(currentDirection, appliedThrottle);

  reversePending = true;
  pendingStage = PendingStage::NONE;
  pendingFinalTargetThrottle = finalTargetThr;
  pendingFinalDirection = dir;
  pendingFinalIsInstant = afterKickIsInstant;
  pendingFinalIsMomentum = afterKickIsMomentum;

  targetDirection = (finalTargetThr == 0) ? Direction::STOP : dir;
  targetThrottle  = finalTargetThr;

  if (debugMode) {
    Serial.print("[KICK] dir=");
    Serial.print((dir == Direction::FWD) ? "FWD" : "REV");
    Serial.print(" hold=");
    Serial.print(kickHoldThrottle);
    Serial.print("ms target=");
    Serial.println(finalTargetThr);
  }
}

static void continueAfterKickIfNeeded() {
  if (!reversePending) return;

  Direction dir = pendingFinalDirection;
  int32_t thr = clampI32(pendingFinalTargetThrottle, 0, 100);
  bool isInstant = pendingFinalIsInstant;
  bool isMomentum = pendingFinalIsMomentum;

  reversePending = false;

  if (isInstant) {
    appliedThrottle = thr;
    currentDirection = (thr == 0) ? Direction::STOP : dir;
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  if (isMomentum) {
    int32_t startThr = appliedThrottle;
    int32_t deltaAbs = abs(thr - startThr);
    uint32_t dur = scaledDurationMs(FULL_MOMENTUM_ACCEL_MS, deltaAbs);
    startRamp(dir, startThr, thr, dur, RampKind::MOMENTUM);
    return;
  }

  appliedThrottle = thr;
  currentDirection = (thr == 0) ? Direction::STOP : dir;
  applyPwmOutputs(currentDirection, appliedThrottle);
}

// ------------------------- Motion command execution -------------------------
static void executeStopRamp(RampKind kind) {
  cancelAllMotionActivities();

  int32_t startThr = appliedThrottle;
  int32_t deltaAbs = abs(startThr);

  uint32_t full = FULL_QUICKSTOP_MS;
  if (kind == RampKind::BRAKE) full = FULL_BRAKE_MS;
  if (kind == RampKind::MOMENTUM) full = FULL_MOMENTUM_DECEL_MS;

  uint32_t dur = scaledDurationMs(full, deltaAbs);

  targetDirection = Direction::STOP;
  targetThrottle  = 0;

  Direction dir = (currentDirection == Direction::STOP) ? Direction::STOP : currentDirection;
  startRamp(dir, startThr, 0, dur, kind);
}

static void scheduleReverseAfterStop(Direction finalDir, int32_t finalThr, bool isInstant, bool isMomentum) {
  pendingFinalTargetThrottle = finalThr;
  pendingFinalDirection = finalDir;
  pendingFinalIsInstant = isInstant;
  pendingFinalIsMomentum = isMomentum;

  reversePending = true;
  pendingStage = PendingStage::NONE;
}

static void executeInstant(Direction dir, int32_t requestedThr) {
  cancelAllMotionActivities();

  int32_t thr = clampI32(requestedThr, 0, 100);

  targetDirection = (thr == 0) ? Direction::STOP : dir;
  targetThrottle  = thr;

  if (thr == 0) {
    stopMotorNow("Instant throttle=0");
    return;
  }

  if (currentDirection != Direction::STOP && currentDirection != dir) {
    scheduleReverseAfterStop(dir, thr, /*instant*/true, /*momentum*/false);
    executeStopRamp(RampKind::QUICKSTOP);
    return;
  }

  bool startingFromStop = (appliedThrottle == 0 && currentDirection == Direction::STOP);
  int32_t effThr = effectiveStartTargetThrottle(thr, startingFromStop);

  targetDirection = (effThr == 0) ? Direction::STOP : dir;
  targetThrottle  = effThr;

  if (shouldKickOnStart(startingFromStop, effThr)) {
    beginKick(dir, effThr, /*afterKickIsInstant*/true, /*afterKickIsMomentum*/false);
    return;
  }

  appliedThrottle = effThr;
  currentDirection = dir;
  applyPwmOutputs(currentDirection, appliedThrottle);
}

static void executeMomentum(Direction dir, int32_t requestedThr) {
  cancelAllMotionActivities();

  int32_t thr = clampI32(requestedThr, 0, 100);

  targetDirection = (thr == 0) ? Direction::STOP : dir;
  targetThrottle  = thr;

  if (thr == 0) {
    executeStopRamp(RampKind::MOMENTUM);
    return;
  }

  if (currentDirection != Direction::STOP && currentDirection != dir) {
    scheduleReverseAfterStop(dir, thr, /*instant*/false, /*momentum*/true);
    executeStopRamp(RampKind::MOMENTUM);
    return;
  }

  bool startingFromStop = (appliedThrottle == 0 && currentDirection == Direction::STOP);
  int32_t effTarget = effectiveStartTargetThrottle(thr, startingFromStop);

  targetDirection = (effTarget == 0) ? Direction::STOP : dir;
  targetThrottle  = effTarget;

  if (startingFromStop) currentDirection = dir;

  if (shouldKickOnStart(startingFromStop, effTarget)) {
    beginKick(dir, effTarget, /*afterKickIsInstant*/false, /*afterKickIsMomentum*/true);
    return;
  }

  int32_t startThr = appliedThrottle;
  int32_t deltaAbs = abs(effTarget - startThr);
  if (deltaAbs == 0) {
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  uint32_t full = (effTarget > startThr) ? FULL_MOMENTUM_ACCEL_MS : FULL_MOMENTUM_DECEL_MS;
  uint32_t dur = scaledDurationMs(full, deltaAbs);
  startRamp(dir, startThr, effTarget, dur, RampKind::MOMENTUM);
}

// ------------------------- BLE callbacks -------------------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    (void)pServer;
    bleConnected = true;

    g_peerMtu = connInfo.getMTU();
    if (g_peerMtu < 23) g_peerMtu = 23;

    // When connected, LEDService will hold SOLID ON automatically
    if (graceActive) {
      graceActive = false;
      debugPrintln("[BLE] Reconnected, grace cancelled");
    } else {
      debugPrintln("[BLE] Connected");
    }

    if (debugMode) {
      Serial.print("[BLE] MTU=");
      Serial.println(g_peerMtu);
    }
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    (void)pServer; (void)connInfo; (void)reason;

    bleConnected = false;
    g_peerMtu = 23;

    // When disconnected, LEDService will blink automatically
    graceActive = true;
    disconnectMs = millis();
    debugPrintln("[BLE] Disconnected, grace started (10s)");

    NimBLEDevice::startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;

    std::string val = pCharacteristic->getValue();
    if (val.empty()) return;

    // ✅ Your new behavior: RX briefly turns LED OFF (dip) while connected/solid
    ledDipRx();

    String preserved = String(val.c_str());
    String original = preserved;

    original.trim();
    original.replace("\r", "");
    original.replace("\n", "");

    String upper = original;
    upper.toUpperCase();

    if (debugMode) {
      Serial.print("[CMD] ");
      Serial.println(preserved);
    }

    bool allowMotionNow = !(forcedStopLatched && !bleConnected);

    // HELP
    if (upper == "?" || upper == "?1") {
      sendACK(preserved);
      sendMENU("F<n> R<n>: momentum 0-100");
      sendMENU("FQ<n> RQ<n>: instant 0-100");
      sendMENU("S: quick stop  B: brake");
      sendMENU("MINSTART<n>  KICK<t>,<ms>");
      sendMENU("D1 debug on  D0 off  ?2 more");
      return;
    }
    if (upper == "?2") {
      sendACK(preserved);
      sendMENU("F<n>: ramp forward to n (smooth)");
      sendMENU("R<n>: ramp reverse to n (smooth)");
      sendMENU("Reverse: ramp to 0, wait 250ms");
      sendMENU("FQ/RQ: quick-stop, wait, jump");
      sendMENU("S: quick stop 1.5s full-scale");
      sendMENU("B: brake 4.0s full-scale");
      sendMENU("MINSTART: only when starting");
      sendMENU("KICK: hold then ramp/jump");
      sendMENU("D1/D0: Serial debug only");
      return;
    }

    // DEBUG
    if (upper == "D1") {
      debugMode = true;
      Serial.begin(115200);
      Serial.println("[DEBUG] ON");
      Serial.print("[FW] ");
      Serial.print(FW_NAME);
      Serial.print(" v");
      Serial.println(FW_VERSION);
      Serial.println("[BOOT] ready");

      if (bleConnected) {
        Serial.print("[BLE] MTU=");
        Serial.println(g_peerMtu);
      }
      sendACK(preserved);
      return;
    }
    if (upper == "D0") {
      sendACK(preserved);
      debugMode = false;
      return;
    }

    // MINSTART
    if (upper.startsWith("MINSTART")) {
      String nStr = original.substring(String("MINSTART").length());
      nStr.trim();
      if (!isDigitStr(nStr)) { sendERR(preserved); return; }
      cfgMinStart = clampI32(nStr.toInt(), 0, 100);
      if (debugMode) { Serial.print("[CFG] MINSTART="); Serial.println(cfgMinStart); }
      sendACK(preserved);
      return;
    }

    // KICK<t>,<ms>
    if (upper.startsWith("KICK")) {
      String args = original.substring(4);
      args.trim();
      int comma = args.indexOf(',');
      if (comma < 0) { sendERR(preserved); return; }

      String tStr  = args.substring(0, comma);
      String msStr = args.substring(comma + 1);
      tStr.trim(); msStr.trim();

      if (!isDigitStr(tStr) || !isDigitStr(msStr)) { sendERR(preserved); return; }

      cfgKickThrottle = clampI32(tStr.toInt(), 0, 100);
      cfgKickMs       = clampI32(msStr.toInt(), 0, 2000);

      if (debugMode) {
        Serial.print("[CFG] KICK=");
        Serial.print(cfgKickThrottle);
        Serial.print(",");
        Serial.println(cfgKickMs);
      }

      sendACK(preserved);
      return;
    }

    // STOPS
    if (upper == "S") {
      if (allowMotionNow) executeStopRamp(RampKind::QUICKSTOP);
      else stopMotorNow("Forced-stop latched; S keeps stopped");
      sendACK(preserved);
      return;
    }
    if (upper == "B") {
      if (allowMotionNow) executeStopRamp(RampKind::BRAKE);
      else stopMotorNow("Forced-stop latched; B keeps stopped");
      sendACK(preserved);
      return;
    }

    // INSTANT: FQ / RQ
    if (upper.startsWith("FQ") || upper.startsWith("RQ")) {
      String nStr = original.substring(2);
      nStr.trim();
      if (!isDigitStr(nStr)) { sendERR(preserved); return; }

      int32_t n = clampI32(nStr.toInt(), 0, 100);

      if (allowMotionNow) {
        if (forcedStopLatched && bleConnected) forcedStopLatched = false;
        executeInstant(upper.startsWith("FQ") ? Direction::FWD : Direction::REV, n);
      } else {
        stopMotorNow("Forced-stop latched; motion ignored until reconnect");
      }

      sendACK(preserved);
      return;
    }

    // MOMENTUM: F / R
    if (upper.startsWith("F") || upper.startsWith("R")) {
      if (upper.startsWith("FQ") || upper.startsWith("RQ")) { sendERR(preserved); return; }

      String nStr = original.substring(1);
      nStr.trim();
      if (!isDigitStr(nStr)) { sendERR(preserved); return; }

      int32_t n = clampI32(nStr.toInt(), 0, 100);

      if (allowMotionNow) {
        if (forcedStopLatched && bleConnected) forcedStopLatched = false;
        executeMomentum(upper.startsWith("F") ? Direction::FWD : Direction::REV, n);
      } else {
        stopMotorNow("Forced-stop latched; motion ignored until reconnect");
      }

      sendACK(preserved);
      return;
    }

    sendERR(preserved);
  }
};

// ------------------------- Setup & Loop -------------------------
static void setupPwm() {
  ledcSetup(PWM_CH_R, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcSetup(PWM_CH_L, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcAttachPin(PIN_RPWM, PWM_CH_R);
  ledcAttachPin(PIN_LPWM, PWM_CH_L);
  applyPwmOutputs(Direction::STOP, 0);
}

static void setupBle() {
  NimBLEDevice::init(FW_NAME);

  pServerGlobal = NimBLEDevice::createServer();
  pServerGlobal->setCallbacks(new ServerCallbacks());

  NimBLEService* svc = pServerGlobal->createService(SERVICE_UUID);

  pTxChar = svc->createCharacteristic(TX_UUID, NIMBLE_PROPERTY::NOTIFY);

  NimBLECharacteristic* rx = svc->createCharacteristic(
    RX_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  rx->setCallbacks(new RxCallbacks());

  svc->start();

  NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setName(FW_NAME);  // shows up as "GScaleThrottle"
  adv->start();

  debugPrintln("[BLE] Advertising");
}

void setup() {
  ledInit();      // LED starts in "disconnected blink" mode by ledService()
  setupPwm();
  stopMotorNow("Boot");
  setupBle();
}

// ------------------------- Main loop tasks -------------------------
static void processRamp() {
  if (!rampActive) return;

  uint32_t now = millis();
  uint32_t elapsed = now - rampStartMs;

  appliedThrottle = smoothstepEasedThrottle(rampStartThrottle, rampTargetThrottle, elapsed, rampDurationMs);

  if (rampTargetThrottle == 0 && appliedThrottle == 0) {
    currentDirection = Direction::STOP;
    applyPwmOutputs(Direction::STOP, 0);
  } else {
    currentDirection = rampDirection;
    applyPwmOutputs(currentDirection, appliedThrottle);
  }

  if (elapsed >= rampDurationMs) {
    rampActive = false;

    appliedThrottle = rampTargetThrottle;
    if (appliedThrottle == 0) {
      currentDirection = Direction::STOP;
      applyPwmOutputs(Direction::STOP, 0);
    } else {
      currentDirection = rampDirection;
      applyPwmOutputs(currentDirection, appliedThrottle);
    }

    if (reversePending) {
      if (appliedThrottle == 0) {
        pendingStage = PendingStage::WAIT_DIR_DELAY;
        pendingStageUntilMs = millis() + DIR_CHANGE_DELAY_MS;
      } else {
        reversePending = false;
        pendingStage = PendingStage::NONE;
      }
    }
  }
}

static void processPendingReverse() {
  if (!reversePending) return;
  if (pendingStage != PendingStage::WAIT_DIR_DELAY) return;

  uint32_t now = millis();
  if ((int32_t)(now - pendingStageUntilMs) < 0) return;

  pendingStage = PendingStage::NONE;

  Direction dir = pendingFinalDirection;
  int32_t thr = clampI32(pendingFinalTargetThrottle, 0, 100);

  bool startingFromStop = (appliedThrottle == 0 && currentDirection == Direction::STOP);
  int32_t effThr = effectiveStartTargetThrottle(thr, startingFromStop);

  targetDirection = (effThr == 0) ? Direction::STOP : dir;
  targetThrottle  = effThr;

  if (shouldKickOnStart(startingFromStop, effThr)) {
    beginKick(dir, effThr, pendingFinalIsInstant, pendingFinalIsMomentum);
    return;
  }

  reversePending = false;

  if (pendingFinalIsInstant) {
    appliedThrottle = effThr;
    currentDirection = (effThr == 0) ? Direction::STOP : dir;
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  if (pendingFinalIsMomentum) {
    uint32_t dur = scaledDurationMs(FULL_MOMENTUM_ACCEL_MS, abs(effThr));
    startRamp(dir, 0, effThr, dur, RampKind::MOMENTUM);
    return;
  }

  appliedThrottle = effThr;
  currentDirection = (effThr == 0) ? Direction::STOP : dir;
  applyPwmOutputs(currentDirection, appliedThrottle);
}

static void processKick() {
  if (!kickActive) return;

  uint32_t now = millis();
  if ((int32_t)(now - kickEndMs) < 0) {
    appliedThrottle = kickHoldThrottle;
    currentDirection = kickDirection;
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  kickActive = false;
  continueAfterKickIfNeeded();
}

static void processBleGrace() {
  if (!graceActive) return;

  uint32_t now = millis();
  if ((now - disconnectMs) >= BLE_GRACE_MS) {
    graceActive = false;
    forcedStopLatched = true;

    stopMotorNow("BLE grace timeout -> forced stop latched");
    debugPrintln("[BLE] Grace timeout: stopped + latched");
  }
}

void loop() {
  // Keep LED behavior running
  ledService();

  processBleGrace();
  processKick();
  processRamp();
  processPendingReverse();

  if (!rampActive && !kickActive) {
    if (appliedThrottle == 0) currentDirection = Direction::STOP;
    applyPwmOutputs(currentDirection, appliedThrottle);
  }
}
