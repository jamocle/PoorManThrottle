/*
  (C) James Theimer 2026 Poor Man's Throttle
  ESP32 BLE Heavy-Train Throttle Controller (FINAL SPEC)

  LED behavior (GPIO2):
  - Default / disconnected: LED blinks continuously
  - Connected: LED stays solid ON
  - Disconnected: LED returns to blinking
  - RX/TX while connected (solid ON): LED briefly turns OFF (a quick dip), then returns solid ON

  Target: ESP32-WROOM-32 (Arduino framework)
  Motor driver: IBT-2 / BTS7960 (RPWM/LPWM)
  BLE: Custom service with RX (Write / WriteNR) + TX (Notify)

    ------------------------- BLE COMMAND REFERENCE -------------------------

  Notes:
  - Commands are case-insensitive.
  - Whitespace + CR/LF are trimmed.
  - For most commands, ESP32 responds via TX Notify with:
      ACK:<original command>   (valid)
      ERR:<original command>   (invalid)
  - Throttle values are clamped to 0..100.
  - Stop-first reversing is enforced (ramps to 0, wait 250ms, then reverse).

  MOTION (Momentum / Nonlinear Ramp)
  - F<n>        : Forward to throttle n (0..100) using momentum ramp (smoothstep)
                Accel full-scale=10s, Decel full-scale=6s
                Example: F40
  - R<n>        : Reverse to throttle n (0..100) using momentum ramp
                Example: R25

  MOTION (Instant)
  - FQ<n>       : Forward immediate to throttle n (0..100)
                If reversing, performs quick-stop ramp -> 250ms -> jump
                Example: FQ60
  - RQ<n>       : Reverse immediate to throttle n (0..100)
                Example: RQ60

  STOPS
  - S           : Quick stop ramp to 0 (full-scale 1.5s)
  - B           : Brake ramp to 0 (full-scale 4.0s)

  START ASSIST CONFIG
  - M<n>        : MINSTART-Set minimum starting throttle (0..100)
                Applied ONLY when starting from stop; does NOT block decel below it
                Example: M15
  - K<t>,<ms>   : KICK-Set start kick throttle and duration
                t = 0..100, ms = 0..2000
                When starting from stop with target > 0, apply max(t, MINSTART) for ms,
                then continue ramp/instant behavior
                Example: K60,200

  DEBUG (Serial only)
  - D<n>        : n=1 Debug ON (prints FW name/version, BLE MTU, events)
                : n=0 Debug OFF

  - ?           : State query (returns raw state text via notify; NO ACK/ERR wrapper)
                Returns one of:
                    STOPPED
                    FORWARD <appliedThrottle>
                    REVERSE <appliedThrottle>
  - ??          : Hardware State query (returns raw state text via notify; NO ACK/ERR wrapper)
                Returns one of:
                    STOPPED
                    FORWARD <appliedThrottle>
                    REVERSE <appliedThrottle>
  - G           : last 3 digits of the GUID
  - V           : Version query (responds as ACK:<FW_VERSION>)
                Example: V  ->  ACK:1.0.6
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <type_traits>

// ------------------------- Startup defaults -------------------------
static const bool DEBUG_AT_STARTUP = false;

// ------------------------- Firmware ID -------------------------
static const char* FW_NAME    = "GScaleThrottle";
static const char* FW_VERSION = "1.0.9";

// ------------------------- BLE UUIDs (custom) -------------------------
static const char* SERVICE_UUID = "9b2b7b30-5f3d-4a51-9bd6-1e8cde2c9000";
static const char* RX_UUID      = "9b2b7b31-5f3d-4a51-9bd6-1e8cde2c9000"; // Write / WriteNR
static const char* TX_UUID      = "9b2b7b32-5f3d-4a51-9bd6-1e8cde2c9000"; // Notify

// ------------------------- Strict no-float guard (compile-time) -------------------------
#ifndef DISABLE_STRICT_NO_FLOAT_GUARD
  #define float  __FORBIDDEN_FLOAT_TYPE_USE_INTEGER_FIXED_POINT__
  #define double __FORBIDDEN_DOUBLE_TYPE_USE_INTEGER_FIXED_POINT__
#endif

// ------------------------- State enums (MUST be before use) -------------------------
enum class Direction : uint8_t { STOP = 0, FWD = 1, REV = 2 };
enum class RampKind  : uint8_t { NONE = 0, MOMENTUM, BRAKE, QUICKSTOP };
enum class PendingStage : uint8_t { NONE = 0, WAIT_DIR_DELAY };

// ------------------------- Hardware readback (debug-only) -------------------------
struct HwSnapshot {
  bool ren;
  bool len;
  bool enabled;

  uint32_t dutyR;
  uint32_t dutyL;

  Direction hwDir;
  int32_t hwThrottlePct;   // 0..100 derived from duty/max
};

// ------------------------- Pins & PWM -------------------------
static const int PIN_RPWM = 25;   // Forward PWM -> RPWM
static const int PIN_LPWM = 26;   // Reverse PWM -> LPWM
static const int PIN_REN = 27;  // IBT-2 R_EN
static const int PIN_LEN = 33;  // IBT-2 L_EN
static const uint32_t PWM_FREQ_HZ  = 20000;   // ~20kHz
static const uint8_t  PWM_RES_BITS = 10;      // 8–10 bits allowed
static const uint8_t  PWM_CH_R     = 0;
static const uint8_t  PWM_CH_L     = 1;
static const uint32_t PWM_MAX_DUTY = (1UL << PWM_RES_BITS) - 1;

// ------------------------- PWM State ---------------------------
static bool pwmInitialized = false;

// ------------------------- Fixed-point easing -------------------------
static const int32_t P_SCALE = 1000; // mandatory

// ------------------------- Timing constants -------------------------
static const uint32_t FULL_MOMENTUM_ACCEL_MS = 20000; // full-scale 100 step
static const uint32_t FULL_MOMENTUM_DECEL_MS = 20000;
static const uint32_t FULL_QUICKRAMP_ACCEL_MS = 2500; // 0->100 in 4s (tune)
static const uint32_t FULL_QUICKRAMP_DECEL_MS = 2500; 
static const uint32_t FULL_BRAKE_MS          = 10000;
static const uint32_t FULL_QUICKSTOP_MS      = 3000;
static const uint32_t DIR_CHANGE_DELAY_MS    = 2000;
static const uint32_t BLE_GRACE_MS           = 15000;
static const uint32_t GRACE_COUNTDOWN_LOG_PERIOD_MS = 1000; // log once per second in debug
static const uint32_t DEBUG_HW_SNAPSHOT_PERIOD_MS = 2000;
static const bool DEBUG_PRINT_PERIODIC_ONLY_ON_MISMATCH = true;
static const bool DEBUG_PRINT_STORED_ONLY_ON_MISMATCH = false;

// Pending reverse needs to remember which ramp constants to use after the delay
static uint32_t pendingFullAccelMs = FULL_MOMENTUM_ACCEL_MS;
static uint32_t pendingFullDecelMs = FULL_MOMENTUM_DECEL_MS;

// ------------------------- Motion/BLE state -------------------------
static volatile bool debugMode = DEBUG_AT_STARTUP;

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

static bool printPeriodic = DEBUG_PRINT_PERIODIC_ONLY_ON_MISMATCH;

// MTU-aware chunking state
static uint16_t g_peerMtu = 23;  // default

// ------------------------- Status LED (GPIO2) -------------------------
// - Disconnected: blink continuously (double-blink search pattern)
// - Connected: solid ON
// - RX/TX while connected: briefly OFF (dip), then back ON
static const int LED_PIN = 2;

// Blink timing while disconnected
static const uint16_t LED_SEARCH_ON_1_MS   = 70;
static const uint16_t LED_SEARCH_GAP_MS    = 90;
static const uint16_t LED_SEARCH_ON_2_MS   = 70;
static const uint16_t LED_SEARCH_PAUSE_MS  = 700;

// Dip timing (brief OFF) on activity while connected
static const uint16_t LED_DIP_MS_RX = 150;
static const uint16_t LED_DIP_MS_TX = 100;

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
      if (!ledIsOn) ledWrite(true);
    }
    return;
  }

  // 2) If disconnected: double-blink search pattern
  ledDipActive = false; // no dips in blink mode

  static uint8_t searchStep = 0;

  if ((int32_t)(now - ledNextToggleMs) < 0) return;

  switch (searchStep) {
    case 0:
      ledWrite(true);
      ledNextToggleMs = now + LED_SEARCH_ON_1_MS;
      searchStep = 1;
      break;

    case 1:
      ledWrite(false);
      ledNextToggleMs = now + LED_SEARCH_GAP_MS;
      searchStep = 2;
      break;

    case 2:
      ledWrite(true);
      ledNextToggleMs = now + LED_SEARCH_ON_2_MS;
      searchStep = 3;
      break;

    default: // pause
      ledWrite(false);
      ledNextToggleMs = now + LED_SEARCH_PAUSE_MS;
      searchStep = 0;
      break;
  }
}

// ------------------------- Helpers -------------------------
static inline void cancelPendingReverse() {
  reversePending = false;
  pendingStage = PendingStage::NONE;
}

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

// ------------------------- Throttle change logging -------------------------
static int32_t lastLoggedAppliedThrottle = -9999;
static Direction lastLoggedDirection = Direction::STOP;
static const char* g_lastDbgReason = "BOOT";

static const char* dirStr(Direction d) {
  switch (d) {
    case Direction::STOP: return "STOP";
    case Direction::FWD:  return "FWD";
    case Direction::REV:  return "REV";
    default:              return "?";
  }
}

// ------------------------- Hardware readback (debug-only) -------------------------
static inline uint32_t readLedcDuty(uint8_t pwmChannel) {
  // Read back the PWM duty using the Arduino-ESP32 API (ledcRead).
  // This reflects the *configured LEDC duty register* (0..PWM_MAX_DUTY),
  // not motor physics (voltage/current/speed).
  // Note: If PWM isn’t initialized yet, return 0 to avoid junk reads.
  if (!pwmInitialized) return 0;
  return (uint32_t)ledcRead(pwmChannel);
}

static HwSnapshot getHwSnapshot() {
  HwSnapshot s{};

  // EN pins as seen by GPIO input buffers
  s.ren = (digitalRead(PIN_REN) == HIGH);
  s.len = (digitalRead(PIN_LEN) == HIGH);
  s.enabled = (s.ren && s.len);

  // PWM duties as currently configured in LEDC hardware
  s.dutyR = readLedcDuty(PWM_CH_R);
  s.dutyL = readLedcDuty(PWM_CH_L);

  // Derive hardware direction + throttle %
  if (!s.enabled || (s.dutyR == 0 && s.dutyL == 0)) {
    s.hwDir = Direction::STOP;
    s.hwThrottlePct = 0;
  } else if (s.dutyR > 0 && s.dutyL == 0) {
    s.hwDir = Direction::FWD;
    s.hwThrottlePct = (int32_t)(((uint64_t)s.dutyR * 100ULL + (PWM_MAX_DUTY / 2)) / (uint64_t)PWM_MAX_DUTY);
  } else if (s.dutyL > 0 && s.dutyR == 0) {
    s.hwDir = Direction::REV;
    s.hwThrottlePct = (int32_t)(((uint64_t)s.dutyL * 100ULL + (PWM_MAX_DUTY / 2)) / (uint64_t)PWM_MAX_DUTY);
  } else {
    // Should never happen in the logic (both sides driven), but useful as a fault indicator
    s.hwDir = Direction::STOP;
    s.hwThrottlePct = 0;
  }

  s.hwThrottlePct = clampI32(s.hwThrottlePct, 0, 100);
  return s;
}

static bool hwMatchesStored(const HwSnapshot& hw) {
  // Treat stored STOP / appliedThrottle==0 as STOP expectation
  Direction storedDir = (appliedThrottle <= 0 || currentDirection == Direction::STOP)
                          ? Direction::STOP
                          : currentDirection;

  int32_t storedPct = clampI32(appliedThrottle, 0, 100);

  // Direction must match, and throttle should be close (PWM quantization can cause +/-1%)
  if (hw.hwDir != storedDir) return false;

  int32_t diff = abs(hw.hwThrottlePct - storedPct);
  return (diff <= 2);
}

static void logThrottleChangeIfNeeded(const char* reason) {
  if (!debugMode) return;

  // Periodic snapshot (C)
  static uint32_t lastPeriodicMs = 0;
  uint32_t now = millis();
  bool periodicDue = (lastPeriodicMs == 0) || ((now - lastPeriodicMs) >= DEBUG_HW_SNAPSHOT_PERIOD_MS);

  bool storedChanged =
      (appliedThrottle != lastLoggedAppliedThrottle) ||
      (currentDirection != lastLoggedDirection);

  // If neither stored change nor periodic due, do nothing.
  if (!storedChanged && !periodicDue) return;

  // Take periodic timestamp *now* so we don't hammer if we decide not to print.
  // (We still "sample" at the requested interval; printing is conditional.)
  if (periodicDue) lastPeriodicMs = now;

  // Read hardware snapshot (A)
  HwSnapshot hw = getHwSnapshot();
  bool match = hwMatchesStored(hw);

  // Decide whether to print:
  // - Always print on stored change
  // - For periodic: print only when mismatch (if toggle enabled), else print every period
  bool shouldPrint = false;

  if (storedChanged) {
    if (DEBUG_PRINT_STORED_ONLY_ON_MISMATCH) {
      shouldPrint = !match;
    } else {
      shouldPrint = true;
    }
  } else if (periodicDue) {
    if (printPeriodic) {
      shouldPrint = !match;
    } else {
      shouldPrint = true;
    }
  }

  if (!shouldPrint) {
  // Even if we suppress printing, we must advance the baseline
  // or "storedChanged" will remain true forever.
  if (storedChanged) {
    lastLoggedAppliedThrottle = appliedThrottle;
    lastLoggedDirection = currentDirection;
  }
    return;
  }


  Serial.print("[THR] ");
  Serial.print(reason);

  if (!storedChanged && periodicDue) {
    Serial.print(" (periodic)");
  }

  // --- STORED ---
  Serial.print(" | STORED dir=");
  Serial.print(dirStr(currentDirection));
  Serial.print(" applied=");
  Serial.print(appliedThrottle);
  Serial.print(" targetDir=");
  Serial.print(dirStr(targetDirection));
  Serial.print(" target=");
  Serial.print(targetThrottle);
  Serial.print(" ramp=");
  Serial.print(rampActive ? "1" : "0");
  Serial.print(" kick=");
  Serial.print(kickActive ? "1" : "0");
  Serial.print(" pending=");
  Serial.print(reversePending ? "1" : "0");
  Serial.print(" ble=");
  Serial.print(bleConnected ? "1" : "0");
  Serial.print(" latch=");
  Serial.print(forcedStopLatched ? "1" : "0");

  // --- HW ---
  Serial.print(" | HW EN=");
  Serial.print(hw.enabled ? "1" : "0");
  Serial.print(" (REN=");
  Serial.print(hw.ren ? "1" : "0");
  Serial.print(" LEN=");
  Serial.print(hw.len ? "1" : "0");
  Serial.print(")");
  Serial.print(" dutyR=");
  Serial.print(hw.dutyR);
  Serial.print(" dutyL=");
  Serial.print(hw.dutyL);
  Serial.print(" hwDir=");
  Serial.print(dirStr(hw.hwDir));
  Serial.print(" hw%=");
  Serial.print(hw.hwThrottlePct);

  // --- Mismatch indicator + quick diff ---
  Serial.print(" | ");
  if (match) {
    Serial.println("OK");
  } else {
    // Use the SAME expectation logic as hwMatchesStored()
    Direction storedDir = (appliedThrottle <= 0 || currentDirection == Direction::STOP)
                            ? Direction::STOP
                            : currentDirection;
    int32_t storedPct = clampI32(appliedThrottle, 0, 100);

    Serial.print("MISMATCH");
    Serial.print(" dDir=");
    Serial.print((int)hw.hwDir - (int)storedDir);
    Serial.print(" d%=");
    Serial.println(hw.hwThrottlePct - storedPct);
  }

  // Update baseline only when stored state changed
  if (storedChanged) {
    lastLoggedAppliedThrottle = appliedThrottle;
    lastLoggedDirection = currentDirection;
  }
}

static inline void setApplied(Direction dir, int32_t thr, const char* reason) {
  thr = clampI32(thr, 0, 100);
  currentDirection = (thr == 0) ? Direction::STOP : dir;
  appliedThrottle  = thr;

  // Debug-only: remember why the last state changed.
  g_lastDbgReason = reason;

  // DO NOT log here anymore (hardware may not be updated yet).
}

static inline void setTarget(Direction dir, int32_t thr, const String& reason) {
  thr = clampI32(thr, 0, 100);
  targetDirection = (thr == 0) ? Direction::STOP : dir;
  targetThrottle  = thr;

  if (debugMode) {
    Serial.print("[TGT] ");
    Serial.print(reason);
    Serial.print(" | targetDir=");
    Serial.print(dirStr(targetDirection));
    Serial.print(" target=");
    Serial.println(targetThrottle);
  }
}

static uint32_t throttleToDuty(int32_t thr) {
  thr = clampI32(thr, 0, 100);
  return (uint32_t)((uint64_t)thr * (uint64_t)PWM_MAX_DUTY / 100ULL);
}

static inline void driverEnable(bool en) {
  digitalWrite(PIN_REN, en ? HIGH : LOW);
  digitalWrite(PIN_LEN, en ? HIGH : LOW);
}

static void applyPwmOutputs(Direction dir, int32_t thr) {
  if (thr <= 0 || dir == Direction::STOP) {
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, 0);
    driverEnable(false);   // EN LOW = true off/coast
    return;
  }

  uint32_t duty = throttleToDuty(thr);

  if (dir == Direction::FWD) {
    driverEnable(true);
    ledcWrite(PWM_CH_R, duty);
    ledcWrite(PWM_CH_L, 0);
  } else { // REV
    driverEnable(true);
    ledcWrite(PWM_CH_R, 0);
    ledcWrite(PWM_CH_L, duty);
  }
}


static void stopMotorNow(const char* reason) {
  setApplied(Direction::STOP, 0, reason);
  setTarget(Direction::STOP, 0, "stopMotorNow target");

  rampActive = false;
  rampKind = RampKind::NONE;

  kickActive = false;

  reversePending = false;
  pendingStage = PendingStage::NONE;

  applyPwmOutputs(Direction::STOP, 0);
  driverEnable(false);  // TRUE STOP: coast/off

  if (debugMode) {
    Serial.print("[STOP] ");
    Serial.println(reason);
  }
}

static String getStateString() {
  if (appliedThrottle <= 0 || currentDirection == Direction::STOP) {
    return "STOPPED";
  }
  if (currentDirection == Direction::FWD) {
    return "FORWARD " + String(appliedThrottle);
  }
  if (currentDirection == Direction::REV) {
    return "REVERSE " + String(appliedThrottle);
  }
  return "STOPPED";
}

static String getHwStateString() {
  HwSnapshot hw = getHwSnapshot();

  if (hw.hwThrottlePct <= 0 || hw.hwDir == Direction::STOP) {
    return "STOPPED";
  }
  if (hw.hwDir == Direction::FWD) {
    return "FORWARD " + String(hw.hwThrottlePct);
  }
  if (hw.hwDir == Direction::REV) {
    return "REVERSE " + String(hw.hwThrottlePct);
  }
  return "STOPPED";
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

    // TX briefly turns LED OFF (dip) while connected/solid
    ledDipTx();

    pTxChar->notify();
    i += take;
  }
}

static void sendACK(const String& original)  { bleNotifyChunked("ACK:" + original); }
static void sendERR(const String& original)  { bleNotifyChunked("ERR:" + original); }

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
  // Do NOT clear reversePending/pendingStage here (used for stop-first reversing).
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
    setApplied(dirDuringRamp, endThr, "Ramp immediate");
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

  setApplied(dir, kickHoldThrottle, "KICK begin");
  applyPwmOutputs(currentDirection, appliedThrottle);

  reversePending = true;
  pendingStage = PendingStage::NONE;
  pendingFinalTargetThrottle = finalTargetThr;
  pendingFinalDirection = dir;
  pendingFinalIsInstant = afterKickIsInstant;
  pendingFinalIsMomentum = afterKickIsMomentum;

  setTarget(dir, finalTargetThr, "KICK target");
}

static void continueAfterKickIfNeeded() {
  if (!reversePending) return;

  Direction dir = pendingFinalDirection;
  int32_t thr = clampI32(pendingFinalTargetThrottle, 0, 100);
  bool isInstant = pendingFinalIsInstant;
  bool isMomentum = pendingFinalIsMomentum;

  reversePending = false;

  if (isInstant) {
    setApplied(dir, thr, "KICK -> instant");
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

  setApplied(dir, thr, "KICK -> apply");
  applyPwmOutputs(currentDirection, appliedThrottle);
}

// ------------------------- Motion command execution -------------------------
static void executeStopRamp(RampKind kind) {
  cancelAllMotionActivities(); // preserve reverse sequencing

  int32_t startThr = appliedThrottle;
  int32_t deltaAbs = abs(startThr);

  uint32_t full = FULL_QUICKSTOP_MS;
  if (kind == RampKind::BRAKE) full = FULL_BRAKE_MS;
  if (kind == RampKind::MOMENTUM) full = FULL_MOMENTUM_DECEL_MS;

  uint32_t dur = scaledDurationMs(full, deltaAbs);

  setTarget(Direction::STOP, 0, "Stop ramp target");

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
  // New motion command overrides any previously queued reverse
  cancelPendingReverse();
  cancelAllMotionActivities();

  int32_t thr = clampI32(requestedThr, 0, 100);

  setTarget(dir, thr, "Instant command target");

  if (thr == 0) {
    stopMotorNow("Instant throttle=0");
    return;
  }

  // Direction change while moving -> stop first (this schedules a NEW pending reverse)
  if (currentDirection != Direction::STOP && currentDirection != dir) {
    scheduleReverseAfterStop(dir, thr, /*instant*/true, /*momentum*/false);
    executeStopRamp(RampKind::QUICKSTOP);
    return;
  }

  bool startingFromStop = (appliedThrottle == 0 && currentDirection == Direction::STOP);
  int32_t effThr = effectiveStartTargetThrottle(thr, startingFromStop);

  setTarget(dir, effThr, "Instant effective target");

  if (shouldKickOnStart(startingFromStop, effThr)) {
    beginKick(dir, effThr, /*afterKickIsInstant*/true, /*afterKickIsMomentum*/false);
    return;
  }

  setApplied(dir, effThr, "Instant apply");
  applyPwmOutputs(currentDirection, appliedThrottle);
}

static void executeRampedMove(Direction dir,
                              int32_t requestedThr,
                              uint32_t fullScaleAccelMs,
                              uint32_t fullScaleDecelMs,
                              RampKind stopFirstRampKind,
                              const char* tagReason) {
  // New motion command overrides any previously queued reverse
  cancelPendingReverse();
  cancelAllMotionActivities();

  int32_t thr = clampI32(requestedThr, 0, 100);
  setTarget(dir, thr, String(tagReason) + " target");

  // If target is 0, ramp down using the chosen stop flavor
  if (thr == 0) {
    executeStopRamp(stopFirstRampKind);
    return;
  }

  // Direction change while moving -> stop first, then wait, then ramp up
  if (currentDirection != Direction::STOP && currentDirection != dir) {
    // Store what to do after stop + delay:
    scheduleReverseAfterStop(dir, thr, /*isInstant*/false, /*isMomentum*/true);

    // Store WHICH ramp constants to use after the delay
    pendingFullAccelMs = fullScaleAccelMs;
    pendingFullDecelMs = fullScaleDecelMs;

    // Ramp to zero first
    executeStopRamp(stopFirstRampKind);
    return;
  }

  bool startingFromStop = (appliedThrottle == 0 && currentDirection == Direction::STOP);
  int32_t effTarget = effectiveStartTargetThrottle(thr, startingFromStop);

  setTarget(dir, effTarget, String(tagReason) + " effective target");

  if (startingFromStop) {
    currentDirection = dir; // bookkeeping
  }

  if (shouldKickOnStart(startingFromStop, effTarget)) {
    // After kick: ramped (not instant). We'll treat the continuation as "momentum-like"
    // but we must also preserve quick vs momentum ramp constants.
    pendingFullAccelMs = fullScaleAccelMs;
    pendingFullDecelMs = fullScaleDecelMs;

    beginKick(dir, effTarget, /*afterKickIsInstant*/false, /*afterKickIsMomentum*/true);
    return;
  }

  int32_t startThr = appliedThrottle;
  int32_t deltaAbs = abs(effTarget - startThr);
  if (deltaAbs == 0) {
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  uint32_t full = (effTarget > startThr) ? fullScaleAccelMs : fullScaleDecelMs;
  uint32_t dur  = scaledDurationMs(full, deltaAbs);

  startRamp(dir, startThr, effTarget, dur, RampKind::MOMENTUM);
}

static void executeMomentum(Direction dir, int32_t requestedThr) {
  executeRampedMove(dir,
                    requestedThr,
                    FULL_MOMENTUM_ACCEL_MS,
                    FULL_MOMENTUM_DECEL_MS,
                    RampKind::MOMENTUM,   // stop-first uses momentum decel feel
                    "Momentum");
}

static void executeQuickRamp(Direction dir, int32_t requestedThr) {
  executeRampedMove(dir,
                    requestedThr,
                    FULL_QUICKRAMP_ACCEL_MS,
                    FULL_QUICKRAMP_DECEL_MS,
                    RampKind::QUICKSTOP,  // stop-first: quick down feels snappier for Q
                    "QuickRamp");
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

    if (debugMode) {
      Serial.print("[BLE] Disconnected, grace started (");
      Serial.print(BLE_GRACE_MS);
      Serial.println("ms)");
    }

    NimBLEDevice::startAdvertising();
  }
};

class RxCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pCharacteristic, NimBLEConnInfo& connInfo) override {
    (void)connInfo;

    std::string val = pCharacteristic->getValue();
    if (val.empty()) return;

    // RX briefly turns LED OFF (dip) while connected/solid
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

    // STATE (no ACK, raw response only)
    if (upper == "??") {
      bleNotifyChunked(getHwStateString());
      return;
    }
    if (upper == "?") {
      bleNotifyChunked(getStateString());
      return;
    }
  
    // VERSION
    if (upper == "V") {
      sendACK(String(FW_VERSION));
      return;
    }

    // GUID
    if (upper == "G") {
      String uuid = String(SERVICE_UUID);

      // Extract last 17 characters: "9bd6-1e8cde2c9000"
      String lastPart = uuid.substring(uuid.length() - 17);
      bleNotifyChunked(lastPart);
      return;
    }

    // DEBUG
    if (upper == "D1") {
      debugMode = true;
      Serial.begin(115200);

      // Reset logging baseline so first change prints
      lastLoggedAppliedThrottle = -9999;
      lastLoggedDirection = (Direction)255;

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

      // Force initial state print
      logThrottleChangeIfNeeded("Debug enabled (baseline)");

      sendACK(preserved);
      return;
    }
    if (upper == "D0") {
      sendACK(preserved);
      debugMode = false;
      return;
    }

    // PERIODIC PRINT CONTROL
    // P1 = periodic prints ONLY when mismatch (default behavior)
    // P0 = periodic prints every period (even if OK)
    if (upper == "P0") {
      printPeriodic = true;

      if (debugMode) {
        Serial.println("[CFG] Periodic prints: ONLY ON MISMATCH");
      }

      sendACK(preserved);
      return;
    }

    if (upper == "P1") {
      printPeriodic = false;

      if (debugMode) {
        Serial.println("[CFG] Periodic prints: ALWAYS (every period)");
      }

      sendACK(preserved);
      return;
    }

    // MINSTART
    if (upper.startsWith("M")) {
      String nStr = original.substring(String("M").length());
      nStr.trim();
      if (!isDigitStr(nStr)) { sendERR(preserved); return; }
      cfgMinStart = clampI32(nStr.toInt(), 0, 100);
      if (debugMode) { Serial.print("[CFG] M="); Serial.println(cfgMinStart); }
      sendACK(preserved);
      return;
    }

    // KICK<t>,<ms>
    if (upper.startsWith("K")) {
      String args = original.substring(1);
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

    // QUICK RAMP: FQ / RQ
    if (upper.startsWith("FQ") || upper.startsWith("RQ")) {
      String nStr = original.substring(2);
      nStr.trim();
      if (!isDigitStr(nStr)) { sendERR(preserved); return; }

      int32_t n = clampI32(nStr.toInt(), 0, 100);

      if (allowMotionNow) {
        if (forcedStopLatched && bleConnected) forcedStopLatched = false;
        //executeInstant(upper.startsWith("FQ") ? Direction::FWD : Direction::REV, n);
        executeQuickRamp(upper.startsWith("FQ") ? Direction::FWD : Direction::REV, n);
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
  pwmInitialized = true; 
  applyPwmOutputs(Direction::STOP, 0);

  if (debugMode) {
    Serial.print("[PWM] init ok, dutyR=");
    Serial.print(ledcRead(PWM_CH_R));
    Serial.print(" dutyL=");
    Serial.println(ledcRead(PWM_CH_L));
  }
}

static void setupDriverPins() {
  //Configure EN pins ----
  pinMode(PIN_REN, OUTPUT);
  pinMode(PIN_LEN, OUTPUT);
  driverEnable(false);   // Start DISABLED (true coast/off)
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
  adv->setName(FW_NAME);
  adv->start();

  debugPrintln("[BLE] Advertising");
}

void setup() {
  if (debugMode) {
    Serial.begin(115200);
    delay(50);
  }

  ledInit();      // LED starts in "disconnected blink" mode by ledService()
  setupDriverPins();
  setupPwm();
  stopMotorNow("Boot");

  if (debugMode) {
    lastLoggedAppliedThrottle = -9999;
    lastLoggedDirection = (Direction)255;

    Serial.println("[DEBUG] ON (startup)");
    Serial.print("[FW] ");
    Serial.print(FW_NAME);
    Serial.print(" v");
    Serial.println(FW_VERSION);
    Serial.println("[BOOT] ready");
  }

  setupBle();
}

// ------------------------- Main loop tasks -------------------------
static void processRamp() {
  if (!rampActive) return;

  uint32_t now = millis();
  uint32_t elapsed = now - rampStartMs;

  int32_t newThr = smoothstepEasedThrottle(
      rampStartThrottle,
      rampTargetThrottle,
      elapsed,
      rampDurationMs);

  if (rampTargetThrottle == 0 && newThr == 0) {
    setApplied(Direction::STOP, 0, "Ramp tick -> stop");
    applyPwmOutputs(Direction::STOP, 0);
  } else {
    setApplied(rampDirection, newThr, "Ramp tick");
    applyPwmOutputs(currentDirection, appliedThrottle);
  }

  if (elapsed >= rampDurationMs) {
    rampActive = false;

    if (rampTargetThrottle == 0) {
      setApplied(Direction::STOP, 0, "Ramp complete -> stop");
      applyPwmOutputs(Direction::STOP, 0);
    } else {
      setApplied(rampDirection, rampTargetThrottle, "Ramp complete");
      applyPwmOutputs(currentDirection, appliedThrottle);
    }

    if (reversePending && appliedThrottle == 0) {
      pendingStage = PendingStage::WAIT_DIR_DELAY;
      pendingStageUntilMs = millis() + DIR_CHANGE_DELAY_MS;
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

  setTarget(dir, effThr, "Reverse complete target");

  if (shouldKickOnStart(startingFromStop, effThr)) {
    beginKick(dir, effThr, pendingFinalIsInstant, pendingFinalIsMomentum);
    return;
  }

  reversePending = false;

  // ----- Instant after reverse -----
  if (pendingFinalIsInstant) {
    setApplied(dir, effThr, "Reverse complete -> instant");
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  // ----- Momentum after reverse -----
  if (pendingFinalIsMomentum) {
    // Use the ramp constants that were active when the command was issued (momentum vs quick ramp)
    uint32_t dur = scaledDurationMs(pendingFullAccelMs, abs(effThr));
    startRamp(dir, 0, effThr, dur, RampKind::MOMENTUM);
    return;
  }

  // ----- Fallback direct apply -----
  setApplied(dir, effThr, "Reverse complete -> apply");
  applyPwmOutputs(currentDirection, appliedThrottle);
}

static void processKick() {
  if (!kickActive) return;

  uint32_t now = millis();
  if ((int32_t)(now - kickEndMs) < 0) {
    setApplied(kickDirection, kickHoldThrottle, "KICK hold");
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  kickActive = false;
  continueAfterKickIfNeeded();
}

static void processBleGrace() {
  if (!graceActive) return;

  uint32_t now = millis();

  // --- Debug countdown logging (once per second, plus immediate on entry) ---
  static uint32_t lastCountdownLogMs = 0;
  static bool countdownPrimed = false;

  if (debugMode) {
    if (!countdownPrimed) {
      // first time we enter grace window
      countdownPrimed = true;
      lastCountdownLogMs = 0; // force immediate log
    }

    if (lastCountdownLogMs == 0 || (now - lastCountdownLogMs) >= GRACE_COUNTDOWN_LOG_PERIOD_MS) {
      lastCountdownLogMs = now;

      uint32_t elapsed = now - disconnectMs;
      uint32_t remaining = (elapsed >= BLE_GRACE_MS) ? 0 : (BLE_GRACE_MS - elapsed);

      Serial.print("[BLE] Grace countdown: ");
      Serial.print(remaining);
      Serial.println("ms remaining");
    }
  } else {
    // If debug is off, don't keep stale primed state
    countdownPrimed = false;
    lastCountdownLogMs = 0;
  }

  // --- Timeout behavior ---
  if ((now - disconnectMs) >= BLE_GRACE_MS) {
    graceActive = false;

    // reset countdown state for next disconnect
    countdownPrimed = false;
    lastCountdownLogMs = 0;

    forcedStopLatched = true;

    // Options:
    // A) original immediate stop:
    // stopMotorNow("BLE grace timeout -> forced stop latched");

    // B) "act like S was sent" behavior:
    executeStopRamp(RampKind::QUICKSTOP);

    if (debugMode) {
      Serial.println("[BLE] Grace expired: forced stop latched");
    }
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

  // ---- DEBUG SNAPSHOT HOOK (non-control, debug only) ----
  if (debugMode && pwmInitialized) {
    logThrottleChangeIfNeeded(g_lastDbgReason);
  }
}
