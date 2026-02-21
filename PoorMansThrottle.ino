/*
  (C) James Theimer 2026 Poor Man's Throttle
  ESP32 BLE Heavy-Train Throttle Controller

  LED behavior (GPIO2):
  - Default / disconnected: LED blinks continuously (double-blink search pattern)
  - Connected: LED stays solid ON
  - RX/TX while connected (solid ON): LED briefly turns OFF (a quick dip), then returns solid ON

  Target: ESP32-WROOM-32 (Arduino framework)
  Motor driver: IBT-2 / BTS7960 (RPWM/LPWM)
  BLE: Custom service with RX (Write / WriteNR) + TX (Notify)

    ------------------------- BLE COMMAND REFERENCE -------------------------

  Notes:
  - Commands are case-insensitive.
  - Whitespace + CR/LF are trimmed.
  - Throttle values are clamped to 0..100.
  - Stop-first reversing is enforced (ramps to 0, waits direction-delay, then reverses).
  - After BLE disconnect, a grace timer runs. If not reconnected,
    a forced stop is latched and a stop ramp is executed.

  Response behavior:
  - Most commands respond with:
      ACK:<original command>   (valid)
      ERR:<original command>   (invalid)
  - The following commands return RAW text (no ACK/ERR wrapper):
      ?, ??, G

  --------------------------------------------------------------------------

  MOTION (Momentum / Nonlinear Ramp - smoothstep easing)
  - F<n>        : Forward to throttle n (0..100) using momentum ramp
  - R<n>        : Reverse to throttle n (0..100) using momentum ramp

  --------------------------------------------------------------------------

  MOTION (Quick Ramp - smoothstep, faster profile)
  - FQ<n>       : Forward to throttle n (0..100) using quick ramp
                  If reversing, performs stop ramp → direction delay → quick ramp up
  - RQ<n>       : Reverse to throttle n (0..100) using quick ramp

  --------------------------------------------------------------------------

  STOPS
  - S           : Quick stop ramp to 0
  - B           : Brake ramp to 0 (slower deceleration profile)

  --------------------------------------------------------------------------

  START ASSIST CONFIG

  - M<n>        : MINSTART – Set minimum starting throttle (0..100)
                  Applied ONLY when starting from stop.
                  Does NOT prevent deceleration below it.

  - K<t>,<ms>
  - K<t>,<ms>,<rampDownMs>,<maxApply>

                  t          = kick throttle (0..100)
                  ms         = kick duration
                  rampDownMs = kick ramp-down duration (optional)
                  maxApply   = only kick if target ≤ maxApply (optional)

                  When starting from stop and target > 0:
                  - Applies max(t, MINSTART) for <ms>
                  - Then transitions into ramp or quick-ramp behavior

  --------------------------------------------------------------------------

  DEBUG / DIAGNOSTICS

  - D1          : Debug ON (Serial @115200)
                  Prints FW name/version, BLE MTU, events, HW comparisons

  - D0          : Debug OFF

  - P0          : Periodic debug prints ONLY when hardware mismatch (default)

  - P1          : Periodic debug prints every period (even if matching)

  --------------------------------------------------------------------------

  STATE QUERIES (RAW notify, no ACK wrapper)

  - ?           : Hardware state query
                  Returns one of:
                      HW-STOPPED
                      HW-FORWARD <percent>
                      HW-REVERSE <percent>

  - ??          : Stored/applied state query
                  Returns one of:
                      STOPPED
                      FORWARD <appliedThrottle>
                      REVERSE <appliedThrottle>

  - G           : Returns last portion of SERVICE_UUID

  - V           : Version query
                  Responds as:
                      ACK:<FW_VERSION>
*/

#include <Arduino.h>
#include <NimBLEDevice.h>
#include <type_traits>
#include <stdarg.h>

// ------------------------- Startup defaults -------------------------
static const bool DEBUG_AT_STARTUP = false;

// ------------------------- Firmware ID -------------------------
static const char* FW_NAME    = "GScaleThrottle";
static const char* FW_VERSION = "1.1.0";

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
static const int PIN_REN = 27;  // IBT-2 R_EN
static const int PIN_LPWM = 26;   // Reverse PWM -> LPWM
static const int PIN_RPWM = 25;   // Forward PWM -> RPWM
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

static bool pendingSkipDirDelay = false;
static bool pendingSuppressKickOnce = false;

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
static int32_t cfgKickRampDownMs = 80;     // default for 2-param K
static int32_t cfgKickMaxApply   = 15;     // default for 2-param K

static bool kickActive = false;
static uint32_t kickEndMs = 0;
static int32_t kickHoldThrottle = 0;
static Direction kickDirection = Direction::STOP;

// Post-kick continuation state (separate from reverse sequencing)
static bool postKickPending = false;
static Direction postKickDir = Direction::STOP;
static int32_t postKickFinalThr = 0;
static bool postKickIsInstant = false;
static bool postKickIsMomentum = false;
static uint32_t postKickFullAccelMs = FULL_MOMENTUM_ACCEL_MS;
static uint32_t postKickFullDecelMs = FULL_MOMENTUM_DECEL_MS;

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
  pendingSuppressKickOnce = false;
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

// ------------------------- Timestamped Debug Printing (ONE println per line) -------------------------
static void dbgPrintf(const char* fmt, ...) {
  if (!debugMode) return;

  // NOTE: big lines (like [THR]) can be long; bump if you ever see truncation.
  char line[768];
  int n = 0;

  uint32_t ms = millis();
  uint32_t totalSeconds = ms / 1000UL;
  uint32_t msec = ms % 1000UL;
  uint32_t sec  = totalSeconds % 60UL;
  uint32_t min  = (totalSeconds / 60UL) % 60UL;
  uint32_t hour = (totalSeconds / 3600UL) % 24UL;

  n = snprintf(line, sizeof(line),
               "%02lu:%02lu:%02lu.%03lu -> ",
               (unsigned long)hour,
               (unsigned long)min,
               (unsigned long)sec,
               (unsigned long)msec);

  if (n < 0 || (size_t)n >= sizeof(line)) return;

  va_list args;
  va_start(args, fmt);
  vsnprintf(line + n, sizeof(line) - (size_t)n, fmt, args);
  va_end(args);

  // Exactly ONE println to reduce interleaving across tasks
  Serial.println(line);
}

static void debugPrintln(const String& s) {
  if (!debugMode) return;
  dbgPrintf("%s", s.c_str());
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

  // Build deltas (for mismatch visibility)
  Direction storedDir = (appliedThrottle <= 0 || currentDirection == Direction::STOP)
                          ? Direction::STOP
                          : currentDirection;
  int32_t storedPct = clampI32(appliedThrottle, 0, 100);
  int32_t dDir = (int32_t)((int)hw.hwDir - (int)storedDir);
  int32_t dPct = (int32_t)(hw.hwThrottlePct - storedPct);

  dbgPrintf(
    "[THR] %s%s | STORED dir=%s applied=%ld targetDir=%s target=%ld ramp=%d kick=%d pending=%d ble=%d latch=%d | "
    "HW EN=%d (REN=%d LEN=%d) dutyR=%lu dutyL=%lu hwDir=%s hw%%=%ld | %s dDir=%ld d%%=%ld",
    reason,
    (!storedChanged && periodicDue) ? " (periodic)" : "",
    dirStr(currentDirection),
    (long)appliedThrottle,
    dirStr(targetDirection),
    (long)targetThrottle,
    rampActive ? 1 : 0,
    kickActive ? 1 : 0,
    reversePending ? 1 : 0,
    bleConnected ? 1 : 0,
    forcedStopLatched ? 1 : 0,
    hw.enabled ? 1 : 0,
    hw.ren ? 1 : 0,
    hw.len ? 1 : 0,
    (unsigned long)hw.dutyR,
    (unsigned long)hw.dutyL,
    dirStr(hw.hwDir),
    (long)hw.hwThrottlePct,
    match ? "OK" : "MISMATCH",
    (long)dDir,
    (long)dPct
  );

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
    dbgPrintf("[TGT] %s | targetDir=%s target=%ld", reason.c_str(), dirStr(targetDirection), (long)targetThrottle);
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

  // Stop any ramp
  rampActive = false;
  rampKind = RampKind::NONE;
  rampStartMs = 0;
  rampDurationMs = 0;
  rampStartThrottle = 0;
  rampTargetThrottle = 0;
  rampDirection = Direction::STOP;

  // Stop any kick + clear kick bookkeeping
  kickActive = false;
  kickEndMs = 0;
  kickHoldThrottle = 0;
  kickDirection = Direction::STOP;

  // Clear post-kick continuation
  postKickPending = false;
  postKickDir = Direction::STOP;
  postKickFinalThr = 0;
  postKickIsInstant = false;
  postKickIsMomentum = false;
  postKickFullAccelMs = FULL_MOMENTUM_ACCEL_MS;
  postKickFullDecelMs = FULL_MOMENTUM_DECEL_MS;

  // Clear any reverse sequencing
  reversePending = false;
  pendingStage = PendingStage::NONE;
  pendingStageUntilMs = 0;
  pendingFinalTargetThrottle = 0;
  pendingFinalDirection = Direction::STOP;
  pendingFinalIsInstant = false;
  pendingFinalIsMomentum = false;
  pendingSkipDirDelay = false;
  pendingSuppressKickOnce = false;

  applyPwmOutputs(Direction::STOP, 0);
  driverEnable(false);  // TRUE STOP: coast/off

  if (debugMode) {
    dbgPrintf("[STOP] %s", reason);
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
    return "HW-STOPPED";
  }
  if (hw.hwDir == Direction::FWD) {
    return "HW-FORWARD " + String(hw.hwThrottlePct);
  }
  if (hw.hwDir == Direction::REV) {
    return "HW-REVERSE " + String(hw.hwThrottlePct);
  }
  return "HW-STOPPED";
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
  // cancelAllMotionActivities() definition:
  //
  // Cancels any *active* time-based motion generators (ramp/kick) AND cancels any
  // kick-related scheduled continuation (postKickPending).
  //
  // IMPORTANT ORDERING RULE:
  // - If you need to start a ramp but also want a post-kick continuation afterward,
  //   you MUST first transfer the continuation into reversePending/pendingFinal...,
  //   because startRamp() calls cancelAllMotionActivities() and will clear postKickPending.

  rampActive = false;
  kickActive = false;

  // If we cancel motion, also cancel any stored post-kick continuation
  postKickPending = false;

  // Do NOT clear reversePending/pendingStage here (used for stop-first reversing).
}

static void startRamp(Direction dirDuringRamp, int32_t startThr, int32_t endThr, uint32_t durationMs, RampKind kind) {
  // NOTE: startRamp() calls cancelAllMotionActivities().
  // That will clear postKickPending (kick continuation staging).
  // Always stage any post-kick continuation into pendingFinal... BEFORE calling startRamp().
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

  // only kick for low commanded values
  if (finalTargetThr > cfgKickMaxApply) return false;

  return true;
}

static void beginKick(Direction dir,
                      int32_t finalTargetThr,
                      bool afterKickIsInstant,
                      bool afterKickIsMomentum,
                      uint32_t accelMs,
                      uint32_t decelMs)
{
  int32_t kickThr = max(cfgKickThrottle, cfgMinStart);
  kickThr = clampI32(kickThr, 0, 100);

  kickActive = true;
  kickDirection = dir;
  kickHoldThrottle = kickThr;
  kickEndMs = millis() + (uint32_t)cfgKickMs;

  setApplied(dir, kickHoldThrottle, "KICK begin");
  applyPwmOutputs(currentDirection, appliedThrottle);

  // Store what to do after the kick finishes (DON'T reuse reversePending)
  postKickPending = true;
  postKickDir = dir;
  postKickFinalThr = finalTargetThr;
  postKickIsInstant = afterKickIsInstant;
  postKickIsMomentum = afterKickIsMomentum;

  // Preserve which ramp constants were active when the command was issued
  postKickFullAccelMs = accelMs;
  postKickFullDecelMs = decelMs;

  setTarget(dir, finalTargetThr, "KICK target");
}

static void continueAfterKickIfNeeded() {
  if (!postKickPending) return;

  Direction dir = postKickDir;
  int32_t finalThr = clampI32(postKickFinalThr, 0, 100);

  // Done with "kick phase"
  postKickPending = false;

  // Recovery target:
  // - Instant command: recover toward finalThr quickly
  // - Ramped command: recover to 0 quickly, then let ramp engine handle inertia up to finalThr
  const bool isInstant = postKickIsInstant;
  int32_t recoverTo = isInstant ? finalThr : 0;

  uint32_t rd = (uint32_t)cfgKickRampDownMs;

  // ---- Stage continuation FIRST (for ramped commands only) ----
  // IMPORTANT: do NOT set WAIT_DIR_DELAY yet. Let processRamp() arm it when we actually hit 0.
  if (!isInstant) {
    pendingFinalDirection = dir;
    pendingFinalTargetThrottle = finalThr;
    pendingFinalIsInstant = false;

    // If Kick is agnostic. Preserve whatever motion type the original command requested.
    // pendingFinalIsMomentum = postKickIsMomentum;
    // If Kick is just a temporary override. The continuation behavior is deterministic.
    pendingFinalIsMomentum = true;

    pendingFullAccelMs = postKickFullAccelMs;
    pendingFullDecelMs = postKickFullDecelMs;

    pendingSkipDirDelay = true;     // no direction-delay for post-kick continuation
    reversePending = true;
    pendingStage = PendingStage::NONE;
    pendingSuppressKickOnce = true;
  }

  // ---- Now perform the fast recovery ----
  if (rd == 0) {
    // Immediate recovery
    setApplied(dir, recoverTo, "KICK recover immediate");
    applyPwmOutputs(currentDirection, appliedThrottle);

    // If ramped command: we must manually arm the continuation because no ramp completion will occur
    if (!isInstant && recoverTo == 0 && reversePending) {
      pendingStage = PendingStage::WAIT_DIR_DELAY;
      pendingStageUntilMs = millis();  // no delay
      pendingSkipDirDelay = false;     // consume it here
    }

    return;
  }

  // Recovery ramp (nonzero duration). When it completes at 0, processRamp() will arm WAIT_DIR_DELAY.
  startRamp(dir, appliedThrottle, recoverTo, rd, RampKind::QUICKSTOP);

  // If it was an instant command, we're done after the recovery ramp completes.
  // (No extra scheduling needed.)
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
    beginKick(dir, effThr,
      /*afterKickIsInstant*/true,
      /*afterKickIsMomentum*/false,
      /*accelMs*/0,
      /*decelMs*/0);
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

    beginKick(dir, effTarget,
          /*afterKickIsInstant*/false,
          /*afterKickIsMomentum*/true,
          fullScaleAccelMs,
          fullScaleDecelMs);
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
      dbgPrintf("[BLE] MTU=%u", (unsigned)g_peerMtu);
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
      dbgPrintf("[BLE] Disconnected, grace started (%lums)", (unsigned long)BLE_GRACE_MS);
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
      dbgPrintf("[CMD] %s", preserved.c_str());
    }

    bool allowMotionNow = !(forcedStopLatched && !bleConnected);

    // STATE (no ACK, raw response only)
    if (upper == "?") {
      bleNotifyChunked(getHwStateString());
      return;
    }
    if (upper == "??") {
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

      dbgPrintf("[DEBUG] ON");
      dbgPrintf("[FW] %s v%s", FW_NAME, FW_VERSION);
      dbgPrintf("[BOOT] ready");

      if (bleConnected) {
        dbgPrintf("[BLE] MTU=%u", (unsigned)g_peerMtu);
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
    // P0 = periodic prints ONLY when mismatch (default behavior)
    // P1 = periodic prints every period (even if OK)
    if (upper == "P0") {
      printPeriodic = true;

      if (debugMode) {
        dbgPrintf("[CFG] Periodic prints: ONLY ON MISMATCH");
      }

      sendACK(preserved);
      return;
    }

    if (upper == "P1") {
      printPeriodic = false;

      if (debugMode) {
        dbgPrintf("[CFG] Periodic prints: ALWAYS (every period)");
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
      if (debugMode) {
        dbgPrintf("[CFG] M=%ld", (long)cfgMinStart);
      }
      sendACK(preserved);
      return;
    }

    // KICK<t>,<ms>[,<rampDownMs>,<maxApply>]
    if (upper.startsWith("K")) {
      String args = original.substring(1);
      args.trim();

      // Tokenize by comma (max 4 tokens)
      String tok[4];
      int tokCount = 0;

      while (tokCount < 4) {
        int comma = args.indexOf(',');
        if (comma < 0) { tok[tokCount++] = args; break; }
        tok[tokCount++] = args.substring(0, comma);
        args = args.substring(comma + 1);
        args.trim();
      }

      if (tokCount != 2 && tokCount != 4) { sendERR(preserved); return; }

      for (int i = 0; i < tokCount; i++) tok[i].trim();

      if (!isDigitStr(tok[0]) || !isDigitStr(tok[1])) { sendERR(preserved); return; }

      cfgKickThrottle = clampI32(tok[0].toInt(), 0, 100);
      cfgKickMs       = clampI32(tok[1].toInt(), 0, 2000);

      if (tokCount == 4) {
        if (!isDigitStr(tok[2]) || !isDigitStr(tok[3])) { sendERR(preserved); return; }
        cfgKickRampDownMs = clampI32(tok[2].toInt(), 0, 2000);
        cfgKickMaxApply   = clampI32(tok[3].toInt(), 0, 100);
      } else {
        cfgKickRampDownMs = 80;
        cfgKickMaxApply   = 15;
      }

      if (debugMode) {
        dbgPrintf("[CFG] KICK=%ld,%ld rd=%ld max=%ld",
                  (long)cfgKickThrottle,
                  (long)cfgKickMs,
                  (long)cfgKickRampDownMs,
                  (long)cfgKickMaxApply);
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
        // executeInstant(upper.startsWith("FQ") ? Direction::FWD : Direction::REV, n);
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
    dbgPrintf("[PWM] init ok, dutyR=%lu dutyL=%lu",
              (unsigned long)ledcRead(PWM_CH_R),
              (unsigned long)ledcRead(PWM_CH_L));
  }
}

static void setupDriverPins() {
  // Configure EN pins ----
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

    dbgPrintf("[DEBUG] ON (startup)");
    dbgPrintf("[FW] %s v%s", FW_NAME, FW_VERSION);
    dbgPrintf("[BOOT] ready");
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

    if (reversePending && (rampTargetThrottle == 0)) {
      pendingStage = PendingStage::WAIT_DIR_DELAY;
      pendingStageUntilMs = millis() + (pendingSkipDirDelay ? 0 : DIR_CHANGE_DELAY_MS);
      pendingSkipDirDelay = false;
    }
  }
}

static void processPendingReverse() {
  if (!reversePending) return;
  if (pendingStage != PendingStage::WAIT_DIR_DELAY) return;

  uint32_t now = millis();
  if ((int32_t)(now - pendingStageUntilMs) < 0) return;

  pendingStage = PendingStage::NONE;

  // Consume one-shot suppression immediately so it can't stick.
  const bool suppressKick = pendingSuppressKickOnce;
  pendingSuppressKickOnce = false;

  Direction dir = pendingFinalDirection;
  int32_t thr = clampI32(pendingFinalTargetThrottle, 0, 100);

  bool startingFromStop = (appliedThrottle == 0 && currentDirection == Direction::STOP);
  int32_t effThr = effectiveStartTargetThrottle(thr, startingFromStop);

  setTarget(dir, effThr, "Reverse complete target");

  if (!suppressKick && shouldKickOnStart(startingFromStop, effThr)) {
    beginKick(dir, effThr,
      pendingFinalIsInstant,
      pendingFinalIsMomentum,
      pendingFullAccelMs,
      pendingFullDecelMs);
    return;
  }

  reversePending = false;

  if (pendingFinalIsInstant) {
    setApplied(dir, effThr, "Reverse complete -> instant");
    applyPwmOutputs(currentDirection, appliedThrottle);
    return;
  }

  if (pendingFinalIsMomentum) {
    uint32_t dur = scaledDurationMs(pendingFullAccelMs, abs(effThr));
    startRamp(dir, 0, effThr, dur, RampKind::MOMENTUM);
    return;
  }

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

      dbgPrintf("[BLE] Grace countdown: %lums remaining", (unsigned long)remaining);
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
      dbgPrintf("[BLE] Grace expired: forced stop latched");
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
