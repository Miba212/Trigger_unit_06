#include "trigger_display.h"
#include "comm_espnow_trigger.h"
#include "trigger_timesync.h"
#include <Arduino.h>
#include <MD_MAX72xx.h>
#include <SPI.h>

// ===== HARDWARE CONFIGURATION =====
#define MAX_DEVICES   4
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

#define DIN_PIN 4  // MOSI
#define CLK_PIN 5  // SCK
#define CS_PIN  6  // CS



extern uint8_t currentStopwatchState;

// ===== DISPLAY OBJECT =====
static MD_MAX72XX mx(HARDWARE_TYPE, DIN_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

 //pinMode(4, INPUT_PULLUP);
 //pinMode(5, INPUT_PULLUP);
 //pinMode(6, INPUT_PULLUP);
 
// ===== RUNTIME CONFIG =====
static uint32_t g_readyFlashPeriodMs = READY_FLASH_PERIOD_MS;
static uint8_t  g_brightness         = DISPLAY_INTENSITY;

static uint32_t lastDisplayedCountdown = UINT32_MAX;

// ===== STATE VARIABLES =====
static bool    displayEnabled      = false;
static uint8_t displayMode         = DISPLAY_MODE_DEFAULT;
static bool resultAlreadyRendered = false;

// Timing state (SIMPLE - just millis() based)
static bool     isTimerRunning    = false;
static uint32_t timerStartMillis  = 0;
static uint32_t frozenTimeMs      = 0;

// Result state
static bool  hasReceivedResult     = false;
static float receivedResultSeconds = 0.0f;

// For READY flashing we keep a "display last result"
static bool  hasDisplayLastResult     = false;
static float displayLastResultSeconds = 0.0f;
static volatile bool displayBusy = false;

static void renderTime(uint32_t elapsedMs);
static void renderCountdown(uint32_t totalSeconds);
static void renderError();
static void renderIdlePattern();
static void renderClock();
static void renderSeconds(float seconds);
static void cancelPostCountdownIfAny();
static void restartDisplayVisuals();

// ===== IDLE CLOCK TIMING CONSTANTS =====
// Adjust these to change alternating behaviour (mode 2)
static const uint32_t IDLE_PATTERN_SHOW_MS  = 4000;   // how long - - - is shown per cycle
static const uint32_t IDLE_CLOCK_SHOW_MS    = 9000;   // how long HH:MM is shown per cycle
static const uint32_t IDLE_PATTERN_FIRST_MS = 30000;  // pattern hold before switching to clock (mode 3)

// ===== IDLE CLOCK STATE VARIABLES =====
static bool     idleClockVisible       = false;  // true = clock showing, false = pattern showing
static uint32_t idlePhaseStartMs       = 0;      // when current phase started
static bool     idleColonOn            = true;   // colon blink state
static uint32_t idleColonLastToggleMs  = 0;      // last colon toggle time
static bool     idleMode3SwitchedOver  = false;  // mode 3: has pattern→clock switch happened

// Countdown state
static bool     isCountdownActive    = false;
static uint16_t countdownDuration    = 0;
static uint32_t countdownStartMillis = 0;
static bool     countdownWasCancelled = false;

static bool postCountdownActive = false;
static bool postCountdownFlash = false;
static unsigned long postCountdownStartMs = 0;
static const unsigned long POST_COUNTDOWN_DURATION_MS = 10000;

// Flash state (READY)
static uint32_t lastFlashToggle = 0;
static bool     flashOn         = true;

// Display update timing
static uint32_t lastTimingUpdate    = 0;
static uint32_t lastCountdownUpdate = 0;

static int32_t g_timezoneOffsetSeconds = 0;

static bool hwInitialized = false;
static bool displayConfigReceived = false;
static bool stateReceived = false;
static bool bootPatternActive = false;
static bool waitingPatternActive = false;
static uint32_t bootPatternStartMs = 0;
static const uint32_t BOOT_PATTERN_DURATION_MS = 700;

// ===== FONTS =====

// 4×7 digit font for timing display
static const uint8_t DIGIT_4x7[10][4] = {
  { 0b1111111, 0b1000001, 0b1000001, 0b1111111 }, // 0
  { 0b0000000, 0b1000001, 0b1111111, 0b0000001 }, // 1
  { 0b1001111, 0b1001001, 0b1001001, 0b1111001 }, // 2
  { 0b1000001, 0b1001001, 0b1001001, 0b1111111 }, // 3
  { 0b1111000, 0b0001000, 0b0001000, 0b1111111 }, // 4
  { 0b1111001, 0b1001001, 0b1001001, 0b1001111 }, // 5
  { 0b1111111, 0b1001001, 0b1001001, 0b1001111 }, // 6
  { 0b1100000, 0b1000011, 0b1001100, 0b1110000 }, // 7
  { 0b1111111, 0b1001001, 0b1001001, 0b1111111 }, // 8
  { 0b1111001, 0b1001001, 0b1001001, 0b1111111 }, // 9
};

static const uint8_t DIGIT_5x8[10][5] = {
  { 0b01111110, 0b10000001, 0b10000001, 0b10000001, 0b01111110 }, // 0
  { 0b00000000, 0b01000001, 0b11111111, 0b00000001, 0b00000000 }, // 1
  { 0b01100001, 0b10000011, 0b10000101, 0b10001001, 0b01110001 }, // 2
  { 0b01000010, 0b10000001, 0b10010001, 0b10010001, 0b01101110 }, // 3
  { 0b11111000, 0b00001000, 0b00001000, 0b00001000, 0b11111111 }, // 4
  { 0b11110010, 0b10010001, 0b10010001, 0b10010001, 0b10001110 }, // 5
  { 0b01111110, 0b10010001, 0b10010001, 0b10010001, 0b01001110 }, // 6
  { 0b00000000, 0b10000111, 0b10001000, 0b10010000, 0b11100000 }, // 7
  { 0b01101110, 0b10010001, 0b10010001, 0b10010001, 0b01101110 }, // 8
  { 0b01110010, 0b10001001, 0b10001001, 0b10001001, 0b01111110 }, // 9
};

// ===== BIG DIGIT 8x8 FONT FOR COUNTDOWN =====
static const uint8_t DIGIT_8x8[10][8] = {
  {0b00111000,0b01000100,0b10000010,0b10000010,0b10000010,0b10000010,0b01000100,0b00111000}, // 0
  {0b00001000,0b00011000,0b00101000,0b00001000,0b00001000,0b00001000,0b00001000,0b00111100}, // 1
  {0b00111000,0b01000100,0b00000100,0b00001000,0b00010000,0b00100000,0b01000000,0b01111100}, // 2
  {0b00111000,0b01000100,0b00000100,0b00011000,0b00000100,0b00000100,0b01000100,0b00111000}, // 3
  {0b00001000,0b00011000,0b00101000,0b01001000,0b11111100,0b00001000,0b00001000,0b00001000}, // 4
  {0b01111100,0b01000000,0b01000000,0b01111000,0b00000100,0b00000100,0b01000100,0b00111000}, // 5
  {0b00111000,0b01000100,0b01000000,0b01111000,0b01000100,0b01000100,0b01000100,0b00111000}, // 6
  {0b01111100,0b00000100,0b00001000,0b00010000,0b00100000,0b00100000,0b00100000,0b00100000}, // 7
  {0b00111000,0b01000100,0b01000100,0b00111000,0b01000100,0b01000100,0b01000100,0b00111000}, // 8
  {0b00111000,0b01000100,0b01000100,0b01000100,0b00111100,0b00000100,0b01000100,0b00111000}  // 9
};

// "Err" pattern for ERROR state (3 characters)
static const uint8_t ERR_PATTERN[11] = {
  // E (5 cols)
  0b11111111, 0b11011011, 0b11011011, 0b11011011, 0b11000011,
  // Space (1 col)
  0b00000000,
  // r (2 cols)
  0b11111111, 0b11100000,
  // Space (1 col)
  0b00000000,
  // r (2 cols)
  0b11111111, 0b11100000
};

// ===== LOW-LEVEL HELPERS =====



static inline bool colInRange(int col) {
  return (col >= 0 && col < 8 * MAX_DEVICES); // 0..31
}

static void drawColumn(int physicalCol, uint8_t pattern) {
  if (!colInRange(physicalCol)) return;
  uint8_t device = physicalCol / 8;
  uint8_t col    = physicalCol % 8;
  mx.setColumn(device, col, pattern);
}

static void drawDigit4x7(int x, uint8_t digit) {
  if (digit > 9) return;
  for (int col = 0; col < 4; col++) {
    drawColumn(x + col, DIGIT_4x7[digit][col]);
  }
}

static void drawDigit5x8(int x, uint8_t digit) {
  if (digit > 9) return;
  for (int col = 0; col < 5; col++) {
    drawColumn(x + col, DIGIT_5x8[digit][col]);
  }
}

static void drawColon(int x) {
  drawColumn(x, 0b0010100);
}

static void drawDot(int x) {
  drawColumn(x, 0b0000001);
}

static void safeUpdate() {
    if (displayBusy) return;
    displayBusy = true;
    mx.update();
    displayBusy = false;
}

// ===== RENDER FUNCTIONS =====

// Convert ms to minutes/seconds/hundredths and draw
static void renderTime(uint32_t elapsedMs) {
  uint32_t totalHundredths = (elapsedMs + 5) / 10;

  uint32_t minutes    = (totalHundredths / 100) / 60;
  uint32_t seconds    = (totalHundredths / 100) % 60;
  uint32_t hundredths = totalHundredths % 100;
  uint32_t tenths     = hundredths / 10;

  mx.clear();
// mx.update(); 
  int x = 4;
  const int digitWidth = 4;
  const int space      = 1;

  if (minutes < 10) {
    // m:ss.xx
    drawDigit4x7(x, (uint8_t)minutes);
    x += digitWidth + space;

    drawColon(x);
    x += 1 + space;

    drawDigit4x7(x, (uint8_t)(seconds / 10));
    x += digitWidth + space;
    drawDigit4x7(x, (uint8_t)(seconds % 10));
    x += digitWidth + space;

    drawDot(x);
    x += 1 + space;

    drawDigit4x7(x, (uint8_t)(hundredths / 10));
    x += digitWidth + space;
    drawDigit4x7(x, (uint8_t)(hundredths % 10));
  } else {
    // mm:ss.x
    uint8_t mm = (minutes > 99) ? 99 : (uint8_t)minutes;

    drawDigit4x7(x, mm / 10);
    x += digitWidth + space;
    drawDigit4x7(x, mm % 10);
    x += digitWidth + space;

    drawColon(x);
    x += 1 + space;

    drawDigit4x7(x, (uint8_t)(seconds / 10));
    x += digitWidth + space;
    drawDigit4x7(x, (uint8_t)(seconds % 10));
    x += digitWidth + space;

    drawDot(x);
    x += 1 + space;

    drawDigit4x7(x, (uint8_t)tenths);
  }

  mx.update();
}

static void renderCountdown(uint32_t totalSeconds) {
  uint32_t minutes = totalSeconds / 60;
  uint32_t seconds = totalSeconds % 60;

  mx.clear();
// mx.update(); 
  int x;
  const int digitWidth = 5;
  const int space      = 1;

  if (minutes < 10) {
    // m:ss centered
    x = 6;
    drawDigit5x8(x, (uint8_t)minutes);
    x += digitWidth + space;

    drawColon(x);
    x += 1 + space;

    drawDigit5x8(x, (uint8_t)(seconds / 10));
    x += digitWidth + space;
    drawDigit5x8(x, (uint8_t)(seconds % 10));
  } else {
    // mm:ss
    x = 2;
    drawDigit5x8(x, (uint8_t)(minutes / 10));
    x += digitWidth + space;
    drawDigit5x8(x, (uint8_t)(minutes % 10));
    x += digitWidth + space;

    drawColon(x);
    x += 1 + space;

    drawDigit5x8(x, (uint8_t)(seconds / 10));
    x += digitWidth + space;
    drawDigit5x8(x, (uint8_t)(seconds % 10));
  }

  mx.update();
}


static void renderError() {
  mx.clear();
  int x = 10;
  for (int col = 0; col < 11; col++) {
    drawColumn(x + col, ERR_PATTERN[col]);
  }
  mx.update();
}

// ===== IDLE PATTERN: static - - - =====
static void renderIdlePattern() {
  mx.clear();
  // Three dashes centered on the 32-column (4 x 8) matrix
  // Each dash is 4 pixels wide, 1 pixel tall (row 3, 0-indexed from bottom)
  // Positions: col 4-7, col 13-16, col 22-25  (gaps of 4 between dashes)
  // Using row bit 0b00001000 = row 3 (middle-ish of 7-row display)
  const uint8_t dashRow = 0b00001000;
  const int dashPositions[] = {4, 5, 6, 7,  13, 14, 15, 16,  22, 23, 24, 25};
  for (int i = 0; i < 12; i++) {
    drawColumn(dashPositions[i], dashRow);
  }
  mx.update();
}

// ===== CLOCK RENDER: HH:MM with blinking colon =====
static void renderClock(bool colonVisible) {
  // Get current unix time in ms from timesync
  uint64_t nowMs = triggerTimeSyncGetUnixMs();
 uint32_t totalSeconds = (uint32_t)(nowMs / 1000ULL) + (uint32_t)g_timezoneOffsetSeconds;
 
  // Extract HH and MM from unix timestamp (UTC)
  uint32_t daySeconds = totalSeconds % 86400UL;
  uint8_t  hours      = (uint8_t)(daySeconds / 3600);
  uint8_t  minutes    = (uint8_t)((daySeconds % 3600) / 60);

  mx.clear();
// mx.update(); 
  // Layout: HH:MM using DIGIT_5x8 (5 cols each) + colon (1 col) + spaces
  // Total: 5+1+5+1+5+1+5 = 23 cols, start at col 4 to center on 32
  const int digitWidth = 5;
  const int space      = 1;
  int x = 3;

  drawDigit5x8(x, hours / 10);   x += digitWidth + space;
  drawDigit5x8(x, hours % 10);   x += digitWidth + space;

  // Colon — blink
  if (colonVisible) {
    drawColon(x);
  }
  x += 1 + space;

  drawDigit5x8(x, minutes / 10); x += digitWidth + space;
  drawDigit5x8(x, minutes % 10);

  mx.update();
}

// ===== RESULT HANDLING =====

// ===== MODE-SPECIFIC STATE CHANGE =====

static void handleMode0StateChange(uint8_t oldState, uint8_t newState) {
  Serial.printf("[DISPLAY][MODE0] Handler called: oldState=%d, newState=%d\n",
                oldState, newState);

  switch (newState) {
   case STOPWATCH_IDLE:
    cancelPostCountdownIfAny();
    resultAlreadyRendered = false;
    Serial.println("[DISPLAY][MODE0] IDLE: Show idle pattern and reset clock state");
    isTimerRunning = false;
    frozenTimeMs   = 0;
    hasReceivedResult = false;

    // Clear "display last result" so READY will start from 0:00.00
    hasDisplayLastResult     = false;
    displayLastResultSeconds = 0.0f;

    // Reset IDLE clock state
    idleClockVisible      = false;
    idlePhaseStartMs      = millis();
    idleColonOn           = true;
    idleColonLastToggleMs = millis();
    idleMode3SwitchedOver = false;

    mx.clear();
    renderIdlePattern();
    break;

    case STOPWATCH_READY:
     cancelPostCountdownIfAny();
      resultAlreadyRendered = false;
        Serial.println("[DISPLAY][MODE0] READY: Start flashing");
        lastFlashToggle = millis();
        flashOn = true;
        isTimerRunning = false;
        frozenTimeMs = 0;

        // Immediately render first frame of READY pattern
        mx.clear();
        if (hasDisplayLastResult) {
            renderSeconds(displayLastResultSeconds);
        } else {
            renderSeconds(0.0f);
        }
        break;

    case STOPWATCH_RUNNING: {
      cancelPostCountdownIfAny();
      Serial.printf("[DISPLAY][MODE0] RUNNING: oldState=%d\n", oldState);
      if (oldState == STOPWATCH_READY || oldState == STOPWATCH_IDLE || oldState == STOPWATCH_STOPPED) {
        isTimerRunning    = true;
        timerStartMillis  = millis();
        hasReceivedResult = false;
        frozenTimeMs      = 0;
        Serial.printf("[DISPLAY][MODE0] ✅ Timer STARTED at %lu ms\n", (unsigned long)timerStartMillis);
      }
      break;
    }

    case STOPWATCH_STOPPED: {
      uint32_t now = millis();
      if (isTimerRunning) {
        frozenTimeMs = now - timerStartMillis;
      }
      isTimerRunning = false;
      resultAlreadyRendered = false;
      Serial.printf("[DISPLAY][MODE0] STOPPED: Freeze timer ⇒ frozenTimeMs=%lu ms\n",
                    (unsigned long)frozenTimeMs);
      break;
    }

    case STOPWATCH_ERROR:
     cancelPostCountdownIfAny();
      Serial.println("[DISPLAY][MODE0] ERROR: Show error pattern");
      isTimerRunning    = false;
      hasReceivedResult = false;
      frozenTimeMs      = 0;
      renderError();
      break;

    default:
      break;
  }
}

// ===== PUBLIC API IMPLEMENTATION =====
static void initDisplayHardware() {
  if (hwInitialized) return;

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  pinMode(DIN_PIN, INPUT_PULLUP);
  pinMode(CLK_PIN, INPUT_PULLUP);

  delay(20);

  SPI.begin(CLK_PIN, -1, DIN_PIN, CS_PIN);
  SPI.setFrequency(1000000);
  delay(30);

  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, g_brightness & 0x0F);
  mx.clear();
  delay(30);

  hwInitialized = true;
  Serial.println("[DISPLAY] Hardware initialized");
}

static void renderBootPattern() {
  if (!hwInitialized) return;

  mx.clear();
  for (int module = 0; module < MAX_DEVICES; module++) {
    int baseCol = module * 8;
    drawColumn(baseCol + 2, 0b00011000);
    drawColumn(baseCol + 3, 0b00111100);
    drawColumn(baseCol + 4, 0b00111100);
    drawColumn(baseCol + 5, 0b00011000);
  }
  mx.update();
}

static void renderWaitingPattern() {
  if (!hwInitialized) return;

  mx.clear();
  for (int module = 0; module < MAX_DEVICES; module++) {
    int baseCol = module * 8;
    drawColumn(baseCol + 3, 0b00011000);
    drawColumn(baseCol + 4, 0b00011000);
  }
  mx.update();
}

void triggerDisplayInit() {
  displayEnabled = false;
  displayMode = DISPLAY_MODE_DEFAULT;
  isTimerRunning = false;
  hasReceivedResult = false;
  frozenTimeMs = 0;

  hwInitialized = false;
  displayConfigReceived = false;
  stateReceived = false;
  bootPatternActive = false;
  waitingPatternActive = false;
  bootPatternStartMs = 0;

  Serial.println("[DISPLAY] Initialized (logical only)");
}

void triggerDisplayPrepareFromSavedConfig(bool enabled, uint8_t mode) {
  displayEnabled = enabled;
  displayMode = mode;

  if (!displayEnabled) {
    Serial.println("[DISPLAY] Saved config says display disabled - keeping OFF");
    return;
  }

  initDisplayHardware();
  renderBootPattern();
  bootPatternActive = true;
  waitingPatternActive = false;
  bootPatternStartMs = millis();

  Serial.println("[DISPLAY] Boot pattern started from saved config");
}

void triggerDisplayOnConfigReceived(bool enabled, uint8_t mode, int32_t offsetSeconds) {
  bool wasEnabled = displayEnabled;

  displayConfigReceived = true;
  displayEnabled = enabled;
  displayMode = mode;
  g_timezoneOffsetSeconds = offsetSeconds;

  Serial.printf("[DISPLAY] Config received: enabled=%d mode=%d tz=%ld\n",
                enabled ? 1 : 0, mode, (long)offsetSeconds);

  if (!displayEnabled) {
    if (hwInitialized) {
      mx.clear();
      mx.update();
    }
    bootPatternActive = false;
    waitingPatternActive = false;
    return;
  }

  if (!hwInitialized) {
    initDisplayHardware();
  } else if (!wasEnabled) {
    // Re-enable path: treat as fresh visual startup
    mx.clear();
    mx.update();
    delay(30);
    mx.control(MD_MAX72XX::INTENSITY, g_brightness & 0x0F);
    delay(20);
  }

  renderBootPattern();
  bootPatternActive = true;
  waitingPatternActive = false;
  bootPatternStartMs = millis();
}

void triggerDisplayMarkStateReceived() {
  stateReceived = true;
  Serial.println("[DISPLAY] State received");
}

void triggerDisplaySetEnabled(bool enabled)
{
    displayEnabled = enabled;
    Serial.printf("[DISPLAY] Enabled set to %d\n", enabled ? 1 : 0);

    if (!displayEnabled && hwInitialized) {
        mx.clear();
        mx.update();
    }
}

bool triggerDisplayIsEnabled() {
  return displayEnabled;
}

static void renderSeconds(float seconds) {
  uint32_t ms = (uint32_t)(seconds * 1000.0f + 0.5f);
  renderTime(ms);
}

void triggerDisplaySetMode(uint8_t mode) {
  displayMode = mode;
  Serial.printf("[DISPLAY] Mode set to %d\n", displayMode);
}

uint8_t triggerDisplayGetMode() {
  return displayMode;
}

void triggerDisplaySetResult(float resultSeconds) {
  hasReceivedResult      = true;
  receivedResultSeconds  = resultSeconds;
  hasDisplayLastResult   = true;
  displayLastResultSeconds = resultSeconds;
  resultAlreadyRendered = false;
  Serial.printf("[DISPLAY] Result stored: %.3f s\n", resultSeconds);
}

void triggerDisplayStartCountdown(uint16_t durationSeconds) {
  cancelPostCountdownIfAny();

  lastDisplayedCountdown = UINT32_MAX;

  if (hwInitialized) {
    restartDisplayVisuals();
  }

  isCountdownActive = true;
  countdownWasCancelled = false;
  countdownDuration = durationSeconds;
  countdownStartMillis = millis();

  Serial.printf("[DISPLAY] Countdown START: %u s\n", durationSeconds);
}

void triggerDisplayFinishCountdown() {
    isCountdownActive = false;
    countdownWasCancelled = false;
    lastDisplayedCountdown = UINT32_MAX;

    if (hwInitialized) {
        restartDisplayVisuals();
    }

    renderCountdown(0);

    postCountdownActive = true;
    postCountdownFlash = true;
    postCountdownStartMs = millis();
    lastFlashToggle = millis();
    flashOn = true;

    Serial.println("[DISPLAY] Countdown FINISH (will flash 0:00)");
}

void triggerDisplayCancelCountdown() {
    isCountdownActive = false;
    countdownWasCancelled = true;
    lastDisplayedCountdown = UINT32_MAX;
    postCountdownActive = false;
    postCountdownFlash = false;
    postCountdownStartMs = 0;

    if (hwInitialized) {
        restartDisplayVisuals();
    }

    // After cancel, immediately render the current stopwatch state
    triggerDisplayUpdateState(currentStopwatchState);

    Serial.println("[DISPLAY] Countdown CANCELLED");
}

void triggerDisplayShowTriggered() {
  Serial.println("[DISPLAY] TRIGGERED blink start");
}

void triggerDisplaySetTimezoneOffset(int32_t offsetSeconds) {
    g_timezoneOffsetSeconds = offsetSeconds;
    Serial.printf("[DISPLAY] Timezone offset set to %ld seconds\n", (long)offsetSeconds);
}

void triggerDisplayUpdateState(uint8_t newState) {
  if (!displayEnabled) {
    cancelPostCountdownIfAny();
    Serial.println("[DISPLAY] ▶️ State update ignored (display disabled)");
    return;
  }

  if (bootPatternActive || waitingPatternActive) {
    Serial.println("[DISPLAY] ▶️ State update deferred (boot/wait active)");
    return;
  }

  static uint8_t lastState = STOPWATCH_IDLE;

  Serial.printf("[DISPLAY] ▶️ State update called: current=%d, new=%d\n",
                lastState, newState);

  if (displayMode == DISPLAY_MODE_DEFAULT ||
      displayMode == DISPLAY_MODE_CLOCK_ALT ||
      displayMode == DISPLAY_MODE_CLOCK_THEN) {
    handleMode0StateChange(lastState, newState);
  }

  lastState = newState;
  Serial.printf("[DISPLAY] ▶️ State changed: %d -> %d (Mode: %d)\n",
                currentStopwatchState, newState, displayMode);
}

void triggerDisplayUpdate() {
  if (!displayEnabled) return;
  if (!hwInitialized) return;

  unsigned long now = millis();

  if (bootPatternActive) {
    if (displayConfigReceived && stateReceived) {
      bootPatternActive = false;
      waitingPatternActive = false;
      Serial.println("[DISPLAY] Boot -> NORMAL");
      triggerDisplayUpdateState(currentStopwatchState);
      return;
    }

    if (now - bootPatternStartMs >= BOOT_PATTERN_DURATION_MS) {
      bootPatternActive = false;
      if (!displayConfigReceived || !stateReceived) {
        waitingPatternActive = true;
        renderWaitingPattern();
        Serial.println("[DISPLAY] Boot -> WAITING");
      }
      return;
    }

    return;
  }

  if (waitingPatternActive) {
    if (displayConfigReceived && stateReceived) {
      waitingPatternActive = false;
      Serial.println("[DISPLAY] Waiting -> NORMAL");
      triggerDisplayUpdateState(currentStopwatchState);
      return;
    }
    return;
  }

  // 1) READY flashing (continuous)
  if (currentStopwatchState == STOPWATCH_READY) {
    if (now - lastFlashToggle >= g_readyFlashPeriodMs) {
      lastFlashToggle = now;
      flashOn = !flashOn;

      mx.clear();
      if (flashOn) {
        if (hasDisplayLastResult) {
          renderSeconds(displayLastResultSeconds);
        } else {
          renderSeconds(0.0f);
        }
      } else {
        mx.clear();
      }
      mx.update();
    }
    return;
  }

  // 2) RUNNING timer update
  if (currentStopwatchState == STOPWATCH_RUNNING && isTimerRunning) {
    const uint32_t UPDATE_INTERVAL = TIMING_UPDATE_MS;
    if (now - lastTimingUpdate >= UPDATE_INTERVAL) {
      lastTimingUpdate = now;
      uint32_t elapsedMs = now - timerStartMillis;
      renderTime(elapsedMs);
      //mx.update();
    }
    return;
  }

  // 3) STOPPED: show result or frozen time
  if (currentStopwatchState == STOPWATCH_STOPPED) {
    if (hasReceivedResult) {
      if (!resultAlreadyRendered) {
        mx.control(MD_MAX72XX::INTENSITY, g_brightness & 0x0F);
        renderSeconds(receivedResultSeconds);
        //mx.update();
        resultAlreadyRendered = true;
      }
    } else {
      if (!resultAlreadyRendered) {
        mx.control(MD_MAX72XX::INTENSITY, g_brightness & 0x0F);
        renderTime(frozenTimeMs);
        //mx.update();
        resultAlreadyRendered = true;
      }
    }
    return;
  }

  // 4) COUNTDOWN
 // static uint32_t lastDisplayedCountdown = UINT32_MAX;

  if (isCountdownActive) {
    uint32_t elapsed   = (now - countdownStartMillis) / 1000;
    uint32_t remaining = (elapsed >= countdownDuration) ? 0 : (countdownDuration - elapsed);

    if (remaining != lastDisplayedCountdown) {
      renderCountdown(remaining);
      lastDisplayedCountdown = remaining;
    }

    if (remaining == 0) {
      // Wait for COUNTDOWN_FINISH packet from main unit
      return;
    }
    return;
}

  // 5) POST-COUNTDOWN FLASHING
  if (postCountdownActive) {
    uint32_t elapsedPost = now - postCountdownStartMs;

    if (elapsedPost >= POST_COUNTDOWN_DURATION_MS) {
      Serial.println("[DISPLAY] Post-countdown window expired -> IDLE");
      postCountdownActive = false;
      postCountdownFlash  = false;
      renderIdlePattern();
      //mx.update();
    } else {
      if (postCountdownFlash) {
        if (now - lastFlashToggle >= g_readyFlashPeriodMs) {
          lastFlashToggle = now;
          flashOn = !flashOn;
          if (flashOn) {
            renderCountdown(0);
          } else {
            mx.clear();
            mx.update();
          }
        }
      }
    }
    return;
  }

  // 6) IDLE — pattern / clock logic
  if (currentStopwatchState == STOPWATCH_IDLE) {

    // Mode 0 or 1: static pattern only — nothing to update continuously
    if (displayMode == DISPLAY_MODE_DEFAULT || displayMode == DISPLAY_MODE_POWER_SAVE) {
      return;
    }

    // Modes 2 and 3: clock involved — but only if real time is available
    // If no real time: behave as mode 0 (pattern already shown on IDLE entry, nothing to do)
    if (!triggerTimeSyncIsRealTime()) {
      return;
    }

    // --- Mode 2: alternating pattern <-> clock ---
    if (displayMode == DISPLAY_MODE_CLOCK_ALT) {
      uint32_t phaseDuration = idleClockVisible ? IDLE_CLOCK_SHOW_MS : IDLE_PATTERN_SHOW_MS;

      if (now - idlePhaseStartMs >= phaseDuration) {
        // Switch phase
        idleClockVisible = !idleClockVisible;
        idlePhaseStartMs = now;

        if (idleClockVisible) {
          idleColonOn           = true;
          idleColonLastToggleMs = now;
          renderClock(idleColonOn);
        } else {
          renderIdlePattern();
        }
      } else if (idleClockVisible) {
        // Update colon blink every 1s while clock is showing
        if (now - idleColonLastToggleMs >= 1000) {
          idleColonOn = !idleColonOn;
          idleColonLastToggleMs = now;
          renderClock(idleColonOn);
        }
      }
      return;
    }

    // --- Mode 3: pattern for IDLE_PATTERN_FIRST_MS, then clock permanently ---
    if (displayMode == DISPLAY_MODE_CLOCK_THEN) {
      if (!idleMode3SwitchedOver) {
        // Still in pattern phase
        if (now - idlePhaseStartMs >= IDLE_PATTERN_FIRST_MS) {
          // Switch to clock permanently
          idleMode3SwitchedOver = true;
          idleClockVisible      = true;
          idleColonOn           = true;
          idleColonLastToggleMs = now;
          renderClock(idleColonOn);
        }
        // else: still showing pattern — nothing to update
      } else {
        // Clock phase — just blink the colon every 1s
        if (now - idleColonLastToggleMs >= 1000) {
          idleColonOn = !idleColonOn;
          idleColonLastToggleMs = now;
          renderClock(idleColonOn);
        }
      }
      return;
    }
  }
}

void triggerDisplaySetReadyFlashPeriod(uint32_t periodMs) {
  if (periodMs < 100) periodMs = 100;
  g_readyFlashPeriodMs = periodMs;
  Serial.printf("[DISPLAY] READY flash period set to %lu ms\n",
                (unsigned long)g_readyFlashPeriodMs);
}

uint32_t triggerDisplayGetReadyFlashPeriod() {
  return g_readyFlashPeriodMs;
}

static void cancelPostCountdownIfAny() {
    postCountdownActive = false;
    postCountdownFlash  = false;
}

static void restartDisplayVisuals() {
    if (!hwInitialized) return;

    mx.clear();
    mx.update();
    delay(30);
    mx.control(MD_MAX72XX::INTENSITY, g_brightness & 0x0F);
    delay(20);
}

void triggerDisplaySetBrightness(uint8_t level) {
  if (level > 15) level = 15;
  g_brightness = level;
  mx.control(MD_MAX72XX::INTENSITY, g_brightness & 0x0F);
  Serial.printf("[DISPLAY] Brightness set to %u\n", g_brightness);
}

uint8_t triggerDisplayGetBrightness() {
  return g_brightness;
}
