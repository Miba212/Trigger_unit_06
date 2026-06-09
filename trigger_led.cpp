#include "trigger_led.h"
#include <FastLED.h>

// LED hardware configuration
#define LED_PIN             48  // 48 on board led, 39 external led pin
//#define LED_PIN             39  // 48 on board led, 39 external led pin
#define NUM_LEDS            1
#define LED_BRIGHTNESS      230  // 47% brightness

// Pairing state colors (Hue value )
#define HUE_DARK_BLUE       160  // Unpaired / Pairing
#define HUE_LIGHT_BLUE      128  // Reconnecting / Reconnected
#define HUE_RED             0    // Timeout / Error

// System state colors (Hue values)
#define HUE_IDLE            45  // Yellow/Orange
#define HUE_READY           96   // Green
#define HUE_RUNNING         96   // Green (flashing)
#define HUE_STOPPED         9   // Red/Orange

// Saturation and value
#define SAT_FULL            255
#define VAL_FULL            255
#define VAL_OFF             0

// Timing definitions
#define FLASH_SLOW_MS       1000  // 1 Hz flash (1 sec period = 500ms on, 500ms off)
#define FLASH_FAST_MS       250   // Fast flash for errors
#define BLINK_SHORT_MS      100   // Short blink for multi-blink patterns
#define BLINK_GAP_MS        200   // Gap between blinks
#define BLINK_PAUSE_MS      1000  // Pause between multi-blink sequences

#define PAIRING_SUCCESS_HOLD_MS    30000  // 1 minute solid after pairing
#define RECONNECT_SUCCESS_HOLD_MS  30000  // 1 minute solid after reconnect

#define LED_RESET_DELAY_MS  50    // Delay after reset to ensure LED is off

// Stopwatch states (from comm_espnow_trigger.h)
#define STOPWATCH_IDLE 0
#define STOPWATCH_READY 1
#define STOPWATCH_RUNNING 2
#define STOPWATCH_STOPPED 3
#define STOPWATCH_ERROR 4

// Pairing states (from comm_espnow_trigger.cpp)
#define STATE_UNPAIRED 0
#define STATE_PAIRING_REQUESTED 1
#define STATE_PAIRED_CONFIGURING 2
#define STATE_PAIRED_OPERATIONAL 3
#define STATE_UNPAIRING 4

// LED state enum
enum LedState {
    LED_OFF,
    LED_PAIRING_UNPAIRED,
    LED_PAIRING_TIMEOUT,
    LED_PAIRING_REQUESTED,
    LED_RECONNECTING,
    LED_PAIRED_SUCCESS,
    LED_RECONNECTED_SUCCESS,
    LED_IDLE,
    LED_READY,
    LED_RUNNING,
    LED_STOPPED,
    LED_ERROR_HEARTBEAT,
    LED_ERROR_SENSOR,
    LED_ERROR_ESPNOW,
    LED_ERROR_GENERAL
};

// Global state variables
static CRGB leds[NUM_LEDS];
static LedState currentLedState = LED_OFF;
static ErrorType currentError = ERROR_NONE;
static bool ledEnabledSetting = true;
static uint8_t currentPairingState = STATE_UNPAIRED;
static bool pairingModeActive = false;
static uint8_t currentStopwatchState = STOPWATCH_IDLE;
static bool pairingTimeoutOccurred = false;

// Flash/blink timing
static unsigned long lastFlashToggle = 0;
static bool flashState = false;
static unsigned long pairingSuccessHoldStart = 0;
static unsigned long reconnectSuccessHoldStart = 0;

// Multi-blink pattern state
static unsigned long blinkPatternStart = 0;
static uint8_t blinkCount = 0;
static uint8_t blinkPhase = 0;  // 0=blink on, 1=gap, 2=pause

void triggerLedInit() {
    Serial.println("[LED] Initializing LED system...");
    
    // Initialize FastLED hardware
    FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    
    // ===== CRITICAL: RESET LED ON REBOOT =====
    // WS2812B retains state when board is reset (stays powered)
    // Explicitly turn OFF before starting normal operation
    leds[0] = CRGB::Black;  // Set to black (off)
    FastLED.show();
    
    Serial.println("[LED] LED reset to OFF (post-reboot clear)");
    
    // Brief delay to ensure LED register update completes
    delay(LED_RESET_DELAY_MS);
    // ===== END CRITICAL SECTION =====
    
    // Initialize state variables
    currentLedState = LED_OFF;
    currentError = ERROR_NONE;
    ledEnabledSetting = true;
    currentPairingState = STATE_UNPAIRED;
    pairingModeActive = false;
    currentStopwatchState = STOPWATCH_IDLE;
    pairingTimeoutOccurred = false;
    
    // Reset timers
    lastFlashToggle = 0;
    flashState = false;
    pairingSuccessHoldStart = 0;
    reconnectSuccessHoldStart = 0;
    blinkPatternStart = 0;
    blinkCount = 0;
    blinkPhase = 0;
    
    Serial.println("[LED] LED system ready - will follow state logic");
}

// Check if 1-minute hold period is active for pairing success
static bool isPairedSuccessHoldActive() {
    if (pairingSuccessHoldStart == 0) return false;
    unsigned long elapsed = millis() - pairingSuccessHoldStart;
    if (elapsed >= PAIRING_SUCCESS_HOLD_MS) {
        pairingSuccessHoldStart = 0;  // Clear hold
        return false;
    }
    return true;
}

// Check if 1-minute hold period is active for reconnect success
static bool isReconnectSuccessHoldActive() {
    if (reconnectSuccessHoldStart == 0) return false;
    unsigned long elapsed = millis() - reconnectSuccessHoldStart;
    if (elapsed >= RECONNECT_SUCCESS_HOLD_MS) {
        reconnectSuccessHoldStart = 0;  // Clear hold
        return false;
    }
    return true;
}

// Map error type to LED state
static LedState mapErrorToLedState(ErrorType error) {
    switch (error) {
        case ERROR_HEARTBEAT_LOST: return LED_ERROR_HEARTBEAT;
        case ERROR_SENSOR_FAILURE: return LED_ERROR_SENSOR;
        case ERROR_ESPNOW_FAILED: return LED_ERROR_ESPNOW;
        case ERROR_GENERAL: return LED_ERROR_GENERAL;
        default: return LED_OFF;
    }
}

// Map pairing state to LED state
static LedState mapPairingStateToLed(uint8_t pairingState, bool pairingActive) {
    if (pairingTimeoutOccurred) {
        return LED_PAIRING_TIMEOUT;
    }
    
    switch (pairingState) {
        case STATE_UNPAIRED:
            return pairingActive ? LED_PAIRING_UNPAIRED : LED_OFF;
        case STATE_PAIRING_REQUESTED:
            return LED_PAIRING_REQUESTED;
        case STATE_PAIRED_CONFIGURING:
            return LED_RECONNECTING;
        case STATE_PAIRED_OPERATIONAL:
            return LED_OFF;  // Will be handled by system states or hold periods
        case STATE_UNPAIRING:
            return LED_OFF;
        default:
            return LED_OFF;
    }
}

// Map stopwatch state to LED state
static LedState mapStopwatchStateToLed(uint8_t stopwatchState) {
    switch (stopwatchState) {
        case STOPWATCH_IDLE: return LED_IDLE;
        case STOPWATCH_READY: return LED_READY;
        case STOPWATCH_RUNNING: return LED_RUNNING;
        case STOPWATCH_STOPPED: return LED_STOPPED;
        case STOPWATCH_ERROR: return LED_ERROR_GENERAL;
        default: return LED_OFF;
    }
}

// Determine which LED state should be shown based on priority
static LedState determineLedState() {
    // PRIORITY 1: Errors (highest)
    if (currentError != ERROR_NONE) {
        return mapErrorToLedState(currentError);
    }
    
    // PRIORITY 2: 1-minute hold periods after pairing/reconnect
    if (isPairedSuccessHoldActive()) {
        return LED_PAIRED_SUCCESS;  // Dark blue solid
    }
    if (isReconnectSuccessHoldActive()) {
        return LED_RECONNECTED_SUCCESS;  // Light blue solid
    }
    
    // PRIORITY 3: Active pairing states
    if (currentPairingState != STATE_PAIRED_OPERATIONAL) {
        return mapPairingStateToLed(currentPairingState, pairingModeActive);
    }
    
    // PRIORITY 4: System states (only if ledEnabled)
    if (ledEnabledSetting) {
        return mapStopwatchStateToLed(currentStopwatchState);
    } else {
        return LED_OFF;  // LED disabled for system states
    }
}

// Update flash state for slow flash (1 Hz)
static bool updateFlashState(unsigned long interval) {
    unsigned long now = millis();
    if (now - lastFlashToggle >= interval / 2) {  // Divide by 2 for on/off cycle
        lastFlashToggle = now;
        flashState = !flashState;
    }
    return flashState;
}

// Update multi-blink pattern (double blink, triple blink)
static bool updateBlinkPattern(uint8_t numBlinks) {
    unsigned long now = millis();
    unsigned long elapsed = now - blinkPatternStart;
    
    // Pattern timing: BLINK_SHORT_MS on, BLINK_GAP_MS off, repeat numBlinks times, then BLINK_PAUSE_MS
    unsigned long singleBlinkCycle = BLINK_SHORT_MS + BLINK_GAP_MS;
    unsigned long totalBlinkTime = numBlinks * singleBlinkCycle;
    unsigned long totalCycle = totalBlinkTime + BLINK_PAUSE_MS;
    
    // Reset pattern if cycle complete
    if (elapsed >= totalCycle) {
        blinkPatternStart = now;
        elapsed = 0;
    }
    
    // Check if we're in the pause phase
    if (elapsed >= totalBlinkTime) {
        return false;  // OFF during pause
    }
    
    // Determine which blink we're in
    uint8_t currentBlink = elapsed / singleBlinkCycle;
    unsigned long blinkPhaseTime = elapsed % singleBlinkCycle;
    
    // ON during blink, OFF during gap
    return (blinkPhaseTime < BLINK_SHORT_MS);
}

// Set LED color based on state
static void setLedColor(LedState state) {
    bool shouldBeOn = true;
    uint8_t hue = 0;
    uint8_t sat = SAT_FULL;
    uint8_t val = VAL_FULL;
    
    switch (state) {
        case LED_OFF:
            leds[0] = CRGB::Black;
            FastLED.show();
            return;
            
        // Pairing states
        case LED_PAIRING_UNPAIRED:
        case LED_PAIRING_REQUESTED:
            hue = HUE_DARK_BLUE;
            shouldBeOn = updateFlashState(FLASH_SLOW_MS);
            break;
            
        case LED_PAIRING_TIMEOUT:
            hue = HUE_RED;
            shouldBeOn = updateFlashState(FLASH_SLOW_MS);
            break;
            
        case LED_RECONNECTING:
            hue = HUE_LIGHT_BLUE;
            shouldBeOn = updateFlashState(FLASH_SLOW_MS);
            break;
            
        case LED_PAIRED_SUCCESS:
            hue = HUE_DARK_BLUE;
            shouldBeOn = true;  // Solid
            break;
            
        case LED_RECONNECTED_SUCCESS:
            hue = HUE_LIGHT_BLUE;
            shouldBeOn = true;  // Solid
            break;
            
        // System states
        case LED_IDLE:
            hue = HUE_IDLE;
            shouldBeOn = true;  // Solid
            break;
            
        case LED_READY:
            hue = HUE_READY;
            shouldBeOn = true;  // Solid
            break;
            
        case LED_RUNNING:
            hue = HUE_RUNNING;
            shouldBeOn = updateFlashState(FLASH_SLOW_MS);
            break;
            
        case LED_STOPPED:
            hue = HUE_STOPPED;
            shouldBeOn = true;  // Solid
            break;
            
        // Error states
        case LED_ERROR_HEARTBEAT:
            hue = HUE_RED;
            shouldBeOn = updateFlashState(FLASH_FAST_MS);
            break;
            
        case LED_ERROR_SENSOR:
            hue = HUE_RED;
            shouldBeOn = updateBlinkPattern(2);  // Double blink
            break;
            
        case LED_ERROR_ESPNOW:
            hue = HUE_RED;
            shouldBeOn = updateBlinkPattern(3);  // Triple blink
            break;
            
        case LED_ERROR_GENERAL:
            hue = HUE_RED;
            shouldBeOn = true;  // Solid
            break;
    }
    
    if (shouldBeOn) {
        leds[0] = CHSV(hue, sat, val);
    } else {
        leds[0] = CRGB::Black;
    }
    FastLED.show();
}

void triggerLedUpdate() {
    // Determine current LED state based on priority
    LedState desiredState = determineLedState();
    
    // Update LED if state changed
    if (desiredState != currentLedState) {
        currentLedState = desiredState;
        // Reset flash/blink timing on state change
        lastFlashToggle = millis();
        flashState = false;
        blinkPatternStart = millis();
    }
    
    // Set LED color based on current state
    setLedColor(currentLedState);
}

void triggerLedSetPairingState(uint8_t state, bool pairingActive) {
    currentPairingState = state;
    pairingModeActive = pairingActive;
}

void triggerLedSetStopwatchState(uint8_t stopwatchState) {
    currentStopwatchState = stopwatchState;
}

void triggerLedSetEnabled(bool enabled) {
    ledEnabledSetting = enabled;
}

void triggerLedNotifyPairingSuccess() {
    pairingSuccessHoldStart = millis();
    Serial.println("[LED] Pairing success - holding dark blue for 1 minute");
}

void triggerLedNotifyReconnectSuccess() {
    reconnectSuccessHoldStart = millis();
    Serial.println("[LED] Reconnect success - holding light blue for 1 minute");
}

void triggerLedSetError(ErrorType error) {
    if (error != ERROR_NONE) {
        currentError = error;
        Serial.printf("[LED] Error set: %d\n", error);
    }
}

void triggerLedClearError() {
    if (currentError != ERROR_NONE) {
        Serial.println("[LED] Error cleared");
        currentError = ERROR_NONE;
    }
}
