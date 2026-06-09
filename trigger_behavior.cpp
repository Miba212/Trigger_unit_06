#include "trigger_behavior.h"
#include "trigger_sensor.h"
#include "trigger_display.h"
//#include "trigger_led.h"
#include <Arduino.h>


// --- Role/State sensor logic variables ---
static uint8_t currentRole = ROLE_START;
static TriggerBehaviorMode currentMode = BEHAVIOR_INACTIVE;
static bool detectionActive = false;
static unsigned long lastActionTime = 0;

// For BOTH role: timing for 2nd trigger event (STOP)
static uint32_t both_lastStartTriggerTimestamp = 0;
static const uint32_t BOTH_ROLE_COOLDOWN_MS = 3000; // 3 second cooldown between triggers

// Forward declarations for internal functions
void handleStartTriggerBehavior(uint8_t oldState, uint8_t newState);
void handleStopTriggerBehavior(uint8_t oldState, uint8_t newState);
void handleBothTriggerBehavior(uint8_t oldState, uint8_t newState);
void handleDisplayOnlyBehavior(uint8_t oldState, uint8_t newState);
void handleSplitBehavior(uint8_t oldState, uint8_t newState);
void startDetection();
void stopDetection();
void showTriggeredState();
void showResult();

void triggerBehaviorInit() {
    currentRole = ROLE_START;
    currentMode = BEHAVIOR_INACTIVE;
    detectionActive = false;
    both_lastStartTriggerTimestamp = 0;
    Serial.println("[BEHAVIOR] Trigger behavior system initialized");
}

void triggerBehaviorUpdate() {
    // No timeout-based sensor deactivation here!
    // All sensor activation/deactivation is now role/state driven
}

void triggerBehaviorHandleStateChange(uint8_t oldState, uint8_t newState, const EspNowSettingsPacket* config) {
    if (!config) return;

    currentRole = config->role;
    lastActionTime = millis();

    Serial.printf("[BEHAVIOR] State change: %d->%d (role=%d)\n", oldState, newState, currentRole);

    switch(currentRole) {
        case ROLE_START:
            handleStartTriggerBehavior(oldState, newState);
            break;
        case ROLE_STOP:
            handleStopTriggerBehavior(oldState, newState);
            break;
        case ROLE_BOTH:
            handleBothTriggerBehavior(oldState, newState);
            break;
        case ROLE_DISPLAY_ONLY:
            handleDisplayOnlyBehavior(oldState, newState);
            break;
        case ROLE_SPLIT:
            handleSplitBehavior(oldState, newState);
            break;
        default:
            Serial.printf("[BEHAVIOR] Unknown role: %d\n", currentRole);
            break;
    }
}

// --- START Role ---
void handleStartTriggerBehavior(uint8_t oldState, uint8_t newState) {
    if (newState == STOPWATCH_READY) {
        Serial.println("[BEHAVIOR] 🎯 START TRIGGER: Ready to detect start");
        startDetection();
    }
    else if (newState == STOPWATCH_RUNNING) { // <-- PATCH: always deactivate on RUNNING!
        Serial.println("[BEHAVIOR] ⚡ START TRIGGER: Race started!");
        showTriggeredState();
        stopDetection();
    }
    else if (newState == STOPWATCH_STOPPED || newState == STOPWATCH_ERROR || newState == STOPWATCH_IDLE) {
        Serial.println("[BEHAVIOR] 📊 START TRIGGER: Reset/stop detection");
        stopDetection();
    }
}

// --- STOP Role ---
void handleStopTriggerBehavior(uint8_t oldState, uint8_t newState) {
    if (newState == STOPWATCH_RUNNING) {
        Serial.println("[BEHAVIOR] 🎯 STOP TRIGGER: Ready to detect finish");
        startDetection();
    }
    else if (newState == STOPWATCH_STOPPED) { // <-- PATCH: always deactivate on STOPPED!
        Serial.println("[BEHAVIOR] ⚡ STOP TRIGGER: Race finished!");
        showTriggeredState();
        stopDetection();
    }
    else if (newState == STOPWATCH_READY || newState == STOPWATCH_IDLE || newState == STOPWATCH_ERROR) {
        Serial.println("[BEHAVIOR] 🔄 STOP TRIGGER: Reset/stop detection");
        stopDetection();
    }
}

// --- BOTH Role ---
void handleBothTriggerBehavior(uint8_t oldState, uint8_t newState) {
    if (newState == STOPWATCH_READY) {
        Serial.println("[BEHAVIOR] 🎯 BOTH TRIGGER: Ready to detect start");
        startDetection();
        both_lastStartTriggerTimestamp = 0; // Reset on READY
    }
    else if (newState == STOPWATCH_RUNNING) { // Keep sensor active in RUNNING
        Serial.println("[BEHAVIOR] ⚡ BOTH TRIGGER: Race started, now ready for finish");
        showTriggeredState();
        
        // Record when we entered RUNNING state to enforce cooldown
        both_lastStartTriggerTimestamp = millis();
        
        // UPDATED: Keep sensor active - DO NOT stop detection
        // Instead of stopping and reactivating, we keep detection active
        if (!detectionActive) {
            Serial.println("[BEHAVIOR] 🎯 BOTH TRIGGER: Ensuring sensor stays active for finish");
            startDetection();
        }
    }
    else if (newState == STOPWATCH_STOPPED) {
        Serial.println("[BEHAVIOR] ⚡ BOTH TRIGGER: Race finished!");
        showResult();
        stopDetection();
        both_lastStartTriggerTimestamp = 0;
    }
    else if (newState == STOPWATCH_IDLE || newState == STOPWATCH_ERROR) {
        Serial.println("[BEHAVIOR] 🔄 BOTH TRIGGER: Reset to idle or error");
        stopDetection();
        both_lastStartTriggerTimestamp = 0;
    }
}

// --- DISPLAY_ONLY Role ---
void handleDisplayOnlyBehavior(uint8_t oldState, uint8_t newState) {
    Serial.printf("[BEHAVIOR] 📊 DISPLAY ONLY: State changed %d -> %d\n", oldState, newState);
//    triggerDisplayUpdateState(newState);
    if (detectionActive) {
        stopDetection();
    }
}

// --- SPLIT Role ---
void handleSplitBehavior(uint8_t oldState, uint8_t newState) {
    if (newState == STOPWATCH_RUNNING) {
        Serial.println("[BEHAVIOR] ⌛ SPLIT TRIGGER: Ready for split time");
        startDetection();
    } else if (newState == STOPWATCH_IDLE || newState == STOPWATCH_READY || 
               newState == STOPWATCH_STOPPED || newState == STOPWATCH_ERROR) {
        Serial.println("[BEHAVIOR] ⌛ SPLIT TRIGGER: Deactivating for non-running state");
        if (detectionActive) {
            stopDetection();
        }
    }
}

// --- Sensor Activation ---
void startDetection() {
    detectionActive = true;
    currentMode = BEHAVIOR_DETECTING;
    lastActionTime = millis();
    Serial.println("[BEHAVIOR] 🎯 Detection ACTIVE");
    triggerSensorActivate();
}

void stopDetection() {
    detectionActive = false;
    currentMode = BEHAVIOR_INACTIVE;
    Serial.println("[BEHAVIOR] ⏹️ Detection STOPPED");
    triggerSensorDeactivate();
}

void showTriggeredState() {
    currentMode = BEHAVIOR_TRIGGERED;
    Serial.println("[BEHAVIOR] ⚡ TRIGGERED!");
    triggerDisplayShowTriggered();
}

void showResult() {
    currentMode = BEHAVIOR_WAITING;
    Serial.println("[BEHAVIOR] 📊 RESULT");
    // Visual feedback handled by LED and display modules
}

void triggerBehaviorSetRole(uint8_t role) {
    currentRole = role;
    Serial.printf("[BEHAVIOR] Role set to: %d\n", role);
}

// --- Event trigger guard logic ---
bool triggerBehaviorShouldTrigger(uint8_t currentState, uint8_t role) {
    // First check if detection is active at all
    if (!detectionActive) return false;

    switch(role) {
        case ROLE_START:
            return (currentState == STOPWATCH_READY);
            
        case ROLE_STOP:
            return (currentState == STOPWATCH_RUNNING);
            
        case ROLE_BOTH:
            if (currentState == STOPWATCH_READY) {
                return true; // Allow START trigger when in READY state
            }
            if (currentState == STOPWATCH_RUNNING && both_lastStartTriggerTimestamp > 0) {
                // Check if cooldown period has passed since entering RUNNING state
                unsigned long timeSinceStart = millis() - both_lastStartTriggerTimestamp;
                if (timeSinceStart >= BOTH_ROLE_COOLDOWN_MS) {
                    return true; // Allow STOP trigger after cooldown period
                } else {
                    Serial.printf("[BEHAVIOR] BOTH: STOP trigger blocked (only %lu ms since START, need %lu ms)\n", 
                               timeSinceStart, BOTH_ROLE_COOLDOWN_MS);
                    return false;
                }
            }
            return false;
            
        case ROLE_DISPLAY_ONLY:
            return false; // Display only never triggers
            
        case ROLE_SPLIT:
            return (currentState == STOPWATCH_RUNNING); // SPLIT only triggers during RUNNING
            
        default:
            return false;
    }
}

TriggerBehaviorMode triggerBehaviorGetMode() {
    return currentMode;
}
