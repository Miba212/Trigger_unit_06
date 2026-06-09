// trigger_test.cpp - Enhanced for separate START/STOP units
#include "trigger_test.h"
#include "comm_espnow_trigger.h"
#include "trigger_sensor.h"
#include "trigger_behavior.h"
#include "trigger_timesync.h"

#if TRIGGER_TEST_ENABLED

// Timestamp mode selection
#define TIMESTAMP_MODE_CALCULATED 1  // Use calculated timestamp (exact 25s)
#define TIMESTAMP_MODE_MEASURED   2  // Use real-time measured timestamp
#define ACTIVE_TIMESTAMP_MODE TIMESTAMP_MODE_CALCULATED  // <-- Set your mode here

// State tracking variables
static unsigned long stateEnteredTime = 0;
static unsigned long runningStateEnteredTime = 0;  // NEW: Track when RUNNING state specifically started
static uint64_t runningStateEnteredTimestamp = 0;  // NEW: Timestamp when entered RUNNING state
static uint8_t lastState = 255;
static bool startTriggered = false;
static bool stopTriggered = false;
static uint64_t triggerTimestamp = 0;
static uint64_t scheduledStopTimestamp = 0;
static uint64_t startTimestamp = 0;

static const char* getStateString(uint8_t state) {
    switch(state) {
        case STOPWATCH_IDLE: return "IDLE";
        case STOPWATCH_READY: return "READY"; 
        case STOPWATCH_RUNNING: return "RUNNING";
        case STOPWATCH_STOPPED: return "STOPPED";
        case STOPWATCH_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

void triggerTestInit() {
    stateEnteredTime = 0;
    runningStateEnteredTime = 0;
    runningStateEnteredTimestamp = 0;
    lastState = 255;
    startTriggered = false;
    stopTriggered = false;
    startTimestamp = 0;
    scheduledStopTimestamp = 0;
    
    Serial.println("[TEST] 🧪 Enhanced test module for separate START/STOP units initialized");
    Serial.printf("[TEST] 🧪 START unit: Will trigger after %d ms in READY state\n", TEST_START_DELAY_MS);
    Serial.printf("[TEST] 🧪 STOP unit: Will trigger %d ms after RUNNING state begins\n", TEST_STOP_DELAY_MS);
    
    if (ACTIVE_TIMESTAMP_MODE == TIMESTAMP_MODE_CALCULATED) {
        Serial.println("[TEST] ⏱️ Using CALCULATED timestamps (precise timing)");
    } else {
        Serial.println("[TEST] ⏱️ Using MEASURED timestamps (actual sensor delay)");
    }
}

void triggerTestUpdate() {
    // Track state changes for both timing and debugging
    if (currentStopwatchState != lastState) {
        Serial.printf("[TEST] State changed: %s -> %s (resetting test timer)\n", 
                     getStateString(lastState), getStateString(currentStopwatchState));
        
        // Update general state timing
        stateEnteredTime = millis();
        
        // NEW: Special handling for RUNNING state entry
        if (currentStopwatchState == STOPWATCH_RUNNING) {
            runningStateEnteredTime = millis();
            runningStateEnteredTimestamp = triggerTimeSyncGetUnixMs();
            stopTriggered = false;
            
            // NEW: For STOP role, schedule stop timer from NOW
            if (triggerRole == ROLE_STOP) {
                scheduledStopTimestamp = runningStateEnteredTimestamp + TEST_STOP_DELAY_MS;
                Serial.printf("[TEST] 🧪 STOP unit: Detected RUNNING state, will trigger in %d ms\n", TEST_STOP_DELAY_MS);
                Serial.printf("[TEST] 📅 STOP scheduled for timestamp %lu\n", scheduledStopTimestamp);
            }
            
            Serial.println("[TEST] 🧪 Ready for STOP test");
        }
        
        // Reset flags on appropriate state transitions
        if (currentStopwatchState == STOPWATCH_READY) {
            startTriggered = false;
            Serial.println("[TEST] 🧪 Ready for START test");
        }
        
        lastState = currentStopwatchState;
    }
    
    // Get current config for role information
    const EspNowSettingsPacket* cfg = commEspNowTriggerGetLastConfigPacket();
    if (!cfg) return;
    
    // Get current timestamp and state time
    uint64_t currentTimestamp = triggerTimeSyncGetUnixMs();
    unsigned long stateTime = millis() - stateEnteredTime;
    
    // ===== START EVENT LOGIC (for START and BOTH roles) =====
    if (currentStopwatchState == STOPWATCH_READY && !startTriggered && 
        (triggerRole == ROLE_START || triggerRole == ROLE_BOTH) && 
        stateTime >= TEST_START_DELAY_MS) {
        
        unsigned long t1 = micros(); // Timing start
        
        // CRITICAL SECTION - No Serial prints
        startTimestamp = triggerTimeSyncGetUnixMs();
        startTriggered = true;
        
        // For ROLE_BOTH, calculate exactly when stop should happen
        if (triggerRole == ROLE_BOTH) {
            scheduledStopTimestamp = startTimestamp + TEST_STOP_DELAY_MS;
        }
        
        // Send trigger event
        TriggerCallback callback = triggerSensorGetCallback();
        if (callback) {
            callback(startTimestamp);
        }
        
        unsigned long t2 = micros(); // Timing end
        
        // Only print AFTER the critical section
        Serial.printf("\n[TEST] 🚀 AUTO-TRIGGERING START EVENT at timestamp %lu\n", startTimestamp);
        if (triggerRole == ROLE_BOTH) {
            Serial.printf("[TEST] 📅 STOP scheduled for timestamp %lu (exactly +%dms)\n", 
                        scheduledStopTimestamp, TEST_STOP_DELAY_MS);
        }
        Serial.printf("[TEST] ⏱️ Processing time: %lu microseconds\n", t2-t1);
        Serial.println("[TEST] ✅ START event triggered\n");
    }
    
    // ===== STOP EVENT LOGIC =====
    // Two separate cases:
    // 1. ROLE_BOTH: Use the original logic based on START trigger timestamp
    // 2. ROLE_STOP: Use the new logic based on RUNNING state entry timestamp
    
    // CASE 1: BOTH role (original logic)
    bool shouldTriggerStop = false;
    uint32_t stopTimestamp = 0;
    
    if (triggerRole == ROLE_BOTH) {
        shouldTriggerStop = (currentStopwatchState == STOPWATCH_RUNNING && !stopTriggered && 
                            startTimestamp > 0 && scheduledStopTimestamp > 0 &&
                            currentTimestamp >= scheduledStopTimestamp);
        
        if (shouldTriggerStop && ACTIVE_TIMESTAMP_MODE == TIMESTAMP_MODE_CALCULATED) {
            stopTimestamp = scheduledStopTimestamp; // Use pre-calculated timestamp
        }
    }
    // CASE 2: STOP role (new logic)
    else if (triggerRole == ROLE_STOP) {
        shouldTriggerStop = (currentStopwatchState == STOPWATCH_RUNNING && !stopTriggered && 
                            runningStateEnteredTimestamp > 0 && scheduledStopTimestamp > 0 &&
                            currentTimestamp >= scheduledStopTimestamp);
        
        if (shouldTriggerStop && ACTIVE_TIMESTAMP_MODE == TIMESTAMP_MODE_CALCULATED) {
            stopTimestamp = scheduledStopTimestamp; // Use pre-calculated timestamp
        }
    }
    
    // If we should trigger a stop event
    if (shouldTriggerStop) {
        unsigned long t1 = micros(); // Timing start
        
        // CRITICAL SECTION - No Serial prints
        stopTriggered = true;
        
        // If we haven't set a stop timestamp yet, use current time
        if (stopTimestamp == 0) {
            stopTimestamp = currentTimestamp; // Use current timestamp
        }
        
        // Send trigger event with the chosen timestamp
        TriggerCallback callback = triggerSensorGetCallback();
        if (callback) {
            callback(stopTimestamp);
        }
        
        unsigned long t2 = micros(); // Timing end
        
        // Only print AFTER the critical section
        Serial.printf("\n[TEST] 🛑 AUTO-TRIGGERING STOP EVENT at timestamp %lu\n", stopTimestamp);
        Serial.printf("[TEST] ⏱️ Processing time: %lu microseconds\n", t2-t1);
        
        // Calculate delta based on role
        if (triggerRole == ROLE_BOTH) {
            uint32_t actualDelta = stopTimestamp - startTimestamp;
            int32_t timingOffset = currentTimestamp - scheduledStopTimestamp;
            
            if (ACTIVE_TIMESTAMP_MODE == TIMESTAMP_MODE_CALCULATED) {
                Serial.printf("[TEST] ⏱️ Exact START-to-STOP delta: %lu ms\n", actualDelta);
            } else {
                Serial.printf("[TEST] ⏱️ Actual START-to-STOP delta: %lu ms (offset: %d ms)\n", 
                             actualDelta, timingOffset);
            }
        } else {
            uint32_t actualDelta = stopTimestamp - runningStateEnteredTimestamp;
            Serial.printf("[TEST] ⏱️ RUNNING-to-STOP delta: %lu ms (target: %d ms)\n", 
                         actualDelta, TEST_STOP_DELAY_MS);
        }
        
        Serial.println("[TEST] ✅ STOP event triggered\n");
    }
}

#else // TRIGGER_TEST_ENABLED is 0

void triggerTestInit() {}
void triggerTestUpdate() {}

#endif // TRIGGER_TEST_ENABLED
