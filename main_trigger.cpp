#include <Arduino.h>
#include <WiFi.h>
#include "comm_espnow_trigger.h"
#include "config_trigger.h"
#include "trigger_behavior.h"
#include "trigger_state.h"
#include "trigger_sensor.h"
#include "trigger_display.h"
#include "trigger_led.h"
#include "trigger_test.h"
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include "wifi_config_trigger.h"
#include "trigger_timesync.h"

// Pairing state constants (must match TriggerPairingState enum)
#define STATE_UNPAIRED 0
#define STATE_PAIRING_REQUESTED 1
#define STATE_PAIRED_CONFIGURING 2
#define STATE_PAIRED_OPERATIONAL 3
#define STATE_UNPAIRING 4

// Forward declarations
const char* getStateString(uint8_t state);
void handleStateUpdate(uint8_t newState);
void onSensorTriggered(uint32_t timestamp);

// Local state variables (not extern)
static uint8_t lastStopwatchState = 255;   // Invalid initial value
static bool stateUpdateReceived = false;

static unsigned long lastLoopTime = 0;
static const unsigned long LOOP_TIMEOUT_WARNING = 200; // ms (can be shorter than main unit)
const char* RESET_COUNT_FILE = "/reset_counter.txt";

// Log reset reason function - KEEP ONLY ONE COPY OF THIS FUNCTION
void logResetReason() {
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("[TRIGGER] Reset Reason: ");
    
    switch (reason) {
        case ESP_RST_POWERON: Serial.println("Power-on reset"); break;
        case ESP_RST_SW: Serial.println("Software reset"); break;
        case ESP_RST_PANIC: Serial.println("Exception/panic"); break;
        case ESP_RST_INT_WDT: Serial.println("Interrupt watchdog"); break;
        case ESP_RST_TASK_WDT: Serial.println("Task watchdog"); break;
        case ESP_RST_WDT: Serial.println("Other watchdog"); break;
        case ESP_RST_BROWNOUT: Serial.println("Brownout"); break;
        default: Serial.println("Other reason");
    }
}

// Print MAC address utility
void printMac(const uint8_t* mac) {
    for(int i = 0; i < 6; ++i) {
        Serial.printf("%02X", mac[i]);
        if(i < 5) Serial.print(":");
    }
}

// Print config struct details
void printConfigPacket(const EspNowSettingsPacket& cfg) {
    Serial.println("=== Received CONFIG PACKET ===");
    Serial.print("Unit ID: "); Serial.println(cfg.unitID);
    Serial.print("Name: "); Serial.println(cfg.name);
    Serial.print("Role: "); Serial.println(cfg.role);
    Serial.print("LED Enabled: "); Serial.println(cfg.ledEnabled ? "true" : "false");
    Serial.print("Display Enabled: "); Serial.println(cfg.displayEnabled ? "true" : "false");
    Serial.print("Display Mode: "); Serial.println(cfg.displayMode);
    Serial.print("Sensor Type: "); Serial.println(cfg.sensorType);
    Serial.print("Distance Threshold: "); Serial.println(cfg.distanceThreshold, 2);
    Serial.println("==============================");
}

// State update handler - called when main unit sends state update
void handleStateUpdate(uint8_t newState) {
    uint8_t oldState = triggerStateGetCurrent();

    Serial.printf("[STATE][CHANGE] UnitID:  %d | Role: %d | Old: %s (%d) -> New: %s (%d)\n", 
        configTrigger.get().unitID, 
        configTrigger.get().role,
        getStateString(oldState), oldState, 
        getStateString(newState), newState
    );

    // *** NEW: Control time sync blocking based on state ***
    if (newState == STOPWATCH_RUNNING) {
        triggerTimeSyncSetTimerActive(true);
        Serial.println("[TRIGGER][TimeSync] ⏱️ Timer ACTIVE - time sync BLOCKED");
    } else if (newState == STOPWATCH_IDLE || 
               newState == STOPWATCH_READY || 
               newState == STOPWATCH_STOPPED) {
        triggerTimeSyncSetTimerActive(false);
        Serial.println("[TRIGGER][TimeSync] ⏸️ Timer INACTIVE - time sync ALLOWED");
    }

    const EspNowSettingsPacket* cfg = commEspNowTriggerGetLastConfigPacket();
    if (cfg) {
        triggerBehaviorHandleStateChange(oldState, newState, cfg);
    }

    Serial.println("[DEBUG] ═══ ABOUT TO CALL triggerDisplayUpdateState() ═══");
    triggerDisplayUpdateState(newState);
    Serial.println("[DEBUG] ═══ triggerDisplayUpdateState() COMPLETED ═══");

    triggerStateSet(newState);
    
    stateUpdateReceived = true;
}
// Get state string for debugging
const char* getStateString(uint8_t state) {
    switch(state) {
        case 0: return "IDLE";
        case 1: return "READY"; 
        case 2: return "RUNNING";
        case 3: return "STOPPED";
        case 4: return "ERROR";
        default: return "UNKNOWN";
    }
}

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("=== Trigger Unit Booting ===");

    // Configure watchdog
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = 5000,
        .idle_core_mask = 0,
        .trigger_panic = true
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);

    // Initialize all subsystems
    triggerBehaviorInit();
    triggerStateInit();
    triggerSensorInit();
    triggerDisplayInit();
    triggerTestInit();
    triggerLedInit();
    Serial.println("[TRIGGER] LED system initialized");
    
    // Load configuration
    configTrigger.load();
    triggerDisplayPrepareFromSavedConfig(
    configTrigger.get().displayEnabled,
    configTrigger.get().displayMode
);
    
    // ==========================================
    // ✅ ADD WIFI INITIALIZATION HERE!
    // ==========================================
    
    Serial.println("[TRIGGER] Initializing WiFi...");
    
    // Start WiFi in STA mode
   WiFi.mode(TRIGGER_WIFI_MODE);
    //WiFi.mode(WIFI_STA);
    
    // Start WiFi hardware (initializes MAC address)
    WiFi.begin();
    
    // Wait for WiFi to be ready
    delay(100);
    
    // Disconnect from any AP (we don't need to connect to one)
    WiFi.disconnect();
    
    // Set ESP-NOW TX power
   // esp_wifi_set_max_tx_power(17 * 4);  // 13 dBm default
   int txPowerDbm = TRIGGER_ESPNOW_TX_POWER_DBM;
   // Optional: clamp to reasonable range (ESP32 typical range 8..20 dBm)
   if (txPowerDbm < 8) txPowerDbm = 8;
   if (txPowerDbm > 20) txPowerDbm = 20;
   esp_err_t res = esp_wifi_set_max_tx_power(txPowerDbm * 4);
   if (res != ESP_OK) {
    Serial.printf("[TRIGGER] ❌ Failed to set TX power (err=%d)\n", res);
   } else {
    Serial.printf("[TRIGGER] TX Power set to %d dBm\n", txPowerDbm);
}
    
    // Print WiFi info
    Serial.println("[TRIGGER] ========================================");
    Serial.println("[TRIGGER] WiFi Configuration");
    Serial.printf("[TRIGGER] Mode: STA (Station - no AP)\n");
    Serial.printf("[TRIGGER] TX Power set to %d dBm\n", txPowerDbm);
    Serial.print("[TRIGGER] MAC Address: ");
    Serial.println(WiFi.macAddress());  // Should show real MAC now!
    Serial.println("[TRIGGER] ========================================");
    
    // Verify MAC is valid
    if (WiFi.macAddress() == "00:00:00:00:00:00") {
        Serial.println("[TRIGGER] ❌ ERROR: Invalid MAC address!");
        Serial.println("[TRIGGER] WiFi not initialized properly!");
        while(1) {
            delay(1000);
        }
    }
    
    // ==========================================
    // NOW Initialize ESP-NOW communication
    // ==========================================
    
    commEspNowTriggerInit();

    // Register callbacks FIRST, before any pairing/reconnect activity
    commEspNowTriggerSetStateCallback(handleStateUpdate);
    triggerSensorSetTriggerCallback(onSensorTriggered);
    
    if (!configTrigger.isPaired()) {
        Serial.println("[TRIGGER] Not paired - entering STATE 1");
        startState1FreshPairing();
    } else {
        Serial.println("[TRIGGER] Previously paired - entering STATE 2");
        startState2Reconnection();
    }
    
    Serial.printf("[TRIGGER] Boot complete - Unit ID: %d\n", configTrigger.get().unitID);
    
    logResetReason();
}

// Sensor trigger callback with timestamp parameter
void onSensorTriggered(uint32_t timestamp) {
    const EspNowSettingsPacket* cfg = commEspNowTriggerGetLastConfigPacket();
    if (!cfg) return;
    
    Serial.printf("[TRIGGER] Sensor triggered at timestamp %lu! Role: %d, State: %s\n", 
                  timestamp, cfg->role, getStateString(currentStopwatchState));
    
    // Only send trigger if we should act on current state
    if (triggerBehaviorShouldTrigger(currentStopwatchState, cfg->role)) {
        Serial.println("[TRIGGER] 🎯 Sending trigger event to main unit!");
        
        // *** NEW: Block time sync when START trigger fires ***
        if (currentStopwatchState == STOPWATCH_READY) {
            // This is a START trigger
            triggerTimeSyncSetTimerActive(true);
            Serial.println("[TRIGGER][TimeSync] ⏱️ START triggered - time sync BLOCKED");
        }
        // *** NEW: Allow time sync when STOP trigger fires ***
        else if (currentStopwatchState == STOPWATCH_RUNNING) {
            // This is a STOP trigger
            triggerTimeSyncSetTimerActive(false);
            Serial.println("[TRIGGER][TimeSync] ⏸️ STOP triggered - time sync ALLOWED");
        }
        
        // Send the actual trigger event via ESP-NOW with the captured timestamp
        triggerCommSendTriggerEvent(timestamp);
        
        // Update visual feedback
//        triggerDisplayShowTriggered();
    } else {
        Serial.println("[TRIGGER] ❌ Not acting - wrong state for this role");
    }
}
void loop() {
    // Feed watchdog at start of loop
    esp_task_wdt_reset();
    
    // Track loop execution time
    unsigned long now = millis();
    unsigned long loopTime = now - lastLoopTime;
    
    // Warning for slow loops (potential issues)
    if (loopTime > LOOP_TIMEOUT_WARNING) {
        Serial.printf("[WATCHDOG] Warning: Loop took %lu ms\n", loopTime);
    }
    
    lastLoopTime = now;
    
    // Update communication
    commEspNowTriggerLoop();
    
    // Update all subsystems
    triggerBehaviorUpdate();
    triggerSensorUpdate();
    triggerDisplayUpdate();  // ✅ This MUST be called!
    triggerTestUpdate();

    // ===== COMPLETE LED UPDATE LOGIC =====
    static uint8_t lastLedJourney = 255;
    static unsigned long successHoldStart = 0;
    static bool inSuccessHold = false;
    static bool lastLedEnabled = true;

    uint8_t currentLedJourney = commEspNowTriggerGetLedJourney();

    // Detect journey changes
    if (currentLedJourney != lastLedJourney) {
        Serial.printf("[LED] Journey changed: %d -> %d\n", lastLedJourney, currentLedJourney);
        lastLedJourney = currentLedJourney;
        
        // Start 60-second hold on success
        if (currentLedJourney == 2) { // LED_JOURNEY_PAIRED_SUCCESS
            triggerLedNotifyPairingSuccess();
            successHoldStart = millis();
            inSuccessHold = true;
            Serial.println("[LED] Starting 60s PAIRING success hold");
        }
        else if (currentLedJourney == 3) { // LED_JOURNEY_RECONNECT_SUCCESS
            triggerLedNotifyReconnectSuccess();
            successHoldStart = millis();
            inSuccessHold = true;
            Serial.println("[LED] Starting 60s RECONNECT success hold");
        }
    }

    // Check if 60-second hold expired
    if (inSuccessHold && (millis() - successHoldStart >= 60000)) {
        inSuccessHold = false;
        commEspNowTriggerSetLedOperational();
        Serial.println("[LED] 60s success hold EXPIRED -> OPERATIONAL");
    }

    // Map journey to LED pairing state
    bool pairingWindowActive = commEspNowTriggerIsPairingWindowActive();

    switch (currentLedJourney) {
        case 0: // LED_JOURNEY_FRESH_PAIRING
            triggerLedSetPairingState(STATE_UNPAIRED, pairingWindowActive);
            break;
            
        case 1: // LED_JOURNEY_RECONNECTING
            triggerLedSetPairingState(STATE_PAIRED_CONFIGURING, pairingWindowActive);
            break;
            
        case 2: // LED_JOURNEY_PAIRED_SUCCESS
        case 3: // LED_JOURNEY_RECONNECT_SUCCESS
            triggerLedSetPairingState(STATE_PAIRED_OPERATIONAL, false);
            break;
            
        case 4: // LED_JOURNEY_TIMEOUT
            triggerLedSetError(ERROR_GENERAL);
            break;
            
        case 5: // LED_JOURNEY_OPERATIONAL
            triggerLedSetPairingState(STATE_PAIRED_OPERATIONAL, false);
            triggerLedClearError(); // Clear any old errors
            break;
    }

    // Update stopwatch state (always)
    triggerLedSetStopwatchState(currentStopwatchState);

    // Update LED enabled setting from config
    const EspNowSettingsPacket* cfgLed = commEspNowTriggerGetLastConfigPacket();
    if (cfgLed && cfgLed->magic == ESPNOW_SETTINGS_MAGIC) {
        if (cfgLed->ledEnabled != lastLedEnabled) {
            triggerLedSetEnabled(cfgLed->ledEnabled);
            lastLedEnabled = cfgLed->ledEnabled;
        }
    }

    // Update LED (call every loop)
    triggerLedUpdate();

    // Handle pairing state changes
    static bool lastPaired = false;
    bool paired = commEspNowTriggerIsPaired();

    if (paired != lastPaired) {
        lastPaired = paired;
        if (paired) {
            uint8_t mac[6];
            commEspNowTriggerGetMainUnitMac(mac);
            Serial.print("[Trigger] ✅ Paired with main unit MAC: ");
            printMac(mac);
            Serial.println();
        } else {
            Serial.println("[Trigger] ❌ Not paired yet, waiting for ACK.. .");
        }
    }
    
    // Config monitoring - check EVERY setting individually (magic never changes!)
    static float lastConfigThreshold = -9999.0;
    static int lastDisplayEnabled = -1;
    static int lastDisplayMode = -1;
    static int lastSensorType = -1;
    static int lastRole = -1;
    static bool configFirstRun = true;

    const EspNowSettingsPacket* cfg = commEspNowTriggerGetLastConfigPacket();
    if (cfg && cfg->magic == ESPNOW_SETTINGS_MAGIC) {
        bool anyChange = false;
        
        // Check threshold
        if (cfg->distanceThreshold != lastConfigThreshold) {
            Serial.printf("[CONFIG] 📏 Threshold:  %.2f -> %.2f\n", 
                         lastConfigThreshold, cfg->distanceThreshold);
            triggerSensorSetThreshold(cfg->distanceThreshold);
            lastConfigThreshold = cfg->distanceThreshold;
            anyChange = true;
        }
        
        // Check sensor type
        if (cfg->sensorType != lastSensorType || configFirstRun) {
            Serial.printf("[CONFIG] 📡 Sensor type: %d -> %d\n", 
                         lastSensorType, cfg->sensorType);
            triggerSensorSetType(cfg->sensorType);
            lastSensorType = cfg->sensorType;
            anyChange = true;
        }
        
        // Check role
        if (cfg->role != lastRole || configFirstRun) {
            Serial.printf("[CONFIG] 🎯 Role:  %d -> %d\n", 
                         lastRole, cfg->role);
            triggerBehaviorSetRole(cfg->role);
            lastRole = cfg->role;
            anyChange = true;
        }
        
       if (cfg->displayEnabled != lastDisplayEnabled || configFirstRun) {
        bool wasEnabled = (lastDisplayEnabled == 1);
        triggerDisplaySetEnabled(cfg->displayEnabled);
        lastDisplayEnabled = cfg->displayEnabled;
        anyChange = true;

        // If display just became enabled, render current state immediately
        if (cfg->displayEnabled && !wasEnabled) {
        triggerDisplayUpdateState(currentStopwatchState);
        }
    }
        // Check display mode
       if (cfg->displayMode != lastDisplayMode || configFirstRun) {
        triggerDisplaySetMode(cfg->displayMode);
        lastDisplayMode = cfg->displayMode;
        anyChange = true;

        // If display is active, re-render current state with new mode
        if (cfg->displayEnabled) {
        triggerDisplayUpdateState(currentStopwatchState);
    }
}
        
        // Clear first run flag
        if (configFirstRun) {
            configFirstRun = false;
        }
        
        // Print full config only if something changed
        if (anyChange) {
            Serial.println("[CONFIG] ══════════════════════════════");
            printConfigPacket(*cfg);
            Serial.println("[CONFIG] ══════════════════════════════");
        }
    }  // ✅ FIXED:  Proper closing brace for config section
    
    // Debug state changes
    if (currentStopwatchState != lastStopwatchState) {
        Serial.printf("[TRIGGER] Stopwatch state:  %s -> %s\n", 
                      getStateString(lastStopwatchState), 
                      getStateString(currentStopwatchState));
        lastStopwatchState = currentStopwatchState;
    }
    
    // Small delay to prevent system overload
    delay(10);
    
    // Feed watchdog at end of loop too
    esp_task_wdt_reset();
}  // ✅ FIXED:  Proper closing brace for loop() function
