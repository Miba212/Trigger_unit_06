#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "comm_espnow_trigger.h"
#include "config_trigger.h"
#include "espnow_timesync.h"
#include "trigger_timesync.h"
#include "trigger_behavior.h"
#include "trigger_sensor.h"
#include "trigger_display.h" 
#include <Arduino.h>

// Command bytes
// #define PAIRING_ACK_CMD     0xA1
// #define DATA_CMD            0xB0
// #define CONFIG_REQ_CMD      0xC1
// #define UNPAIR_CMD          0xD0
// #define UNPAIR_ACK_CMD      0xD1
// #define TRIGGER_EVENT_CMD   0xB1  // New command for trigger events

// RESET_PAIRING_CMD 0xF1 is defined in the header file

// Main unit identifier in pairing ACK
#define MAIN_UNIT_IDENTIFIER 0xA5

// --- Enhanced state management ---
enum TriggerPairingState {
    STATE_UNPAIRED = 0,          // Not paired with any main unit
    STATE_PAIRING_REQUESTED,     // Sent pairing request, waiting for response
    STATE_PAIRED_CONFIGURING,    // Paired but waiting for config
    STATE_PAIRED_OPERATIONAL,    // Fully paired and configured
    STATE_UNPAIRING              // In process of unpairing
};

// CRITICAL: Define global variables here (only in .cpp file)
uint8_t currentStopwatchState = 0;  // STOPWATCH_IDLE
uint8_t triggerRole = 0; // 0=Start, 1=Stop, 2=Both  
uint8_t unitID = 0;

// State update callback
static void (*stateUpdateCallback)(uint8_t) = nullptr;

static bool triggerEventPending = false;
static uint8_t pendingTriggerType = 0;
static uint32_t pendingTriggerTimestamp = 0;
static unsigned long triggerEventSentTime = 0;
static int triggerEventRetryCount = 0;
static const int MAX_TRIGGER_RETRIES = 3;
static const unsigned long TRIGGER_RETRY_INTERVAL = 500; // 500ms

// --- Global variables ---
static EspNowSettingsPacket lastConfigPacket = {0};
static bool espNowInitDone = false;
static uint8_t mainUnitMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static unsigned long lastPairingSend = 0;
static unsigned long lastHeartbeatSend = 0;
static unsigned long lastConfigRequest = 0;
static TriggerPairingState pairingState = STATE_UNPAIRED;

// RSSI monitoring - NEW for Core 3.3.6
static int8_t mainUnitRSSI = -127;  // Invalid/no signal initially (valid range: -10 to -100 dBm)

// Pairing window state
static bool pairingMode = false;
static bool manualPairingMode = false;
static unsigned long pairingWindowEnd = 0;

// Cooldown after unpairing
static bool recentlyUnpaired = false;
static unsigned long unpairedTimestamp = 0;
static const unsigned long UNPAIR_COOLDOWN_MS = 5000; // 5 seconds cooldown

// Timing constants
static const unsigned long HEARTBEAT_INTERVAL_MS = 3000;     // Send heartbeat every 3 seconds
static const unsigned long CONFIG_REQUEST_INTERVAL_MS = 5000; // Request config every 5 seconds if needed
static const unsigned long PAIRING_RETRY_INTERVAL_MS = 3000; // Retry pairing every 3 seconds

// Channel scanning range
#define CHANNEL_SCAN_MIN 1
#define CHANNEL_SCAN_MAX 13

// STATE 1: Fresh trigger timing
#define STATE1_PAIRING_WINDOW_MS 120000      // 2 minutes
#define STATE1_SCAN_INTERVAL_MS 5000         // Scan every 5 seconds
#define STATE1_CHANNEL_DWELL_MS 250          // 250ms per channel

// STATE 2: Previously paired timing
#define STATE2_QUICK_RETRY_DURATION_MS 5000  // Phase 1: 5 seconds
#define STATE2_EXTENDED_WINDOW_MS 120000     // Phase 2: 2 minutes
#define STATE2_TOTAL_WINDOW_MS (STATE2_QUICK_RETRY_DURATION_MS + STATE2_EXTENDED_WINDOW_MS)
#define STATE2_RETRY_INTERVAL_MS 1000        // Phase 1: every 1 second
#define STATE2_SCAN_INTERVAL_MS 10000        // Phase 2: every 10 seconds
#define STATE2_CHANNEL_DWELL_MS 250          // 250ms per channel

// STATE 4: After pairing reset
#define STATE4_PART1_TOTAL_WINDOW_MS STATE2_TOTAL_WINDOW_MS
#define STATE4_PART2_PAIRING_WINDOW_MS STATE1_PAIRING_WINDOW_MS

// Channel scanning state variables
static uint8_t currentScanChannel = 0;
static unsigned long channelScanStartTime = 0;
static unsigned long lastChannelSwitchTime = 0;
static bool channelScanActive = false;
static uint8_t scanAttemptNumber = 0;
static bool inExtendedPairingWindow = false;
static unsigned long extendedWindowStartTime = 0;
static int reconnectionPhase = 0;
static bool pairingResponseReceived = false;
static bool configResponseReceived = false;
static bool resetPairingReceived = false;

// LED journey tracking
static LedJourneyState ledJourney = LED_JOURNEY_FRESH_PAIRING;
static bool pairingTimeoutOccurred = false;

// RSSI quality thresholds (in dBm)
static const int8_t RSSI_EXCELLENT_THRESHOLD = -50;
static const int8_t RSSI_GOOD_THRESHOLD = -60;
static const int8_t RSSI_FAIR_THRESHOLD = -70;
static const int8_t RSSI_WEAK_THRESHOLD = -80;

// Local config storage
static uint8_t triggerUnitID = 255;
static char triggerName[ESPNOW_MAX_NAME] = "Unset";
static bool triggerLedEnabled = false;
static bool triggerDisplayEnabled = false;
static uint8_t triggerDisplayMode = 0;
static uint8_t triggerSensorType = 0;
static float triggerDistanceThreshold = 0.0f;

static bool isMacBroadcast(const uint8_t* mac);

// --- Channel Scanning Functions ---

static void setWiFiChannel(uint8_t channel) {
    if (channel < CHANNEL_SCAN_MIN || channel > CHANNEL_SCAN_MAX) {
        Serial.printf("[TRIGGER][ERROR] Invalid channel: %d\n", channel);
        return;
    }
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    delay(10);
    Serial.printf("[TRIGGER] 📡 Switched to channel %d\n", channel);
}

static void scanChannelsWithPairingHello() {
    Serial.println("[TRIGGER] 🔍 Starting channel scan with PAIRING_HELLO");
    pairingResponseReceived = false;
    
    for (uint8_t ch = CHANNEL_SCAN_MIN; ch <= CHANNEL_SCAN_MAX; ch++) {
        setWiFiChannel(ch);
        
        // Send PAIRING_HELLO broadcast
        struct EspNowHelloPacket {
            uint32_t magic;
            uint8_t  protoVer;
            uint8_t  reserved[3];
        };
        
        EspNowHelloPacket pkt = { ESPNOW_PAIR_MAGIC, 1, {0,0,0} };
        uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        
        esp_now_send(bcast, (uint8_t*)&pkt, sizeof(pkt));
        Serial.printf("[TRIGGER] 📤 Sent PAIRING_HELLO on channel %d\n", ch);
        
        // Dwell on this channel
        delay(STATE1_CHANNEL_DWELL_MS);
        
        // Check if we got a response
        if (pairingResponseReceived) {
            Serial.printf("[TRIGGER] ✅ Pairing response received on channel %d\n", ch);
            configTrigger.setChannel(ch);
            configTrigger.save();
            return;
        }
    }
    
    Serial.println("[TRIGGER] ⚠️ Channel scan completed, no response");
}

static void scanChannelsWithConfigRequest() {
    Serial.println("[TRIGGER] 🔍 Starting channel scan with CONFIG_REQUEST");
    configResponseReceived = false;
    resetPairingReceived = false;
    
    for (uint8_t ch = CHANNEL_SCAN_MIN; ch <= CHANNEL_SCAN_MAX; ch++) {
        setWiFiChannel(ch);
        
        // Send CONFIG_REQUEST to saved MAC
        if (!isMacBroadcast(mainUnitMac)) {
            uint8_t req[4] = {CONFIG_REQ_CMD, 0xA1, 0x12, 0x34};
            esp_now_send(mainUnitMac, req, 4);
            Serial.printf("[TRIGGER] 📤 Sent CONFIG_REQUEST on channel %d\n", ch);
        }
        
        // Dwell on this channel
        delay(STATE2_CHANNEL_DWELL_MS);
        
        // Check if we got a response
        if (configResponseReceived) {
            Serial.printf("[TRIGGER] ✅ Config response received on channel %d\n", ch);
            if (configTrigger.getChannel() != ch) {
                Serial.printf("[TRIGGER] 🔄 Updating saved channel: %d -> %d\n", 
                             configTrigger.getChannel(), ch);
                configTrigger.setChannel(ch);
                configTrigger.save();
            }
            return;
        }
        
        if (resetPairingReceived) {
            Serial.printf("[TRIGGER] 🔄 RESET_PAIRING received on channel %d\n", ch);
            return;
        }
    }
    
    Serial.println("[TRIGGER] ⚠️ Channel scan completed, no response");
}

    void startState1FreshPairing() {
    Serial.println("[TRIGGER] 🆕 STATE 1: Fresh pairing mode");
    Serial.println("[TRIGGER] Duration: 2 minutes");
    Serial.println("[TRIGGER] Scanning all channels every 5 seconds");
    
    inExtendedPairingWindow = true;
    extendedWindowStartTime = millis();
    channelScanActive = true;
    channelScanStartTime = millis();
    scanAttemptNumber = 0;
    pairingState = STATE_UNPAIRED;
    reconnectionPhase = 0;
    
    ledJourney = LED_JOURNEY_FRESH_PAIRING;
    pairingTimeoutOccurred = false;
}

    void startState2Reconnection() {
    Serial.println("[TRIGGER] 🔄 STATE 2: Reconnection mode");
    Serial.println("[TRIGGER] Phase 1: Quick retry on saved channel (5 seconds)");
    Serial.println("[TRIGGER] Phase 2: Channel scanning (2 minutes)");
    
    inExtendedPairingWindow = true;
    extendedWindowStartTime = millis();
    reconnectionPhase = 1; // Start with Phase 1
    channelScanActive = false;
    
    ledJourney = LED_JOURNEY_RECONNECTING;
    pairingTimeoutOccurred = false;
    
    // Set to saved channel for Phase 1
    uint8_t savedChannel = configTrigger.getChannel();
    if (savedChannel >= CHANNEL_SCAN_MIN && savedChannel <= CHANNEL_SCAN_MAX) {
        Serial.printf("[TRIGGER] 📡 Setting to saved channel %d\n", savedChannel);
        setWiFiChannel(savedChannel);
    } else {
        Serial.println("[TRIGGER] ⚠️ No valid saved channel, using channel 1");
        setWiFiChannel(1);
    }
}

// --- Helper functions ---
static void saveLocalTriggerConfig() {
    configTrigger.get().unitID = triggerUnitID;
    configTrigger.get().role = triggerRole;
    configTrigger.get().displayEnabled = triggerDisplayEnabled;
    configTrigger.get().displayMode = triggerDisplayMode;
    configTrigger.save();

    Serial.println("[TRIGGER] ✅ Configuration saved to flash memory");
}

static void printTriggerConfig() {
    Serial.println("┌───────────────────────────────────────────┐");
    Serial.println("│           TRIGGER CONFIGURATION           │");
    Serial.println("├───────────────────────────────────────────┤");
    Serial.printf("│ Unit ID:           %-24d │\n", triggerUnitID);
    Serial.printf("│ Name:              %-24s │\n", triggerName);
    Serial.printf("│ Role:              %-24d │\n", triggerRole);
    Serial.printf("│ LED Enabled:       %-24s │\n", triggerLedEnabled ? "true" : "false");
    Serial.printf("│ Display Enabled:   %-24s │\n", triggerDisplayEnabled ? "true" : "false");
    Serial.printf("│ Display Mode:      %-24d │\n", triggerDisplayMode);
    Serial.printf("│ Sensor Type:       %-24d │\n", triggerSensorType);
    Serial.printf("│ Distance Threshold: %-23.3f │\n", triggerDistanceThreshold);
    Serial.println("└───────────────────────────────────────────┘");
}

// Utility to print MAC addresses
static void printMacAddress(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        Serial.printf("%02X", mac[i]);
        if (i < 5) Serial.print(":");
    }
}

// Check if MAC is broadcast (all FFs)
static bool isMacBroadcast(const uint8_t* mac) {
    for (int i = 0; i < 6; i++) {
        if (mac[i] != 0xFF) return false;
    }
    return true;
}

// Send config request to main unit
static void sendConfigRequest() {
    if (isMacBroadcast(mainUnitMac)) {
        Serial.println("[TRIGGER][ERROR] Cannot request config - no main unit MAC");
        return;
    }
    
    Serial.println("[TRIGGER] Sending config request to main unit");
    uint8_t req[4] = {CONFIG_REQ_CMD, 0xA1, 0x12, 0x34};
    esp_err_t result = esp_now_send(mainUnitMac, req, 4);
    
    if (result == ESP_OK) {
        lastConfigRequest = millis();
        Serial.println("[TRIGGER] Config request sent successfully");
    } else {
        Serial.printf("[TRIGGER][ERROR] Failed to send config request: %d\n", result);
    }
}

// Send heartbeat to main unit
static void sendHeartbeat() {
    if (isMacBroadcast(mainUnitMac)) {
        return;
    }
    
    struct DataPacket {
        uint8_t cmd;        // 0xB0
        uint8_t unitID;     // Our unit ID
        uint8_t status;     // Status byte
        int8_t rssi;        // RSSI from main unit (NEW)
        uint8_t reserved[4]; // Reduced from 5 to 4
    };
    
    DataPacket pkt = { DATA_CMD, triggerUnitID, 0x01, mainUnitRSSI, {0,0,0,0} };
    esp_err_t result = esp_now_send(mainUnitMac, (uint8_t*)&pkt, sizeof(pkt));
    
    if (result == ESP_OK) {
        lastHeartbeatSend = millis();
        Serial.printf("[TRIGGER] Heartbeat sent | Our RSSI from main: %d dBm\n", mainUnitRSSI);
    } else {
        Serial.printf("[TRIGGER][ERROR] Failed to send heartbeat: %d\n", result);
    }
}

// Send pairing request
static void sendPairingRequest() {
    struct EspNowHelloPacket {
        uint32_t magic;
        uint8_t  protoVer;
        uint8_t  reserved[3];
    };
    
    EspNowHelloPacket pkt = { ESPNOW_PAIR_MAGIC, 1, {0,0,0} };
    
    // Determine target based on pairing mode
    static const uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t* target;
    if (manualPairingMode && !isMacBroadcast(mainUnitMac)) {
        target = mainUnitMac;
        Serial.println("[TRIGGER] MANUAL pairing: Sending to known main unit");
    } else {
        target = (uint8_t*)bcast;
        Serial.println("[TRIGGER] AUTO pairing: Broadcasting pairing request");
    }
    
    esp_err_t result = esp_now_send(target, (uint8_t*)&pkt, sizeof(pkt));
    
    if (result == ESP_OK) {
        lastPairingSend = millis();
        pairingState = STATE_PAIRING_REQUESTED;
        Serial.print("[TRIGGER] Pairing request sent to: ");
        printMacAddress(target);
        Serial.println();
    } else {
        Serial.printf("[TRIGGER][ERROR] Failed to send pairing request: %d\n", result);
    }
}

// Update pairing state with appropriate logging
static void updatePairingState(TriggerPairingState newState) {
    if (newState == pairingState) return;
    
    Serial.printf("[TRIGGER] State change: %d -> %d (", pairingState, newState);
    
    // Print state names for better debugging
    const char* stateNames[] = {
        "UNPAIRED", "PAIRING_REQUESTED", "PAIRED_CONFIGURING", 
        "PAIRED_OPERATIONAL", "UNPAIRING"
    };
    
    Serial.printf("%s -> %s)\n", 
                 stateNames[pairingState], stateNames[newState]);
    
    pairingState = newState;
    
    // Request fresh config when entering PAIRED_OPERATIONAL
    if (newState == STATE_PAIRED_OPERATIONAL && !isMacBroadcast(mainUnitMac)) {
        Serial.println("[TRIGGER] Entered operational state - requesting current configuration");
        sendConfigRequest();
    }
}

// Handle state update packets
static void handleStateUpdateInOnDataRecv(const uint8_t* data, int len) {
    if (len >= 8 && data[0] == STATE_UPDATE_CMD) {
        StateUpdatePacket* pkt = (StateUpdatePacket*)data;
        uint8_t newState = pkt->stopwatchState;

        Serial.printf("[TRIGGER] State update received: %d -> %d\n",
                      currentStopwatchState, newState);

        currentStopwatchState = newState;
        unitID = triggerUnitID;

        triggerDisplayMarkStateReceived();

        if (stateUpdateCallback) {
            stateUpdateCallback(newState);
        }

        triggerCommSendStateAck();
        return;
    }
}

// --- ESP-NOW callbacks ---
void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    // Extract MAC address - use 'da' (destination address) instead of 'mac'
//    const uint8_t *mac_addr = tx_info->da;
    
    // Keep all existing code below unchanged
    if (status == ESP_NOW_SEND_SUCCESS) {
        Serial.println("[TRIGGER] ✅ Data sent successfully");
    } else {
        Serial.println("[TRIGGER] ❌ Send failed");
    }
}

void commEspNowTriggerSendSensorSetupData(uint16_t measuredDistance, uint8_t rangeStatus) {
    if (isMacBroadcast(mainUnitMac)) {
        Serial.println("[TRIGGER][ERROR] Cannot send setup data - no main unit MAC");
        return;
    }
   SensorSetupDataPacket pkt = {
    SENSOR_SETUP_DATA_CMD,
    triggerUnitID,
    triggerSensorGetType(), // <-- This now works!
    measuredDistance,
    rangeStatus
};
    esp_err_t result = esp_now_send(mainUnitMac, (uint8_t*)&pkt, sizeof(pkt));
    if (result == ESP_OK) {
        Serial.printf("[TRIGGER] 🚀 Sent SENSOR_SETUP_DATA: %u mm, status=%u\n", measuredDistance, rangeStatus);
    } else {
        Serial.printf("[TRIGGER][ERROR] Failed to send sensor setup data: %d\n", result);
    }
}


static void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    // Defensive null pointer checks
    if (!info || !info->src_addr || !info->rx_ctrl) {
        Serial.println("[TRIGGER][ERROR] Invalid ESP-NOW info structure");
        return;
    }
    
    // Extract MAC address from info structure
    const uint8_t* mac = info->src_addr;
    
    // Extract and store RSSI
    int8_t rssi = info->rx_ctrl->rssi;
    
    // If this is from our main unit, store the RSSI
    if (memcmp(mac, mainUnitMac, 6) == 0) {
        mainUnitRSSI = rssi;
        
        // Log signal quality
        const char* quality = "Unknown";
        if (rssi >= RSSI_EXCELLENT_THRESHOLD) quality = "Excellent";
        else if (rssi >= RSSI_GOOD_THRESHOLD) quality = "Good";
        else if (rssi >= RSSI_FAIR_THRESHOLD) quality = "Fair";
        else if (rssi >= RSSI_WEAK_THRESHOLD) quality = "Weak";
        else quality = "Very Weak";
        
        Serial.printf("[TRIGGER] Packet from main | RSSI: %d dBm (%s)\n", rssi, quality);
    }
    
    Serial.printf("[TRIGGER][DEBUG] ESP-NOW packet received, len=%d:  ", len);
    for (int i = 0; i < len; ++i) Serial.printf("%02X ", data[i]);
    Serial.println();

    // Handle state updates first
    if (len >= 8 && data[0] == STATE_UPDATE_CMD) {
        handleStateUpdateInOnDataRecv(data, len);
        return;
    }

    if (len == sizeof(EspNowTimeSyncPacket)) {
        triggerTimeSyncOnPacket((const EspNowTimeSyncPacket*)data);
        return;
    }

    // ===== NEW:  RESULT PACKET =====
   // In your trigger unit ESP-NOW receive callback:
if (len == sizeof(ResultPacket) && data[0] == RESULT_PACKET_CMD) {
    const ResultPacket* pkt = (const ResultPacket*)data;
    float resultSeconds = pkt->resultSeconds;

    // Tell display module about new result
    triggerDisplaySetResult(resultSeconds);

    // Convert to MM:SS.CC format only for serial debug
    int totalCentiseconds = (int)(resultSeconds * 100);
    int minutes = totalCentiseconds / 6000;
    int seconds = (totalCentiseconds % 6000) / 100;
    int centiseconds = totalCentiseconds % 100;

    char displayBuf[16];
    sprintf(displayBuf, "%02d:%02d.%02d", minutes, seconds, centiseconds);
    Serial.printf("[TRIGGER] Result received: %s\n", displayBuf);

    return;
}
    
   // ===== NEW: COUNTDOWN START PACKET =====
if (len == sizeof(CountdownStartPacket) && data[0] == COUNTDOWN_START_CMD) {
    const CountdownStartPacket* pkt = (const CountdownStartPacket*)data;
    
    Serial.printf("[TRIGGER] ⏱️ Received COUNTDOWN START: %u seconds\n", pkt->durationSeconds);
    
    triggerDisplayStartCountdown(pkt->durationSeconds);
    return;
}

// ===== NEW: COUNTDOWN FINISH PACKET =====
if (len == sizeof(CountdownFinishPacket) && data[0] == COUNTDOWN_FINISH_CMD) {
    Serial.println("[TRIGGER] 🏁 Received COUNTDOWN FINISH");
    
    triggerDisplayFinishCountdown();
    return;
}

// ===== NEW: COUNTDOWN CANCEL PACKET =====   <<< ADD THIS BLOCK
if (len == sizeof(CountdownCancelPacket) && data[0] == COUNTDOWN_CANCEL_CMD) {
    Serial.println("[TRIGGER] 🛑 Received COUNTDOWN CANCEL");
    
    // This function should stop timer and go back to idle display
    triggerDisplayCancelCountdown();   // see next step
    return;
}
    // --- RESET_PAIRING_CMD from main unit ---
    if (len == 1 && data[0] == RESET_PAIRING_CMD) {
        Serial.print("[TRIGGER] Received from:  ");
        printMacAddress(mac);
        Serial.println(" - RESET_PAIRING command");
        
        // Set flag for channel scanning
        resetPairingReceived = true;
        
        // Clear pairing state
        configTrigger.setPaired(false);
        memset(configTrigger.get().mainUnitMac, 0xFF, 6);
        configTrigger.setChannel(0);
        configTrigger.save();
        
        // Clear runtime state
        memset(mainUnitMac, 0xFF, 6);
        updatePairingState(STATE_UNPAIRED);
        
        Serial.println("[TRIGGER] ✅ Pairing state reset - entering pairing mode");
        
        // Start fresh pairing (STATE 4 Part 2)
        startState1FreshPairing();
        return;
    }

    // --- SETTINGS PACKET from main unit (hybrid config) ---
    if (len == sizeof(EspNowSettingsPacket)) {
        const EspNowSettingsPacket* pkt = (const EspNowSettingsPacket*)data;
        if (pkt->magic == ESPNOW_SETTINGS_MAGIC) {
            Serial.print("[TRIGGER] Received from:  ");
            printMacAddress(mac);
            Serial.println(" - Configuration packet");
            
            // Set flag for channel scanning
            configResponseReceived = true;

            // Store config values
            triggerUnitID = pkt->unitID;
            unitID = triggerUnitID;
            strncpy(triggerName, pkt->name, ESPNOW_MAX_NAME-1);
            triggerName[ESPNOW_MAX_NAME-1] = 0;
            triggerRole = pkt->role;
            triggerLedEnabled = pkt->ledEnabled;
            triggerDisplayEnabled = pkt->displayEnabled;
            triggerDisplayMode = pkt->displayMode;
            triggerSensorType = pkt->sensorType;
            triggerDistanceThreshold = pkt->distanceThreshold;
            
            triggerDisplayOnConfigReceived(
             pkt->displayEnabled,
             pkt->displayMode,
             pkt->timezoneOffsetSeconds
            );

            // Save config and update state
            saveLocalTriggerConfig();
            Serial.println("[TRIGGER] Received new configuration from main unit:");
            printTriggerConfig();
            lastConfigPacket = *pkt;
            
            // Update LED journey based on previous state
            if (ledJourney == LED_JOURNEY_FRESH_PAIRING) {
                // Keep FRESH_PAIRING - will transition to PAIRED_SUCCESS on next pairing ACK
                Serial.println("[TRIGGER] LED Journey: Staying in FRESH_PAIRING");
            } else if (ledJourney == LED_JOURNEY_RECONNECTING) {
                ledJourney = LED_JOURNEY_RECONNECT_SUCCESS;
                Serial.println("[TRIGGER] LED Journey: RECONNECT_SUCCESS");
            }
            
            // Update state to fully paired and operational
            updatePairingState(STATE_PAIRED_OPERATIONAL);
            return;
        }
    }

    // --- CONFIG INVALID notification from main unit ---
    if (len == 1 && data[0] == CONFIG_INVALID_CMD) {
        Serial.print("[TRIGGER] Received from: ");
        printMacAddress(mac);
        Serial.println(" - CONFIG_INVALID_CMD");
        
        sendConfigRequest();
        return;
    }

    // --- PAIRING ACK from main unit ONLY ---
    if (len == 8) {
        uint32_t magic = *(uint32_t*)data;
        uint8_t ok = data[4];
        uint8_t mainUnitFlag = data[5];
        
        if (magic == ESPNOW_PAIR_MAGIC && ok == 1 && mainUnitFlag == MAIN_UNIT_IDENTIFIER) {
            Serial.print("[TRIGGER] Received from: ");
            printMacAddress(mac);
            Serial.println(" - Valid pairing confirmation");
            
            // Set flag for channel scanning
            pairingResponseReceived = true;
            
            // Store the main unit MAC and mark as paired
            memcpy(mainUnitMac, mac, 6);
            configTrigger.setPaired(true);
            memcpy(configTrigger.get().mainUnitMac, mainUnitMac, 6);
            configTrigger.save();

            // Exit unpair cooldown if active
            recentlyUnpaired = false;

            // Make sure peer is registered
            esp_now_peer_info_t mainPeer = {};
            memcpy(mainPeer.peer_addr, mainUnitMac, 6);
            mainPeer.channel = 0;
            mainPeer.encrypt = false;
            if (esp_now_is_peer_exist(mainUnitMac)) {
                esp_now_del_peer(mainUnitMac);
            }
            esp_err_t addPeerStatus = esp_now_add_peer(&mainPeer);
            
            if (addPeerStatus == ESP_OK) {
                Serial.print("[TRIGGER] ✅ Paired with main unit:  ");
                printMacAddress(mainUnitMac);
                Serial.println();
                
                ledJourney = LED_JOURNEY_PAIRED_SUCCESS;
                Serial.println("[TRIGGER] LED Journey: PAIRED_SUCCESS");
                
                // Request configuration after successful pairing
                updatePairingState(STATE_PAIRED_CONFIGURING);
                sendConfigRequest();
                commEspNowTriggerStopPairingWindow();
            } else {
                Serial.printf("[TRIGGER] Failed to add main unit peer: %d\n", addPeerStatus);
                updatePairingState(STATE_UNPAIRED);
            }
            return;
        }
    }
    
    // --- SENSOR SETUP DATA REQUEST ---
    if (len == 1 && data[0] == SENSOR_SETUP_DATA_CMD) {
        uint16_t distance = 0;
        uint8_t status = 255;
        triggerSensorDoSetupMeasurement(&distance, &status);
        commEspNowTriggerSendSensorSetupData(distance, status);
        return;
    }

    // --- TRIGGER EVENT ACK ---
    if (len >= 8 && data[0] == TRIGGER_EVENT_ACK_CMD) {
        uint8_t ackUnitID = data[1];
        uint8_t ackTriggerType = data[2];
        uint32_t ackTimestamp = *((uint32_t*)&data[4]);
        
        if (triggerEventPending && 
            ackUnitID == triggerUnitID && 
            ackTriggerType == pendingTriggerType &&
            ackTimestamp == pendingTriggerTimestamp) {
            
            triggerEventPending = false;
            Serial.println("[TRIGGER] ✅ Trigger event ACK received - no more retries needed");
        } else {
            Serial.println("[TRIGGER] ⚠️ Received ACK but it doesn't match pending event");
        }
        
        return;
    }
    
    // --- ENHANCED UNPAIR HANDLING (with magic) ---
    if (len >= 8) {
        uint32_t magic = *(uint32_t*)data;
        uint8_t cmd = data[4];
        if (magic == ESPNOW_PAIR_MAGIC && cmd == 0xD0) {
            Serial.print("[TRIGGER] Received from: ");
            printMacAddress(mac);
            Serial.println(" - UNPAIR command with magic");

            // Send acknowledgment
            uint8_t ack[1] = { UNPAIR_ACK_CMD };
            esp_now_send(mac, ack, 1);
            delay(10);

            // Update internal state
            updatePairingState(STATE_UNPAIRING);
            
            // Clear pairing data
            configTrigger.setPaired(false);
            memset(configTrigger.get().mainUnitMac, 0xFF, 6);
            configTrigger.save();
            memset(mainUnitMac, 0xFF, 6);

            // Remove ESP-NOW peer
            if (esp_now_is_peer_exist(mac)) {
                esp_now_del_peer(mac);
            }

            // Enter cooldown period
            recentlyUnpaired = true;
            unpairedTimestamp = millis();

            Serial.println("[TRIGGER] Unpaired and config cleared");
            commEspNowTriggerStopPairingWindow();
            updatePairingState(STATE_UNPAIRED);
            return;
        }
    }

    // --- LEGACY UNPAIR COMMAND HANDLING ---
    if (len == 1 && data[0] == UNPAIR_CMD) {
        Serial.print("[TRIGGER] Received from: ");
        printMacAddress(mac);
        Serial.println(" - Legacy UNPAIR command");

        // Send acknowledgment
        uint8_t ack[1] = { UNPAIR_ACK_CMD };
        esp_err_t ackResult = esp_now_send(mac, ack, 1);
        Serial.printf("[TRIGGER] UNPAIR_ACK sent, result: %d\n", ackResult);

        delay(10);

        // Update internal state
        updatePairingState(STATE_UNPAIRING);
        
        // Clear pairing data
        configTrigger. setPaired(false);
        memset(configTrigger.get().mainUnitMac, 0xFF, 6);
        configTrigger.save();
        memset(mainUnitMac, 0xFF, 6);

        // Remove ESP-NOW peer
        if (esp_now_is_peer_exist(mac)) {
            esp_now_del_peer(mac);
        }

        // Enter cooldown period
        recentlyUnpaired = true;
        unpairedTimestamp = millis();

        Serial.println("[TRIGGER] Unpaired and config cleared via legacy command");
        commEspNowTriggerStopPairingWindow();
        updatePairingState(STATE_UNPAIRED);
        return;
    }

    // Unknown packets are ignored silently
}

// --- Public functions ---
void commEspNowTriggerStartPairingWindow(unsigned long durationMs, bool manualOnly) {
    if (recentlyUnpaired && millis() - unpairedTimestamp < UNPAIR_COOLDOWN_MS) {
        Serial.println("[TRIGGER] Cannot start pairing - in cooldown period");
        return;
    }
    
    pairingMode = true;
    manualPairingMode = manualOnly;
    pairingWindowEnd = millis() + durationMs;
    Serial.printf("[TRIGGER] %s pairing window STARTED for %lu ms\n",
                 manualOnly ? "Manual" : "Auto", durationMs);
}


void triggerCommSendStateAck() {
    uint8_t ackPacket[8] = {0};
    ackPacket[0] = STATE_UPDATE_ACK_CMD;
    ackPacket[1] = triggerUnitID;
    ackPacket[2] = currentStopwatchState;
    
    esp_now_send(mainUnitMac, ackPacket, 8);
    Serial.printf("[TRIGGER] Sent state ACK: unitID=%d, state=%d\n", triggerUnitID, currentStopwatchState);
}

void commEspNowTriggerStopPairingWindow() {
    pairingMode = false;
    manualPairingMode = false;
    pairingWindowEnd = 0;
    Serial.println("[TRIGGER] Pairing window STOPPED");
}

void commEspNowTriggerManualPairingMode() {
    commEspNowTriggerInit();
    Serial.println("[TRIGGER] Entering MANUAL pairing mode (existing main unit only)");
    commEspNowTriggerStartPairingWindow(60000, true);
}

void commEspNowTriggerPairingMode() {
    commEspNowTriggerInit();
    configTrigger.setPaired(false);
    configTrigger.save();
    memset(mainUnitMac, 0xFF, 6);
    recentlyUnpaired = false;
    Serial.println("[TRIGGER] Entering PAIRING mode");
    commEspNowTriggerStartPairingWindow(60000, false);
    updatePairingState(STATE_UNPAIRED);
}

void commEspNowTriggerInit() {
    if (espNowInitDone) return;

    WiFi.mode(WIFI_STA);
    Serial.print("[TRIGGER] MAC Address: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("[TRIGGER] ❌ ERROR: Failed to initialize ESP-NOW");
        return;
    }
    Serial.println("[TRIGGER] ✅ ESP-NOW initialized successfully");

    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    // Register broadcast address for discovery
    uint8_t bcast[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, bcast, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (!esp_now_is_peer_exist(bcast)) {
        esp_err_t addPeerStatus = esp_now_add_peer(&peerInfo);
        Serial.printf("[TRIGGER] Added broadcast peer: %d\n", addPeerStatus);
    }

    // Load previous pairing state
    configTrigger.load();
    Serial.printf("[TRIGGER] Loaded pairing state: %s\n", 
                 configTrigger.isPaired() ? "PAIRED" : "NOT PAIRED");

    // Try to reconnect if previously paired
    if(configTrigger.isPaired()) {
        // Load saved data
        memcpy(mainUnitMac, configTrigger.get().mainUnitMac, 6);
        // IMPORTANT: Load unitID from saved config
        triggerUnitID = configTrigger.get().unitID;
        triggerRole = configTrigger.get().role;
        unitID = triggerUnitID; // Update global unitID
        
        Serial.print("[TRIGGER] ℹ️ Loaded main unit MAC: ");
        printMacAddress(mainUnitMac);
        Serial.println();
        
        // Show initial configuration from flash
        Serial.println("[TRIGGER] ℹ️ Loaded initial configuration from flash:");
        Serial.printf("[TRIGGER] ℹ️ Unit ID: %d, Role: %d\n", triggerUnitID, triggerRole);
        printTriggerConfig();

        // Set LED journey to RECONNECTING
        ledJourney = LED_JOURNEY_RECONNECTING;
        Serial.println("[TRIGGER] LED Journey: RECONNECTING");

        // Register main unit as peer
        esp_now_peer_info_t mainPeer = {};
        memcpy(mainPeer.peer_addr, mainUnitMac, 6);
        mainPeer.channel = 0;
        mainPeer.encrypt = false;
        if (!esp_now_is_peer_exist(mainUnitMac)) {
            esp_err_t addPeerStatus = esp_now_add_peer(&mainPeer);
            Serial.printf("[TRIGGER] Added main unit peer: %d\n", addPeerStatus);
        }

        // Enter operational state directly and request fresh config
        Serial.println("[TRIGGER] ✅ Entering operational state directly - already paired");
        updatePairingState(STATE_PAIRED_OPERATIONAL);
        
        // IMPORTANT: Always request fresh config after reconnecting
        Serial.println("[TRIGGER] 🔄 Requesting fresh configuration from main unit");
        sendConfigRequest();
        
        // DON'T send heartbeat yet - wait for reconnection confirmation
    } else {
        ledJourney = LED_JOURNEY_FRESH_PAIRING;
        Serial.println("[TRIGGER] LED Journey: FRESH_PAIRING");
    }

    espNowInitDone = true;
}

// Add this function to comm_espnow_trigger.cpp

// Modify to accept a timestamp parameter
void triggerCommSendTriggerEvent(uint32_t eventTimestamp) {
    if (isMacBroadcast(mainUnitMac)) {
        Serial.println("[TRIGGER][ERROR] Cannot send trigger - no main unit MAC");
        return;
    }
    
    // Determine trigger type based on current state and role
    uint8_t triggerType;
    
    // The type of trigger event is determined by the CURRENT STATE, not just the role
    if (currentStopwatchState == STOPWATCH_READY) {
        triggerType = 0; // START trigger
    } 
    else if (currentStopwatchState == STOPWATCH_RUNNING) {
        if (triggerRole == ROLE_SPLIT) {
            triggerType = 2; // SPLIT trigger
        } else {
            triggerType = 1; // STOP trigger
        }
    } 
    else {
        // This shouldn't happen if triggerBehaviorShouldTrigger() is working properly
        Serial.printf("[TRIGGER][WARNING] Unexpected state for trigger: %d\n", currentStopwatchState);
        triggerType = 0; // Default to START as a fallback
    }
    
    struct TriggerEventPacket {
        uint8_t cmd;         // 0xB1
        uint8_t unitID;      // Which trigger unit is sending the event
        uint8_t triggerType; // 0=START, 1=STOP, 2=SPLIT
        uint8_t reserved;    // For alignment
        uint32_t timestamp;  // Timestamp when triggered
    };
    
    TriggerEventPacket pkt = { 
        TRIGGER_EVENT_CMD, 
        triggerUnitID,
        triggerType,
        0, // reserved
        eventTimestamp // Use the timestamp provided from sensor detection
    };
    
    Serial.printf("[TRIGGER] 🚀 Sending TRIGGER EVENT to main unit (type: %d, timestamp: %lu)\n", 
                  triggerType, eventTimestamp);
    
    
    esp_err_t result = esp_now_send(mainUnitMac, (uint8_t*)&pkt, sizeof(pkt));
    
    if (result == ESP_OK) {
        // Track this event for potential retry
        triggerEventPending = true;
        pendingTriggerType = triggerType;
        pendingTriggerTimestamp = eventTimestamp;
        triggerEventSentTime = millis();
        triggerEventRetryCount = 0;
        
        Serial.println("[TRIGGER] ✅ Trigger event sent successfully, waiting for ACK");
    } else {
        Serial.printf("[TRIGGER][ERROR] Failed to send trigger event: %d\n", result);
    }
}

// Add this function to retry sending trigger events
void triggerCommRetryTriggerEvent() {
    if (!triggerEventPending) return;
    
    triggerEventRetryCount++;
    triggerEventSentTime = millis();
    
    struct TriggerEventPacket {
        uint8_t cmd;         // 0xB1
        uint8_t unitID;      // Which trigger unit is sending the event
        uint8_t triggerType; // 0=START, 1=STOP, 2=SPLIT
        uint8_t reserved;    // For alignment
        uint32_t timestamp;  // Timestamp when triggered
    };
    
    TriggerEventPacket pkt = { 
        TRIGGER_EVENT_CMD, 
        triggerUnitID,
        pendingTriggerType,
        0, // reserved
        pendingTriggerTimestamp 
    };
    
    esp_err_t result = esp_now_send(mainUnitMac, (uint8_t*)&pkt, sizeof(pkt));
    
    Serial.printf("[TRIGGER] 🔄 Trigger event RETRY: type=%d (attempt %d/%d)\n", 
                  pendingTriggerType, triggerEventRetryCount, MAX_TRIGGER_RETRIES);
    
    if (result != ESP_OK) {
        Serial.printf("[TRIGGER][ERROR] Failed to send retry: %d\n", result);
    }
}

void commEspNowTriggerLoop() {
    if (!espNowInitDone) return;
    
    unsigned long now = millis();

    // Handle cooldown after unpair
    if (recentlyUnpaired) {
        if (now - unpairedTimestamp < UNPAIR_COOLDOWN_MS) {
            return;
        } else {
            recentlyUnpaired = false;
            Serial.println("[TRIGGER] Unpair cooldown expired");
        }
    }
    
    // Handle trigger event retries
    if (triggerEventPending) {
        unsigned long timeSinceLastSend = millis() - triggerEventSentTime;
        
        if (timeSinceLastSend > TRIGGER_RETRY_INTERVAL) {
            if (triggerEventRetryCount < MAX_TRIGGER_RETRIES) {
                triggerCommRetryTriggerEvent();
            } else {
                Serial.printf("[TRIGGER] ❌ Trigger event failed after %d retries, giving up\n", 
                              MAX_TRIGGER_RETRIES);
                triggerEventPending = false; // Give up
            }
        }
    }
    
    // Pairing window management
    if (pairingMode && now > pairingWindowEnd) {
        commEspNowTriggerStopPairingWindow();
    }

    // === STATE MACHINE IMPLEMENTATION ===
    
    // STATE 1: Fresh pairing (never paired before)
    if (pairingState == STATE_UNPAIRED && inExtendedPairingWindow && reconnectionPhase == 0) {
        unsigned long elapsed = now - extendedWindowStartTime;
        unsigned long remaining = STATE1_PAIRING_WINDOW_MS - elapsed;
        
        // Check if 2-minute window expired
        if (elapsed >= STATE1_PAIRING_WINDOW_MS) {
            Serial.println("[TRIGGER] ❌ STATE 1: Pairing timeout - NO MAIN FOUND");
            ledJourney = LED_JOURNEY_TIMEOUT;
            pairingTimeoutOccurred = true;
            inExtendedPairingWindow = false;
            channelScanActive = false;
            updatePairingState(STATE_UNPAIRED);
            return;
        }
        
        // Scan channels every 5 seconds
        if (now - channelScanStartTime >= STATE1_SCAN_INTERVAL_MS) {
            scanAttemptNumber++;
            Serial.printf("[TRIGGER] STATE 1: Scan attempt %d (Time remaining: %lu seconds)\n", 
                         scanAttemptNumber, remaining / 1000);
            scanChannelsWithPairingHello();
            channelScanStartTime = now;
            
            // If we got a pairing response, stop scanning
            if (pairingResponseReceived) {
                Serial.println("[TRIGGER] ✅ Pairing successful, stopping scan");
                inExtendedPairingWindow = false;
                channelScanActive = false;
            }
        }
        return;
    }
    
    // STATE 2: Previously paired reconnection
    if (pairingState == STATE_PAIRED_OPERATIONAL && inExtendedPairingWindow && reconnectionPhase > 0) {
        unsigned long elapsed = now - extendedWindowStartTime;
        
        // Phase 1: Quick retry on saved channel (0-5 seconds)
        if (reconnectionPhase == 1 && elapsed < STATE2_QUICK_RETRY_DURATION_MS) {
            static unsigned long lastRetry = 0;
            if (now - lastRetry >= STATE2_RETRY_INTERVAL_MS) {
                Serial.printf("[TRIGGER] STATE 2 Phase 1: Quick retry (%.1fs)\n", elapsed / 1000.0);
                sendConfigRequest();
                // REMOVED: sendHeartbeat(); // Don't send until reconnected
                lastRetry = now;
                
                // Check if we got a response
                if (configResponseReceived) {
                    Serial.println("[TRIGGER] ✅ Reconnected on saved channel");
                    inExtendedPairingWindow = false;
                    reconnectionPhase = 0;
                    sendHeartbeat(); // First heartbeat AFTER reconnection confirmed
                }
            }
            return;
        }
        
        // Transition to Phase 2
        if (reconnectionPhase == 1 && elapsed >= STATE2_QUICK_RETRY_DURATION_MS) {
            Serial.println("[TRIGGER] STATE 2: Transitioning to Phase 2 (channel scanning)");
            reconnectionPhase = 2;
            channelScanStartTime = now;
        }
        
        // Phase 2: Channel scanning (5s-2:05)
        if (reconnectionPhase == 2) {
            // Check if total window expired
            if (elapsed >= STATE2_TOTAL_WINDOW_MS) {
                if (resetPairingReceived) {
                    Serial.println("[TRIGGER] STATE 2: RESET_PAIRING received, handled separately");
                } else {
                    Serial.println("[TRIGGER] ❌ STATE 2: Reconnection timeout - MAIN OFFLINE");
                    ledJourney = LED_JOURNEY_TIMEOUT;
                    pairingTimeoutOccurred = true;
                    updatePairingState(STATE_UNPAIRED);
                }
                inExtendedPairingWindow = false;
                reconnectionPhase = 0;
                channelScanActive = false;
                return;
            }
            
            // Scan channels every 10 seconds
            if (now - channelScanStartTime >= STATE2_SCAN_INTERVAL_MS) {
                scanAttemptNumber++;
                unsigned long remaining = STATE2_TOTAL_WINDOW_MS - elapsed;
                Serial.printf("[TRIGGER] STATE 2 Phase 2: Scan attempt %d (Time remaining: %lu seconds)\n", 
                             scanAttemptNumber, remaining / 1000);
                scanChannelsWithConfigRequest();
                channelScanStartTime = now;
                
                // If we got a config response or reset, stop scanning
                if (configResponseReceived) {
                    Serial.println("[TRIGGER] ✅ Reconnected, stopping scan");
                    inExtendedPairingWindow = false;
                    reconnectionPhase = 0;
                    channelScanActive = false;
                }
                
                if (resetPairingReceived) {
                    // Already handled in OnDataRecv callback
                    inExtendedPairingWindow = false;
                    reconnectionPhase = 0;
                    channelScanActive = false;
                }
            }
            return;
        }
    }
    
    // Normal state machine for already paired and operational
    switch (pairingState) {
        case STATE_UNPAIRED:
            // Attempt pairing if in pairing mode
            if (pairingMode && now - lastPairingSend > PAIRING_RETRY_INTERVAL_MS) {
                sendPairingRequest();
            }
            break;
            
        case STATE_PAIRING_REQUESTED:
            // Retry pairing if no response received
            if (now - lastPairingSend > PAIRING_RETRY_INTERVAL_MS && pairingMode) {
                Serial.println("[TRIGGER] Pairing request timed out, retrying");
                sendPairingRequest();
            }
            break;
            
        case STATE_PAIRED_CONFIGURING:
            // Request config if not received
            if (now - lastConfigRequest > CONFIG_REQUEST_INTERVAL_MS) {
                Serial.println("[TRIGGER] No config received, requesting again");
                sendConfigRequest();
            }
            break;
            
        case STATE_PAIRED_OPERATIONAL:
            // Send regular heartbeats (only if not in extended window)
            if (!inExtendedPairingWindow && 
                !pairingTimeoutOccurred &&
                now - lastHeartbeatSend > HEARTBEAT_INTERVAL_MS) {
                sendHeartbeat();
            }
            break;
            
        case STATE_UNPAIRING:
            // Will be handled by the unpair callback
            break;
    }
}

bool commEspNowTriggerIsPaired() {
    return (pairingState == STATE_PAIRED_OPERATIONAL || 
            pairingState == STATE_PAIRED_CONFIGURING);
}

void commEspNowTriggerUnpair() {
    Serial.println("[TRIGGER] Manual unpair initiated");
    
    // Clear pairing data
    memset(mainUnitMac, 0xFF, 6);
    configTrigger.setPaired(false);
    memset(configTrigger.get().mainUnitMac, 0xFF, 6);
    configTrigger.save();
    
    // Exit the unpair cooldown if active
    recentlyUnpaired = false;
    
    // Update state
    updatePairingState(STATE_UNPAIRED);
    
    Serial.println("[TRIGGER] Unpairing complete");
}

bool triggerCommShouldActOnState(uint8_t newState) {
    switch (triggerRole) {
        case 0: // Start trigger
            return (currentStopwatchState == STOPWATCH_READY && newState == STOPWATCH_RUNNING);
            
        case 1: // Stop trigger  
            return (currentStopwatchState == STOPWATCH_RUNNING && newState == STOPWATCH_STOPPED);
            
        case 2: // Both (start/stop)
            return ((currentStopwatchState == STOPWATCH_READY && newState == STOPWATCH_RUNNING) ||
                    (currentStopwatchState == STOPWATCH_RUNNING && newState == STOPWATCH_STOPPED));
            
        default:
            return false;
    }
}

void triggerCommInit() {
    // This functionality is handled by commEspNowTriggerInit()
    Serial.println("[TRIGGER] triggerCommInit() called - redirecting to main init");
    commEspNowTriggerInit();
}

// Get RSSI from main unit (signal strength)
int8_t commEspNowTriggerGetMainRSSI() {
    return mainUnitRSSI;
}

void triggerCommLoop() {
    // This functionality is handled by commEspNowTriggerLoop()
    commEspNowTriggerLoop();
}

void commEspNowTriggerGetMainUnitMac(uint8_t* macOut) {
    memcpy(macOut, mainUnitMac, 6);
}

const EspNowSettingsPacket* commEspNowTriggerGetLastConfigPacket() {
    if (lastConfigPacket.magic == ESPNOW_SETTINGS_MAGIC) {
        return &lastConfigPacket;
    } else {
        return nullptr;
    }
}

// Set state update callback
void commEspNowTriggerSetStateCallback(void (*callback)(uint8_t)) {
    stateUpdateCallback = callback;
    Serial.println("[TRIGGER] State update callback registered");
}

uint8_t commEspNowTriggerGetLedJourney() {
    return (uint8_t)ledJourney;
}

void commEspNowTriggerSetLedOperational() {
    if (ledJourney == LED_JOURNEY_PAIRED_SUCCESS || 
        ledJourney == LED_JOURNEY_RECONNECT_SUCCESS) {
        ledJourney = LED_JOURNEY_OPERATIONAL;
        Serial.println("[TRIGGER] LED Journey: OPERATIONAL");
    }
}

bool commEspNowTriggerIsPairingWindowActive() {
    return inExtendedPairingWindow || pairingMode;
}
