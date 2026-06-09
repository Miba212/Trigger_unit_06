#pragma once
#include <Arduino.h>

#define ESPNOW_PAIR_MAGIC   0x53545750 // 'STWP'
#define ESPNOW_SETTINGS_MAGIC 0xAABBCCDD
#define ESPNOW_MAX_NAME 32
#define CONFIG_INVALID_CMD 0xF0
#define RESET_PAIRING_CMD 0xF1  // NEW: Add reset pairing command
#define STATE_UPDATE_CMD 0xE6
#define STATE_UPDATE_ACK_CMD 0xE7
#define TRIGGER_EVENT_CMD 0xB1
#define TRIGGER_EVENT_ACK_CMD 0xB2
#define SENSOR_SETUP_DATA_CMD 0xC2 // New command for sensor setup data  
#define RESULT_PACKET_CMD       0xC3
#define COUNTDOWN_START_CMD     0xC4
#define COUNTDOWN_FINISH_CMD    0xC5
#define COUNTDOWN_CANCEL_CMD    0xC6   // <<< ADD THIS

#define PAIRING_ACK_CMD     0xA1
#define DATA_CMD            0xB0
#define CONFIG_REQ_CMD      0xC1
#define UNPAIR_CMD          0xD0
#define UNPAIR_ACK_CMD      0xD1

#define ROLE_START 0
#define ROLE_STOP 1  
#define ROLE_BOTH 2
#define ROLE_DISPLAY_ONLY 3
#define ROLE_SPLIT 4

// Remove duplicate definitions - define only once
#ifndef STOPWATCH_STATES_DEFINED
#define STOPWATCH_STATES_DEFINED
#define STOPWATCH_IDLE 0
#define STOPWATCH_READY 1
#define STOPWATCH_RUNNING 2
#define STOPWATCH_STOPPED 3
#define STOPWATCH_ERROR 4
#endif

// CHANGE: Use extern declarations in header (not definitions)
extern uint8_t currentStopwatchState;
extern uint8_t triggerRole;  // 0=Start, 1=Stop, 2=Both
extern uint8_t unitID;

// State update packet structure
struct StateUpdatePacket {
    uint8_t cmd;              // STATE_UPDATE_CMD (0xE6)
    uint8_t stopwatchState;   // Current stopwatch state
    uint8_t reserved[6];      // For future use
};

struct SensorSetupDataPacket {
    uint8_t cmd;           // SENSOR_SETUP_DATA_CMD (0xC2)
    uint8_t unitID;
    uint8_t sensorType;
    uint16_t measuredDistance; // in mm
    uint8_t rangeStatus;
};
void commEspNowTriggerSendSensorSetupData(uint16_t measuredDistance, uint8_t rangeStatus);

struct TriggerEventPacket {
    uint8_t cmd;         // 0xB1
    uint8_t unitID;      // Which trigger unit is sending the event
    uint8_t triggerType; // 0=START, 1=STOP, 2=SPLIT
    uint8_t reserved;    // For alignment
    uint32_t timestamp;  // Timestamp when triggered
};

struct EspNowSettingsPacket {
    uint32_t magic;
    uint8_t unitID;
    char name[ESPNOW_MAX_NAME];
    uint8_t role;
    uint8_t ledEnabled;
    uint8_t displayEnabled;
    uint8_t displayMode;
    uint8_t sensorType;
    float distanceThreshold;
    int32_t timezoneOffsetSeconds;
};
struct ResultPacket {
    uint8_t cmd;           // RESULT_PACKET_CMD (0xC3)
    uint8_t reserved;
    uint16_t padding;
    float resultSeconds;
};

struct CountdownStartPacket {
    uint8_t cmd;              // COUNTDOWN_START_CMD (0xC4)
    uint8_t reserved;
    uint16_t durationSeconds;
    uint32_t padding;
};
struct CountdownCancelPacket {          // <-- ADD THIS
    uint8_t cmd;        // COUNTDOWN_CANCEL_CMD (0xC6)
    uint8_t reserved[7];
};
struct CountdownFinishPacket {
    uint8_t cmd;        // COUNTDOWN_FINISH_CMD (0xC5)
    uint8_t reserved[7];
};
void triggerCommSendTriggerEvent();
void commEspNowTriggerInit();
void commEspNowTriggerPairingMode();
void commEspNowTriggerManualPairingMode(); // <-- NEW: manual pairing mode
void commEspNowTriggerLoop();
bool commEspNowTriggerIsPaired();
void commEspNowTriggerUnpair();
void commEspNowTriggerGetMainUnitMac(uint8_t* macOut);
//void triggerCommSendTriggerEvent(uint32_t eventTimestamp);
void triggerCommSendTriggerEvent(uint32_t eventTimestamp);
const EspNowSettingsPacket* commEspNowTriggerGetLastConfigPacket();

// NEW: start/stop pairing window
void commEspNowTriggerStartPairingWindow(unsigned long durationMs, bool manualOnly);
void commEspNowTriggerStopPairingWindow();

// NEW: Channel scanning functions
void startState1FreshPairing();
void startState2Reconnection();

void triggerCommInit();
void triggerCommSendStateAck();
void triggerCommLoop();
bool triggerCommShouldActOnState(uint8_t newState);

void commEspNowTriggerSetStateCallback(void (*callback)(uint8_t));

// Get RSSI from main unit (signal strength)
int8_t commEspNowTriggerGetMainRSSI();

// LED journey state tracking
enum LedJourneyState {
    LED_JOURNEY_FRESH_PAIRING = 0,      // Never paired, searching for main
    LED_JOURNEY_RECONNECTING = 1,        // Previously paired, reconnecting
    LED_JOURNEY_PAIRED_SUCCESS = 2,      // Just paired (first time) - 60s hold
    LED_JOURNEY_RECONNECT_SUCCESS = 3,   // Just reconnected - 60s hold
    LED_JOURNEY_TIMEOUT = 4,             // Pairing/reconnect timeout (permanent)
    LED_JOURNEY_OPERATIONAL = 5          // Fully operational (show system state)
};

// Get LED journey state for indication (read-only)
uint8_t commEspNowTriggerGetLedJourney();
void commEspNowTriggerSetLedOperational();

// Check if pairing window is currently active
bool commEspNowTriggerIsPairingWindowActive();
