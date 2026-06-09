#pragma once
#include <Arduino.h>

// Error types enum
enum ErrorType {
    ERROR_NONE = 0,
    ERROR_HEARTBEAT_LOST,
    ERROR_SENSOR_FAILURE,
    ERROR_ESPNOW_FAILED,
    ERROR_GENERAL = 255
};

// Function declarations
void triggerLedInit();
void triggerLedUpdate();
void triggerLedSetPairingState(uint8_t state, bool pairingModeActive);
void triggerLedSetStopwatchState(uint8_t stopwatchState);
void triggerLedSetEnabled(bool enabled);
void triggerLedNotifyPairingSuccess();
void triggerLedNotifyReconnectSuccess();
void triggerLedSetError(ErrorType error);
void triggerLedClearError();
