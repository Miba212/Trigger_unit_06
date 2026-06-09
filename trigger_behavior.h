#pragma once
#include <stdint.h>
#include "comm_espnow_trigger.h"

// Role definitions (must match main unit)
#define ROLE_START 0
#define ROLE_STOP 1  
#define ROLE_BOTH 2
#define ROLE_DISPLAY_ONLY 3
#define ROLE_SPLIT 4

// State definitions (must match main unit)
#define STOPWATCH_IDLE 0
#define STOPWATCH_READY 1
#define STOPWATCH_RUNNING 2
#define STOPWATCH_STOPPED 3
#define STOPWATCH_ERROR 4

// Behavior modes
enum TriggerBehaviorMode {
    BEHAVIOR_INACTIVE = 0,
    BEHAVIOR_DETECTING = 1,
    BEHAVIOR_TRIGGERED = 2,
    BEHAVIOR_WAITING = 3
};

void triggerBehaviorInit();
void triggerBehaviorUpdate();
void triggerBehaviorHandleStateChange(uint8_t oldState, uint8_t newState, const EspNowSettingsPacket* config);
void triggerBehaviorSetRole(uint8_t role);
bool triggerBehaviorShouldTrigger(uint8_t currentState, uint8_t role);
TriggerBehaviorMode triggerBehaviorGetMode();
