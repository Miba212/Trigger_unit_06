#include "trigger_state.h"
#include "trigger_behavior.h"

static uint8_t currentState = 0; // STOPWATCH_IDLE

void triggerStateInit() {
    currentState = 0; // STOPWATCH_IDLE
}

uint8_t triggerStateGetCurrent() {
    return currentState;
}

void triggerStateSet(uint8_t state) {
    currentState = state;
}

const char* triggerStateGetString() {
    switch(currentState) {
        case 0: return "IDLE";
        case 1: return "READY"; 
        case 2: return "RUNNING";
        case 3: return "STOPPED";
        case 4: return "ERROR";
        default: return "UNKNOWN";
    }
}

bool triggerStateIsDetecting() {
    return triggerBehaviorGetMode() == BEHAVIOR_DETECTING;
}
