#include <Arduino.h>
#include "espnow_timesync.h"
#include "trigger_timesync.h"

// === Trigger unit time sync state ===
bool isTimerActive = false;
static uint64_t triggerUnixBaseMs = 1000 * 1000;   // Last received unix time (ms)
static uint32_t triggerBaseMillis = 0;             // millis() when unix base was set
static bool triggerHasRealTime = false;

// Called when a time sync packet is received
void triggerTimeSyncOnPacket(const EspNowTimeSyncPacket* pkt) {
    if (pkt->magic == ESPNOW_TIME_SYNC_MAGIC && pkt->cmd == TIME_SYNC_CMD) {
        if (!isTimerActive) {
            triggerUnixBaseMs = pkt->masterTimeMs;
            triggerBaseMillis = millis();
            triggerHasRealTime = (pkt->isRealTime != 0);
            Serial.printf("[TRIGGER][TimeSync] Time sync RECEIVED! Synced ms: %llu (real=%d)\n",
                triggerTimeSyncGetUnixMs(),
                triggerTimeSyncIsRealTime()
            );
        } else {
            Serial.println("[TRIGGER][TimeSync] Time sync IGNORED: timer/race is active");
        }
    }
}

// Returns current unix ms time for this trigger (real or fake)
uint64_t triggerTimeSyncGetUnixMs() {
    uint32_t millisElapsed = millis() - triggerBaseMillis;
    return triggerUnixBaseMs + millisElapsed;
}

bool triggerTimeSyncIsRealTime() {
    return triggerHasRealTime;
}

void triggerTimeSyncSetTimerActive(bool active) {
    isTimerActive = active;
}
