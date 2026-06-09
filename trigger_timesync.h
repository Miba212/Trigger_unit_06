#pragma once

#include "espnow_timesync.h" // For EspNowTimeSyncPacket

void triggerTimeSyncOnPacket(const EspNowTimeSyncPacket* pkt);
uint64_t triggerTimeSyncGetUnixMs(); // <-- updated to uint64_t
bool triggerTimeSyncIsRealTime();
void triggerTimeSyncSetTimerActive(bool active);
