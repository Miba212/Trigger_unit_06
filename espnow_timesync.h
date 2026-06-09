#pragma once
#include <stdint.h>

#define ESPNOW_TIME_SYNC_MAGIC 0xABCDEF01
#define TIME_SYNC_CMD 0xE1

struct EspNowTimeSyncPacket {
    uint32_t magic;        // 0xABCDEF01
    uint8_t  cmd;          // 0xE1
    uint8_t  reserved[2];  // for future use
    uint8_t  isRealTime;   // 1 = real Unix, 0 = fake
    uint64_t masterTimeMs; // Unix time in ms (NOT seconds!)
};
