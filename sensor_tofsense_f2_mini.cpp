#include "sensor_tofsense_f2_mini.h"
#include "trigger_timesync.h"

// ==== Debug Configuration ====
#ifndef F2_MINI_DEBUG
#define F2_MINI_DEBUG 0  // Set to 1 for verbose debugging
#endif

#define F2_MINI_DEBUG_RAW_BYTES 0  // Set to 1 to see all incoming bytes

// ==== HARDCODED Hardware Configuration ====
#define F2_MINI_RX_PIN      16      // ESP32 RX <- Sensor TX (green wire)
#define F2_MINI_TX_PIN      17      // ESP32 TX -> Sensor RX (white wire)
#define F2_MINI_BAUD        921600
#define F2_MINI_SERIAL_PORT 1       // HardwareSerial(1)

// ==== HARDCODED Sensor Parameters ====
static float    F2_THRESHOLD_M        = 1.0f;    // Default (updated by setThreshold)
static uint16_t F2_MIN_SIGNAL         = 40;      // Min signal quality 0-100
static uint16_t F2_LOW_THRESHOLD_MM   = 100;     // Ignore readings below this (10cm)
static uint16_t F2_REFRESH_RATE       = 50;      // Hz (sensor native rate)
static uint16_t F2_BAND_WIDTH_MM      = 7000;   // Max range 7m

// ==== Frame Protocol Constants ====
#define FRAME_HEADER        0x57
#define FUNC_DATA           0x00
#define FUNC_SETTING        0x04
#define DATA_FRAME_LEN      16
#define SETTING_FRAME_LEN   32

// ==== Internal State ====
static HardwareSerial* sensorSerial = nullptr;
static TriggerCallback triggerCallback = nullptr;

static bool     sensorActive = false;
static bool     sensorInitialized = false;
static float    lastReading = 0.0f;
static uint32_t lastTimestamp = 0;
static uint16_t lastSignal = 0;
static uint8_t  lastStatus = 0xFF;

static bool     validTriggerOccurred = false;
static uint32_t triggerTimestampMs = 0;

// Debug counters
static uint32_t goodFrames = 0;
static uint32_t badFrames = 0;
static uint32_t triggerCount = 0;
static uint32_t totalBytesRead = 0;

// ==== Frame Parser State ====
static uint8_t  frameBuffer[DATA_FRAME_LEN];
static uint8_t  framePos = 0;
static bool     frameSynced = false;
static uint8_t  frameFuncMark = 0x00;
static uint8_t  frameExpectedLen = DATA_FRAME_LEN;

// ==== Averaging for stability ====
#define AVG_WINDOW 6
static uint16_t signalBuf[AVG_WINDOW];
static uint8_t  avgIdx = 0;
static uint8_t  avgCount = 0;

// ==== Confirmation logic (prevent false triggers) ====
static const uint8_t F2_CONFIRM_READS = 2;
static const unsigned long F2_CONFIRM_WINDOW_MS = 60;
static bool     confirmPending = false;
static uint32_t confirmFirstTsMs = 0;
static unsigned long confirmFirstMillis = 0;
static uint8_t  confirmValidCount = 0;

// ============================================================
//  Utilities
// ============================================================
static void updateSignalAverage(uint16_t sig, uint16_t* outAvgSig) {
    signalBuf[avgIdx] = sig;
    avgIdx = (avgIdx + 1) % AVG_WINDOW;
    if (avgCount < AVG_WINDOW) ++avgCount;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < avgCount; ++i) {
        sum += signalBuf[i];
    }
    *outAvgSig = (uint16_t)(sum / avgCount);
}

static int32_t parseInt24(uint8_t b0, uint8_t b1, uint8_t b2) {
    int32_t v = (int32_t)b0 | ((int32_t)b1 << 8) | ((int32_t)b2 << 16);
    if (v & 0x800000) v |= 0xFF000000;  // Sign extend
    return v;
}

// ============================================================
//  Build and send 0x57 0x04 setting frame
// ============================================================
static void sendSettingFrame(uint32_t systemTime) {
    const uint8_t SLEN = SETTING_FRAME_LEN;
    uint8_t f[SLEN];
    memset(f, 0xFF, SLEN);

    f[0]  = FRAME_HEADER;   // 0x57
    f[1]  = FUNC_SETTING;   // 0x04
    f[2]  = 0x00;           // mix: write
    f[3]  = 0xFF;           // reserved
    f[4]  = 0x00;           // id = 0

    // System time (inject ESP32 millis for sync)
    f[5]  = (systemTime >>  0) & 0xFF;
    f[6]  = (systemTime >>  8) & 0xFF;
    f[7]  = (systemTime >> 16) & 0xFF;
    f[8]  = (systemTime >> 24) & 0xFF;

    f[9]  = 0x00;   // mode: active, short range, UART
    f[10] = 0xFF;   // reserved
    f[11] = 0xFF;   // reserved

    // uart_baudrate: 921600 = 0x0E1000
    f[12] = 0x00;
    f[13] = 0x10;
    f[14] = 0x0E;

    f[15] = 0xFF; f[16] = 0xFF; f[17] = 0xFF; f[18] = 0xFF; // reserved

    // band_start = 0 mm
    f[19] = 0x00;
    f[20] = 0x00;

    // band_width
    f[21] = (F2_BAND_WIDTH_MM >> 0) & 0xFF;
    f[22] = (F2_BAND_WIDTH_MM >> 8) & 0xFF;

    f[23] = 0xFF;   // reserved

    // refresh_rate
    f[24] = (F2_REFRESH_RATE >> 0) & 0xFF;
    f[25] = (F2_REFRESH_RATE >> 8) & 0xFF;

    // filter_factor = 5 (default from manual)
    f[26] = 0x05;

    f[27] = 0xFF; f[28] = 0xFF; f[29] = 0xFF; f[30] = 0xFF; // reserved

    // Checksum
    uint8_t sum = 0;
    for (uint8_t i = 0; i < SLEN - 1; i++) sum += f[i];
    f[31] = sum;

    sensorSerial->write(f, SLEN);
    sensorSerial->flush();
}

// ============================================================
//  Parse Data Frame (0x57 0x00) - PRODUCTION VERSION
// ============================================================
static void parseDataFrame(const uint8_t* f) {
    // Validate checksum
    uint8_t sum = 0;
    for (uint8_t i = 0; i < DATA_FRAME_LEN - 1; i++) sum += f[i];
    if (sum != f[DATA_FRAME_LEN - 1]) {
        badFrames++;
        return;  // Silent fail
    }

    goodFrames++;

    // Extract fields
    uint32_t sensorTime = (uint32_t)f[4] | ((uint32_t)f[5] << 8)
                        | ((uint32_t)f[6] << 16) | ((uint32_t)f[7] << 24);
    int32_t  distMM     = parseInt24(f[8], f[9], f[10]);
    uint8_t  status     = f[11];
    uint16_t signal     = (uint16_t)f[12] | ((uint16_t)f[13] << 8);

    // Update state
    lastReading   = distMM / 1000.0f;
    lastTimestamp = sensorTime;
    lastSignal    = signal;
    lastStatus    = status;

    // Valid measurement detection
    // Status 0x00 + signal=0 + distance=0 = Sunlight saturation / invalid
    // Status 0x01 + signal>0 + distance>0 = Valid measurement
    const bool validMeasurement = (status == 0x01 && signal > 0 && distMM > 0);
    
    if (!sensorActive) return;

    // Handle invalid measurements (sunlight/no target) - SILENT
    if (!validMeasurement) {
        if (confirmPending) {
            confirmPending = false;
            confirmValidCount = 0;
        }
        return;
    }

    // *** From here on: VALID measurement ***
    
    // Update signal averaging
    uint16_t avgSignal = 0;
    updateSignalAverage(signal, &avgSignal);

    // Detection logic
    const uint16_t lowMM  = F2_LOW_THRESHOLD_MM;
    const uint16_t highMM = (uint16_t)(F2_THRESHOLD_M * 1000.0f);
    const bool inWindow = (distMM >= lowMM && distMM <= highMM);

    // Quality check
    bool qualityOk = (avgSignal >= F2_MIN_SIGNAL);
    
    // Distance validation
    const bool tooShort = (distMM < lowMM);
    const bool shortAllowed = !tooShort || (avgSignal >= (F2_MIN_SIGNAL * 2));

    const bool sampleValid = inWindow && qualityOk && shortAllowed;

    // Confirmation logic
    unsigned long nowMs = millis();

    if (!confirmPending) {
        if (sampleValid) {
            confirmPending = true;
            confirmFirstTsMs = triggerTimeSyncGetUnixMs();
            confirmFirstMillis = nowMs;
            confirmValidCount = 1;
            
            // *** ALWAYS PRINT: Trigger pending ***
            Serial.printf("[F2_MINI] 🎯 PENDING | D:%4dmm (%.2fm) | Sig:%3d (avg:%3d)\n", 
                          distMM, lastReading, signal, avgSignal);
        }
    } else {
        if (sampleValid) {
            confirmValidCount++;
        } else {
            confirmPending = false;
            confirmValidCount = 0;
            
            // *** ALWAYS PRINT: Trigger canceled ***
            Serial.println("[F2_MINI] ⏸️ CANCELED (object moved)");
            return;
        }

        if (confirmValidCount >= F2_CONFIRM_READS) {
            // *** TRIGGER CONFIRMED! ALWAYS PRINT ***
            validTriggerOccurred = true;
            triggerTimestampMs = (uint32_t)confirmFirstTsMs;

            if (triggerCallback) {
                triggerCallback(triggerTimestampMs);
            }

            triggerCount++;
            
            // *** ALWAYS PRINT: Confirmed trigger with details ***
            Serial.printf("[F2_MINI] ✅ TRIGGER #%lu | D:%4dmm (%.2fm) | Sig:%3d (avg:%3d) | TS:%lu\n",
                          triggerCount, distMM, lastReading, signal, avgSignal, triggerTimestampMs);

            confirmPending = false;
            confirmValidCount = 0;
        } else if (nowMs - confirmFirstMillis > F2_CONFIRM_WINDOW_MS) {
            confirmPending = false;
            confirmValidCount = 0;
            
            // *** ALWAYS PRINT: Timeout ***
            Serial.println("[F2_MINI] ⏱️ TIMEOUT (not enough valid reads)");
        }
    }

    // Release logic
    if (validTriggerOccurred) {
        if (!validMeasurement || lastReading < 0.08f || lastReading > (F2_THRESHOLD_M + 0.2f)) {
            validTriggerOccurred = false;
            
            // *** ALWAYS PRINT: Trigger released ***
            Serial.println("[F2_MINI] 🔓 RELEASED (object left zone)");
        }
    }
}

// ============================================================
//  Byte-by-byte parser - SILENT
// ============================================================
static void processByte(uint8_t b) {
    if (!frameSynced) {
        if (framePos == 0) {
            if (b == FRAME_HEADER) {
                frameBuffer[framePos++] = b;
            }
        } else {
            frameBuffer[framePos++] = b;
            frameFuncMark = b;
            frameExpectedLen = (b == FUNC_DATA) ? DATA_FRAME_LEN : SETTING_FRAME_LEN;
            frameSynced = true;
        }
        return;
    }

    frameBuffer[framePos++] = b;

    if (framePos >= frameExpectedLen) {
        if (frameFuncMark == FUNC_DATA) {
            parseDataFrame(frameBuffer);
        }
        framePos = 0;
        frameSynced = false;
    }

    if (framePos >= DATA_FRAME_LEN) {
        framePos = 0;
        frameSynced = false;
    }
}

// ============================================================
//  Public API Implementation
// ============================================================

void sensorF2MiniInit() {
    Serial.println("[F2_MINI] Initializing...");
    
    if (sensorSerial == nullptr) {
        sensorSerial = new HardwareSerial(F2_MINI_SERIAL_PORT);
    }

    sensorSerial->begin(F2_MINI_BAUD, SERIAL_8N1, F2_MINI_RX_PIN, F2_MINI_TX_PIN);
    delay(100);

    // Clear buffer
    while (sensorSerial->available()) {
        sensorSerial->read();
    }

    sendSettingFrame(millis());
    delay(200);

    sensorInitialized = true;
    
    Serial.printf("[F2_MINI] ✅ Ready | Threshold:%.2fm | MinSig:%d\n",
                  F2_THRESHOLD_M, F2_MIN_SIGNAL);
}

void sensorF2MiniUpdate() {
    if (!sensorInitialized) return;

    // Process incoming bytes (SILENT)
    while (sensorSerial->available()) {
        uint8_t b = (uint8_t)sensorSerial->read();
        processByte(b);
        totalBytesRead++;
    }

    // Periodic stats (every 5 minutes)
    static unsigned long lastStats = 0;
    if (millis() - lastStats >= 300000) {  // 5 minutes
        lastStats = millis();
        Serial.printf("[F2_MINI] Stats | Frames:%lu | Triggers:%lu | Bytes:%lu\n",
                      goodFrames, triggerCount, totalBytesRead);
    }
}

bool sensorF2MiniDoSetupMeasurement(uint16_t* distanceOut, uint8_t* statusOut) {
    if (!sensorInitialized) {
        sensorF2MiniInit();
    }

    Serial.println("[F2_MINI] 🔬 Setup measurement starting...");

    // Temporarily activate sensor to get fresh readings
    bool wasActive = sensorActive;
    sensorActive = true;
    
    // Clear old state
    lastReading = 0.0f;
    lastSignal = 0;
    lastStatus = 0xFF;
    
    // Reset averaging buffer for fresh data
    avgCount = 0;
    avgIdx = 0;
    for (uint8_t i = 0; i < AVG_WINDOW; i++) {
        signalBuf[i] = 0;
    }

    // Sample for 300ms to get several fresh frames (15 frames @ 50Hz)
    unsigned long start = millis();
    uint32_t framesAtStart = goodFrames;
    uint32_t validFrames = 0;
    
    while (millis() - start < 300) {
        // Process incoming bytes
        while (sensorSerial->available()) {
            uint8_t b = (uint8_t)sensorSerial->read();
            processByte(b);
            totalBytesRead++;
        }
        
        // Count valid frames received
        if (goodFrames > framesAtStart) {
            validFrames = goodFrames - framesAtStart;
        }
        
        delay(10);
    }

    // Restore previous active state
    sensorActive = wasActive;

    // Check if measurement is valid
    bool isValid = (lastStatus == 0x01 && lastReading > 0.0f && validFrames > 0);

    Serial.printf("[F2_MINI] Setup complete: %lu frames | D:%.2fm | Sig:%d | St:0x%02X | Valid:%d\n",
                  validFrames, lastReading, lastSignal, lastStatus, isValid);

    // Return distance in millimeters
    if (distanceOut) {
        *distanceOut = (uint16_t)(lastReading * 1000.0f);
    }
    
    // *** CRITICAL FIX: Convert F2_Mini status to TOF400C format ***
    // F2_Mini: 0x01  =valid, 0x00 = invalid
    // TOF400C: 0 = valid, 255 = invalid
    if (statusOut) {
        if (isValid) {
            *statusOut = 0;  // ← VALID (TOF400C format)
        } else {
            *statusOut = 255;  // ← INVALID
        }
    }

    if (!isValid) {
        Serial.println("[F2_MINI] ⚠️ Setup measurement FAILED - no valid data");
    }
    
    return isValid;
}
void sensorF2MiniActivate() {
    if (sensorActive) return;
    if (!sensorInitialized) sensorF2MiniInit();

    sensorActive = true;
    validTriggerOccurred = false;
    confirmPending = false;

    Serial.println("[F2_MINI] 🟢 Activated");
}

void sensorF2MiniDeactivate() {
    sensorActive = false;
    validTriggerOccurred = false;
    confirmPending = false;

    Serial.println("[F2_MINI] ⏸️ Deactivated");
}

void sensorF2MiniSetThreshold(float newThresholdMeters) {
    F2_THRESHOLD_M = newThresholdMeters;
    Serial.printf("[F2_MINI] Threshold: %.2fm\n", F2_THRESHOLD_M);
}

void sensorF2MiniSetTriggerCallback(TriggerCallback callback) {
    triggerCallback = callback;
}

bool sensorF2MiniIsTriggered() {
    return validTriggerOccurred;
}

void sensorF2MiniResetInterruptFlag() {
    validTriggerOccurred = false;
    confirmPending = false;
    confirmValidCount = 0;
}

float sensorF2MiniGetLastReading() {
    return lastReading;
}

uint32_t sensorF2MiniGetTimestamp() {
    return triggerTimestampMs;
}

TriggerCallback sensorF2MiniGetCallback() {
    return triggerCallback;
}

// ============================================================
//  Diagnostics (optional - for troubleshooting)
// ============================================================
void sensorF2MiniDiagnostics() {
    Serial.println("\n[F2_MINI] ═══════════════════════════════════");
    Serial.println("[F2_MINI] 🔍 DIAGNOSTICS");
    Serial.println("[F2_MINI] ═══════════════════════════════════");
    Serial.printf("[F2_MINI] Initialized: %s\n", sensorInitialized ? "YES" : "NO");
    Serial.printf("[F2_MINI] Active: %s\n", sensorActive ? "YES" : "NO");
    Serial.printf("[F2_MINI] Serial port: %p\n", (void*)sensorSerial);
    
    if (sensorSerial) {
        Serial.printf("[F2_MINI] RX Pin: %d\n", F2_MINI_RX_PIN);
        Serial.printf("[F2_MINI] TX Pin: %d\n", F2_MINI_TX_PIN);
        Serial.printf("[F2_MINI] Baud: %d\n", F2_MINI_BAUD);
        Serial.printf("[F2_MINI] Available bytes: %d\n", sensorSerial->available());
    }
    
    Serial.printf("[F2_MINI] Good frames: %lu\n", goodFrames);
    Serial.printf("[F2_MINI] Bad frames: %lu\n", badFrames);
    Serial.printf("[F2_MINI] Triggers: %lu\n", triggerCount);
    Serial.printf("[F2_MINI] Total bytes read: %lu\n", totalBytesRead);
    Serial.printf("[F2_MINI] Last reading: %.3fm\n", lastReading);
    Serial.printf("[F2_MINI] Last signal: %d\n", lastSignal);
    Serial.printf("[F2_MINI] Last status: 0x%02X\n", lastStatus);
    Serial.printf("[F2_MINI] Threshold: %.2fm\n", F2_THRESHOLD_M);
    Serial.println("[F2_MINI] ═══════════════════════════════════\n");
}
