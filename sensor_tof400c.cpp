#include "sensor_tof400c.h"
#include "trigger_timesync.h"
#include <Wire.h>
#include "SparkFun_VL53L1X.h"

// ==== Debug switch (set to 0 to silence extra prints) ====
#ifndef TOF_DEBUG
#define TOF_DEBUG 0
#endif

// ==== Sensor Parameters (editable) ====

// --- Pin assignments ---
uint8_t TOF_INT_PIN   = 11;
uint8_t TOF_SDA_PIN   = 8;
uint8_t TOF_SCL_PIN   = 9;
uint8_t TOF_SHUT_PIN  = 10;

// --- Sensor settings ---
uint8_t  TOF_ROI_X          = 6;    // ROI width (SPAD columns)
uint8_t  TOF_ROI_Y          = 12;   // ROI height (SPAD rows)
uint8_t  TOF_ROI_SPAD       = 199;  // Center SPAD
uint8_t  TOF_DISTANCE_MODE  = 3;    // 1=Short, 3=Long
float    TOF_GLOBAL_THRESHOLD_M = 1.0f; // Default threshold (meters)
uint16_t TOF_LOW_THRESHOLD_MM = 100; // Filter for short distance 10 cm

// --- Timing settings ---
unsigned long STABILIZATION_DELAY_MS = 1000;
unsigned long REARM_COOLDOWN_MS     = 3000;
uint8_t       MAX_REARM_ATTEMPTS    = 3;

// EXPLICIT TIMING CONTROL
static uint16_t TIMING_BUDGET_MS      = 25;  // e.g., 20/25/33/50
static uint16_t INTERMEASUREMENT_MS   = 35;  // must be >= TIMING_BUDGET_MS

// --- Detection tuning ---
static const uint16_t TOF_MIN_SIGNAL = 35;         // min per-SPAD signal to accept (tune)
static const uint32_t TOF_MAX_AMBIENT = 11000;     // reject readings when ambient above this (tune)
static const uint8_t  TOF_CONFIRM_READS = 2;       // # valid reads to confirm fallback
static const unsigned long TOF_FALLBACK_CONFIRM_WINDOW_MS = 30; // fallback confirmation window (ms)

// --- Averaging window for signal/ambient to improve robustness on low-reflective surfaces ---
static const uint8_t AVG_WINDOW = 6;

// Fallback confirmation state
static bool   fallbackPending = false;
static uint64_t fallbackFirstTsMs = 0;
static unsigned long fallbackFirstMillis = 0;
static uint8_t  fallbackValidCount = 0;

// ==== Internal State ====
static TriggerCallback triggerCallback = nullptr;
static SFEVL53L1X distanceSensor(Wire);

static volatile bool tofInterruptOccurred = false;
static volatile uint32_t tofInterruptTimestamp = 0;
static volatile uint64_t syncedInterruptTimestampMs = 0;
static bool tofInitialized = false;
static bool sensorActive = false;
static float lastReading = 0.0f;

static bool interruptProcessed = false;
static bool validTriggerOccurred = false;
static bool rearmingInProgress = false;
static int rearmAttemptCount = 0;
static unsigned long rearmCooldownStart = 0;

// Debug counters
static uint32_t g_intTriggerCount = 0;
static uint32_t g_fallbackTriggerCount = 0;

// ---- averaging buffers ----
static uint16_t sigBuf[AVG_WINDOW];
static uint16_t ambBuf[AVG_WINDOW];
static uint8_t  avgIdx = 0;
static uint8_t  avgCount = 0;

// ==== ISR ====
// Keep ISR minimal: only capture a fast hardware timestamp and set flag.
void IRAM_ATTR tofSensorInterrupt() {
    tofInterruptTimestamp = micros();
    tofInterruptOccurred = true;
    // DO NOT call triggerTimeSyncGetUnixMs() or any blocking call here.
}

// Helper to update circular buffers and compute averages
static void updateAverages(uint16_t sig, uint16_t amb, uint16_t *outAvgSig, uint16_t *outAvgAmb) {
    sigBuf[avgIdx] = sig;
    ambBuf[avgIdx] = amb;
    avgIdx = (avgIdx + 1) % AVG_WINDOW;
    if (avgCount < AVG_WINDOW) ++avgCount;

    uint32_t ssum = 0;
    uint32_t asum = 0;
    for (uint8_t i = 0; i < avgCount; ++i) {
        ssum += sigBuf[i];
        asum += ambBuf[i];
    }
    *outAvgSig = (uint16_t)(ssum / avgCount);
    *outAvgAmb = (uint16_t)(asum / avgCount);
}

// Helper to apply timing controls safely
static void applyTiming() {
    uint16_t effectiveInter = INTERMEASUREMENT_MS;
    if (effectiveInter < TIMING_BUDGET_MS) {
        effectiveInter = TIMING_BUDGET_MS;
    }
    distanceSensor.setTimingBudgetInMs(TIMING_BUDGET_MS);
    distanceSensor.setIntermeasurementPeriod(effectiveInter);
}

// ==== Sensor Configuration ====
static bool configureVL53L1XInterrupt() {
    distanceSensor.setInterruptPolarityLow();

    uint16_t configThresholdMM = (uint16_t)(TOF_GLOBAL_THRESHOLD_M * 1000.0f);
    uint16_t lowThreshold = TOF_LOW_THRESHOLD_MM;
    uint16_t highThreshold = configThresholdMM;

    // Window mode 0 (IN-window): INT asserted when distance in [low, high]
    distanceSensor.setDistanceThreshold(lowThreshold, highThreshold, 0);
    distanceSensor.clearInterrupt();
    delay(100);

    return true;
}

// ==== Sensor Initialization ====
static bool robustInitTOFSensor() {
    interruptProcessed = false;
    validTriggerOccurred = false;
    tofInterruptOccurred = false;
    avgIdx = 0;
    avgCount = 0;

    pinMode(TOF_SHUT_PIN, OUTPUT);
    digitalWrite(TOF_SHUT_PIN, HIGH);
    delay(50);
    Wire.begin(TOF_SDA_PIN, TOF_SCL_PIN);
    delay(50);

    if (distanceSensor.begin() != 0) {
        return false;
    }

    switch (TOF_DISTANCE_MODE) {
        case 1: distanceSensor.setDistanceModeShort(); break;
        case 2: /* medium */ 
        case 3: default: distanceSensor.setDistanceModeLong(); break;
    }

    distanceSensor.setROI(TOF_ROI_X, TOF_ROI_Y, TOF_ROI_SPAD);
    applyTiming();
    configureVL53L1XInterrupt();
    
 pinMode(TOF_INT_PIN, INPUT_PULLUP);
   

    distanceSensor.clearInterrupt();
    delay(100);

    bool intPinState = digitalRead(TOF_INT_PIN);
    if (intPinState == LOW) {
        distanceSensor.clearInterrupt();
        delay(200);
    }

    attachInterrupt(digitalPinToInterrupt(TOF_INT_PIN), tofSensorInterrupt, FALLING);

    distanceSensor.startRanging();
    delay(STABILIZATION_DELAY_MS);

    distanceSensor.clearInterrupt();
    tofInterruptOccurred = false;
    delay(100);

    uint16_t distance = distanceSensor.getDistance();
    lastReading = distance / 1000.0f;

    tofInitialized = true;
    return true;
}

// ==== API Implementation ====

// Init
void sensorTof400cInit() {
    pinMode(TOF_SHUT_PIN, OUTPUT);
    digitalWrite(TOF_SHUT_PIN, HIGH);
    pinMode(TOF_INT_PIN, INPUT);
    robustInitTOFSensor();
}

// Update
void sensorTof400cUpdate() {
    if (!sensorActive && !rearmingInProgress) return;

    unsigned long now = millis();
    static unsigned long lastStatusTime = 0;
    static unsigned long lastRangePrint = 0;

    // Print current ranging distance every 2 seconds
    if (now - lastRangePrint > 2000) {
        if (tofInitialized) {
            uint16_t distance = distanceSensor.getDistance();
            uint8_t status = distanceSensor.getRangeStatus();
            uint16_t signal = distanceSensor.getSignalPerSpad();
            uint16_t ambient = distanceSensor.getAmbientRate();

            Serial.printf("[TOF400C] Distance: %d mm (%.2f m), Mode: %s | status=%u signal=%u ambient=%u\n",
                distance,
                distance / 1000.0f,
                (TOF_DISTANCE_MODE == 1 ? "Short" : "Long"),
                status,
                signal,
                ambient
            );
        }
        lastRangePrint = now;
    }

    if (rearmingInProgress) {
        if (now - rearmCooldownStart >= REARM_COOLDOWN_MS) {
            rearmAttemptCount++;
            if (robustInitTOFSensor()) {
                sensorActive = true;
                rearmingInProgress = false;
                rearmAttemptCount = 0;
            } else {
                if (rearmAttemptCount >= MAX_REARM_ATTEMPTS) {
                    rearmingInProgress = false;
                    sensorActive = false;
                } else {
                    rearmCooldownStart = now;
                }
            }
        }
        return;
    }

    if (now - lastStatusTime > 5000) {
        if (tofInitialized) {
            uint16_t distance = distanceSensor.getDistance();
            lastReading = distance / 1000.0f;
            lastStatusTime = now;
        }
    }

    // INT-based trigger processing
    if (tofInitialized && tofInterruptOccurred && !interruptProcessed) {
        // Capture a synced timestamp now (safe outside ISR)
        syncedInterruptTimestampMs = triggerTimeSyncGetUnixMs();

        // Read diagnostics
        uint16_t distance = distanceSensor.getDistance();
        uint8_t  status   = distanceSensor.getRangeStatus();
        uint16_t signal   = distanceSensor.getSignalPerSpad();
        uint16_t ambient  = distanceSensor.getAmbientRate();

        lastReading = distance / 1000.0f;

        // Update running averages
        uint16_t avgSig = 0, avgAmb = 0;
        updateAverages(signal, ambient, &avgSig, &avgAmb);

        // Window check
        const uint16_t lowMM  = TOF_LOW_THRESHOLD_MM;
        const uint16_t highMM = (uint16_t)(TOF_GLOBAL_THRESHOLD_M * 1000.0f);
        const bool inWindow = (distance >= lowMM && distance <= highMM);

        // Diagnostics using averaged values (more robust for low-reflective materials)
        // First reject if ambient too high (flooding / sunlight)
        if (avgAmb > TOF_MAX_AMBIENT) {
            #if TOF_DEBUG
            Serial.printf("[TOF400C][INT] Sample | dist=%u | status=%u | sig=%u | amb=%u | avgSig=%u | avgAmb=%u | inWindow=%d | shortAllowed=%d | ts=%llu\n",
                          distance, status, signal, ambient, avgSig, avgAmb, inWindow, (distance < lowMM ? 0 : 1), (unsigned long long)syncedInterruptTimestampMs);
            Serial.println("[TOF400C][INT] Rejected -> reason: high-ambient");
            #endif
            distanceSensor.clearInterrupt();
            tofInterruptOccurred = false;
            return;
        }

        // Accept if:
        //  - status==0 AND avgSig >= TOF_MIN_SIGNAL (good reading)
        //  - OR avgSig >= (TOF_MIN_SIGNAL/2) AND object is relatively close (<= 1000mm)
        //  - OR avgSig >= TOF_MIN_SIGNAL and distance is inside window (fallback for some status codes)
        bool diagOk = false;
        if (status == 0 && avgSig >= TOF_MIN_SIGNAL) {
            diagOk = true;
        } else if (avgSig >= (TOF_MIN_SIGNAL / 2) && distance <= 1000) {
            // weaker signal tolerated for close objects (low reflectivity)
            diagOk = true;
        } else if (avgSig >= TOF_MIN_SIGNAL && inWindow) {
            // signal strong enough even if status not 0
            diagOk = true;
        }

        const bool tooShort = (distance < lowMM);
        const bool shortAllowed = (!tooShort) || (status == 0 && avgSig >= TOF_MIN_SIGNAL);

        // Log sample always (helps debugging)
        #if TOF_DEBUG
        Serial.printf("[TOF400C][INT] Sample | dist=%u mm | status=%u | sig=%u | amb=%u | avgSig=%u | avgAmb=%u | inWindow=%d | diagOk=%d | shortAllowed=%d | ts=%llu\n",
                      distance, status, signal, ambient, avgSig, avgAmb, inWindow, diagOk, shortAllowed, (unsigned long long)syncedInterruptTimestampMs);
        #endif

        if (!(inWindow && diagOk && shortAllowed)) {
            #if TOF_DEBUG
            Serial.print("[TOF400C][INT] Rejected -> reason:");
            if (!inWindow) Serial.print(" out-of-window");
            if (!diagOk) {
                if (avgAmb > TOF_MAX_AMBIENT) Serial.print(" high-avg-ambient");
                else Serial.print(" diag-fail(avgSig too low or bad-status)");
            }
            if (!shortAllowed) Serial.print(" short-not-allowed");
            Serial.print("\n");
            #endif

            distanceSensor.clearInterrupt();
            tofInterruptOccurred = false;
            return;
        }

        // Accept the interrupt — update bookkeeping and call callback using synced timestamp
        interruptProcessed = true;
        validTriggerOccurred = true;

        if (triggerCallback) {
            triggerCallback((uint32_t)syncedInterruptTimestampMs);
        }

        g_intTriggerCount++;
        #if TOF_DEBUG
        Serial.printf("[TOF400C][INT]  Trigger #%lu ACCEPTED | dist=%u mm (%.2f m) | status=%u | sig=%u | amb=%u | avgSig=%u | avgAmb=%u | ts=%llu\n",
                      (unsigned long)g_intTriggerCount,
                      distance, distance / 1000.0f,
                      status,
                      signal,
                      ambient,
                      avgSig,
                      avgAmb,
                      (unsigned long long)syncedInterruptTimestampMs);
        #endif

        // Clear sensor interrupt and local flag
        distanceSensor.clearInterrupt();
        tofInterruptOccurred = false;

        // Small pause to let sensor reset; keep it small to avoid blocking
        delay(30);
    }

    // Fallback (no INT) - uses averaged diagnostics as well
    if (tofInitialized && !interruptProcessed && sensorActive) {
        const uint16_t lowMM = TOF_LOW_THRESHOLD_MM;
        const uint16_t highMM = (uint16_t)(TOF_GLOBAL_THRESHOLD_M * 1000.0f);

        uint16_t d   = distanceSensor.getDistance();
        uint8_t  s   = distanceSensor.getRangeStatus();
        uint16_t sig = distanceSensor.getSignalPerSpad();
        uint16_t amb = distanceSensor.getAmbientRate();
        lastReading = d / 1000.0f;

        // Update averages with this sample
        uint16_t avgSig = 0, avgAmb = 0;
        updateAverages(sig, amb, &avgSig, &avgAmb);

        const bool inWindow = (d >= lowMM && d < highMM);

        // Diagnostics (same logic using averages)
        if (avgAmb > TOF_MAX_AMBIENT) {
            #if TOF_DEBUG
            Serial.printf("[TOF400C][FALLBACK] Sample | dist=%u | status=%u | sig=%u | amb=%u | avgSig=%u | avgAmb=%u -> reject high-ambient\n",
                          d, s, sig, amb, avgSig, avgAmb);
            #endif
        } else {
            bool diagnosticsOk = false;
            if (s == 0 && avgSig >= TOF_MIN_SIGNAL) diagnosticsOk = true;
            else if (avgSig >= (TOF_MIN_SIGNAL / 2) && d <= 1000) diagnosticsOk = true;
            else if (avgSig >= TOF_MIN_SIGNAL && inWindow) diagnosticsOk = true;

            const bool tooShort = (d < lowMM);
            const bool shortAllowed = (!tooShort) || (s == 0 && avgSig >= TOF_MIN_SIGNAL);
            const bool sampleValid = inWindow && diagnosticsOk && shortAllowed;

            #if TOF_DEBUG
            Serial.printf("[TOF400C][FALLBACK] Sample | dist=%u | status=%u | sig=%u | amb=%u | avgSig=%u | avgAmb=%u | inWindow=%d | diagnosticsOk=%d\n",
                          d, s, sig, amb, avgSig, avgAmb, inWindow, diagnosticsOk);
            #endif

            unsigned long nowMs = millis();

            if (!fallbackPending) {
                if (sampleValid) {
                    fallbackPending = true;
                    fallbackFirstTsMs = triggerTimeSyncGetUnixMs();
                    fallbackFirstMillis = nowMs;
                    fallbackValidCount = 1;
                    #if TOF_DEBUG
                    Serial.printf("[TOF400C][FALLBACK] Pending detection started | firstDist=%u mm | avgSig=%u | avgAmb=%u | ts=%llu\n",
                                  d, avgSig, avgAmb, (unsigned long long)fallbackFirstTsMs);
                    #endif
                }
            } else {
                if (sampleValid) fallbackValidCount++;
                if (fallbackValidCount >= TOF_CONFIRM_READS) {
                    interruptProcessed = true;
                    validTriggerOccurred = true;
                    if (triggerCallback) triggerCallback((uint32_t)fallbackFirstTsMs);
                    g_fallbackTriggerCount++;
                    #if TOF_DEBUG
                    Serial.printf("[TOF400C][FALLBACK] Confirmed trigger #%lu | firstTs=%llu | confirmDist=%u mm | avgSig=%u | avgAmb=%u\n",
                                  (unsigned long)g_fallbackTriggerCount,
                                  (unsigned long long)fallbackFirstTsMs, d, avgSig, avgAmb);
                    #endif
                    fallbackPending = false;
                    fallbackValidCount = 0;
                } else if (nowMs - fallbackFirstMillis > TOF_FALLBACK_CONFIRM_WINDOW_MS) {
                    #if TOF_DEBUG
                    Serial.printf("[TOF400C][FALLBACK] Pending canceled (timeout) | firstTs=%llu\n", (unsigned long long)fallbackFirstTsMs);
                    #endif
                    fallbackPending = false;
                    fallbackValidCount = 0;
                }
            }
        }
    }

    // Release logic (unchanged)
    if (validTriggerOccurred && sensorActive) {
        uint16_t distance = distanceSensor.getDistance();
        lastReading = distance / 1000.0f;

        if (lastReading < 0.08f || lastReading > (TOF_GLOBAL_THRESHOLD_M + 0.2f)) {
            validTriggerOccurred = false;
            interruptProcessed = false;
        }
    }
}

// Returns true if object detected, false if no object (distance == 0 or invalid)
bool sensorTof400cDoSetupMeasurement(uint16_t* distanceOut, uint8_t* statusOut) {
    if (!tofInitialized) robustInitTOFSensor();
    uint16_t validDistance = 0;
    uint8_t validStatus = 255;
    for (int i = 0; i < 5; ++i) {
        uint16_t d = distanceSensor.getDistance();
        uint8_t s = distanceSensor.getRangeStatus();
        delay(50);
        if (s == 0 && d > 0) { validDistance = d; validStatus = s; break; }
    }
    if (distanceOut) *distanceOut = validDistance;
    if (statusOut) *statusOut = validStatus;
    return (validStatus == 0 && validDistance > 0);
}

// Activate
void sensorTof400cActivate() {
    if (sensorActive) return;
    if (rearmingInProgress) return;
    if (robustInitTOFSensor()) {
        sensorActive = true;
        interruptProcessed = false;
        validTriggerOccurred = false;
    } else {
        rearmingInProgress = true;
        rearmAttemptCount = 0;
        rearmCooldownStart = millis();
    }
}

// Deactivate
void sensorTof400cDeactivate() {
    rearmingInProgress = false;
    if (tofInitialized) {
        detachInterrupt(digitalPinToInterrupt(TOF_INT_PIN));
        distanceSensor.stopRanging();
        tofInitialized = false;
        tofInterruptOccurred = false;
        interruptProcessed = false;
        validTriggerOccurred = false;
    }
    sensorActive = false;
}

// Set threshold
void sensorTof400cSetThreshold(float newThreshold) {
    TOF_GLOBAL_THRESHOLD_M = newThreshold;
    if (tofInitialized) {
        uint16_t thresholdMM = (uint16_t)(TOF_GLOBAL_THRESHOLD_M * 1000.0f);
        distanceSensor.setDistanceThreshold(TOF_LOW_THRESHOLD_MM, thresholdMM, 0);
        distanceSensor.clearInterrupt();
    }
}

// Set timing (public setter)
void sensorTof400cSetTiming(uint16_t timingBudgetMs, uint16_t intermeasurementMs) {
    TIMING_BUDGET_MS = timingBudgetMs;
    INTERMEASUREMENT_MS = intermeasurementMs;
    if (tofInitialized) {
        applyTiming();
        distanceSensor.clearInterrupt();
    }
}

// Set callback
void sensorTof400cSetTriggerCallback(TriggerCallback callback) {
    triggerCallback = callback;
}

// Query triggered
bool sensorTof400cIsTriggered() {
    return validTriggerOccurred;
}

// Reset interrupt flag
void sensorTof400cResetInterruptFlag() {
    tofInterruptOccurred = false;
    interruptProcessed = false;
    validTriggerOccurred = false;
    if (sensorActive && tofInitialized) {
        distanceSensor.clearInterrupt();
    }
}

// Get last reading
float sensorTof400cGetLastReading() {
    return lastReading;
}

// Get timestamp
uint32_t sensorTof400cGetTimestamp() {
    return (uint32_t)syncedInterruptTimestampMs;
}

// Get callback (for test)
TriggerCallback sensorTof400cGetCallback() {
    return triggerCallback;
} 
