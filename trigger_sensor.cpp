#include "trigger_sensor.h"
#include "sensor_tof400c.h"
#include "sensor_tofsense_f2_mini.h"  // ← NEW

// ============================================================
//  Internal State (following your pattern)
// ============================================================
static uint8_t sensorType = TOF400C;  // ← Your existing default
static TriggerCallback onTriggerCallback = nullptr;
static float currentThreshold = 1.0f;

// ============================================================
//  Your existing function
// ============================================================
uint8_t triggerSensorGetType() {
    return sensorType;
}

// ============================================================
//  Initialization
// ============================================================
void triggerSensorInit() {
    Serial.printf("[SENSOR] Initializing sensor type: %d\n", sensorType);
    
    switch (sensorType) {
        case TOF400C:
            sensorTof400cInit();
            if (onTriggerCallback) {
                sensorTof400cSetTriggerCallback(onTriggerCallback);
            }
            sensorTof400cSetThreshold(currentThreshold);
            break;
            
        case TOFSENSE_F2_MINI:
            sensorF2MiniInit();
            if (onTriggerCallback) {
                sensorF2MiniSetTriggerCallback(onTriggerCallback);
            }
            sensorF2MiniSetThreshold(currentThreshold);
            break;
            
        default:
            Serial.printf("[SENSOR] ❌ Unknown sensor type: %d\n", sensorType);
            break;
    }
}

// ============================================================
//  Sensor Type Switching
// ============================================================
void triggerSensorSetType(uint8_t type) {
    if (type == sensorType) return;  // No change
    
    Serial.printf("[SENSOR] Switching sensor type: %d -> %d\n", sensorType, type);
    
    // Deactivate old sensor
    switch (sensorType) {
        case TOF400C:
            sensorTof400cDeactivate();
            break;
        case TOFSENSE_F2_MINI:
            sensorF2MiniDeactivate();
            break;
    }
    
    // Update type
    sensorType = type;
    
    // Initialize new sensor
    triggerSensorInit();
}

// ============================================================
//  Update Loop
// ============================================================
void triggerSensorUpdate() {
    switch (sensorType) {
        case TOF400C:
            sensorTof400cUpdate();
            break;
        case TOFSENSE_F2_MINI:
            sensorF2MiniUpdate();
            break;
    }
}

// ============================================================
//  Setup Measurement (your existing pattern)
// ============================================================
bool triggerSensorDoSetupMeasurement(uint16_t* distanceOut, uint8_t* statusOut) {
    switch (sensorType) {
        case TOF400C:
            return sensorTof400cDoSetupMeasurement(distanceOut, statusOut);
            
        case TOFSENSE_F2_MINI:
            return sensorF2MiniDoSetupMeasurement(distanceOut, statusOut);
            
        default:
            if (distanceOut) *distanceOut = 0;
            if (statusOut) *statusOut = 255;
            return false;
    }
}

// ============================================================
//  Activation
// ============================================================
void triggerSensorActivate() {
    Serial.printf("[SENSOR] Activating sensor type: %d\n", sensorType);
    
    switch (sensorType) {
        case TOF400C:
            sensorTof400cActivate();
            break;
        case TOFSENSE_F2_MINI:
            sensorF2MiniActivate();
            break;
    }
}

void triggerSensorDeactivate() {
    Serial.printf("[SENSOR] Deactivating sensor type: %d\n", sensorType);
    
    switch (sensorType) {
        case TOF400C:
            sensorTof400cDeactivate();
            break;
        case TOFSENSE_F2_MINI:
            sensorF2MiniDeactivate();
            break;
    }
}

// ============================================================
//  Threshold Configuration (from Web UI)
// ============================================================
void triggerSensorSetThreshold(float newThresholdMeters) {
    currentThreshold = newThresholdMeters;
    
    Serial.printf("[SENSOR] Setting threshold to %.2fm\n", newThresholdMeters);
    
    switch (sensorType) {
        case TOF400C:
            sensorTof400cSetThreshold(newThresholdMeters);
            break;
        case TOFSENSE_F2_MINI:
            sensorF2MiniSetThreshold(newThresholdMeters);
            break;
    }
}

// ============================================================
//  Callback Registration
// ============================================================
void triggerSensorSetTriggerCallback(TriggerCallback callback) {
    onTriggerCallback = callback;
    
    switch (sensorType) {
        case TOF400C:
            sensorTof400cSetTriggerCallback(callback);
            break;
        case TOFSENSE_F2_MINI:
            sensorF2MiniSetTriggerCallback(callback);
            break;
    }
}

// ============================================================
//  Status Queries
// ============================================================
bool triggerSensorIsTriggered() {
    switch (sensorType) {
        case TOF400C:
            return sensorTof400cIsTriggered();
        case TOFSENSE_F2_MINI:
            return sensorF2MiniIsTriggered();
        default:
            return false;
    }
}

void triggerSensorResetInterruptFlag() {
    switch (sensorType) {
        case TOF400C:
            sensorTof400cResetInterruptFlag();
            break;
        case TOFSENSE_F2_MINI:
            sensorF2MiniResetInterruptFlag();
            break;
    }
}

float triggerSensorGetLastReading() {
    switch (sensorType) {
        case TOF400C:
            return sensorTof400cGetLastReading();
        case TOFSENSE_F2_MINI:
            return sensorF2MiniGetLastReading();
        default:
            return 0.0f;
    }
}

uint32_t triggerSensorGetTimestamp() {
    switch (sensorType) {
        case TOF400C:
            return sensorTof400cGetTimestamp();
        case TOFSENSE_F2_MINI:
            return sensorF2MiniGetTimestamp();
        default:
            return 0;
    }
}
