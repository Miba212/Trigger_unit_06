#pragma once
#include <stdint.h>

// Sensor types
#define TOF400C 1
#define TOFSENSE_F2_MINI        2  // ← Replaces SENSOR_TYPE_ULTRASONIC
#define SENSOR_TYPE_PROXIMITY 3
#define SENSOR_TYPE_MANUAL 4

bool triggerSensorDoSetupMeasurement(uint16_t* distanceOut, uint8_t* statusOut);

typedef void (*TriggerCallback)(uint32_t timestamp);

void triggerSensorInit();
void triggerSensorUpdate();
void triggerSensorActivate();
void triggerSensorDeactivate();
bool triggerSensorIsTriggered();
void triggerSensorSetType(uint8_t type);
void triggerSensorSetThreshold(float threshold);
void triggerSensorSetTriggerCallback(TriggerCallback callback);

void triggerSensorResetInterruptFlag();
float triggerSensorGetLastReading();
uint32_t triggerSensorGetTimestamp();
uint8_t triggerSensorGetType();
TriggerCallback triggerSensorGetCallback();
