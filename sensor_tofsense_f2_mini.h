#ifndef SENSOR_TOFSENSE_F2_MINI_H
#define SENSOR_TOFSENSE_F2_MINI_H

#include <Arduino.h>
#include <HardwareSerial.h>

// Callback type (matches sensor_tof400c.h)
typedef void (*TriggerCallback)(uint32_t timestamp);

// ============================================================
//  Public API (matches sensor_tof400c.h interface)
// ============================================================
void sensorF2MiniInit();
void sensorF2MiniUpdate();
bool sensorF2MiniDoSetupMeasurement(uint16_t* distanceOut, uint8_t* statusOut);
void sensorF2MiniActivate();
void sensorF2MiniDeactivate();
void sensorF2MiniSetThreshold(float newThresholdMeters);
void sensorF2MiniSetTriggerCallback(TriggerCallback callback);
bool sensorF2MiniIsTriggered();
void sensorF2MiniResetInterruptFlag();
float sensorF2MiniGetLastReading();
uint32_t sensorF2MiniGetTimestamp();
TriggerCallback sensorF2MiniGetCallback();  // For testing

// Diagnostics
void sensorF2MiniDiagnostics();

#endif
