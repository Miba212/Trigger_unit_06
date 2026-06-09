#pragma once
#include <stdint.h>
#include <Arduino.h>
#include "trigger_sensor.h"

// === TOF400C Sensor Parameters (can be changed in sensor_tof400c.cpp) ===
extern uint8_t TOF_ROI_X;
extern uint8_t TOF_ROI_Y;
extern uint8_t TOF_ROI_SPAD;
extern uint8_t TOF_DISTANCE_MODE; // 1=Short, 2=Medium, 3=Long
extern float   TOF_GLOBAL_THRESHOLD_M;
extern unsigned long STABILIZATION_DELAY_MS;
extern unsigned long REARM_COOLDOWN_MS;
extern uint8_t MAX_REARM_ATTEMPTS;
extern uint16_t TOF_LOW_THRESHOLD_MM;
extern uint8_t TOF_INT_PIN;
extern uint8_t TOF_SDA_PIN;
extern uint8_t TOF_SCL_PIN;
extern uint8_t TOF_SHUT_PIN;

bool sensorTof400cDoSetupMeasurement(uint16_t* distanceOut, uint8_t* statusOut);

// === Sensor API ===
void sensorTof400cInit();
void sensorTof400cUpdate();
void sensorTof400cActivate();
void sensorTof400cDeactivate();
void sensorTof400cSetThreshold(float threshold);
void sensorTof400cSetTiming(uint16_t timingBudgetMs, uint16_t intermeasurementMs); // timing control
void sensorTof400cSetTriggerCallback(TriggerCallback callback);
bool sensorTof400cIsTriggered();
void sensorTof400cResetInterruptFlag();
float sensorTof400cGetLastReading();
uint32_t sensorTof400cGetTimestamp();
TriggerCallback sensorTof400cGetCallback();
