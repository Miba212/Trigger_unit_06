// trigger_test.h
#pragma once
#include <Arduino.h>

// Set this to 1 to enable automatic testing, 0 to disable
#define TRIGGER_TEST_ENABLED 0

// Configure test timing (in milliseconds)
#define TEST_START_DELAY_MS 5000   // Wait 5 seconds before triggering start
#define TEST_STOP_DELAY_MS 25000   // Wait 25 seconds before triggering stop

void triggerTestInit();
void triggerTestUpdate();
