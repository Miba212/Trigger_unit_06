#pragma once
#include <stdint.h>

void triggerStateInit();
uint8_t triggerStateGetCurrent();
void triggerStateSet(uint8_t state);
const char* triggerStateGetString();
bool triggerStateIsDetecting();
