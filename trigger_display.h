#pragma once
#include <stdint.h>

// // Display modes (matching config from main unit)
// #define DISPLAY_MODE_DEFAULT 0     // Timing, result or countdown
// #define DISPLAY_MODE_RESULT_CD 1   // Result and countdown (battery save)
// #define DISPLAY_MODE_STATE 2       // System state (future - bitmaps)

// Display modes (matching config from main unit)
#define DISPLAY_MODE_DEFAULT    0   // Always on — idle pattern, timing, result, countdown
#define DISPLAY_MODE_POWER_SAVE 1   // Power saving — idle pulse 5s ON/20s OFF, timing OFF
#define DISPLAY_MODE_CLOCK_ALT  2   // Idle alternates: - - - and HH:MM clock
#define DISPLAY_MODE_CLOCK_THEN 3   // Idle shows - - - for 30s then switches to HH:MM permanently

// Display configuration
#define READY_FLASH_PERIOD_MS 1100  // READY state flash period (easy to change)
#define TIMING_UPDATE_MS 60         // Live timing refresh rate
#define DISPLAY_INTENSITY 4       // Brightness 0-15

void triggerDisplayPrepareFromSavedConfig(bool enabled, uint8_t mode);
void triggerDisplayOnConfigReceived(bool enabled, uint8_t mode, int32_t offsetSeconds);
void triggerDisplayMarkStateReceived();
void triggerDisplaySetEnabled(bool enabled);
void triggerDisplaySetMode(uint8_t mode);
void triggerDisplaySetResult(float resultSeconds);

void triggerDisplaySetTimezoneOffset(int32_t offsetSeconds);

// Initialize display subsystem
void triggerDisplayInit();

// Update display (call in main loop)
void triggerDisplayUpdate();

// Enable/disable display
void triggerDisplaySetEnabled(bool enabled);
bool triggerDisplayIsEnabled();


// Set display mode
void triggerDisplaySetMode(uint8_t mode);
uint8_t triggerDisplayGetMode();

// Update display based on stopwatch state
void triggerDisplayUpdateState(uint8_t stopwatchState);

// Handle received data from main unit
void triggerDisplaySetResult(float resultSeconds);

void triggerDisplayStartCountdown(uint16_t durationSeconds);
void triggerDisplayFinishCountdown();
void triggerDisplayCancelCountdown();

// Show specific content (for testing/debugging)
void triggerDisplayShowText(const char* text);
void triggerDisplayShowTriggered();
void triggerDisplayStartTimer();

// Clear display
void triggerDisplayClear();

// Test display
void triggerDisplayTest();

