#pragma once
#include <stdint.h>

// ==========================================
// TRIGGER UNIT WIFI CONFIGURATION
// ==========================================
// Change these values to configure WiFi/ESP-NOW for your trigger unit
// After changing, recompile and upload

// ==========================================
// ESP-NOW TX POWER CONFIGURATION
// ==========================================

// ESP-NOW TX Power (dBm)
// Should match or be slightly higher than main unit power
// Adjust based on distance to main unit and interference needs
//
// Power levels:
// - 20 dBm: Maximum power (~100m range) - outdoor, long distance
// - 17 dBm: High power (~50m range) - large indoor spaces
// - 13 dBm: Medium power (~20m range) - standard indoor use
// - 11 dBm: Low power (~10m range) - same room as main unit
// - 8  dBm: Very low power (~5m range) - very close to main unit
//
// Recommended settings:
// - Match main unit power for balanced communication
// - Use 2-3 dBm higher than main unit if triggers are farther away
// - Use lower power if multiple systems are very close (< 5m apart)
#define TRIGGER_ESPNOW_TX_POWER_DBM 18

// ==========================================
// WIFI MODE
// ==========================================
// Trigger units operate in STA (station) mode only
// No Access Point functionality needed
#define TRIGGER_WIFI_MODE WIFI_STA

// ==========================================
// CHANNEL DETECTION
// ==========================================
// Trigger units automatically detect and lock to the WiFi channel
// used by the main unit during pairing process.
// No manual channel configuration is needed!
//
// How it works:
// 1. Trigger sends pairing request on all channels (broadcast scan)
// 2. Main unit responds on its configured channel
// 3. Trigger locks to that channel for all future communication
// 4. After pairing, trigger always uses the paired channel

// ==========================================
// PRESETS FOR QUICK CONFIGURATION
// ==========================================
// To use a preset, comment out the line 29 definition above and
// uncomment ONE preset below

// PRESET 1: Standard range (match main unit medium power)
// #define TRIGGER_ESPNOW_TX_POWER_DBM 13

// PRESET 2: Close range (match main unit low power, less interference)
// #define TRIGGER_ESPNOW_TX_POWER_DBM 11

// PRESET 3: Long range (match main unit high power)
// #define TRIGGER_ESPNOW_TX_POWER_DBM 17

// PRESET 4: Maximum range (outdoor use)
// #define TRIGGER_ESPNOW_TX_POWER_DBM 20

// PRESET 5: Minimal interference (very close systems)
// #define TRIGGER_ESPNOW_TX_POWER_DBM 8

// ==========================================
// CONFIGURATION GUIDE
// ==========================================
//
// Matching Main Unit Settings:
// - If main unit uses 11 dBm → set trigger to 11 dBm
// - If main unit uses 13 dBm → set trigger to 13 dBm
// - If main unit uses 17 dBm → set trigger to 17 dBm
//
// For Multiple Systems Close Together:
// - System A: Main 11 dBm, Triggers 11 dBm, Channel 1
// - System B: Main 11 dBm, Triggers 11 dBm, Channel 6
// - System C: Main 11 dBm, Triggers 11 dBm, Channel 11
//
// For Distant Triggers:
// - If trigger is farther from main than other triggers
// - Add 2-3 dBm to trigger power (e.g., main 13 → trigger 15)
//
// For Very Close Units (< 2m apart):
// - Use 8 dBm on both main and triggers
// - Different channels (1, 6, 11)
