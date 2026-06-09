#pragma once
#include <Arduino.h>
#include <Preferences.h>

#define TRIGGER_NAME_LEN 32
struct TriggerConfig {
    bool paired;
    uint8_t mainUnitMac[6];
    uint8_t unitID;
    uint8_t role;
    char name[TRIGGER_NAME_LEN];
    uint8_t mainUnitChannel;

    bool displayEnabled;
    uint8_t displayMode;
};

class ConfigTrigger {
public:
    ConfigTrigger();

    bool load();         // Load config from NVS
    void save();         // Save config to NVS
    void setDefaults();  // Set default config values

    bool isPaired() const;
    void setPaired(bool p);
    
    uint8_t getChannel() const;
    void setChannel(uint8_t ch);

    TriggerConfig& get();
private:
    Preferences prefs;
    TriggerConfig cfg;
};

extern ConfigTrigger configTrigger;
