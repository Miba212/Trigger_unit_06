#include "config_trigger.h"

ConfigTrigger configTrigger;

ConfigTrigger::ConfigTrigger() {
    setDefaults();
}

bool ConfigTrigger::load() {
    prefs.begin("trigger_cfg", true); // readonly
    cfg.paired = prefs.getBool("paired", false);
    prefs.getBytes("mainMac", cfg.mainUnitMac, 6);
    cfg.unitID = prefs.getUChar("unitID", 0);
    cfg.role = prefs.getUChar("role", 0);
    prefs.getBytes("name", cfg.name, TRIGGER_NAME_LEN);
    cfg.mainUnitChannel = prefs.getUChar("channel", 0);

    cfg.displayEnabled = prefs.getBool("dispEn", false);
    cfg.displayMode = prefs.getUChar("dispMode", 0);

    prefs.end();
    return true;
}

void ConfigTrigger::save() {
    prefs.begin("trigger_cfg", false); // read-write
    prefs.putBool("paired", cfg.paired);
    prefs.putBytes("mainMac", cfg.mainUnitMac, 6);
    prefs.putUChar("unitID", cfg.unitID);
    prefs.putUChar("role", cfg.role);
    prefs.putBytes("name", cfg.name, TRIGGER_NAME_LEN);
    prefs.putUChar("channel", cfg.mainUnitChannel);

    prefs.putBool("dispEn", cfg.displayEnabled);
    prefs.putUChar("dispMode", cfg.displayMode);

    prefs.end();
}

void ConfigTrigger::setDefaults() {
    memset(&cfg, 0, sizeof(cfg));
    cfg.paired = false;
    memset(cfg.mainUnitMac, 0, 6);
    cfg.unitID = 0;
    cfg.role = 0;
    cfg.mainUnitChannel = 0;
    strncpy(cfg.name, "Trigger", TRIGGER_NAME_LEN - 1);
    cfg.name[TRIGGER_NAME_LEN - 1] = 0;

    cfg.displayEnabled = false;
    cfg.displayMode = 0;
}

bool ConfigTrigger::isPaired() const {
    return cfg.paired;
}

void ConfigTrigger::setPaired(bool p) {
    cfg.paired = p;
}

TriggerConfig& ConfigTrigger::get() {
    return cfg;
}

uint8_t ConfigTrigger::getChannel() const {
    return cfg.mainUnitChannel;
}

void ConfigTrigger::setChannel(uint8_t ch) {
    cfg.mainUnitChannel = ch;
}
