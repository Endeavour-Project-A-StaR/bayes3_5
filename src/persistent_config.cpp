#include "persistent_config.h"
#include <EEPROM.h>
#include <Arduino.h>

PersistentCfg_t config;

void config_set_defaults()
{
    config.magic = CFG_MAGIC;

    config.pid_pitch = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};
    config.pid_roll = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};
    config.pid_yaw = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};
    config.pid_i_max = 10.0f;

    config.servo_center_us = 1500.0f;
    config.servo_us_per_deg = 10.0f;
    config.servo_limit_max_deg = 30.0f;

    config.parachute_charge_timeout_ms = 60000;

    config.log_interval_ms = 50;
    config.log_flush_interval_ms = 200;

    config.test_mode_en = false;
}

void config_init()
{
    EEPROM.get(0, config);

    if (config.magic == CFG_MAGIC)
    {
        Serial.println("CONFIG LOADED FROM EEPROM");
    }
    else
    {
        Serial.println("EEPROM INVALID, RESTORING TO DEFAULT");
        config_set_defaults();
        config_save();
    }
}

void config_save()
{
    EEPROM.put(0, config);
    Serial.println("CONFIG WRITTEN TO EEPROM");
}