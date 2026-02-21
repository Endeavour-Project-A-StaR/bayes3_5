#include "eeprom_config.h"
#include <EEPROM.h>
#include <Arduino.h>

EEPROMCfg_t config;

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

    config.motor_burn_time_ms = 3000;
    config.parachute_charge_timeout_ms = 60000;

    config.log_interval_ms = 10;
    config.log_flush_interval_ms = 100;

    config.en_servo_in_burn = false;
    config.test_mode_en = false;
}

void config_init()
{
    EEPROM.get(0, config);

    if (config.magic == CFG_MAGIC)
    {
        Serial1.println("MSG: CONFIG LOADED FROM EEPROM");
    }
    else
    {
        Serial1.println("MSG: EEPROM INVALID, RESTORING TO DEFAULT");
        config_set_defaults();
        config_save();
    }
}

void config_save()
{
    EEPROM.put(0, config);
    Serial1.println("MSG: EEPROM WRITE SUCCESS");
}