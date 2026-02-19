#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "types.h"

// MUST be incremented when config struct is changed
// Epoch 1 0xDEAD0001 FEB-19-2026
#define CFG_MAGIC 0xDEAD0001

typedef struct
{
    uint32_t magic;
    
    PIDCoeff_t pid_pitch;
    PIDCoeff_t pid_roll;
    PIDCoeff_t pid_yaw;

    float pid_i_max;

    float servo_center_us;
    float servo_limit_max_deg;
    float servo_us_per_deg;

    uint32_t parachute_charge_timeout_ms;
    uint32_t log_interval_ms;
    uint32_t log_flush_interval_ms;
    bool test_mode_en;
} PersistentCfg_t;

extern PersistentCfg_t config;

void config_init();
void config_save();
void config_set_defaults();