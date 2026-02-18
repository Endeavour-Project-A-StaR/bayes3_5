#pragma once

#include <stdint.h>
#include <stdbool.h>

// State machine defs
typedef enum
{
    STATE_DIAG,   // Self test, start data logging
    STATE_PREFLT, // Preflight wait, initial attitude determined using accel data
    STATE_NAVLK,  // Navigation switched to gyro only before ignition to prevent attitude getting fucked up by motor shock
    STATE_BURN,   // Motor burn, stability ctrl enabled
    STATE_COAST,  // Motor burnout, coasting with stability ctrl
    STATE_RECVY,  // Parachute out, stability ctrl off
    STATE_OVRD    // Ground override for testing
} FltStates_t;

typedef struct
{

    // Raw sensor data
    float accel[3]; // x, y, z
    float gyro[3];  // x, y, z
    float pressure;

    // Computed altitude
    float altitude;

    // Orientation data
    float quat[4]; // w, x, y, z
    float pitch, yaw, roll;

    // Control outputs
    float servo_out[4];

    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};

} FltData_t;

typedef struct
{

    float kp, ki, kd;
} PIDCoeff_t;
