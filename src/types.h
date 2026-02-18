#pragma once

#include <stdint.h>
#include <stdbool.h>

// State machine defs
typedef enum {
  STATE_DIAG,      // Self test, start data logging
  STATE_PREFLT,    // Preflight wait, cont gyro cal
  STATE_BURN,      // Motor burn, stability ctrl enabled
  STATE_COAST,     // Motor burnout, coasting with stability ctrl
  STATE_RECVY,     // Parachute out, stability ctrl off
  STATE_OVRD       // Ground override for testing
} FltStates_t;

typedef struct {

    // Raw sensor data
    float accel[3];
    float gyro[3];
    float pressure;

    // Computed altitude
    float altitude;

    // Orientation data
    float quat[4];
    float pitch, yaw, roll;

    // Control outputs
    float servo_out[4];

    // Accel fusion kill sw
    bool accel_fusion_en;
} FltData_t;

typedef struct {

    float kp, ki, kd;
} PIDCoeff_t;
