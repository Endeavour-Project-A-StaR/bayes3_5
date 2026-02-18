#include "nav.h"

static PIDCoeff_t pidcoeff_roll = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};
static PIDCoeff_t pidcoeff_pitch = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};
static PIDCoeff_t pidcoeff_yaw = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};
