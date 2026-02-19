#pragma once

#include "types.h"

#define PIN_SERVO_1 37
#define PIN_SERVO_2 14
#define PIN_SERVO_3 36
#define PIN_SERVO_4 33

void servo_init(FltData_t *fltdata);
void servo_swing_test();
void servo_write(FltData_t *fltdata);