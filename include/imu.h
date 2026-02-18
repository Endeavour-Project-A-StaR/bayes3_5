#pragma once

#include "types.h"

bool imu_init();
bool imu_read(FltData_t *fltdata);
void imu_calc_initial_att(FltData_t *fltdata);
void imu_calc_att(FltData_t *fltdata, float dt);
