#pragma once

#include "types.h"

bool imu_init();                                 // IMU initialization
void imu_cal_gyro(FltData_t* fltdata);                             // Calculates the gyro bias when stationary
bool imu_read(FltData_t *fltdata);               // Reads raw accel and compensated gyro data from IMU
void imu_calc_initial_att(FltData_t *fltdata);   // Calculates initial attitude when stationary from pure accel data by finding grav vector
void imu_calc_att(FltData_t *fltdata, float dt); // Calculates real time attitude in flight using pure gyro integration
