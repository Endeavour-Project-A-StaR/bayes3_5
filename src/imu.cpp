#include "imu.h"
#include <math.h>
#include <Wire.h>
#include <ICM45686.h>

static const uint16_t ACCEL_FSR_G = 16;
static const uint16_t GYRO_FSR_DPS = 2000;
static const uint16_t ODR_HZ = 1600;

static const float G_MS2 = 9.80665f;
static const float DEG_2_RAD = 3.14159265f / 180.0f;

static const float ACCEL_SCALE = (float)ACCEL_FSR_G / 32768.0f * G_MS2;
static const float GYRO_SCALE  = (float)GYRO_FSR_DPS / 32768.0f * DEG_2_RAD;

static ICM456xx IMU(Wire, 0);

static float gyro_bias[3] = {0.0f,0.0f,0.0f};

bool imu_init() 
{
    int ret = IMU.begin();
    if (ret != 0) return false;

    ret = IMU.startAccel(ODR_HZ, ACCEL_FSR_G);
    if (ret != 0) return false;

    ret = IMU.startGyro(ODR_HZ, GYRO_FSR_DPS);
    if (ret != 0) return false;

    return true;
}

bool imu_read(FltData_t* fltdata)
{
    return true;
}

void imu_gyro_cal(FltData_t* fltdata)
{
    return;
}

void imu_run_fusion(FltData_t* fltdata, float dt)
{
    return;
}