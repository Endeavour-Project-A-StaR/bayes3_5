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
static const float GYRO_SCALE = (float)GYRO_FSR_DPS / 32768.0f * DEG_2_RAD;

static ICM456xx IMU(Wire, 0);

static void quat2euler(FltData_t *fltdata)
{
    float qw = fltdata->quat[0];
    float qx = fltdata->quat[1];
    float qy = fltdata->quat[2];
    float qz = fltdata->quat[3];

    fltdata->roll = atan2f(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy));

    float sinp = 2.0f * (qw * qy - qz * qx);
    if (abs(sinp) >= 1)
        fltdata->pitch = copysign(M_PI / 2, sinp);
    else
        fltdata->pitch = asinf(sinp);

    fltdata->yaw = atan2f(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz));
}

bool imu_init()
{
    int ret = IMU.begin();
    if (ret != 0)
        return false;

    ret = IMU.startAccel(ODR_HZ, ACCEL_FSR_G);
    if (ret != 0)
        return false;

    ret = IMU.startGyro(ODR_HZ, GYRO_FSR_DPS);
    if (ret != 0)
        return false;

    return true;
}

bool imu_read(FltData_t *fltdata)
{
    inv_imu_sensor_data_t imu_data;

    int ret = IMU.getDataFromRegisters(imu_data);
    if (ret != 0)
        return false;

    fltdata->accel[0] = (float)imu_data.accel_data[0] * ACCEL_SCALE;
    fltdata->accel[1] = (float)imu_data.accel_data[1] * ACCEL_SCALE;
    fltdata->accel[2] = (float)imu_data.accel_data[2] * ACCEL_SCALE;

    fltdata->gyro[0] = (float)imu_data.gyro_data[0] * GYRO_SCALE;
    fltdata->gyro[1] = (float)imu_data.gyro_data[1] * GYRO_SCALE;
    fltdata->gyro[2] = (float)imu_data.gyro_data[2] * GYRO_SCALE;

    return true;
}

void imu_calc_initial_att(FltData_t *fltdata)
{
    float ax = fltdata->accel[0];
    float ay = fltdata->accel[1];
    float az = fltdata->accel[2];

    float roll = 0.0f;
    float pitch = atan2f(ax, sqrtf(ay * ay + az * az));
    float yaw = atan2f(ay, az);

    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);

    fltdata->quat[0] = cr * cp * cy + sr * sp * sy;
    fltdata->quat[1] = sr * cp * cy - cr * sp * sy;
    fltdata->quat[2] = cr * sp * cy + sr * cp * sy;
    fltdata->quat[3] = cr * cp * sy - sr * sp * cy;

    fltdata->pitch = pitch;
    fltdata->roll = roll;
    fltdata->yaw = yaw;
}

void imu_calc_att(FltData_t *fltdata, float dt)
{
    float q0 = fltdata->quat[0], q1 = fltdata->quat[1], q2 = fltdata->quat[2], q3 = fltdata->quat[3];
    float gx = fltdata->gyro[0], gy = fltdata->gyro[1], gz = fltdata->gyro[2];

    // Rate of change of quaternion from Gyro
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    // Integrate
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize
    float recipNorm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    fltdata->quat[0] = q0 * recipNorm;
    fltdata->quat[1] = q1 * recipNorm;
    fltdata->quat[2] = q2 * recipNorm;
    fltdata->quat[3] = q3 * recipNorm;

    // Output Euler
    quat2euler(fltdata);
}
