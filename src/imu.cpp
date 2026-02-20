#include "imu.h"
#include "eeprom_config.h"
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

void imu_cal_gyro(FltData_t *fltdata)
{
    const int smps = 500;

    float s_x = 0;
    float s_y = 0;
    float s_z = 0;

    inv_imu_sensor_data_t imu_data;

    for (int i = 0; i < smps; i++)
    {
        IMU.getDataFromRegisters(imu_data);
        s_x += (float)imu_data.gyro_data[0] * GYRO_SCALE;
        s_y += (float)imu_data.gyro_data[1] * GYRO_SCALE;
        s_z += (float)imu_data.gyro_data[2] * GYRO_SCALE;
        delay(1);
    }

    float sign_flip = config.test_mode_en ? -1.0f : 1.0f;

    if (sign_flip != 1.0f)
        Serial.println("MSG: INVERTED TEST MODE ENABLED");

    fltdata->gyro_bias[0] = (s_x / smps) * sign_flip;
    fltdata->gyro_bias[1] = s_y / smps;
    fltdata->gyro_bias[2] = (s_z / smps) * sign_flip;
}

bool imu_read(FltData_t *fltdata)
{
    inv_imu_sensor_data_t imu_data;

    int ret = IMU.getDataFromRegisters(imu_data);
    if (ret != 0)
        return false;

    fltdata->accel[0] = ((float)imu_data.accel_data[0] * ACCEL_SCALE);
    fltdata->accel[1] = ((float)imu_data.accel_data[1] * ACCEL_SCALE);
    fltdata->accel[2] = ((float)imu_data.accel_data[2] * ACCEL_SCALE);

    fltdata->gyro[0] = ((float)imu_data.gyro_data[0] * GYRO_SCALE);
    fltdata->gyro[1] = ((float)imu_data.gyro_data[1] * GYRO_SCALE);
    fltdata->gyro[2] = ((float)imu_data.gyro_data[2] * GYRO_SCALE);

    if (config.test_mode_en)
    {
        fltdata->accel[0] = -fltdata->accel[0];
        fltdata->accel[2] = -fltdata->accel[2];
        fltdata->gyro[0] = -fltdata->gyro[0];
        fltdata->gyro[2] = -fltdata->gyro[2];
    }

    fltdata->gyro[0] -= fltdata->gyro_bias[0];
    fltdata->gyro[1] -= fltdata->gyro_bias[1];
    fltdata->gyro[2] -= fltdata->gyro_bias[2];

    return true;
}

void imu_calc_initial_att(FltData_t *fltdata)
{
    float ax = fltdata->accel[0];
    float ay = fltdata->accel[1];
    float az = fltdata->accel[2];

    // 1. Normalize the measured gravity vector
    float norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    float gx = ax * norm;
    float gy = ay * norm;
    float gz = az * norm;

    // 2. Rotate the Measured Gravity vector TO the Target UP vector (1, 0, 0).
    // This generates the correct Body-to-World quaternion.
    // q.w = 1 + dot_product(Measured, Target)
    // q.xyz = cross_product(Measured, Target)

    fltdata->quat[0] = 1.0f + gx; // w
    fltdata->quat[1] = 0.0f;      // x (gy*0 - gz*0)
    fltdata->quat[2] = gz;        // y (gz*1 - gx*0)
    fltdata->quat[3] = -gy;       // z (gx*0 - gy*1)

    // 3. Normalize the resulting quaternion
    float q_norm = 1.0f / sqrtf(
                              fltdata->quat[0] * fltdata->quat[0] +
                              fltdata->quat[1] * fltdata->quat[1] +
                              fltdata->quat[2] * fltdata->quat[2] +
                              fltdata->quat[3] * fltdata->quat[3]);

    fltdata->quat[0] *= q_norm;
    fltdata->quat[1] *= q_norm;
    fltdata->quat[2] *= q_norm;
    fltdata->quat[3] *= q_norm;
}

void imu_calc_att(FltData_t *fltdata, float dt)
{
    float q0 = fltdata->quat[0], q1 = fltdata->quat[1], q2 = fltdata->quat[2], q3 = fltdata->quat[3];
    float gx = fltdata->gyro[0], gy = fltdata->gyro[1], gz = fltdata->gyro[2];

    // Rate of change of quaternion from Gyro
    float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

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
}
