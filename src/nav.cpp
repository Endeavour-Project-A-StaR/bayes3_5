#include "nav.h"
#include "persistent_config.h"
#include <math.h>

static const float SERVO_CENTER = 90.0f;

static float i_roll = 0.0f;
static float i_pitch = 0.0f;
static float i_yaw = 0.0f;

static float constrain_f(float val, float min, float max)
{
    if (val < min)
        return min;
    if (val > max)
        return max;
    return val;
}

void nav_rst_integral()
{
    i_roll = 0.0f;
    i_pitch = 0.0f;
    i_yaw = 0.0f;
}

void nav_update_pid(FltData_t *fltdata, float dt)
{

    // 1. EXTRACT QUATERNION ERROR
    // Because our target is perfectly vertical (1, 0, 0, 0), the error quaternion
    // is simply the conjugate (inverse) of our current orientation.
    // For small angles, the x, y, z components of the conjugate quaternion exactly
    // represent half of the rotational error in radians. We multiply by 2.

    float err_roll_rad = -fltdata->quat[1] * 2.0f;  // Rotation around X
    float err_pitch_rad = -fltdata->quat[2] * 2.0f; // Rotation around Y
    float err_yaw_rad = -fltdata->quat[3] * 2.0f;   // Rotation around Z

    // Convert to degrees so our PID tuning feels more human-readable
    float err_roll = err_roll_rad * (180.0f / 3.14159265f);
    float err_pitch = err_pitch_rad * (180.0f / 3.14159265f);
    float err_yaw = err_yaw_rad * (180.0f / 3.14159265f);

    // 2. PID CALCULATIONS

    // Note on Derivative (D) term:
    // Instead of doing (err - prev_err)/dt which magnifies sensor noise,
    // we use the directly measured gyro rate (-gyro). This is a standard
    // aerospace control technique called "Derivative on Measurement".

    // Roll PID
    i_roll += err_roll * dt;
    i_roll = constrain_f(i_roll, -config.pid_i_max, config.pid_i_max);
    float out_roll = (config.pid_roll.kp * err_roll) + (config.pid_roll.ki * i_roll) - (config.pid_roll.kd * fltdata->gyro[0]);

    // Pitch PID
    i_pitch += err_pitch * dt;
    i_pitch = constrain_f(i_pitch, -config.pid_i_max, config.pid_i_max);
    float out_pitch = (config.pid_pitch.kp * err_pitch) + (config.pid_pitch.ki * i_pitch) - (config.pid_pitch.kd * fltdata->gyro[1]);

    // Yaw PID
    i_yaw += err_yaw * dt;
    i_yaw = constrain_f(i_yaw, -config.pid_i_max, config.pid_i_max);
    float out_yaw = (config.pid_yaw.kp * err_yaw) + (config.pid_yaw.ki * i_yaw) - (config.pid_yaw.kd * fltdata->gyro[2]);

    // 3. PLUS (+)-CONFIGURATION SERVO MIXER
    // Maps the 3 rotational requests into 4 physical servo movements.
    // Assuming fins are aligned directly with the IMU axes:
    // S1: Top Fin (+Z axis)    -> Controls Pitch & Roll
    // S2: Right Fin (+Y axis)  -> Controls Yaw & Roll
    // S3: Bottom Fin (-Z axis) -> Controls Pitch & Roll
    // S4: Left Fin (-Y axis)   -> Controls Yaw & Roll

    // Note: The specific + and - signs here depend on which way your servos
    // are physically mounted (e.g. horn pointing forward vs backward).
    // If a pair moves backwards during your push test, just flip the signs for that pair!

    float m1 = out_pitch + out_roll;  // Top
    float m2 = out_yaw + out_roll;    // Right
    float m3 = -out_pitch + out_roll; // Bottom
    float m4 = -out_yaw + out_roll;   // Left

    // 4. APPLY LIMITS & CENTER OFFSET
    fltdata->servo_out[0] = SERVO_CENTER + constrain_f(m1, -config.servo_limit_max_deg, config.servo_limit_max_deg);
    fltdata->servo_out[1] = SERVO_CENTER + constrain_f(m2, -config.servo_limit_max_deg, config.servo_limit_max_deg);
    fltdata->servo_out[2] = SERVO_CENTER + constrain_f(m3, -config.servo_limit_max_deg, config.servo_limit_max_deg);
    fltdata->servo_out[3] = SERVO_CENTER + constrain_f(m4, -config.servo_limit_max_deg, config.servo_limit_max_deg);
}
