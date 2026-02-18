#include "nav.h"
#include <math.h>

static const float SERVO_CENTER = 90.0f;
static const float SERVO_LIMIT_MAX = 30.0f;

static PIDCoeff_t pid_roll = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};
static PIDCoeff_t pid_pitch = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};
static PIDCoeff_t pid_yaw = {.kp = 1.0f, .ki = 0.0f, .kd = 0.0f};

static float i_roll = 0.0f;
static float i_pitch = 0.0f;
static float i_yaw = 0.0f;
static const float I_TERM_MAX = 10.0f;

static float constrain_f(float val, float min, float max) {
    if (val < min) return min;
    if (val > max) return max;
    return val;
}

void nav_rst_integral() {
    i_roll = 0.0f;
    i_pitch = 0.0f;
    i_yaw = 0.0f;
}

void nav_update_pid(FltData_t *fltdata, float dt) {
    
    // 1. EXTRACT QUATERNION ERROR
    // Because our target is perfectly vertical (1, 0, 0, 0), the error quaternion 
    // is simply the conjugate (inverse) of our current orientation. 
    // For small angles, the x, y, z components of the conjugate quaternion exactly 
    // represent half of the rotational error in radians. We multiply by 2.
    
    float err_roll_rad  = -fltdata->quat[1] * 2.0f; // Rotation around X
    float err_pitch_rad = -fltdata->quat[2] * 2.0f; // Rotation around Y
    float err_yaw_rad   = -fltdata->quat[3] * 2.0f; // Rotation around Z

    // Convert to degrees so our PID tuning feels more human-readable
    float err_roll  = err_roll_rad  * (180.0f / 3.14159265f);
    float err_pitch = err_pitch_rad * (180.0f / 3.14159265f);
    float err_yaw   = err_yaw_rad   * (180.0f / 3.14159265f);

    // 2. PID CALCULATIONS
    
    // Note on Derivative (D) term: 
    // Instead of doing (err - prev_err)/dt which magnifies sensor noise, 
    // we use the directly measured gyro rate (-gyro). This is a standard 
    // aerospace control technique called "Derivative on Measurement".
    
    // Roll PID
    i_roll += err_roll * dt;
    i_roll = constrain_f(i_roll, -I_TERM_MAX, I_TERM_MAX);
    float out_roll = (pid_roll.kp * err_roll) + (pid_roll.ki * i_roll) - (pid_roll.kd * fltdata->gyro[0]);

    // Pitch PID
    i_pitch += err_pitch * dt;
    i_pitch = constrain_f(i_pitch, -I_TERM_MAX, I_TERM_MAX);
    float out_pitch = (pid_pitch.kp * err_pitch) + (pid_pitch.ki * i_pitch) - (pid_pitch.kd * fltdata->gyro[1]);

    // Yaw PID
    i_yaw += err_yaw * dt;
    i_yaw = constrain_f(i_yaw, -I_TERM_MAX, I_TERM_MAX);
    float out_yaw = (pid_yaw.kp * err_yaw) + (pid_yaw.ki * i_yaw) - (pid_yaw.kd * fltdata->gyro[2]);


    // 3. X-CONFIGURATION SERVO MIXER
    // Maps the 3 rotational requests into 4 physical servo movements.
    // Assuming standard quad-layout:
    // S1: Top-Right (+Y, +Z)
    // S2: Bottom-Right (-Y, +Z)
    // S3: Bottom-Left (-Y, -Z)
    // S4: Top-Left (+Y, -Z)
    
    float m1 =  out_pitch - out_yaw + out_roll;
    float m2 = -out_pitch - out_yaw + out_roll;
    float m3 = -out_pitch + out_yaw + out_roll;
    float m4 =  out_pitch + out_yaw + out_roll;

    // 4. APPLY LIMITS & CENTER OFFSET
    // We clamp the mix to +/- 30 degrees, then add the 90 degree center point
    
    fltdata->servo_out[0] = SERVO_CENTER + constrain_f(m1, -SERVO_LIMIT_MAX, SERVO_LIMIT_MAX);
    fltdata->servo_out[1] = SERVO_CENTER + constrain_f(m2, -SERVO_LIMIT_MAX, SERVO_LIMIT_MAX);
    fltdata->servo_out[2] = SERVO_CENTER + constrain_f(m3, -SERVO_LIMIT_MAX, SERVO_LIMIT_MAX);
    fltdata->servo_out[3] = SERVO_CENTER + constrain_f(m4, -SERVO_LIMIT_MAX, SERVO_LIMIT_MAX);
}
