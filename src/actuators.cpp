#include <Arduino.h>
#include <Servo.h>
#include "actuators.h"
#include "persistent_config.h"

/*
Our servos uses non standard pulse widths (sad)
pulse length: 800uS - 2200uS
pulse length for -50 / 0 / +50 deg: 1000uS, 1500uS, 2000uS.
*/

static Servo servo_1;
static Servo servo_2;
static Servo servo_3;
static Servo servo_4;

void servo_swing_test_deg2us(int pos)
{
    int us = (int)roundf(config.servo_center_us + (pos - 90.0f) * config.servo_us_per_deg);
    servo_1.writeMicroseconds(us);
    servo_2.writeMicroseconds(us);
    servo_3.writeMicroseconds(us);
    servo_4.writeMicroseconds(us);
}

void servo_init(FltData_t *fltdata)
{
    servo_1.attach(PIN_SERVO_1);
    servo_2.attach(PIN_SERVO_2);
    servo_3.attach(PIN_SERVO_3);
    servo_4.attach(PIN_SERVO_4);

    fltdata->servo_out[0] = 90.0f;
    fltdata->servo_out[1] = 90.0f;
    fltdata->servo_out[2] = 90.0f;
    fltdata->servo_out[3] = 90.0f;
}

void servo_swing_test()
{
    for (int pos = 90; pos <= 135; pos++)
    {
        servo_swing_test_deg2us(pos);
        delay(20);
    }
    for (int pos = 135; pos >= 45; pos--)
    {
        servo_swing_test_deg2us(pos);
        delay(20);
    }
    for (int pos = 45; pos <= 90; pos++)
    {
        servo_swing_test_deg2us(pos);
        delay(20);
    }
}

void servo_write(FltData_t *fltdata)
{
    int us1 = (int)roundf(config.servo_center_us + (fltdata->servo_out[0] - 90.0f) * config.servo_us_per_deg);
    int us2 = (int)roundf(config.servo_center_us + (fltdata->servo_out[1] - 90.0f) * config.servo_us_per_deg);
    int us3 = (int)roundf(config.servo_center_us + (fltdata->servo_out[2] - 90.0f) * config.servo_us_per_deg);
    int us4 = (int)roundf(config.servo_center_us + (fltdata->servo_out[3] - 90.0f) * config.servo_us_per_deg);

    servo_1.writeMicroseconds(us1);
    servo_2.writeMicroseconds(us2);
    servo_3.writeMicroseconds(us3);
    servo_4.writeMicroseconds(us4);
}
