#include <Arduino.h>
#include <Wire.h>
#include "types.h"
#include "imu.h"

FltStates_t state = STATE_DIAG; // Default startup to self test
FltData_t fltdata;              // Init shared flight data struct

uint32_t last_loop_time = 0;
uint32_t last_print_time = 0;

void setup()
{

  Serial.begin(0); // ACM doesnt need baud rate

  Wire.begin();
  Wire.setClock(1000000);

  while (!Serial.available())
    delay(1);

  Serial.println("------ Bayes V3_5 Test ------");

  delay(500);

  Serial.println("Will init IMU");

  int ret = imu_init();
  Serial.println(ret ? "IMU Init Success" : "IMU Init Failed");
  if (!ret)
    while (1)
      delay(1);

  delay(500);

  Serial.println("Will calibrate gyro bias in 5 sec");
  delay(5000);
  imu_cal_gyro(&fltdata);
  Serial.println("Gyro calibration complete");

  delay(500);

  Serial.println("Bayes init complete. Will enter Preflight state");

  state = STATE_PREFLT;

  last_loop_time = micros();
}

void loop()
{
  uint32_t current_time = micros();
  float dt = (current_time - last_loop_time) / 1000000.0f;

  if (dt >= (1.0f / 1600.0f)) // 1600Hz
  {
    last_loop_time = current_time;

    if (imu_read(&fltdata))
    {
      switch (state)
      {

      case STATE_PREFLT:

        imu_calc_initial_att(&fltdata);
        if (Serial.available())
        {
          char c = Serial.read();
          if (c == 'A')
          {
            state = STATE_NAVLK;
            Serial.println("------ NAV LOCK COMPLETE ------");
          }
        }
        break;

      case STATE_NAVLK:

        imu_calc_att(&fltdata, dt);

        if (fltdata.accel[0] > 20.0f)
        {
          state = STATE_BURN;
          Serial.println("------ LIFTOFF ------");
        }
        break;

      case STATE_BURN:

        imu_calc_att(&fltdata, dt);

        if (fltdata.accel[0] < 0.0f)
        {
          state = STATE_COAST;
          Serial.println("------ BURNOUT ------");
        }

        break;

      case STATE_COAST:

        imu_calc_att(&fltdata, dt);

        if (false)
        {
          state = STATE_RECVY;
          Serial.println("------ APOGEE REACHED ------");
        }
        break;

      case STATE_RECVY:
        break;

      case STATE_OVRD:

        imu_calc_att(&fltdata, dt);
        break;
      }
    }
    else
    {
      Serial.println("IMU READ FAIL");
    }
  }
}
