#include <Arduino.h>
#include <Wire.h>
#include "types.h"
#include "imu.h"
#include "nav.h"
#include "log.h"

FltStates_t state = STATE_DIAG; // Default startup to self test
FltData_t fltdata;              // Init shared flight data struct

uint32_t last_loop_time = 0;
uint32_t last_print_time = 0;

void print_telem()
{
  if ((state == STATE_OVRD || state == STATE_PREFLT) && (millis() - last_print_time >= 50))
  {
    last_print_time = millis();

    static char ser_buf[4096];

    int len = serializer(ser_buf, sizeof(ser_buf), millis(), state, &fltdata);

    if (len > 0 && len < sizeof(ser_buf))
      Serial.println(ser_buf);
  }
}

void command_parser()
{
  if (Serial.available())
  {
    char c = Serial.read();
    if (state == STATE_NAVLK || state == STATE_BURN || state == STATE_COAST || state == STATE_RECVY)
    {
      if (c == 'O' || c == 'P' || c == 'A')
      {
        Serial.println("COMMAND IGNORED IN FLT LOCKOUT");
      }
    }
    else
    {

      switch (c)
      {
      case 'A':
        state = STATE_NAVLK;
        nav_rst_integral();
        Serial.println("------ GUIDANCE IS INTERNAL ------");
        break;

      case 'O':
        state = STATE_OVRD;
        nav_rst_integral();
        Serial.println("------ OVRD MODE ------");
        break;

      case 'P':
        state = STATE_PREFLT;
        Serial.println("------ REVERTED TO PREFLT ------");
        break;

      default:
        break;
      }
    }
  }
}

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

  Serial.println("Bayes HW init complete. Will enter Preflight state");

  Serial.println("\nCommands Available: A = NAV LOCK | O = GROUND OVERRIDE | P = REVERT TO PREFLIGHT");

  nav_rst_integral();

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
        nav_update_pid(&fltdata, dt);

        if (fltdata.accel[0] < 0.0f)
        {
          state = STATE_COAST;
          Serial.println("------ BURNOUT ------");
        }

        break;

      case STATE_COAST:

        imu_calc_att(&fltdata, dt);
        nav_update_pid(&fltdata, dt);

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
        nav_update_pid(&fltdata, dt);

        break;

      default:

        break;
      }

      print_telem();
    }
    else
    {
      Serial.println("IMU READ FAIL");
    }
  }

  command_parser();
}
