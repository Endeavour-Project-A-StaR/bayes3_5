#include <Arduino.h>
#include <Wire.h>
#include "types.h"
#include "imu.h"
#include "nav.h"
#include "log.h"
#include "actuators.h"
#include "persistent_config.h"
#include "comms.h"
#include "baro.h"

FltStates_t state = STATE_DIAG; // Default startup to self test
FltData_t fltdata;              // Init shared flight data struct

uint32_t last_loop_time;
uint32_t burn_start;

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(0); // ACM doesnt need baud rate

  Wire.begin();
  Wire.setClock(1000000);

  while (!Serial.available())
    delay(1);

  Serial.println("RACS Development Booting Up");

  config_init();

  if (!imu_init())
    while (1)
      delay(1);
  Serial.println("MSG: IMU INIT SUCCESS");

  if (!baro_init())
    while (1)
      delay(1);
  Serial.println("MSG: BARO INIT SUCCESS");

  servo_init(&fltdata);

  Serial.println("MSG: WILL TEST SERVO");

  servo_swing_test();

  Serial.println("MSG: SERVO RECENTERED");

  Serial.println("MSG: GYRO CAL IN 5 SEC");
  delay(5000);
  imu_cal_gyro(&fltdata);

  Serial.println("MSG: BAYES READY");

  nav_rst_integral();

  state = STATE_PREFLT;

  digitalWrite(LED_BUILTIN, HIGH);

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
          burn_start = millis();
          Serial.println("MSG: LIFTOFF");
        }
        break;

      case STATE_BURN:

        imu_calc_att(&fltdata, dt);

        if (config.en_servo_in_burn)
          nav_update_pid(&fltdata, dt);

        if ((millis() - burn_start) >= config.motor_burn_time_ms)
        {
          state = STATE_COAST;
          Serial.println("MSG: BURN TIMER EXPIRED, UNLOCKING FINS");
        }

        break;

      case STATE_COAST:

        imu_calc_att(&fltdata, dt);
        nav_update_pid(&fltdata, dt);

        if ((millis() - burn_start) >= config.parachute_charge_timeout_ms)
        {
          state = STATE_RECVY;
          Serial.println("MSG: PARACHUTE DELAY CHARGE TIMER EXPIRED, DISABLING CONTROL");
        }

        break;

      case STATE_RECVY:
        imu_calc_att(&fltdata, dt);
        break;

      case STATE_OVRD:

        imu_calc_att(&fltdata, dt);
        nav_update_pid(&fltdata, dt);

        break;

      default:

        break;
      }

      servo_write(&fltdata);

      static uint32_t last_baro_read = 0;
      if ((current_time - last_baro_read) >= 15625)
      {
        baro_read(&fltdata);
        last_baro_read = current_time;
      }

      comms_send_telem(state, &fltdata);
    }
    else
    {
      Serial.println("MSG: IMU READ FAIL");
    }
  }

  comms_read_cmd(&state);
}
