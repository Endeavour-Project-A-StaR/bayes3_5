#include <Arduino.h>
#include <Wire.h>
#include "types.h"
#include "imu.h"
#include "nav.h"
#include "log.h"
#include "actuators.h"
#include "eeprom_config.h"
#include "comms.h"
#include "baro.h"

#define INIT_MAX_RETRIES 3

FltStates_t state = STATE_DIAG; // Default startup to self test
FltData_t fltdata;              // Init shared flight data struct

uint32_t last_loop_time; // last flight loop run timestamp
uint32_t burn_start;     // Ignition timestamp

void setup()
{

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); // onboard led will be turned on after successful initialization

  Serial.begin(0);      // USB ACM Serial, doesnt need baud rate
  Serial1.begin(38400); // Bluetooth serial

  Wire.begin();
  Wire.setClock(1000000); // Use 1MHz fast mode plus I2C (IMU needs fast readout)

  while (!Serial1.available()) // Stall until wakeup command
    delay(1);

  Serial1.println("RACS Development Booting Up");

  config_init(); // check EEPROM config integrity

  if (!log_init()) // initialize sd card and logfile
    while (1)
      delay(1);

  if (!imu_init())
    while (1)
      delay(1);
  Serial1.println("MSG: IMU INIT SUCCESS");

  if (!baro_init())
    while (1)
      delay(1);
  Serial1.println("MSG: BARO INIT SUCCESS");

  servo_init(&fltdata); // initialize and center servos in fltdata

  Serial1.println("MSG: WILL TEST SERVO");

  servo_swing_test();

  Serial1.println("MSG: SERVO RECENTERED");

  Serial1.println("MSG: GYRO CAL IN 3 SEC");
  delay(3000);
  imu_cal_gyro(&fltdata); // gyro bias calibration, 500 smps averaged

  Serial1.println("MSG: BAYES READY");

  nav_rst_integral(); // reset integral in all PID controllers

  state = STATE_PREFLT;

  Serial1.println("MSG: The rocket knows where it is at all times.");

  if (config.test_mode_en == 0)
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
          Serial1.println("MSG: LIFTOFF");
        }
        break;

      case STATE_BURN:

        imu_calc_att(&fltdata, dt);

        if (config.en_servo_in_burn)
          nav_update_pid(&fltdata, dt);

        if ((millis() - burn_start) >= config.motor_burn_time_ms)
        {
          state = STATE_COAST;
          Serial1.println("MSG: BURN TIMER EXPIRED, UNLOCKING FINS");
        }

        break;

      case STATE_COAST:

        imu_calc_att(&fltdata, dt);
        nav_update_pid(&fltdata, dt);

        if ((millis() - burn_start) >= config.parachute_charge_timeout_ms)
        {
          state = STATE_RECVY;
          Serial1.println("MSG: PARACHUTE DELAY CHARGE TIMER EXPIRED, DISABLING CONTROL");
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

      static uint32_t last_log_time = 0;
      if ((millis() - last_log_time) >= config.log_interval_ms)
      {
        log_write_frame(&fltdata, state, millis());
        last_log_time = millis();
      }

      comms_send_telem(state, &fltdata);
    }
  }

  comms_read_cmd(&state);
}
