#include <Arduino.h>
#include <Wire.h>
#include "types.h"
#include "imu.h"

FltStates_t state = STATE_DIAG;  // Default startup to self test
FltData_t fltdata;  // Init shared flight data struct

void setup() {

  Wire.begin();
  Wire.setClock(1000000);
  
  Serial.begin(0); // ACM doesnt need baud rate

  while(!Serial.available()) delay(1);

  int ret = imu_init();
  Serial.println(ret ? "IMU Init success" : "fuck you");

}

void loop() {
}
