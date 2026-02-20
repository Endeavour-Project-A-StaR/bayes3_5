#include <Arduino.h>
#include <Adafruit_DPS310.h>
#include "baro.h"

static Adafruit_DPS310 dps;

bool baro_init()
{
    if (!dps.begin_I2C())
        return false;

    dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
    dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);

    return true;
}

void baro_read(FltData_t *fltdata)
{
    if (dps.temperatureAvailable() && dps.pressureAvailable()){
        sensors_event_t temp_evt, pressure_evt;

        dps.getEvents(&temp_evt, &pressure_evt);

        fltdata->pressure = pressure_evt.pressure;
    }
}
