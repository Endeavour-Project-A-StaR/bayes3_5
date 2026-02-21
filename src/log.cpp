#include "log.h"
#include "eeprom_config.h"
#include <SdFat.h>

static SdFs sd;
static FsFile logfile;

static bool sd_ready = false;
static bool logfile_open = false;

int serializer(char *buf, size_t buf_size, uint32_t timestamp, FltStates_t state, const FltData_t *data)
{
    return snprintf(buf, buf_size,
                    "{\"timestamp\":%lu,\"state\":%d,"
                    "\"raw_accel\":[%.3f,%.3f,%.3f],"
                    "\"raw_gyro\":[%.3f,%.3f,%.3f],"
                    "\"pressure\":%.3f,\"altitude\":%.3f,"
                    "\"quats\":[%.3f,%.3f,%.3f,%.3f],"
                    "\"servo\":[%.1f,%.1f,%.1f,%.1f],"
                    "\"gyro_bias\":[%.3f,%.3f,%.3f]}",
                    timestamp, (int)state,
                    data->accel[0], data->accel[1], data->accel[2],
                    data->gyro[0], data->gyro[1], data->gyro[2],
                    data->pressure, data->altitude,
                    data->quat[0], data->quat[1], data->quat[2], data->quat[3],
                    data->servo_out[0], data->servo_out[1], data->servo_out[2], data->servo_out[3],
                    data->gyro_bias[0], data->gyro_bias[1], data->gyro_bias[2]);
}

bool log_init()
{
    if (sd_init())
    {
        Serial1.println("MSG: SD INIT SUCCESS");
        if (logfile_init())
        {
            Serial1.println("MSG: LOG FILE CREATED");
        }
        else
        {
            Serial1.println("MSG: LOG FILE CREATION FAILED");
            return false;
        }
    }
    else
    {
        Serial1.println("MSG: SD INIT FAILED");
        return false;
    }
    return true;
}

bool sd_init()
{
    if (!sd.begin(SdioConfig(FIFO_SDIO)))
    {
        sd_ready = false;
        return false;
    }

    sd_ready = true;
    return true;
}

bool logfile_init()
{
    if (!sd_ready)
        return false;

    char filename[32];

    for (int i = 0; i < 1000; i++)
    {
        snprintf(filename, sizeof(filename), "flightlog_%03d.json", i);
        if (!sd.exists(filename))
            break;
    }

    if (!logfile.open(filename, O_WRONLY | O_CREAT | O_EXCL))
    {
        logfile_open = false;
        return false;
    }

    logfile_open = true;

    logfile.println("[");
    logfile.sync();

    return true;
}

bool log_write_frame(FltData_t *fltdata, FltStates_t state, uint32_t timestamp)
{
    if (!logfile_open)
        return false;

    char buf[512];
    int len = serializer(buf, sizeof(buf), timestamp, state, fltdata);

    if (((len > 0) && ((size_t)len < sizeof(buf))))
    {
        logfile.print(buf);
        logfile.println(",");

        static uint32_t last_sync_time = 0;

        if (timestamp - last_sync_time > config.log_flush_interval_ms)
        {
            logfile.sync();
            last_sync_time = timestamp;
        }

        return true;
    }

    return false;
}