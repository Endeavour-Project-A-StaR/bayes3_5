#include "log.h"

int serializer(char *buf, size_t buf_size, uint32_t timestamp, FltStates_t state, const FltData_t *data)
{
    return snprintf(buf, buf_size,
                    "{\"timestamp\":%lu,\"state\":%d,"
                    "\"raw_accel\":[%.3f,%.3f,%.3f],"
                    "\"raw_gyro\":[%.3f,%.3f,%.3f],"
                    "\"pressure\":%.3f,\"altitude\":%.3f,"
                    "\"quats\":[%.3f,%.3f,%.3f,%.3f],"
                    "\"euler_angle\":[%.3f,%.3f,%.3f],"
                    "\"servo\":[%.1f,%.1f,%.1f,%.1f],"
                    "\"gyro_bias\":[%.3f,%.3f,%.3f]}",
                    timestamp, (int)state,
                    data->accel[0], data->accel[1], data->accel[2],
                    data->gyro[0], data->gyro[1], data->gyro[2],
                    data->pressure, data->altitude,
                    data->quat[0], data->quat[1], data->quat[2], data->quat[3],
                    data->pitch, data->yaw, data->roll,
                    data->servo_out[0], data->servo_out[1], data->servo_out[2], data->servo_out[3],
                    data->gyro_bias[0], data->gyro_bias[1], data->gyro_bias[2]);
}