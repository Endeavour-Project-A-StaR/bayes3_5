#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "log.h"
#include "nav.h"
#include "eeprom_config.h"
#include "comms.h"

static uint32_t last_print_time = 0;

static char cmd_buf[64];
static uint8_t cmd_idx = 0;

typedef enum
{
    T_F32,
    T_U32,
    T_BOOL
} VarType_t;

typedef struct
{
    const char *name;
    void *ptr;
    VarType_t type;
} ConfigEntry_t;

const ConfigEntry_t config_table[] = {

    {"PITCH_KP", &config.pid_pitch.kp, T_F32},
    {"PITCH_KI", &config.pid_pitch.ki, T_F32},
    {"PITCH_KD", &config.pid_pitch.kd, T_F32},

    {"ROLL_KP", &config.pid_roll.kp, T_F32},
    {"ROLL_KI", &config.pid_roll.ki, T_F32},
    {"ROLL_KD", &config.pid_roll.kd, T_F32},

    {"YAW_KP", &config.pid_yaw.kp, T_F32},
    {"YAW_KI", &config.pid_yaw.ki, T_F32},
    {"YAW_KD", &config.pid_yaw.kd, T_F32},

    {"PID_I_MAX", &config.pid_i_max, T_F32},

    {"SERVO_CENTER_US", &config.servo_center_us, T_F32},
    {"SERVO_FLT_LIM_DEG", &config.servo_limit_max_deg, T_F32},
    {"SERVO_US_PER_DEG", &config.servo_us_per_deg, T_F32},

    {"PARACHUTE_TIMEOUT_FROM_IGN_MS", &config.parachute_charge_timeout_ms, T_U32},
    {"MOTOR_BURN_MS", &config.motor_burn_time_ms, T_U32},

    {"LOG_RATE_MS", &config.log_interval_ms, T_U32},
    {"LOG_FLUSH_MS", &config.log_flush_interval_ms, T_U32},

    {"SERVO_BURN_EN", &config.en_servo_in_burn, T_BOOL},
    {"INVERTED_TEST_EN", &config.test_mode_en, T_BOOL}};

const size_t NUM_CONFIG_ENTRIES = sizeof(config_table) / sizeof(config_table[0]);

// TELEM ONLY SENT TO USB ACM
void comms_send_telem(FltStates_t state, FltData_t *fltdata)
{
    if ((state == STATE_OVRD || state == STATE_PREFLT) &&
        (millis() - last_print_time >= 50))
    {
        last_print_time = millis();

        static char ser_buf[512];

        int len = serializer(ser_buf, sizeof(ser_buf), millis(), state, fltdata);

        if (len > 0 && (size_t)len < sizeof(ser_buf))
            Serial.println(ser_buf);
    }
}

static void cmd_processor(char *cmd_str, FltStates_t *state)
{
    bool flt_lockout_en = (*state == STATE_NAVLK || *state == STATE_BURN || *state == STATE_COAST || *state == STATE_RECVY);

    if (flt_lockout_en)
    {
        Serial1.println("MSG: COMMAND IGNORED IN FLIGHT LOCKOUT");
        return;
    }

    char *cmd = strtok(cmd_str, " ");
    if (!cmd)
        return;
    char *arg1 = strtok(NULL, " ");
    char *arg2 = strtok(NULL, " ");

    if (strcmp(cmd, "ARM") == 0)
    {
        *state = STATE_NAVLK;
        nav_rst_integral();
        Serial1.println("MSG: GUIDANCE IS INTERNAL");
    }
    else if (strcmp(cmd, "OVRD") == 0)
    {
        *state = STATE_OVRD;
        nav_rst_integral();
        Serial1.println("MSG: GROUND OVERRIDE MODE");
    }
    else if (strcmp(cmd, "PREFLT") == 0)
    {
        *state = STATE_PREFLT;
        Serial1.println("MSG: REVERTED TO PREFLT");
    }

    else if (strcmp(cmd, "SET") == 0)
    {
        if (!arg1 || !arg2)
        {
            Serial1.println("MSG: SYNTAX ERROR. USE: SET <VAR> <VALUE>");
            return;
        }

        bool found = false;

        for (size_t i = 0; i < NUM_CONFIG_ENTRIES; i++)
        {
            if (strcmp(arg1, config_table[i].name) == 0)
            {

                if (config_table[i].type == T_F32)
                {
                    *(float *)config_table[i].ptr = atof(arg2);
                    Serial1.printf("MSG: %s = %.3f\n", config_table[i].name, *(float *)config_table[i].ptr);
                }
                else if (config_table[i].type == T_U32)
                {
                    *(uint32_t *)config_table[i].ptr = strtoul(arg2, NULL, 10);
                    Serial1.printf("MSG: %s = %lu\n", config_table[i].name, *(uint32_t *)config_table[i].ptr);
                }
                else if (config_table[i].type == T_BOOL)
                {
                    *(bool *)config_table[i].ptr = atoi(arg2) > 0;
                    Serial1.printf("MSG: %s = %d\n", config_table[i].name, *(bool *)config_table[i].ptr);
                }

                found = true;
                break;
            }
        }
        if (!found)
            Serial1.println("MSG: UNKNOWN TUNEABLE VARIABLE");
    }

    else if (strcmp(cmd, "DUMP") == 0)
    {

        for (size_t i = 0; i < NUM_CONFIG_ENTRIES; i++)
        {
            if (config_table[i].type == T_F32)
            {
                Serial1.printf("CFG: %s %.3f\n", config_table[i].name, *(float *)config_table[i].ptr);
            }
            else if (config_table[i].type == T_U32)
            {
                Serial1.printf("CFG: %s %lu\n", config_table[i].name, *(uint32_t *)config_table[i].ptr);
            }
            else if (config_table[i].type == T_BOOL)
            {
                Serial1.printf("CFG: %s %d\n", config_table[i].name, *(bool *)config_table[i].ptr);
            }
        }
    }

    else if (strcmp(cmd, "SAVE") == 0)
    {
        config_save();
        Serial1.println("MSG: CONFIG SAVED TO EEPROM");
    }

    else if (strcmp(cmd, "DEFAULT") == 0)
    {
        config_set_defaults();
        config_save();
        Serial1.println("MSG: EEPROM RESET TO DEFAULTS");
    }

    else if (strcmp(cmd, "MAGICRESET") == 0)
    {
        SCB_AIRCR = 0x05FA0004;
    }

    else
    {
        Serial1.println("MSG: UNKNOWN COMMAND");
    }
}

void comms_read_cmd(FltStates_t *state)
{
    while (Serial1.available())
    {
        char c = Serial1.read();

        if (c == '\n' || c == '\r')
        {
            if (cmd_idx > 0)
            {
                cmd_buf[cmd_idx] = '\0';
                cmd_processor(cmd_buf, state);
                cmd_idx = 0;
            }
        }
        else if (cmd_idx < sizeof(cmd_buf) - 1)
        {
            cmd_buf[cmd_idx++] = c;
        }
    }
}
