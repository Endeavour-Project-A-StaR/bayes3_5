#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "log.h"
#include "nav.h"
#include "persistent_config.h"
#include "comms.h"

static uint32_t last_print_time = 0;

static char cmd_buf[64];
static uint8_t cmd_idx = 0;

void comms_send_telem(FltStates_t state, FltData_t *fltdata)
{
    if ((state == STATE_OVRD || state == STATE_PREFLT) &&
        (millis() - last_print_time >= config.log_interval_ms))
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
        Serial.println("MSG: COMMAND IGNORED IN FLIGHT LOCKOUT");
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
        Serial.println("MSG: GUIDANCE IS INTERNAL");
    }
    else if (strcmp(cmd, "OVRD") == 0)
    {
        *state = STATE_OVRD;
        nav_rst_integral();
        Serial.println("MSG: GROUND OVERRIDE MODE");
    }
    else if (strcmp(cmd, "PREFLT") == 0)
    {
        *state = STATE_PREFLT;
        Serial.println("MSG: REVERTED TO PREFLT");
    }

    else if (strcmp(cmd, "SET") == 0)
    {
        if (!arg1 || !arg2)
        {
            Serial.println("MSG: SYNTAX ERROR. SET <VAR> <VALUE>");
            return;
        }

        float val = atof(arg2);

        if (strcmp(arg1, "PITCH_KP") == 0)
        {
            config.pid_pitch.kp = val;
            Serial.printf("MSG: PITCH_KP %.3f\n", config.pid_pitch.kp);
        }
        else if (strcmp(arg1, "PITCH_KI") == 0)
        {
            config.pid_pitch.ki = val;
            Serial.printf("MSG: PITCH_KI %.3f\n", config.pid_pitch.ki);
        }
        else if (strcmp(arg1, "PITCH_KD") == 0)
        {
            config.pid_pitch.kd = val;
            Serial.printf("MSG: PITCH_KD %.3f\n", config.pid_pitch.kd);
        }

        else if (strcmp(arg1, "ROLL_KP") == 0)
        {
            config.pid_roll.kp = val;
            Serial.printf("MSG: ROLL_KP %.3f\n", config.pid_roll.kp);
        }
        else if (strcmp(arg1, "ROLL_KI") == 0)
        {
            config.pid_roll.ki = val;
            Serial.printf("MSG: ROLL_KI %.3f\n", config.pid_roll.ki);
        }
        else if (strcmp(arg1, "ROLL_KD") == 0)
        {
            config.pid_roll.kd = val;
            Serial.printf("MSG: ROLL_KD %.3f\n", config.pid_roll.kd);
        }

        else if (strcmp(arg1, "YAW_KP") == 0)
        {
            config.pid_yaw.kp = val;
            Serial.printf("MSG: YAW_KP %.3f\n", config.pid_yaw.kp);
        }
        else if (strcmp(arg1, "YAW_KI") == 0)
        {
            config.pid_yaw.ki = val;
            Serial.printf("MSG: YAW_KI %.3f\n", config.pid_yaw.ki);
        }
        else if (strcmp(arg1, "YAW_KD") == 0)
        {
            config.pid_yaw.kd = val;
            Serial.printf("MSG: YAW_KD %.3f\n", config.pid_yaw.kd);
        }

        else if (strcmp(arg1, "SERVO_CENTER") == 0)
        {
            config.servo_center_us = val;
            Serial.printf("MSG: SERVO_CENTER %.1f\n", config.servo_center_us);
        }
        else if (strcmp(arg1, "SERVO_LIMIT") == 0)
        {
            config.servo_limit_max_deg = val;
            Serial.printf("MSG: SERVO_LIMIT %.1f\n", config.servo_limit_max_deg);
        }

        else if (strcmp(arg1, "CHUTE_TIMEOUT_MS") == 0)
        {
            config.parachute_charge_timeout_ms = (uint32_t)val;
            Serial.printf("MSG: CHUTE_TIMEOUT_MS %lu\n", config.parachute_charge_timeout_ms);
        }

        // Telemetry Rate
        else if (strcmp(arg1, "TELEM_RATE") == 0)
        {
            config.log_interval_ms = (uint32_t)val;
            Serial.printf("MSG: TELEM_RATE %lu\n", config.log_interval_ms);
        }
        else if (strcmp(arg1, "TELEM_FLUSH_RATE") == 0)
        {
            config.log_flush_interval_ms = (uint32_t)val;
            Serial.printf("MSG: FLUSH_RATE %lu\n", config.log_flush_interval_ms);
        }

        else if (strcmp(arg1, "SERVO_BURN_EN") == 0)
        {
            config.en_servo_in_burn = ((uint32_t)val == 0 ? false : true);
            Serial.printf("MSG: SERVO_BURN_EN %d\n", config.en_servo_in_burn);
        }
        else if (strcmp(arg1, "TEST_MODE_EN") == 0)
        {
            config.test_mode_en = ((uint32_t)val == 0 ? false : true);
            Serial.printf("MSG: TEST_MODE_EN %d\n", config.test_mode_en);
        }

        else
        {
            Serial.println("MSG: UNKNOWN TUNEABLE VARIABLE");
        }
    }

    else if (strcmp(cmd, "DUMP") == 0)
    {

        Serial.printf("CFG: PITCH_KP %.3f\n", config.pid_pitch.kp);
        Serial.printf("CFG: PITCH_KI %.3f\n", config.pid_pitch.ki);
        Serial.printf("CFG: PITCH_KD %.3f\n", config.pid_pitch.kd);

        Serial.printf("CFG: ROLL_KP %.3f\n", config.pid_roll.kp);
        Serial.printf("CFG: ROLL_KI %.3f\n", config.pid_roll.ki);
        Serial.printf("CFG: ROLL_KD %.3f\n", config.pid_roll.kd);

        Serial.printf("CFG: YAW_KP %.3f\n", config.pid_yaw.kp);
        Serial.printf("CFG: YAW_KI %.3f\n", config.pid_yaw.ki);
        Serial.printf("CFG: YAW_KD %.3f\n", config.pid_yaw.kd);

        Serial.printf("CFG: PID_I_MAX %.3f\n", config.pid_i_max);

        Serial.printf("CFG: SERVO_CENTER %.1f\n", config.servo_center_us);
        Serial.printf("CFG: SERVO_LIMIT %.1f\n", config.servo_limit_max_deg);
        Serial.printf("CFG: SERVO_US_PER_DEG %.1f\n", config.servo_us_per_deg);

        Serial.printf("CFG: CHUTE_TIMEOUT_MS %lu\n", config.parachute_charge_timeout_ms);
        Serial.printf("CFG: TELEM_RATE %lu\n", config.log_interval_ms);
        Serial.printf("CFG: FLUSH_RATE %lu\n", config.log_flush_interval_ms);
        Serial.printf("CFG: TEST_MODE_EN %d\n", config.test_mode_en);
        Serial.printf("CFG: SERVO_BURN_EN %d\n", config.en_servo_in_burn);
    }

    else if (strcmp(cmd, "SAVE") == 0)
    {
        config_save();
        Serial.println("MSG: CONFIG SAVED TO EEPROM");
    }

    else if (strcmp(cmd, "DEFAULT") == 0)
    {
        config_set_defaults();
        config_save();
        Serial.println("MSG: EEPROM RESET TO DEFAULTS");
    }

    else if (strcmp(cmd, "MAGICRESET") == 0)
    {
        SCB_AIRCR = 0x05FA0004;
    }

    else
    {
        Serial.println("MSG: UNKNOWN COMMAND");
    }
}

void comms_read_cmd(FltStates_t *state)
{
    while (Serial.available())
    {
        char c = Serial.read();

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
