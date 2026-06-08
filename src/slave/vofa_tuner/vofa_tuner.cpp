#include "slave/vofa_tuner/vofa_tuner.h"

#if SLAVE_VOFA_TUNER_ENABLED

#include <Arduino.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "common/math/clamp.h"
#include "slave/config/slave_config.h"
#include "slave/control/slave_coordinate_mapper.h"
#include "slave/control/slave_motion.h"
#include "slave/hardware/slave_hardware.h"

namespace {

struct SlaveVofaCommand {
    uint32_t sequence;
    bool active;
    AxisId axis;
    SlaveVofaTunerMode mode;
    float p;
    float i;
    float d;
    float amplitude;
    float current_limit_a;
    float voltage_limit_v;
    float velocity_limit_rad_s;
    float angle_limit_rad;
    uint32_t wave_period_ms;
    uint32_t sample_period_ms;
};

struct SlaveVofaSample {
    uint32_t timestamp_us;
    uint32_t sequence;
    bool active;
    AxisId axis;
    SlaveVofaTunerMode mode;
    float setpoint;
    float input;
    float output;
    float error;
    float p;
    float i;
    float d;
    float aux1;
    float aux2;
    float aux3;
};

portMUX_TYPE command_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE sample_mux = portMUX_INITIALIZER_UNLOCKED;
SlaveVofaCommand pending_command = {
    0U,
    false,
    AXIS_X,
    SLAVE_VOFA_MODE_OFF,
    0.0f,
    0.0f,
    0.0f,
    SLAVE_VOFA_CURRENT_LIMIT_DEFAULT_A,
    SLAVE_VOFA_CURRENT_LIMIT_DEFAULT_A,
    SLAVE_VOFA_VOLTAGE_LIMIT_DEFAULT_V,
    SLAVE_VOFA_VELOCITY_LIMIT_DEFAULT_RAD_S,
    SLAVE_VOFA_ANGLE_LIMIT_DEFAULT_RAD,
    SLAVE_VOFA_WAVE_PERIOD_DEFAULT_MS,
    SLAVE_VOFA_SAMPLE_PERIOD_DEFAULT_MS,
};
SlaveVofaSample latest_sample = {};
volatile bool tuner_requested = false;
volatile bool tuner_control_active = false;

const char *modeName(SlaveVofaTunerMode mode) {
    switch (mode) {
        case SLAVE_VOFA_MODE_CURRENT_Q:
            return "CURRENT_Q";
        case SLAVE_VOFA_MODE_VELOCITY:
            return "VELOCITY";
        case SLAVE_VOFA_MODE_ANGLE:
            return "ANGLE";
        case SLAVE_VOFA_MODE_OFF:
        default:
            return "OFF";
    }
}

uint8_t motorTuningMode(SlaveVofaTunerMode mode) {
    switch (mode) {
        case SLAVE_VOFA_MODE_CURRENT_Q:
            return SLAVE_MOTOR_TUNING_CURRENT_Q;
        case SLAVE_VOFA_MODE_VELOCITY:
            return SLAVE_MOTOR_TUNING_VELOCITY;
        case SLAVE_VOFA_MODE_ANGLE:
            return SLAVE_MOTOR_TUNING_ANGLE;
        case SLAVE_VOFA_MODE_OFF:
        default:
            return 0;
    }
}

bool axisEnabled(AxisId axis) {
    return axis == AXIS_X ? (SLAVE_VOFA_TUNER_X_ENABLED != 0)
                          : (SLAVE_VOFA_TUNER_Y_ENABLED != 0);
}

float parseFloat(const char *text, bool *valid) {
    char *end = nullptr;
    const float value = strtof(text, &end);
    *valid = text != end && end != nullptr && *end == '\0' && isfinite(value);
    return value;
}

uint32_t parseUint32(const char *text, bool *valid) {
    char *end = nullptr;
    const unsigned long value = strtoul(text, &end, 10);
    *valid = text != end && end != nullptr && *end == '\0';
    return static_cast<uint32_t>(value);
}

float pidMaxP(SlaveVofaTunerMode mode) {
    return mode == SLAVE_VOFA_MODE_CURRENT_Q
               ? SLAVE_VOFA_CURRENT_PID_P_MAX
               : (mode == SLAVE_VOFA_MODE_VELOCITY ? SLAVE_VOFA_VELOCITY_PID_P_MAX
                                                    : SLAVE_VOFA_ANGLE_PID_P_MAX);
}

float pidMaxI(SlaveVofaTunerMode mode) {
    return mode == SLAVE_VOFA_MODE_CURRENT_Q
               ? SLAVE_VOFA_CURRENT_PID_I_MAX
               : (mode == SLAVE_VOFA_MODE_VELOCITY ? SLAVE_VOFA_VELOCITY_PID_I_MAX
                                                    : SLAVE_VOFA_ANGLE_PID_I_MAX);
}

float pidMaxD(SlaveVofaTunerMode mode) {
    return mode == SLAVE_VOFA_MODE_CURRENT_Q
               ? SLAVE_VOFA_CURRENT_PID_D_MAX
               : (mode == SLAVE_VOFA_MODE_VELOCITY ? SLAVE_VOFA_VELOCITY_PID_D_MAX
                                                    : SLAVE_VOFA_ANGLE_PID_D_MAX);
}

void setDefaultPid(SlaveVofaCommand &command) {
    const SlaveMotorFocConfig &config =
        command.axis == AXIS_X ? kSlaveXMotorFoc : kSlaveYMotorFoc;
    switch (command.mode) {
        case SLAVE_VOFA_MODE_CURRENT_Q:
            command.p = config.current_loop.q.p;
            command.i = config.current_loop.q.i;
            command.d = config.current_loop.q.d;
            command.amplitude = fminf(SLAVE_VOFA_CURRENT_LIMIT_DEFAULT_A,
                                      command.current_limit_a);
            break;
        case SLAVE_VOFA_MODE_VELOCITY:
            command.p = config.velocity.p;
            command.i = config.velocity.i;
            command.d = config.velocity.d;
            command.amplitude = fminf(SLAVE_VOFA_VELOCITY_LIMIT_DEFAULT_RAD_S,
                                      command.velocity_limit_rad_s);
            break;
        case SLAVE_VOFA_MODE_ANGLE:
            command.p = config.position.p;
            command.i = config.position.i;
            command.d = config.position.d;
            command.amplitude = fminf(SLAVE_VOFA_ANGLE_LIMIT_DEFAULT_RAD,
                                      command.angle_limit_rad);
            break;
        case SLAVE_VOFA_MODE_OFF:
        default:
            command.p = 0.0f;
            command.i = 0.0f;
            command.d = 0.0f;
            command.amplitude = 0.0f;
            break;
    }
}

float activeAmplitudeLimit(const SlaveVofaCommand &command) {
    switch (command.mode) {
        case SLAVE_VOFA_MODE_CURRENT_Q:
            return command.current_limit_a;
        case SLAVE_VOFA_MODE_VELOCITY:
            return command.velocity_limit_rad_s;
        case SLAVE_VOFA_MODE_ANGLE:
            return command.angle_limit_rad;
        case SLAVE_VOFA_MODE_OFF:
        default:
            return 0.0f;
    }
}

void publishCommand(SlaveVofaCommand &command) {
    command.sequence++;
    portENTER_CRITICAL(&command_mux);
    pending_command = command;
    tuner_requested = command.active;
    portEXIT_CRITICAL(&command_mux);
}

SlaveVofaCommand snapshotCommand() {
    SlaveVofaCommand command = {};
    portENTER_CRITICAL(&command_mux);
    command = pending_command;
    portEXIT_CRITICAL(&command_mux);
    return command;
}

void publishSample(const SlaveVofaSample &sample) {
    portENTER_CRITICAL(&sample_mux);
    latest_sample = sample;
    portEXIT_CRITICAL(&sample_mux);
}

SlaveVofaSample snapshotSample() {
    SlaveVofaSample sample = {};
    portENTER_CRITICAL(&sample_mux);
    sample = latest_sample;
    portEXIT_CRITICAL(&sample_mux);
    return sample;
}

void lowerAscii(char *line) {
    while (*line != '\0') {
        *line = static_cast<char>(tolower(static_cast<unsigned char>(*line)));
        line++;
    }
}

AxisId parseAxis(const char *token, bool *valid) {
    if (strcmp(token, "x") == 0) {
        *valid = true;
        return AXIS_X;
    }
    if (strcmp(token, "y") == 0) {
        *valid = true;
        return AXIS_Y;
    }
    *valid = false;
    return AXIS_X;
}

SlaveVofaTunerMode parseMode(const char *token) {
    if (strcmp(token, "current_q") == 0 || strcmp(token, "current") == 0) {
        return SLAVE_VOFA_MODE_CURRENT_Q;
    }
    if (strcmp(token, "velocity") == 0 || strcmp(token, "vel") == 0) {
        return SLAVE_VOFA_MODE_VELOCITY;
    }
    if (strcmp(token, "angle") == 0 || strcmp(token, "position") == 0) {
        return SLAVE_VOFA_MODE_ANGLE;
    }
    return SLAVE_VOFA_MODE_OFF;
}

void printHelp() {
    Serial.println("# VOFA tuner commands");
    Serial.println("# vofa start <x|y> <current_q|velocity|angle>");
    Serial.println("# vofa pid <p> <i> <d>");
    Serial.println("# vofa amp <value>");
    Serial.println("# vofa limits <current_a> <voltage_v> <velocity_rad_s> <angle_rad>");
    Serial.println("# vofa sample <20..50_ms>");
    Serial.println("# vofa wave <2000..60000_ms>");
    Serial.println("# vofa status");
    Serial.println("# vofa stop");
    Serial.println("# tune order: CURRENT_Q -> VELOCITY -> ANGLE");
}

void printStatus(const SlaveVofaCommand &command) {
    Serial.printf("# active=%u axis=%s mode=%s pid=%.6f,%.6f,%.6f amp=%.6f limits=%.6fA,%.6fV,%.6frad/s,%.6frad wave=%lums sample=%lums\n",
                  command.active ? 1U : 0U,
                  axisIdName(command.axis),
                  modeName(command.mode),
                  command.p,
                  command.i,
                  command.d,
                  command.amplitude,
                  command.current_limit_a,
                  command.voltage_limit_v,
                  command.velocity_limit_rad_s,
                  command.angle_limit_rad,
                  static_cast<unsigned long>(command.wave_period_ms),
                  static_cast<unsigned long>(command.sample_period_ms));
}

void printCsvHeader() {
    Serial.println("# timestamp,setpoint,input,output,error,p,i,d,aux1,aux2,aux3");
}

void handleCommand(char *line, SlaveVofaCommand &command) {
    lowerAscii(line);
    char *save = nullptr;
    char *root = strtok_r(line, " \t", &save);
    if (root == nullptr) {
        return;
    }
    if (strcmp(root, "vofa") != 0) {
        Serial.println("# error: command must start with 'vofa'");
        return;
    }

    char *action = strtok_r(nullptr, " \t", &save);
    if (action == nullptr || strcmp(action, "help") == 0) {
        printHelp();
        return;
    }
    if (strcmp(action, "status") == 0) {
        printStatus(command);
        return;
    }
    if (strcmp(action, "stop") == 0) {
        command.active = false;
        command.mode = SLAVE_VOFA_MODE_OFF;
        command.amplitude = 0.0f;
        publishCommand(command);
        Serial.println("# stopped");
        return;
    }
    if (strcmp(action, "start") == 0) {
        char *axis_token = strtok_r(nullptr, " \t", &save);
        char *mode_token = strtok_r(nullptr, " \t", &save);
        bool axis_valid = false;
        if (axis_token == nullptr || mode_token == nullptr) {
            Serial.println("# error: start requires axis and mode");
            return;
        }
        const AxisId axis = parseAxis(axis_token, &axis_valid);
        const SlaveVofaTunerMode mode = parseMode(mode_token);
        if (!axis_valid || mode == SLAVE_VOFA_MODE_OFF || !axisEnabled(axis)) {
            Serial.println("# error: axis or mode is not enabled");
            return;
        }
        command.active = true;
        command.axis = axis;
        command.mode = mode;
        setDefaultPid(command);
        publishCommand(command);
        printCsvHeader();
        printStatus(command);
        return;
    }
    if (strcmp(action, "pid") == 0) {
        char *p_text = strtok_r(nullptr, " \t", &save);
        char *i_text = strtok_r(nullptr, " \t", &save);
        char *d_text = strtok_r(nullptr, " \t", &save);
        bool p_valid = false;
        bool i_valid = false;
        bool d_valid = false;
        if (p_text == nullptr || i_text == nullptr || d_text == nullptr) {
            Serial.println("# error: pid requires p i d");
            return;
        }
        const float p = parseFloat(p_text, &p_valid);
        const float i = parseFloat(i_text, &i_valid);
        const float d = parseFloat(d_text, &d_valid);
        if (!p_valid || !i_valid || !d_valid || !command.active) {
            Serial.println("# error: invalid pid or tuner is not active");
            return;
        }
        command.p = clampFloat(p, 0.0f, pidMaxP(command.mode));
        command.i = clampFloat(i, 0.0f, pidMaxI(command.mode));
        command.d = clampFloat(d, 0.0f, pidMaxD(command.mode));
        publishCommand(command);
        printStatus(command);
        return;
    }
    if (strcmp(action, "amp") == 0) {
        char *value_text = strtok_r(nullptr, " \t", &save);
        bool valid = false;
        if (value_text == nullptr || !command.active) {
            Serial.println("# error: amp requires an active tuner");
            return;
        }
        const float value = parseFloat(value_text, &valid);
        if (!valid) {
            Serial.println("# error: invalid amplitude");
            return;
        }
        command.amplitude = clampFloat(fabsf(value), 0.0f, activeAmplitudeLimit(command));
        publishCommand(command);
        printStatus(command);
        return;
    }
    if (strcmp(action, "limits") == 0) {
        char *current_text = strtok_r(nullptr, " \t", &save);
        char *voltage_text = strtok_r(nullptr, " \t", &save);
        char *velocity_text = strtok_r(nullptr, " \t", &save);
        char *angle_text = strtok_r(nullptr, " \t", &save);
        bool valid[4] = {};
        if (current_text == nullptr || voltage_text == nullptr ||
            velocity_text == nullptr || angle_text == nullptr) {
            Serial.println("# error: limits requires current voltage velocity angle");
            return;
        }
        const float current = parseFloat(current_text, &valid[0]);
        const float voltage = parseFloat(voltage_text, &valid[1]);
        const float velocity = parseFloat(velocity_text, &valid[2]);
        const float angle = parseFloat(angle_text, &valid[3]);
        if (!valid[0] || !valid[1] || !valid[2] || !valid[3]) {
            Serial.println("# error: invalid limits");
            return;
        }
        command.current_limit_a =
            clampFloat(fabsf(current), SLAVE_VOFA_CURRENT_LIMIT_MIN_A, SLAVE_VOFA_CURRENT_LIMIT_MAX_A);
        command.voltage_limit_v =
            clampFloat(fabsf(voltage), SLAVE_VOFA_VOLTAGE_LIMIT_MIN_V, SLAVE_VOFA_VOLTAGE_LIMIT_MAX_V);
        command.velocity_limit_rad_s =
            clampFloat(fabsf(velocity),
                       SLAVE_VOFA_VELOCITY_LIMIT_MIN_RAD_S,
                       SLAVE_VOFA_VELOCITY_LIMIT_MAX_RAD_S);
        command.angle_limit_rad =
            clampFloat(fabsf(angle), SLAVE_VOFA_ANGLE_LIMIT_MIN_RAD, SLAVE_VOFA_ANGLE_LIMIT_MAX_RAD);
        command.amplitude = fminf(command.amplitude, activeAmplitudeLimit(command));
        publishCommand(command);
        printStatus(command);
        return;
    }
    if (strcmp(action, "sample") == 0 || strcmp(action, "wave") == 0) {
        char *value_text = strtok_r(nullptr, " \t", &save);
        bool valid = false;
        if (value_text == nullptr) {
            Serial.println("# error: period value is required");
            return;
        }
        const uint32_t value = parseUint32(value_text, &valid);
        if (!valid) {
            Serial.println("# error: invalid period");
            return;
        }
        if (strcmp(action, "sample") == 0) {
            command.sample_period_ms =
                static_cast<uint32_t>(clampFloat(static_cast<float>(value),
                                                 static_cast<float>(SLAVE_VOFA_SAMPLE_PERIOD_MIN_MS),
                                                 static_cast<float>(SLAVE_VOFA_SAMPLE_PERIOD_MAX_MS)));
        } else {
            command.wave_period_ms =
                static_cast<uint32_t>(clampFloat(static_cast<float>(value),
                                                 static_cast<float>(SLAVE_VOFA_WAVE_PERIOD_MIN_MS),
                                                 static_cast<float>(SLAVE_VOFA_WAVE_PERIOD_MAX_MS)));
        }
        publishCommand(command);
        printStatus(command);
        return;
    }

    Serial.println("# error: unknown command");
    printHelp();
}

float triangleValue(uint32_t elapsed_us, uint32_t period_ms, float amplitude) {
    const uint32_t period_us = period_ms * 1000UL;
    const float phase =
        static_cast<float>(elapsed_us % period_us) / static_cast<float>(period_us);
    if (phase < 0.25f) {
        return amplitude * (phase * 4.0f);
    }
    if (phase < 0.50f) {
        return amplitude * (2.0f - phase * 4.0f);
    }
    if (phase < 0.75f) {
        return -amplitude * ((phase - 0.50f) * 4.0f);
    }
    return -amplitude * (4.0f - phase * 4.0f);
}

float clampAngleTarget(AxisId axis, float target_rad) {
    const float angle_a = slaveAxisPaperMmToGimbalAngleRad(axis, slaveAxisLimitMinMm(axis));
    const float angle_b = slaveAxisPaperMmToGimbalAngleRad(axis, slaveAxisLimitMaxMm(axis));
    return clampFloat(target_rad, fminf(angle_a, angle_b), fmaxf(angle_a, angle_b));
}

void fillSample(const SlaveVofaCommand &command,
                uint32_t timestamp_us,
                float setpoint,
                const SlaveMotorTuningFeedback &feedback,
                SlaveVofaSample &sample) {
    sample.timestamp_us = timestamp_us;
    sample.sequence++;
    sample.active = true;
    sample.axis = command.axis;
    sample.mode = command.mode;
    sample.setpoint = setpoint;
    sample.p = command.p;
    sample.i = command.i;
    sample.d = command.d;
    sample.aux1 = feedback.shaft_angle_rad;
    sample.aux2 = feedback.shaft_velocity_rad_s;
    sample.aux3 = feedback.current_q_a;

    switch (command.mode) {
        case SLAVE_VOFA_MODE_CURRENT_Q:
            sample.input = feedback.current_q_a;
            sample.output = feedback.voltage_q_v;
            break;
        case SLAVE_VOFA_MODE_VELOCITY:
            sample.input = feedback.shaft_velocity_rad_s;
            sample.output = feedback.current_setpoint_a;
            break;
        case SLAVE_VOFA_MODE_ANGLE:
            sample.input = feedback.shaft_angle_rad;
            sample.output = feedback.current_setpoint_a;
            break;
        case SLAVE_VOFA_MODE_OFF:
        default:
            sample.input = 0.0f;
            sample.output = 0.0f;
            break;
    }
    sample.error = sample.setpoint - sample.input;
}

}  // namespace

void runSlaveVofaTunerIoStep() {
    static char line[128] = {};
    static size_t line_length = 0;
    static SlaveVofaCommand io_command = pending_command;
    static uint32_t last_print_ms = 0;
    static uint32_t last_sample_sequence = 0;
    static bool ready_printed = false;

    if (!ready_printed) {
        Serial.printf("# VOFA tuner ready x=%u y=%u baud=115200\n",
                      SLAVE_VOFA_TUNER_X_ENABLED ? 1U : 0U,
                      SLAVE_VOFA_TUNER_Y_ENABLED ? 1U : 0U);
        printCsvHeader();
        printHelp();
        ready_printed = true;
    }

    while (Serial.available() > 0) {
        const int value = Serial.read();
        if (value < 0) {
            break;
        }
        const char ch = static_cast<char>(value);
        if (ch == '\r' || ch == '\n') {
            if (line_length > 0) {
                line[line_length] = '\0';
                handleCommand(line, io_command);
                line_length = 0;
                line[0] = '\0';
            }
            continue;
        }
        if (line_length + 1U < sizeof(line)) {
            line[line_length++] = ch;
        } else {
            line_length = 0;
            line[0] = '\0';
            Serial.println("# error: command too long");
        }
    }

    const uint32_t now_ms = millis();
    if (!io_command.active ||
        static_cast<uint32_t>(now_ms - last_print_ms) < io_command.sample_period_ms) {
        return;
    }
    const SlaveVofaSample sample = snapshotSample();
    if (!sample.active || sample.sequence == last_sample_sequence) {
        return;
    }
    Serial.printf("%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                  static_cast<unsigned long>(sample.timestamp_us / 1000UL),
                  sample.setpoint,
                  sample.input,
                  sample.output,
                  sample.error,
                  sample.p,
                  sample.i,
                  sample.d,
                  sample.aux1,
                  sample.aux2,
                  sample.aux3);
    last_print_ms = now_ms;
    last_sample_sequence = sample.sequence;
}

bool runSlaveVofaTunerControlStep(float dt_s, SlaveControlStepTiming *timing) {
    (void)dt_s;
    static SlaveVofaCommand active_command = pending_command;
    static uint32_t applied_sequence = 0;
    static uint32_t wave_start_us = 0;
    static AxisId configured_axis = AXIS_X;
    static bool configured = false;
    static float hold_x_rad = 0.0f;
    static float hold_y_rad = 0.0f;
    static SlaveVofaSample sample = {};

    if (timing != nullptr) {
        *timing = {};
    }

    const SlaveVofaCommand command = snapshotCommand();
    if (command.sequence != applied_sequence) {
        if (configured) {
            restoreSlaveMotorTuning(configured_axis);
            configured = false;
        }
        applied_sequence = command.sequence;
        active_command = command;
        if (!command.active) {
            tuner_control_active = false;
            sample.active = false;
            publishSample(sample);
            return false;
        }

        const SlaveMotorTuningFeedback x_feedback = snapshotSlaveMotorTuning(AXIS_X);
        const SlaveMotorTuningFeedback y_feedback = snapshotSlaveMotorTuning(AXIS_Y);
        hold_x_rad = x_feedback.shaft_angle_rad;
        hold_y_rad = y_feedback.shaft_angle_rad;
        if (!configureSlaveMotorTuning(command.axis,
                                       motorTuningMode(command.mode),
                                       command.p,
                                       command.i,
                                       command.d,
                                       command.current_limit_a,
                                       command.voltage_limit_v,
                                       command.velocity_limit_rad_s)) {
            tuner_control_active = false;
            sample.active = false;
            publishSample(sample);
            return true;
        }
        configured_axis = command.axis;
        configured = true;
        wave_start_us = micros();
        tuner_control_active = true;
    }

    if (!active_command.active || !configured) {
        tuner_control_active = false;
        return active_command.active;
    }

    const uint32_t now_us = micros();
    const float wave = triangleValue(now_us - wave_start_us,
                                     active_command.wave_period_ms,
                                     active_command.amplitude);
    float target = wave;
    if (active_command.mode == SLAVE_VOFA_MODE_ANGLE) {
        const float center = active_command.axis == AXIS_X ? hold_x_rad : hold_y_rad;
        target = clampAngleTarget(active_command.axis, center + wave);
    }

    if (active_command.axis == AXIS_X) {
        (void)applySlaveXMotorTarget(target, hold_x_rad, nullptr);
        (void)applySlaveYMotorTarget(hold_y_rad, hold_y_rad, nullptr);
    } else {
        (void)applySlaveXMotorTarget(hold_x_rad, hold_x_rad, nullptr);
        (void)applySlaveYMotorTarget(target, hold_y_rad, nullptr);
    }

    const SlaveMotorTuningFeedback feedback = snapshotSlaveMotorTuning(active_command.axis);
    fillSample(active_command, now_us, target, feedback, sample);
    publishSample(sample);
    return true;
}

bool isSlaveVofaTunerRequestedOrActive() {
    return tuner_requested || tuner_control_active;
}

#else

void runSlaveVofaTunerIoStep() {}

bool runSlaveVofaTunerControlStep(float dt_s, SlaveControlStepTiming *timing) {
    (void)dt_s;
    (void)timing;
    return false;
}

bool isSlaveVofaTunerRequestedOrActive() {
    return false;
}

#endif
