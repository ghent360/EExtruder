#pragma once

#include <stdint.h>

class TempSensor;
class SigmaDeltaPwm;

class TemperatureControl {
public:
    TemperatureControl();
    ~TemperatureControl();

    //static bool load_controls(ConfigReader& cr);
    void on_halt(bool flg);
    bool is_halted();

    void set_desired_temperature(float desired_temperature);
    float get_temperature();

private:
    friend class PID_Autotuner;

    //bool configure(ConfigReader& cr, ConfigReader::section_map_t& m);

    void thermistor_read_tick(void);
    void pid_process(float);

    void setPIDp(float p) {
        p_factor = p;
    }

    void setPIDi(float i) {
        i_factor = i * PIDdt;
    }

    void setPIDd(float d) {
        d_factor = d / PIDdt;
    }

    void check_runaway();
    //bool handle_mcode(GCode& gcode, OutputStream& os);
    //bool handle_M6(GCode& gcode, OutputStream& os);
    //bool handle_autopid(GCode& gcode, OutputStream& os);

    float target_temperature;
    float max_temp, min_temp;

    float preset1;
    float preset2;

    //TempSensor *sensor{nullptr};
    float i_max;
    int o;
    float last_reading;
    float readings_per_second;

    SigmaDeltaPwm *heater_pin{nullptr};

    float hysteresis;
    float iTerm;
    float lastInput;
    // PID settings
    float p_factor;
    float i_factor;
    float d_factor;
    float PIDdt;

    float runaway_error_range;

    enum RUNAWAY_TYPE {
        NOT_HEATING,
        HEATING_UP,
        COOLING_DOWN,
        TARGET_TEMPERATURE_REACHED
    };

    uint8_t tool_id{0};

    RUNAWAY_TYPE runaway_state;
    // Temperature runaway config options
    uint8_t runaway_range; // max 63
    uint16_t runaway_heating_timeout; // 4088 secs
    uint16_t runaway_cooling_timeout; // 4088 secs
    uint16_t runaway_timer;
    uint8_t tick;
    bool use_bangbang;
    bool temp_violated;
    bool active;
    bool readonly;
    bool windup;
    bool sensor_settings;
};

