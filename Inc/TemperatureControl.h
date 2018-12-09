#pragma once

#include <stdint.h>

class TemperatureSensor;
class SigmaDeltaPwm;

class TemperatureControl {
public:
    TemperatureControl(SigmaDeltaPwm& heater, TemperatureSensor& tempSensor);

    void on_halt(bool flg);
    bool is_halted() const {
        return halted;
    }

    void set_desired_temperature(float desired_temperature);
    float get_temperature();
    void broadcast_halt(bool halt) {
        halted = halt;
        on_halt(halt);
    }
    void set_max_temp(float max_temp) {
        this->max_temp = max_temp;
    }
    void thermistor_read_tick(void);
    void check_runaway();
    void setPIDp(float p) {
        p_factor = p;
    }

    void setPIDi(float i) {
        i_factor = i * PIDdt;
    }

    void setPIDd(float d) {
        d_factor = d / PIDdt;
    }

    void handle_autopid(float target, int ncycles);
private:
    friend class PID_Autotuner;

    //bool configure(ConfigReader& cr, ConfigReader::section_map_t& m);

    void pid_process(float);


    float target_temperature;
    float max_temp;
    float min_temp;

    TemperatureSensor& sensor;
    float i_max;
    int o;
    float last_reading;
    float readings_per_second;

    SigmaDeltaPwm& heater;

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

    RUNAWAY_TYPE runaway_state;
    // Temperature runaway config options
    uint8_t runaway_range; // max 63
    uint16_t runaway_heating_timeout; // 4088 secs
    uint16_t runaway_cooling_timeout; // 4088 secs
    uint16_t runaway_timer;
    uint8_t tick;
    bool use_bangbang;
    bool temp_violated;
    bool windup;
    bool halted;
};

