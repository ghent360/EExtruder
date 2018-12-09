#include "TemperatureControl.h"
#include "SigmaDeltaPwm.h"
#include "TemperatureSensor.h"
#include "PID_Autotuner.h"

#include <math.h>
#include <string.h>

// Temp sensor implementations:
//#include "Thermistor.h"
//#include "max31855.h"
//#include "AD8495.h"
//#include "PT100_E3D.h"

#define UNDEFINED -1

TemperatureControl::TemperatureControl(SigmaDeltaPwm& heaterPin, TemperatureSensor& tempSensor) 
    : target_temperature(UNDEFINED),
      max_temp(300),
      min_temp(0),
      sensor(tempSensor),
      i_max(heaterPin.max_pwm()),
      o(0),
      last_reading(0),
      readings_per_second(5),
      heater(heaterPin),
      hysteresis(2),
      iTerm(0),
      lastInput(-1),
      p_factor(10),
      i_factor(0.3),
      d_factor(200),
      PIDdt(1.0 / this->readings_per_second),
      runaway_error_range(1.0),
      runaway_state(NOT_HEATING),
      runaway_range(20),
      runaway_heating_timeout(900),
      runaway_cooling_timeout(0), // Disable
      runaway_timer(0),
      tick(0),
      use_bangbang(false),
      temp_violated(false),
      windup(true),
      halted(false) {}

/*
bool TemperatureControl::configure(ConfigReader& cr, ConfigReader::section_map_t& m)
{
    // We start not desiring any temp
    this->target_temperature = UNDEFINED;
    this->sensor_settings = false; // set to true if sensor settings have been overriden

    // General config
    this->set_m_code          = cr.get_int(m, set_m_code_key, tool_id < 100 ? 104 : 140); // hotend vs bed
    this->set_and_wait_m_code = cr.get_int(m, set_and_wait_m_code_key, tool_id < 100 ? 109 : 190); // hotend vs bed
    this->get_m_code          = cr.get_int(m, get_m_code_key, 105);
    this->readings_per_second = cr.get_int(m, readings_per_second_key, 20);

    // Runaway parameters
    uint32_t n = cr.get_int(m, runaway_range_key, 20);
    if (n > 63) n = 63;
    this->runaway_range = n;

    // TODO probably do not need to pack these anymore
    // these need to fit in 9 bits after dividing by 8 so max is 4088 secs or 68 minutes
    n = cr.get_int(m, runaway_heating_timeout_key, 900);
    if (n > 4088) n = 4088;
    this->runaway_heating_timeout = n / 8; // we have 8 second ticks
    n = cr.get_int(m, runaway_cooling_timeout_key, 0); // disable by default
    if (n > 4088) n = 4088;
    this->runaway_cooling_timeout = n / 8;

    this->runaway_error_range = cr.get_float(m, runaway_error_range_key, 1.0F);

    // Max and min temperatures we are not allowed to get over (Safety)
    this->max_temp = cr.get_float(m, max_temp_key, 300);
    this->min_temp = cr.get_float(m, min_temp_key, 0);

    // Heater pin
    this->heater_pin = new SigmaDeltaPwm();
    this->heater.from_string(cr.get_string(m, heater_pin_key, "nc"));
    if (this->heater.connected()) {
        this->heater.as_output();
        this->readonly = false;
    } else {
        this->readonly = true;
        delete heater_pin;
        heater_pin = nullptr;
    }

    // For backward compatibility, default to a thermistor sensor.
    std::string sensor_type = cr.get_string(m, sensor_key, "thermistor");

    // Instantiate correct sensor
    delete sensor;
    sensor = nullptr; // In case we fail to create a new sensor.
    if (sensor_type.compare("thermistor") == 0) {
        sensor = new Thermistor();
    // } else if (sensor_type.compare("max31855") == 0) { // needs porting
    //     sensor = new Max31855();
        // } else if (sensor_type.compare("ad8495") == 0) {
        //     sensor = new AD8495();
        // } else if (sensor_type.compare("pt100_e3d") == 0) {
        //     sensor = new PT100_E3D();
    } else {
        sensor = new TempSensor(); // A dummy implementation
    }

    // allow sensor to read the config
    if (!sensor->configure(cr, m)) {
        printf("configure-temperature: %s sensor %s failed to configure\n", get_instance_name(), sensor_type.c_str());
        return false;
    }

    // sigma-delta output modulation
    this->o = 0;

    if (!this->readonly) {
        // used to enable bang bang control of heater
        this->use_bangbang = cr.get_bool(m, bang_bang_key, false);
        this->hysteresis = cr.get_float(m, hysteresis_key, 2);
        this->windup = cr.get_bool(m, windup_key, false);
        this->heater.max_pwm( cr.get_float(m, max_pwm_key, 255) );
        this->heater.set(0);
        //set_low_on_debug(heater.port_number, heater.pin);
        // TODO use single fasttimer for all sigma delta
        float freq= cr.get_float(m, pwm_frequency_key, 2000);
        if (freq >= FastTicker::get_min_frequency()) { // if >= 1KHz use FastTicker
            if (FastTicker::getInstance()->attach((uint32_t)freq, std::bind(&SigmaDeltaPwm::on_tick, this->heater_pin)) < 0) {
                printf("configure-temperature: ERROR Fast Ticker was not set (Too slow?)\n");
                return false;
            }

        }else{
            if (SlowTicker::getInstance()->attach((uint32_t)freq, std::bind(&SigmaDeltaPwm::on_tick, this->heater_pin)) < 0) {
                printf("configure-temperature: ERROR Slow Ticker was not set (Too fast?)\n");
                return false;
            }
        }
    }

    // runaway timer
    SlowTicker::getInstance()->attach(1, std::bind(&TemperatureControl::check_runaway, this));

    // sensor reading tick
    SlowTicker::getInstance()->attach( this->readings_per_second, std::bind(&TemperatureControl::thermistor_read_tick, this));
    this->PIDdt = 1.0 / this->readings_per_second;

    // PID
    setPIDp( cr.get_float(m, p_factor_key, 10 ) );
    setPIDi( cr.get_float(m, i_factor_key, 0.3f) );
    setPIDd( cr.get_float(m, d_factor_key, 200) );

    if (!this->readonly) {
        // set to the same as max_pwm by default
        this->i_max = cr.get_float(m, i_max_key, this->heater.max_pwm());
    }

    this->iTerm = 0.0;
    this->lastInput = -1.0;
    this->last_reading = 0.0;

    // register gcodes and mcodes
    using std::placeholders::_1;
    using std::placeholders::_2;

    Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 6, std::bind(&TemperatureControl::handle_M6,    this, _1, _2));

    Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, this->get_m_code, std::bind(&TemperatureControl::handle_mcode, this, _1, _2));
    Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 305, std::bind(&TemperatureControl::handle_mcode, this, _1, _2));

    if (!this->readonly) {
        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 143, std::bind(&TemperatureControl::handle_mcode, this, _1, _2));
        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 301, std::bind(&TemperatureControl::handle_mcode, this, _1, _2));
        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 303, std::bind(&TemperatureControl::handle_autopid, this, _1, _2));
        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, 500, std::bind(&TemperatureControl::handle_mcode, this, _1, _2));

        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, set_m_code, std::bind(&TemperatureControl::handle_mcode, this, _1, _2));
        Dispatcher::getInstance()->add_handler(Dispatcher::MCODE_HANDLER, set_and_wait_m_code, std::bind(&TemperatureControl::handle_mcode, this, _1, _2));
    }

    return true;
}
*/

void TemperatureControl::on_halt(bool flg) {
    if (flg) {
        // turn off heater
        o = 0;
        heater.set(0);
        target_temperature = UNDEFINED;
    }
}

void TemperatureControl::handle_autopid(float target, int ncycles)
{
    //os.printf("Running autopid on toolid %d, control X to abort\n", tool_id);
    PID_Autotuner autopid(this);
    // will not return until complete
    autopid.start(target, ncycles);
}

void TemperatureControl::set_desired_temperature(float desired_temperature)
{
    // Never go over the configured max temperature
    if ( desired_temperature > this->max_temp ) {
        desired_temperature = this->max_temp;
    }

    float last_target_temperature = target_temperature;
    target_temperature = desired_temperature;
    if (desired_temperature <= 0.0F) {
        // turning it off
        heater.set((this->o = 0));

    } else if (last_target_temperature <= 0.0F) {
        // if it was off and we are now turning it on we need to initialize
        this->lastInput = last_reading;
        // set to whatever the output currently is See http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
        this->iTerm = this->o;
        if (this->iTerm > this->i_max) this->iTerm = this->i_max;
        else if (this->iTerm < 0.0) this->iTerm = 0.0;
    }

    // reset the runaway state, even if it was a temp change
    this->runaway_state = NOT_HEATING;
}

float TemperatureControl::get_temperature()
{
    return last_reading;
}

void TemperatureControl::thermistor_read_tick()
{
    float temperature = (float)sensor.getTemperature() / 100.0f;
    if (target_temperature > 2) {
        if (isinf(temperature) || temperature < min_temp || temperature > max_temp) {
            target_temperature = UNDEFINED;
            heater.set((this->o = 0));

            // we schedule a call back in command context to print the errors
            printf("ERROR: MINTEMP or MAXTEMP triggered. Check your temperature sensors!\nHALT asserted - reset or M999 required\n");

            // force into ALARM state
            broadcast_halt(true);
        } else {
            pid_process(temperature);
        }
    }

    last_reading = temperature;
    return;
}

/**
 * Based on https://github.com/br3ttb/Arduino-PID-Library
 */
void TemperatureControl::pid_process(float temperature)
{
    if (use_bangbang) {
        // bang bang is very simple, if temp is < target - hysteresis turn on full else if  temp is > target + hysteresis turn heater off
        // good for relays
        if (temperature > (target_temperature + hysteresis) && this->o > 0) {
            heater.set(false);
            this->o = 0; // for display purposes only

        } else if (temperature < (target_temperature - hysteresis) && this->o <= 0) {
            if (heater.max_pwm() >= 255) {
                // turn on full
                this->heater.set(true);
                this->o = 255; // for display purposes only
            } else {
                // only to whatever max pwm is configured
                this->heater.pwm(heater.max_pwm());
                this->o = heater.max_pwm(); // for display purposes only
            }
        }
        return;
    }

    // regular PID control
    float error = target_temperature - temperature;

    float new_I = this->iTerm + (error * this->i_factor);
    if (new_I > this->i_max) new_I = this->i_max;
    else if (new_I < 0.0) new_I = 0.0;
    if (!this->windup) this->iTerm = new_I;

    float d = (temperature - this->lastInput);

    // calculate the PID output
    // TODO does this need to be scaled by max_pwm/256? I think not as p_factor already does that
    this->o = (this->p_factor * error) + new_I - (this->d_factor * d);

    if (this->o >= heater.max_pwm())
        this->o = heater.max_pwm();
    else if (this->o < 0)
        this->o = 0;
    else if (this->windup)
        this->iTerm = new_I; // Only update I term when output is not saturated.

    this->heater.pwm(this->o);
    this->lastInput = temperature;
}

// called every second
void TemperatureControl::check_runaway()
{
    if (is_halted()) return;

    // see if runaway detection is enabled
    if (this->runaway_heating_timeout == 0 && this->runaway_range == 0) return;

    // check every 8 seconds, depends on tick being 3 bits
    if (++tick != 0) return;

    // Check whether or not there is a temperature runaway issue, if so stop everything and report it

    if (this->target_temperature <= 0) { // If we are not trying to heat, state is NOT_HEATING
        this->runaway_state = NOT_HEATING;

    } else {
        float current_temperature = this->get_temperature();
        // heater is active
        switch( this->runaway_state ) {
            case NOT_HEATING: // If we were previously not trying to heat, but we are now, change to state WAITING_FOR_TEMP_TO_BE_REACHED
                this->runaway_state = (this->target_temperature >= current_temperature || this->runaway_cooling_timeout == 0) ? HEATING_UP : COOLING_DOWN;
                this->runaway_timer = 0;
                tick = 0;
                break;

            case HEATING_UP:
            case COOLING_DOWN:
                // check temp has reached the target temperature within the given error range
                if ( (runaway_state == HEATING_UP && current_temperature >= (this->target_temperature - this->runaway_error_range)) ||
                    (runaway_state == COOLING_DOWN && current_temperature <= (this->target_temperature + this->runaway_error_range)) ) {
                    this->runaway_state = TARGET_TEMPERATURE_REACHED;
                    this->runaway_timer = 0;
                    tick = 0;

                } else {
                    uint16_t t = (runaway_state == HEATING_UP) ? this->runaway_heating_timeout : this->runaway_cooling_timeout;
                    // we are still heating up see if we have hit the max time allowed
                    if (t > 0 && ++this->runaway_timer > t) {
                        // this needs to go to any connected terminal, so do it in command thread context
                        printf("ERROR: Temperature took too long to be reached, HALT asserted, TURN POWER OFF IMMEDIATELY - reset or M999 required\n");

                        broadcast_halt(true);
                        this->runaway_state = NOT_HEATING;
                        this->runaway_timer = 0;
                    }
                }
                break;

            case TARGET_TEMPERATURE_REACHED:
                if (this->runaway_range != 0) {
                    // we are in state TARGET_TEMPERATURE_REACHED, check for thermal runaway
                    float delta = current_temperature - this->target_temperature;

                    // If the temperature is outside the acceptable range for 8 seconds, this allows for some noise spikes without halting
                    if (fabsf(delta) > this->runaway_range) {
                        if (this->runaway_timer++ >= 1) { // this being 8 seconds
                            printf("ERROR: Temperature runaway (delta temp %f), HALT asserted, TURN POWER OFF IMMEDIATELY - reset or M999 required\n", delta);

                            broadcast_halt(true);
                            this->runaway_state = NOT_HEATING;
                            this->runaway_timer = 0;
                        }

                    } else {
                        this->runaway_timer = 0;
                    }
                }

                break;
        }
    }
}
