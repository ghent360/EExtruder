#pragma once

#include "FastIOPin.h"

class SigmaDeltaPwm {
    static constexpr int16_t PID_PWM_MAX = 256;
public:
    constexpr SigmaDeltaPwm(uint16_t pin_no)
        : _pin(pin_no),
          _max(PID_PWM_MAX - 1),
          _pwm(-1),
          _sd_direction(false),
          _sd_accumulator(0) {}

    void     on_tick(void);

    void     max_pwm(int16_t);
    int      max_pwm(void) const { return _max; }

    void     pwm(int16_t);
    int      get_pwm() const { return _pwm; }

    void     set(bool);

private:
    FastIOPin _pin;
    int16_t  _max;
    int16_t  _pwm;
    bool     _sd_direction;
    int32_t  _sd_accumulator;
};
