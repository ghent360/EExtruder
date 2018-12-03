#pragma once

#include "FastIOPin.h"

class SigmaDeltaPwm {
public:
    SigmaDeltaPwm(uint16_t pin_no);

    void     on_tick(void);

    void     max_pwm(int);
    int      max_pwm(void);

    void     pwm(int);
    int      get_pwm() const { return _pwm; }
    void     set(bool);

private:
    FastIOPin _pin;
    int  _max;
    int  _pwm;
    bool _sd_direction;
    int  _sd_accumulator;
};
