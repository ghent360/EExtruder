#include "pwm_ticker.h"
#include "SigmaDeltaPwm.h"

SigmaDeltaPwm heater0(FAST_IO_PIN(PORTB, 11));
SigmaDeltaPwm heater1(FAST_IO_PIN(PORTB, 12));

void setPwmHeater0(int16_t pwm) {
    heater0.pwm(pwm);
}

void setPwmHeater1(int16_t pwm) {
    heater1.pwm(pwm);
}

void pwmTicker() {
    heater0.on_tick();
    heater1.on_tick();
}