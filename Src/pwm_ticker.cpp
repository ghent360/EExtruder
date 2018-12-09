#include "pwm_ticker.h"
#include "SigmaDeltaPwm.h"

SigmaDeltaPwm heater1(FAST_IO_PIN(PORTB, 11));
SigmaDeltaPwm heater2(FAST_IO_PIN(PORTB, 12));

void pwmTicker() {
    heater1.on_tick();
    heater2.on_tick();
}