#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void pwmTicker();
void setPwmHeater0(int16_t pwm);
void setPwmHeater1(int16_t pwm);

#ifdef __cplusplus
}
#endif
