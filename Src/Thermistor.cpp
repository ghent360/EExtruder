#include "Thermistor.h"
#include "adc.h"
#include <math.h>

static float toFp256(uint32_t v) {
    typedef union {
        float fv;
        uint32_t iv;
    } float_as_int;
    float_as_int y;
    y.fv = v;
    uint32_t exponent = y.iv & 0x7f800000; // extract exponent bits 30..23
    exponent -= (8 << 23);                 // subtract 8 from exponent (aka divide by 256)
    y.iv = (y.iv & ~0x7f800000) | exponent;// insert modified exponent back into bits 30..23
    return y.fv;
}

float Thermistor::calcTemperature() const {
    //float r = 4700.0f / ((float)vref/ vin - 1.0f);
    if (vref < vTherm_ || (vref - vTherm_) < 50) {
        return -1;
    }
    uint32_t r_int = ((uint32_t)(4700 * 256) * vTherm_) / (vref - vTherm_);
    float r = toFp256(r_int);
    return (1.0F / (k_ + (j_ * logf(r / r0_)))) - 273.15f;
}

int16_t Thermistor::getTemperature() const {
    return (int16_t)ceilf(calcTemperature() * 100.0);
}