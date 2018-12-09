#pragma once

#include <stdint.h>

class TemperatureSensor {
public:
    virtual ~TemperatureSensor() {}

    // Result is fixed point with two decimal positions.
    virtual int16_t getTemperature() const = 0;
};