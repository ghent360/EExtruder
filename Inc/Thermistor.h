#pragma once

#include <stdint.h>

class Thermistor {
public:
  constexpr Thermistor(float beta, float t0, float r0)
    : beta_(beta),
      t0_(t0),
      r0_(r0),
      j_(1.0f / beta),
      k_(1.0f / (t0 + 273.15f)) {}

  const float beta_;
  const float t0_;
  const float r0_;
  const float j_;
  const float k_;

  float getTemperature(uint16_t vin) const;
};