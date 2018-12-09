#pragma once

#include "TemperatureSensor.h"
#include <stdint.h>

class Thermistor : public TemperatureSensor {
public:
  constexpr Thermistor(float beta, float t0, float r0, uint16_t& vTherm)
    : beta_(beta),
      t0_(t0),
      r0_(r0),
      j_(1.0f / beta),
      k_(1.0f / (t0 + 273.15f)),
      vTherm_(vTherm) {}

  const float beta_;
  const float t0_;
  const float r0_;
  const float j_;
  const float k_;

  virtual int16_t getTemperature() const;
private:
  uint16_t& vTherm_;

  float calcTemperature() const;
};

extern Thermistor heater1thm;
extern Thermistor heater2thm;