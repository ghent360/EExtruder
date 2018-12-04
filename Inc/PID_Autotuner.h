/**
 * Based on https://github.com/br3ttb/Arduino-PID-AutoTune-Library
 */

#pragma once

#include <stdint.h>

class TemperatureControl;

class PID_Autotuner
{
public:
    PID_Autotuner(TemperatureControl *);
    void start(
        float target = 150.0,
        int ncycles = 8,
        float noiseBand = 0.5,
        int nLookBack = 10);

private:
    void run_auto_pid(float target, int ncycles);
    void finishUp();
    void abort();

    TemperatureControl *temp_control;
    float target_temperature;

    float *peaks;
    int requested_cycles;
    float noiseBand;
    unsigned long peak1, peak2;
    int sampleTime;
    int nLookBack;
    int lookBackCnt;
    int peakType;
    float *lastInputs;
    int peakCount;
    float absMax, absMin;
    float oStep;
    int output;
    unsigned long tickCnt;
    bool justchanged;
    bool firstPeak;
};
