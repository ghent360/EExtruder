#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "task_start.h"
#include "pwm_ticker.h"
#include <string.h>
#include <math.h>

osTimerId  adcTimerId;

void adcConversionComplete() {
}

void StartAdcMeasureTick(void const * argument) {
    startAdcRead();
}

#define t0   25.0f
#define beta 4267.0f
#define r0   100000.0f

#define j (1.0f / beta)
#define k (1.0f / (t0 + 273.15f))

static int32_t solveThermistor(uint16_t vin) {
    float r = 4700.0f / ((float)vref/ vin - 1.0f);
    if (r > 3 * r0) {
        return -1;
    }
    float t = (1.0F / (k + (j * logf(r / r0)))) - 273.15f;
    return (int32_t)(t * 100 + 50);
}

void taskStart() {
    /* Infinite loop */
    osTimerDef(adcTimer, StartAdcMeasureTick);
    adcTimerId = osTimerCreate(osTimer(adcTimer), osTimerPeriodic, NULL);
    osTimerStart(adcTimerId, 50);
    osDelay( 250 );
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim14);
    setPwmHeater0(80);
    for(;;)
    {
        printf("Vref    %d.%03d\n\r", vref / 1000, vref % 1000);
        printf("IntTemp %d.%02d\n\r", chipTemp / 100, chipTemp % 100);
        int32_t temp1 = solveThermistor(vTemp1);
        int32_t temp2 = solveThermistor(vTemp2);
        printf("vTemp1  %d.%03d", vTemp1 / 1000, vTemp1 % 1000);
        if (temp1 > 0) {
            printf(" %ld.%02ld", temp1 / 100, temp1 % 100);
        }
        printf("\n\rvTemp2  %d.%03d", vTemp2 / 1000, vTemp2 % 1000);
        if (temp2 > 0) {
            printf(" %ld.%02ld", temp2 / 100, temp2 % 100);
        }
        printf("\n\r");
        osDelay( 2000 );
    }
}