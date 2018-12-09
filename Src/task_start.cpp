#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "task_start.h"
#include "pwm_ticker.h"
#include "Thermistor.h"
#include <string.h>
#include <math.h>

osTimerId  adcTimerId;

extern "C" void adcConversionComplete() {
}

void StartAdcMeasureTick(void const * argument) {
    startAdcRead();
}

Thermistor heater0thm(4267.0f, 25.0f, 100000.0f);

extern "C" void taskStart() {
    osTimerDef(adcTimer, StartAdcMeasureTick);
    adcTimerId = osTimerCreate(osTimer(adcTimer), osTimerPeriodic, NULL);
    osTimerStart(adcTimerId, 50);
    osDelay( 250 );
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim14);
    //setPwmHeater0(140);
    /* Infinite loop */
    for(;;)
    {
        printf("Vref    %d.%03d\n\r", vref / 1000, vref % 1000);
        printf("IntTemp %d.%02d\n\r", chipTemp / 100, chipTemp % 100);
        int32_t temp1 = heater0thm.getTemperature(vTemp1);
        int32_t temp2 = heater0thm.getTemperature(vTemp2);
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