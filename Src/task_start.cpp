#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "task_start.h"
#include "SigmaDeltaPwm.h"
#include "Thermistor.h"
#include "TemperatureControl.h"
#include <string.h>
#include <math.h>

static osTimerId  adcTimerId;
osThreadId tempTaskHandle;
Thermistor heater1thm(4267.0f, 25.0f, 100000.0f, vTemp1);
Thermistor heater2thm(4267.0f, 25.0f, 100000.0f, vTemp2);
TemperatureControl tcHeater1(heater1, heater1thm);

extern "C" void adcConversionComplete() {
}

static void SlowTicker(void const * argument) {
    tcHeater1.thermistor_read_tick();
    startAdcRead();
}

static void TempReporter(void const * argument) {
    /* Infinite loop */
    for(;;)
    {
        printf("Vref    %d.%03d\n\r", vref / 1000, vref % 1000);
        printf("IntTemp %d.%02d\n\r", chipTemp / 100, chipTemp % 100);
        int16_t temp1 = heater1thm.getTemperature();
        int16_t temp2 = heater2thm.getTemperature();
        printf("vTemp1  %d.%03d", vTemp1 / 1000, vTemp1 % 1000);
        if (temp1 > 0) {
            printf(" %d.%02d", temp1 / 100, temp1 % 100);
        }
        printf("\n\rvTemp2  %d.%03d", vTemp2 / 1000, vTemp2 % 1000);
        if (temp2 > 0) {
            printf(" %d.%02d", temp2 / 100, temp2 % 100);
        }
        printf("\n\r");
        osDelay( 2000 );
    }
}

extern "C" void taskStart() {
    osTimerDef(adcTimer, SlowTicker);
    adcTimerId = osTimerCreate(osTimer(adcTimer), osTimerPeriodic, NULL);
    osTimerStart(adcTimerId, 200);
    osDelay( 250 );
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Base_Start_IT(&htim14);

    osThreadDef(tempTask, TempReporter, osPriorityNormal, 0, 256);
    tempTaskHandle = osThreadCreate(osThread(tempTask), NULL);

    //setPwmHeater0(140);
    tcHeater1.setPIDp(88.9);
    tcHeater1.setPIDi(2.223);
    tcHeater1.setPIDd(889);
    //tcHeater1.handle_autopid(55, 10);
    tcHeater1.set_desired_temperature(45);
    /* Infinite loop */
    for(;;)
    {
/*        
        printf("Vref    %d.%03d\n\r", vref / 1000, vref % 1000);
        printf("IntTemp %d.%02d\n\r", chipTemp / 100, chipTemp % 100);
        int16_t temp1 = heater1thm.getTemperature();
        int16_t temp2 = heater2thm.getTemperature();
        printf("vTemp1  %d.%03d", vTemp1 / 1000, vTemp1 % 1000);
        if (temp1 > 0) {
            printf(" %d.%02d", temp1 / 100, temp1 % 100);
        }
        printf("\n\rvTemp2  %d.%03d", vTemp2 / 1000, vTemp2 % 1000);
        if (temp2 > 0) {
            printf(" %d.%02d", temp2 / 100, temp2 % 100);
        }
        printf("\n\r");
*/
        osDelay( 2000 );
    }
}