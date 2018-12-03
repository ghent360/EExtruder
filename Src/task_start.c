#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "adc.h"
#include "tim.h"
#include "task_start.h"
#include "pwm_ticker.h"
#include <string.h>

static char buffer[30];
osTimerId  adcTimerId;

void adcConversionComplete() {
}

void StartAdcMeasureTick(void const * argument) {
    startAdcRead();
}

void serialSendBuffer() {
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 0xffff);
}

void taskStart() {
    /* Infinite loop */
    osTimerDef(adcTimer, StartAdcMeasureTick);
    adcTimerId = osTimerCreate(osTimer(adcTimer), osTimerPeriodic, NULL);
    osTimerStart(adcTimerId, 50);
    osDelay( 250 );
    HAL_TIM_Base_Start_IT(&htim3);
    setPwmHeater0(10);
    for(;;)
    {
        snprintf(buffer, sizeof(buffer), "Vref    %d.%03d\n\r", vref / 1000, vref % 1000);
        serialSendBuffer();
        snprintf(buffer, sizeof(buffer), "IntTemp %d.%02d\n\r", chipTemp / 100, chipTemp % 100);
        serialSendBuffer();
        snprintf(buffer, sizeof(buffer), "vTemp1  %d.%03d\n\r", vTemp1 / 1000, vTemp1 % 1000);
        serialSendBuffer();
        snprintf(buffer, sizeof(buffer), "vTemp2  %d.%03d\n\r", vTemp2 / 1000, vTemp2 % 1000);
        serialSendBuffer();
        HAL_UART_Transmit(&huart1, (uint8_t*)"Alive...\n\r", 10, 0xffff);
        osDelay( 2000 );
    }
}