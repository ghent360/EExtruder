#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "adc.h"
#include "task_start.h"
#include <string.h>

static char buffer[30];
osThreadId adcTaskHandle;

void serialSendBuffer() {
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 0xffff);
}

void adcConversionComplete() {
    xTaskResumeFromISR(adcTaskHandle);
}

void StartAdcTask(void const * argument) {
    startAdcRead();
    vTaskSuspend(adcTaskHandle);
    for(;;)
    {
        startAdcRead();
        vTaskSuspend(adcTaskHandle);
        osDelay( 100 );
    }
}

void taskStart() {
    /* Infinite loop */
    osThreadDef(adcTask, StartAdcTask, osPriorityNormal, 0, 128);
    adcTaskHandle = osThreadCreate(osThread(adcTask), NULL);
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
        osDelay( 500 );
    }
}