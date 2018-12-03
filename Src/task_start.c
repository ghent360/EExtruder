#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include "adc.h"
#include <string.h>

static char buffer[30];
void serialSendBuffer() {
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 0xffff);
}

void taskStart() {
    uint16_t cnt = 0;

    /* Infinite loop */
    startAdcRead();
    for(;;)
    {
        if (adcComplete) {
            snprintf(buffer, sizeof(buffer), "Vref    %d.%03d\n\r", vref / 1000, vref % 1000);
            serialSendBuffer();
            snprintf(buffer, sizeof(buffer), "IntTemp %d.%02d\n\r", chipTemp / 100, chipTemp % 100);
            serialSendBuffer();
            snprintf(buffer, sizeof(buffer), "vTemp1  %d.%03d\n\r", vTemp1 / 1000, vTemp1 % 1000);
            serialSendBuffer();
            snprintf(buffer, sizeof(buffer), "vTemp2  %d.%03d\n\r", vTemp2 / 1000, vTemp2 % 1000);
            serialSendBuffer();
            startAdcRead();
        } else {
            snprintf(buffer, sizeof(buffer), "Alive %d\r", cnt);
            serialSendBuffer();
        }
        cnt += 1;
        osDelay( 1000 );
    }
}