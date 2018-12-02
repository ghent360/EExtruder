#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "usart.h"
#include <string.h>

void taskStart() {
  uint16_t cnt = 0;

  /* Infinite loop */
  for(;;)
  {
    static char buffer[30];
    snprintf(buffer, sizeof(buffer), "Alive %d\r", cnt);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), 0xffff);
    if ((cnt % 20) == 0)
    {
      HAL_UART_Transmit(&huart1, (uint8_t*)"\n\r", 2, 0xffff);
    }
    cnt += 1;
    osDelay( 1000 );
  }
}