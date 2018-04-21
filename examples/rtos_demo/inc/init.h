#ifndef _INIT_H_
#define _INIT_H_

#include "stm32f7xx_hal.h"

/*-----------------------------------------------------------*/
/* Enable Clock, LED, USART3 hardware, RNG */
void vSetupHardware(UART_HandleTypeDef *xConsole, RNG_HandleTypeDef *xRNG, ADC_HandleTypeDef *xADC, DMA_HandleTypeDef *xDMA);

/*-----------------------------------------------------------*/
/* HAL Init Functions */
void HAL_UART_MspInit(UART_HandleTypeDef *husart);

#endif