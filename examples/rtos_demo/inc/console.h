#ifndef _CONSOLE_H_
#define _CONSOLE_H_

#include "stm32f7xx_hal.h"

/*-----------------------------------------------------------*/
/* Console Global Definitions */

void vConsoleInit(RNG_HandleTypeDef *xRNG);
void vCommandConsoleTask(void *pvParameters);


#endif