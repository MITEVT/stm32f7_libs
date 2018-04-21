#ifndef _BOARD_H_
#define _BOARD_H_


#define boardTOGGLE_LED1()	HAL_GPIO_TogglePin(GPIOB, LD1_PIN)
#define boardTOGGLE_LED2()	HAL_GPIO_TogglePin(GPIOB, LD2_PIN)
#define boardTOGGLE_LED3()	HAL_GPIO_TogglePin(GPIOB, LD3_PIN)

#endif