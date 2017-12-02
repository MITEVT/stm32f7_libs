#ifndef _BOARD_H_
#define _BOARD_H_

#define LD1_PIN GPIO_PIN_0
#define LD2_PIN GPIO_PIN_7
#define LD3_PIN GPIO_PIN_14

/* Toggle LED functions (using HAL) */
#define mainTOGGLE_LED1()	HAL_GPIO_TogglePin(GPIOB, LD1_PIN)
#define mainTOGGLE_LED2()	HAL_GPIO_TogglePin(GPIOB, LD2_PIN)
#define mainTOGGLE_LED3()	HAL_GPIO_TogglePin(GPIOB, LD3_PIN)

#endif