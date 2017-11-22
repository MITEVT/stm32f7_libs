#include <string.h>

#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"

#define LD1_PIN GPIO_PIN_0
#define LD2_PIN GPIO_PIN_7
#define LD3_PIN GPIO_PIN_14

void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void HAL_UART_MspInit(UART_HandleTypeDef *husart) {
	if (husart->Instance == USART3) {
		__HAL_RCC_USART3_CLK_ENABLE();

		__HAL_RCC_GPIOD_CLK_ENABLE();
		GPIO_InitTypeDef UG;
		UG.Mode = GPIO_MODE_AF_PP;
		UG.Pull = GPIO_PULLUP;
		UG.Speed = GPIO_SPEED_FREQ_HIGH;
		UG.Alternate = GPIO_AF7_USART3;
		UG.Pin = GPIO_PIN_8; HAL_GPIO_Init(GPIOD, &UG);
		UG.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOD, &UG);
	}
}


int main(void) {
	
	HAL_Init();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef LED;
	LED.Mode = GPIO_MODE_OUTPUT_PP;
	LED.Pull = GPIO_NOPULL;
	LED.Speed = GPIO_SPEED_FREQ_LOW;
	LED.Pin = LD1_PIN; HAL_GPIO_Init(GPIOB, &LED);
	LED.Pin = LD2_PIN; HAL_GPIO_Init(GPIOB, &LED);
	LED.Pin = LD3_PIN; HAL_GPIO_Init(GPIOB, &LED);

	UART_HandleTypeDef usart3;
	memset(&usart3, 0, sizeof(UART_HandleTypeDef)); // Prevents need to initialize unused things
	usart3.Instance = USART3;
	usart3.Init.BaudRate = UART_BAUD;
	usart3.Init.WordLength = UART_WORDLENGTH_8B;
	usart3.Init.StopBits = UART_STOPBITS_1;
	usart3.Init.Parity = UART_PARITY_NONE;
	usart3.Init.Mode = UART_MODE_TX_RX;
	usart3.Init.OverSampling = USART_OVERSAMPLING_16;
	if (HAL_UART_Init(&usart3) == HAL_ERROR) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 0);
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, 1);
	}

	uint32_t last_ms = HAL_GetTick();
	while (1) {
		uint8_t c;
		if (HAL_UART_Receive(&usart3, &c, 1, 1) == HAL_OK) {
			HAL_GPIO_TogglePin(GPIOB, LD2_PIN);
			HAL_UART_Transmit(&usart3, &c, 1, 10);
		}

		if (HAL_GetTick() - last_ms > 1000) {
			HAL_GPIO_TogglePin(GPIOB, LD1_PIN);
			last_ms = HAL_GetTick();
		}

	}


	return 0;
}
