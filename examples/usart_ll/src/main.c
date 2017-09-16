#include <stm32f7xx.h>

#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_rcc.h>
// #include <stm32f7xx_ll_utils.h>
#include <stm32f7xx_ll_bus.h>
#include <stm32f7xx_ll_usart.h>



volatile uint32_t ticks_ms;
void SysTick_Handler(void) {
	ticks_ms++;
}

int main(void) {
	
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOD);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

	// Initialize SysTick
	ticks_ms = 0;
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);

	// Blue LED PB7
	LL_GPIO_InitTypeDef PB7;
	LL_GPIO_StructInit(&PB7);
	PB7.Pin = LL_GPIO_PIN_7; PB7.Mode = LL_GPIO_MODE_OUTPUT;
	PB7.Speed = LL_GPIO_SPEED_FREQ_LOW; PB7.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	PB7.Pull = LL_GPIO_PULL_NO; PB7.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOB, &PB7);


	LL_USART_InitTypeDef USART3Init;
	USART3Init.BaudRate = 9600;
	USART3Init.DataWidth = LL_USART_DATAWIDTH_8B;
	USART3Init.StopBits = LL_USART_STOPBITS_1;
	USART3Init.Parity = LL_USART_PARITY_NONE;
	USART3Init.TransferDirection = LL_USART_DIRECTION_TX_RX;
	USART3Init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
	USART3Init.OverSampling = LL_USART_OVERSAMPLING_16;

	if (LL_USART_Init(USART3, &USART3Init) != SUCCESS) {
		while(1) {
			// Hang bc wtf
		}
	}

	LL_USART_Enable(USART3); // Enable USART3

	LL_GPIO_InitTypeDef PD8, PD9; // TX and RX for USART3
	LL_GPIO_StructInit(&PD8); LL_GPIO_StructInit(&PD9);
	PD8.Pin = LL_GPIO_PIN_8; PD8.Mode = LL_GPIO_MODE_ALTERNATE; PD8.Alternate = LL_GPIO_AF_7;
	PD9.Pin = LL_GPIO_PIN_9; PD9.Mode = LL_GPIO_MODE_ALTERNATE; PD9.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOD, &PD8); LL_GPIO_Init(GPIOD, &PD9);

	uint32_t last_ms = ticks_ms;
	while (1) {
		if (ticks_ms - last_ms > 500) {
			LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_7);
			if (LL_USART_IsActiveFlag_TXE(USART3)) {
				LL_USART_TransmitData8(USART3, 'e');
			}
			last_ms = ticks_ms;
		}

		if (LL_USART_IsActiveFlag_RXNE(USART3)) { // echo some data
			LL_USART_TransmitData8(USART3, LL_USART_ReceiveData8(USART3));
		}
	}


	return 0;
}