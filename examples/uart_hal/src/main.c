#include <stm32f7xx.h>

#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_rcc.h>
#include <stm32f7xx_ll_utils.h>
#include <stm32f7xx_ll_bus.h>
#include <stm32f7xx_hal_conf.h>



volatile uint32_t ticks_ms;
void SysTick_Handler(void) {
	ticks_ms++;
}


int main(void) {

	/* Enable PLL and set as SYSCLK (216MHz)
	 * See User Manual Section 5.3.2
	 *
	 * f_in = 16Mhz
	 * f_in / PLLM = 2Mhz -> PLLM = 8
	 * f_vco = f_in * PLLN / PLLM = 432 Mhz -> PLLN = 216
	 * f_out = f_vco / PLLP = 216 Mhz -> PLLP = 2
	 * f_out2 = f_vco / PLLQ = 48 Mhz -> PLLQ = 9
	 *
	 * APB1 Prescaler = f_out / 54 Mhz = 4
	 * APB2 Prescaler = f_out / 108 Mhz = 2
	 */


	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA 
								| LL_AHB1_GRP1_PERIPH_GPIOB 
								| LL_AHB1_GRP1_PERIPH_GPIOC
								| LL_AHB1_GRP1_PERIPH_GPIOD);

	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);


	// Initialize SysTick
	ticks_ms = 0;
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);
	

	GPIO_InitTypeDef DP8;
	DP8.Pin = GPIO_PIN_8;
	DP8.Mode = GPIO_MODE_AF_PP;
	DP8.Pull = GPIO_NOPULL;
	DP8.Speed = GPIO_SPEED_FREQ_HIGH;
	DP8.Alternate = GPIO_AF7_USART3;


	GPIO_InitTypeDef DP9;
	DP9.Pin = GPIO_PIN_9;
	DP9.Mode = GPIO_MODE_AF_PP;
	DP9.Pull = GPIO_NOPULL;
	DP9.Speed = GPIO_SPEED_FREQ_HIGH;
	DP9.Alternate = GPIO_AF7_USART3;

	HAL_GPIO_Init(GPIOD, &DP9);
	HAL_GPIO_Init(GPIOD, &DP8);


	UART_HandleTypeDef uart3;
	uart3.Instance = USART3;
	

	uart3.Init.BaudRate = 9600;
	uart3.Init.WordLength = UART_WORDLENGTH_8B;
	uart3.Init.StopBits = UART_STOPBITS_1;
	uart3.Init.Parity = UART_PARITY_NONE;
	uart3.Init.Mode = UART_MODE_TX_RX;
	uart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart3.Init.OverSampling = UART_OVERSAMPLING_16;
	uart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;


	HAL_UART_Init(&uart3);

	uint32_t last_ms = ticks_ms;
	while (1) {
		if (ticks_ms - last_ms > 500) {
			uint8_t data[4] = {0xAA, 0xAA, 0xAA, 0xAA};
			HAL_UART_Transmit(&uart3, data, 4,100);
		}
	}


	return 0;
}
