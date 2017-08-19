#include <stm32f7xx.h>

#include <stm32f7xx_ll_gpio.h>
#include <stm32f7xx_ll_rcc.h>
#include <stm32f7xx_ll_utils.h>
#include <stm32f7xx_ll_bus.h>



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
	LL_UTILS_PLLInitTypeDef pll_init;
	pll_init.PLLM = LL_RCC_PLLM_DIV_8;
	pll_init.PLLN = 216;
	pll_init.PLLP = LL_RCC_PLLP_DIV_2;
	LL_UTILS_ClkInitTypeDef clk_init;
	clk_init.AHBCLKDivider = LL_RCC_SYSCLK_DIV_1;
	clk_init.APB1CLKDivider = LL_RCC_APB1_DIV_4;
	clk_init.APB2CLKDivider = LL_RCC_APB2_DIV_2;

	LL_PLL_ConfigSystemClock_HSI(&pll_init, &clk_init);


	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA | LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOC);

	/* Configure SYSCLK out on MCO1 (PA8) and PLL out MCO2 (PC9)
	 * See User Manual Sections 5.2.10
	 */
	RCC->CFGR |= RCC_CFGR_MCO1; // Set MCO1 = 3 for PLL
	RCC->CFGR &= ~RCC_CFGR_MCO1PRE; // clear prescaler
	RCC->CFGR |= 6U << RCC_CFGR_MCO1PRE_Pos; // Prescale by 4

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable Port A
	LL_GPIO_InitTypeDef PA8;
	PA8.Pin = LL_GPIO_PIN_8;
	PA8.Mode = LL_GPIO_MODE_ALTERNATE;
	PA8.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	PA8.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	PA8.Pull = LL_GPIO_PULL_NO; 
	PA8.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOA, &PA8);

	RCC->CFGR &= ~RCC_CFGR_MCO2; // Set MCO2 = 0 for SYSCLK
	RCC->CFGR &= ~RCC_CFGR_MCO2PRE; // clear prescaler
	RCC->CFGR |= 6U << RCC_CFGR_MCO2PRE_Pos; // Prescale by 4

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable Port C
	LL_GPIO_InitTypeDef PC9;
	PC9.Pin = LL_GPIO_PIN_9;
	PC9.Mode = LL_GPIO_MODE_ALTERNATE;
	PC9.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
	PC9.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	PC9.Pull = LL_GPIO_PULL_NO; 
	PC9.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOC, &PC9);

	// Initialize SysTick
	ticks_ms = 0;
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);

	// Blue LED PB7
	LL_GPIO_InitTypeDef PB7;
	PB7.Pin = LL_GPIO_PIN_7;
	PB7.Mode = LL_GPIO_MODE_OUTPUT;
	PB7.Speed = LL_GPIO_SPEED_FREQ_LOW;
	PB7.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	PB7.Pull = LL_GPIO_PULL_NO;
	PB7.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOB, &PB7);

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock to GPIOB

	uint32_t last_ms = ticks_ms;
	while (1) {
		if (ticks_ms - last_ms > 500) {
			LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_7);
			last_ms = ticks_ms;
		}
	}


	return 0;
}
