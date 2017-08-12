#include <stm32f7xx.h>

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
	 */
	#define PLLM 8U
	#define PLLN 216U
	#define PLLP 2U
	#define PLLQ 9U
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; // HSI as source
	RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP | RCC_PLLCFGR_PLLQ);
	RCC->PLLCFGR |= PLLM << RCC_PLLCFGR_PLLM_Pos;
	RCC->PLLCFGR |= PLLN << RCC_PLLCFGR_PLLN_Pos;
	RCC->PLLCFGR |= ((PLLP >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos;
	RCC->PLLCFGR |= PLLQ << RCC_PLLCFGR_PLLQ_Pos;

	/* Start PLL and enable over drive mode for high frequency
	 * switching. Also ensure components have appropriate freqs.
	 * See User Manual Sections 3.3.2, 4.1.4 and Data Sheet 5.3.1
	 */
	RCC->CR |= RCC_CR_PLLON; // Enable PLL
	RCC->APB1ENR |= RCC_APB1ENR_PWREN; // Enable PWR
	PWR->CR1 |= PWR_CR1_ODEN; // Enable overdrive mode
	while(!(PWR->CSR1 & PWR_CSR1_ODRDY)); // Wait for overdrive mode
	PWR->CR1 |= PWR_CR1_ODSWEN; // Switch to overdrive
	while(!(PWR->CSR1 & PWR_CSR1_ODSWRDY));// Wait for overdrive
	while(!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to lock

	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |= 7 << FLASH_ACR_LATENCY_Pos;
	RCC->CFGR &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2); // Clear APB1/2 precaler
	RCC->CFGR |= 5 << RCC_CFGR_PPRE1_Pos; // Set APB1 prescaler to 4 (54 Mhz (Max))
	RCC->CFGR |= 4 << RCC_CFGR_PPRE2_Pos; // Set APB2 prescaler to 2 (108 Mhz (Max))

	RCC->CFGR |= 2 << RCC_CFGR_SW_Pos; // Switch to PLL
	while(!(RCC->CFGR & (0x2 << RCC_CFGR_SWS_Pos)));

	/* Configure SYSCLK out on MCO2 (PC9) and PLL out MCO1 (PA8)
	 * See User Manual Sections 5.2.10
	 */
	RCC->CFGR |= RCC_CFGR_MCO1; // Set MCO1 = 3 for PLL
	RCC->CFGR &= ~RCC_CFGR_MCO1PRE; // clear prescaler
	RCC->CFGR |= 6U << RCC_CFGR_MCO1PRE_Pos; // Prescale by 4

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable Port A
	GPIOA->OSPEEDR |= 3U << GPIO_OSPEEDER_OSPEEDR8_Pos; // Very High Speed
	GPIOA->MODER &= ~GPIO_MODER_MODER8;
	GPIOA->MODER |= GPIO_MODER_MODER8_1; // 0b10 - Alternate Mode
	GPIOA->AFR[1] &= ~GPIO_AFRH_AFRH0; // Alternate Mode 0 (8 + AFRH0 = AFRH8)

	RCC->CFGR &= ~RCC_CFGR_MCO2; // Set MCO2 = 0 for SYSCLK
	RCC->CFGR &= ~RCC_CFGR_MCO2PRE; // clear prescaler
	RCC->CFGR |= 6U << RCC_CFGR_MCO2PRE_Pos; // Prescale by 4

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // Enable Port C
	GPIOC->OSPEEDR |= 3U << GPIO_OSPEEDER_OSPEEDR9_Pos; // Very High Speed
	GPIOC->MODER &= ~GPIO_MODER_MODER9;
	GPIOC->MODER |= GPIO_MODER_MODER9_1; // 0b10 - Alternate Mode
	GPIOC->AFR[1] &= ~GPIO_AFRH_AFRH1; // Alternate Mode 0 (8 + AFRH1 = AFRH9)

	// Initialize SysTick
	ticks_ms = 0;
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/1000);

	// Blue LED PB7
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // Enable clock to GPIOB
	
	GPIOB->MODER &= ~GPIO_MODER_MODER7; //
	GPIOB->MODER |= GPIO_MODER_MODER7_0; // Set as output
	GPIOB->ODR &= ~GPIO_ODR_ODR_7;

	uint32_t last_ms = ticks_ms;
	while (1) {
		if (ticks_ms - last_ms > 500) {
			GPIOB->ODR ^= GPIO_ODR_ODR_7;
			last_ms = ticks_ms;
		}
	}


	return 0;
}
