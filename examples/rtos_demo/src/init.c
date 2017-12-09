#include "init.h"

#include <string.h>

#include "board.h"

#include "stm32f7xx.h"

/* Configure system clock for 216Mhz */
static void prvSystemClockConfig(void);

void vSetupHardware(UART_HandleTypeDef *xConsole, RNG_HandleTypeDef *xRNG) {

	/* Set Interrupt Group Priority */
	NVIC_SetPriorityGrouping((uint32_t)0x00000003U);

	/* Configure the System clock to have a frequency of 216 MHz */
	prvSystemClockConfig();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef LED;
	LED.Mode = GPIO_MODE_OUTPUT_PP;
	LED.Pull = GPIO_NOPULL;
	LED.Speed = GPIO_SPEED_FREQ_LOW;
	LED.Pin = LD1_PIN; HAL_GPIO_Init(GPIOB, &LED);
	LED.Pin = LD2_PIN; HAL_GPIO_Init(GPIOB, &LED);
	LED.Pin = LD3_PIN; HAL_GPIO_Init(GPIOB, &LED);

	memset(xConsole, 0, sizeof(UART_HandleTypeDef)); // Prevents need to initialize unused things
	xConsole->Instance = USART3;
	xConsole->Init.BaudRate = UART_BAUD;
	xConsole->Init.WordLength = UART_WORDLENGTH_8B;
	xConsole->Init.StopBits = UART_STOPBITS_1;
	xConsole->Init.Parity = UART_PARITY_NONE;
	xConsole->Init.Mode = UART_MODE_TX_RX;
	xConsole->Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(xConsole);

	xRNG->Instance = RNG;
	HAL_RNG_Init(xRNG);
}

/*-----------------------------------------------------------*/

static void prvSystemClockConfig(void) {
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

	SystemCoreClockUpdate();
}

/*-----------------------------------------------------------*/
/* HAL Init Functions*/

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

void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng) {
	__HAL_RCC_RNG_CLK_ENABLE();
}