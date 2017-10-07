#include <stdio.h>
#include <string.h>

#include "stm32f7xx.h"

#include "stm32f7xx_ll_gpio.h"
#include "stm32f7xx_ll_rcc.h"
#include "stm32f7xx_ll_bus.h"
#include "stm32f7xx_ll_usart.h"

/* FreeRTOS include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"



// volatile uint32_t ticks_ms;
// void SysTick_Handler(void) {
// 	ticks_ms++;
// }

/* Priorities at which the tasks are created. */
#define mainCOMMAND_CONSOLE_TASK_PRIORITY	(tskIDLE_PRIORITY + 1)
#define mainQUEUE_RECEIVE_TASK_PRIORITY		(tskIDLE_PRIORITY + 2)
#define	mainQUEUE_SEND_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)

/* The rate at which data is sent to the queue.  The 200ms value is converted
to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			(500 / portTICK_PERIOD_MS)

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					(1)

/* The LED is used to show the demo status. (not connected on Rev A hardware) */
#define mainTOGGLE_LED1()	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_0)
#define mainTOGGLE_LED2()	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_7)
#define mainTOGGLE_LED3()	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_14)

/*-----------------------------------------------------------*/

/*
 * Configure the hardware as necessary to run this demo.
 */
static void prvSetupHardware(void);

/*
 * Configure the system clock for maximum speed.
 */
static void prvSystemClockConfig(void);

static void prvQueueSendTask(void *pvParameters);
static void prvQueueReceiveTask(void *pvParameters);

/* Prototypes for the standard FreeRTOS callback/hook functions implemented
within this file. */
void vApplicationMallocFailedHook(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationTickHook(void);

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/*-----------------------------------------------------------*/
/* Command Line Definitions */

void vCommandConsoleTask(void *pvParameters);
static BaseType_t prvToggleLEDCommand(char*, size_t, const char*);
static BaseType_t prvFreqCommand(char*, size_t, const char*);

static const CLI_Command_Definition_t xToggleLEDCommand = {
    "toggle-led",
    "toggle-led <num>: Toggle LED<num>\r\n",
    prvToggleLEDCommand,
    1
};

static const CLI_Command_Definition_t xFreqCommand = {
    "freq",
    "freq: Get CPU Frequency\r\n",
    prvFreqCommand,
    0
};

/*-----------------------------------------------------------*/

int main(void) {
	/* Configure the hardware ready to run the demo. */
	prvSetupHardware();

	FreeRTOS_CLIRegisterCommand(&xToggleLEDCommand);
	FreeRTOS_CLIRegisterCommand(&xFreqCommand);
	xTaskCreate(vCommandConsoleTask, "console", configMINIMAL_STACK_SIZE*4, USART3, mainCOMMAND_CONSOLE_TASK_PRIORITY, NULL);

	/* Create the queue. */
	xQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof( uint32_t ));

	if(xQueue != NULL) {
		/* Start the two tasks as described in the comments at the top of this
		file. */
		xTaskCreate(prvQueueReceiveTask,				/* The function that implements the task. */
					"Rx", 								/* The text name assigned to the task - for debug only as it is not used by the kernel. */
					configMINIMAL_STACK_SIZE, 			/* The size of the stack to allocate to the task. */
					NULL, 								/* The parameter passed to the task - not used in this case. */
					mainQUEUE_RECEIVE_TASK_PRIORITY, 	/* The priority assigned to the task. */
					NULL);								/* The task handle is not required, so NULL is passed. */

		xTaskCreate(prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL);

		/* Start the tasks and timer running. */
		vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following
	line will never be reached.  If the following line does execute, then
	there was insufficient FreeRTOS heap memory available for the Idle and/or
	timer tasks to be created.  See the memory management section on the
	FreeRTOS web site for more details on the FreeRTOS heap
	http://www.freertos.org/a00111.html. */
	for( ;; );

	return 0;
}

/*-----------------------------------------------------------*/

static void prvQueueSendTask(void *pvParameters) {
	TickType_t xNextWakeTime;
	const unsigned long ulValueToSend = 100UL;

	/* Remove compiler warning about unused parameter. */
	(void)pvParameters;

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Place this task in the blocked state until it is time to run again. */
		vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

		/* Send to the queue - causing the queue receive task to unblock and
		toggle the LED.  0 is used as the block time so the sending operation
		will not block - it shouldn't need to block as the queue should always
		be empty at this point in the code. */
		xQueueSend( xQueue, &ulValueToSend, 0U );
	}
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask(void *pvParameters) {
	unsigned long ulReceivedValue;
	const unsigned long ulExpectedValue = 100UL;

	/* Remove compiler warning about unused parameter. */
	( void ) pvParameters;

	for( ;; )
	{
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h. */
		xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, toggle the LED. */
		if( ulReceivedValue == ulExpectedValue )
		{
			mainTOGGLE_LED1();
			ulReceivedValue = 0U;
		}
	}
}

/*-----------------------------------------------------------*/

static void prvSetupHardware(void) {

	/* Set Interrupt Group Priority */
	NVIC_SetPriorityGrouping((uint32_t)0x00000003U);

	/* Configure the System clock to have a frequency of 200 MHz */
	prvSystemClockConfig();

	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB | LL_AHB1_GRP1_PERIPH_GPIOD);
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART3);

	// Green LED PB0 (LD1)
	LL_GPIO_InitTypeDef PB0;
	LL_GPIO_StructInit(&PB0);
	PB0.Pin = LL_GPIO_PIN_0; PB0.Mode = LL_GPIO_MODE_OUTPUT;
	PB0.Speed = LL_GPIO_SPEED_FREQ_LOW; PB0.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	PB0.Pull = LL_GPIO_PULL_NO; PB0.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOB, &PB0);

	// Blue LED PB7 (LD2)
	LL_GPIO_InitTypeDef PB7;
	LL_GPIO_StructInit(&PB7);
	PB7.Pin = LL_GPIO_PIN_7; PB7.Mode = LL_GPIO_MODE_OUTPUT;
	PB7.Speed = LL_GPIO_SPEED_FREQ_LOW; PB7.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	PB7.Pull = LL_GPIO_PULL_NO; PB7.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOB, &PB7);

	// Red LED PB14 (LD3)
	LL_GPIO_InitTypeDef PB14;
	LL_GPIO_StructInit(&PB14);
	PB14.Pin = LL_GPIO_PIN_14; PB14.Mode = LL_GPIO_MODE_OUTPUT;
	PB14.Speed = LL_GPIO_SPEED_FREQ_LOW; PB14.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	PB14.Pull = LL_GPIO_PULL_NO; PB14.Alternate = LL_GPIO_AF_0;
	LL_GPIO_Init(GPIOB, &PB14);


	LL_GPIO_InitTypeDef PD8, PD9; // TX and RX for USART3
	LL_GPIO_StructInit(&PD8); LL_GPIO_StructInit(&PD9);
	PD8.Pin = LL_GPIO_PIN_8; PD8.Mode = LL_GPIO_MODE_ALTERNATE; PD8.Alternate = LL_GPIO_AF_7;
	PD9.Pin = LL_GPIO_PIN_9; PD9.Mode = LL_GPIO_MODE_ALTERNATE; PD9.Alternate = LL_GPIO_AF_7;
	LL_GPIO_Init(GPIOD, &PD8); LL_GPIO_Init(GPIOD, &PD9);

	LL_USART_InitTypeDef USART3Init;
	USART3Init.BaudRate = UART_BAUD;
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

void vApplicationMallocFailedHook( void ) {
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName ) {
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void ) {
	volatile size_t xFreeHeapSpace;

	/* This is just a trivial example of an idle hook.  It is called on each
	cycle of the idle task.  It must *NOT* attempt to block.  In this case the
	idle task just queries the amount of FreeRTOS heap that remains.  See the
	memory management section on the http://www.FreeRTOS.org web site for memory
	management options.  If there is a lot of heap memory free then the
	configTOTAL_HEAP_SIZE value in FreeRTOSConfig.h can be reduced to free up
	RAM. */
	xFreeHeapSpace = xPortGetFreeHeapSize();

	/* Remove compiler warning about xFreeHeapSpace being set but never used. */
	( void ) xFreeHeapSpace;
}
/*-----------------------------------------------------------*/

void vAssertCalled( uint32_t ulLine, const char *pcFile ) {
	volatile unsigned long ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	taskENTER_CRITICAL();
	{
		/* Set ul to a non-zero value using the debugger to step out of this
		function. */
		while( ul == 0 )
		{
			__NOP();
		}
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void ) {
	// #if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 0 )
	// {
	// 	/* The full demo includes a software timer demo/test that requires
	// 	prodding periodically from the tick interrupt. */
	// 	vTimerPeriodicISRTests();

	// 	/* Call the periodic queue overwrite from ISR demo. */
	// 	vQueueOverwritePeriodicISRDemo();

	// 	 Call the periodic event group from ISR demo. 
	// 	vPeriodicEventGroupsProcessing();

	// 	/* Call the code that uses a mutex from an ISR. */
	// 	vInterruptSemaphorePeriodicTest();

	// 	/* Use a queue set from an ISR. */
	// 	vQueueSetAccessQueueSetFromISR();

	// 	/* Use task notifications from an ISR. */
	// 	xNotifyTaskFromISR();
	// }
	// #endif
}

/*-----------------------------------------------------------*/

#define MAX_INPUT_LENGTH    50
#define MAX_OUTPUT_LENGTH   100

static const char* const pcWelcomeMessage =
  "\r\n***************************************************\r\n \
   \t\tFreeRTOS command server.\r\nType 'help' to view a list of registered commands.\r\n>";

static const char* const pcBackSpace = "\033[1D \033[1D";
void vCommandConsoleTask( void *pvParameters )
{
	USART_TypeDef *xConsole;
	char cRxedChar = 0;
	int8_t cInputIndex = 0;
	BaseType_t xMoreDataToFollow;
	/* The input and output buffers are declared static to keep them off the stack. */
	static char pcOutputString[MAX_OUTPUT_LENGTH], pcInputString[MAX_INPUT_LENGTH];

    /* This code assumes the peripheral being used as the console has already
    been opened and configured, and is passed into the task as the task
    parameter.  Cast the task parameter to the correct type. */
    xConsole = (USART_TypeDef*) pvParameters;
    // xConsole = USART3;
    /* Send a welcome message to the user knows they are connected. */
    uint32_t i;
    for (i = 0; i < strlen(pcWelcomeMessage); i++) {
    	while(!LL_USART_IsActiveFlag_TXE(xConsole)) {taskYIELD();}
    	LL_USART_TransmitData8(xConsole, pcWelcomeMessage[i]);
    }


    for( ;; )
    {
        /* This implementation reads a single character at a time. Yield if no character available */
 		while(!LL_USART_IsActiveFlag_RXNE(xConsole)) {/*taskYIELD();*/}
 		cRxedChar = LL_USART_ReceiveData8(xConsole);

        if( cRxedChar == '\n' || cRxedChar == '\r')
        {
            /* A newline character was received, so the input command string is
            complete and can be processed.  Transmit a line separator, just to
            make the output easier to read. */
            while(!LL_USART_IsActiveFlag_TXE(xConsole)) {/*taskYIELD();*/}
    		LL_USART_TransmitData8(xConsole, '\n');
    		while(!LL_USART_IsActiveFlag_TXE(xConsole)) {/*taskYIELD();*/}
    		LL_USART_TransmitData8(xConsole, '\r');
    		// while(!LL_USART_IsActiveFlag_TXE(USART3)) {/*taskYIELD();*/}
    		// LL_USART_TransmitData8(xConsole, '\n');

            /* The command interpreter is called repeatedly until it returns
            pdFALSE.  See the "Implementing a command" documentation for an
            explanation of why this is. */
            if (cInputIndex != 0) {
	            do {
	                /* Send the command string to the command interpreter.  Any
	                output generated by the command interpreter will be placed in the
	                pcOutputString buffer. */
	                xMoreDataToFollow = FreeRTOS_CLIProcessCommand
	                              (
	                                  pcInputString,   /* The command string.*/
	                                  pcOutputString,  /* The output buffer. */
	                                  MAX_OUTPUT_LENGTH/* The size of the output buffer. */
	                              );

	                /* Write the output generated by the command interpreter to the
	                console. */
	                for (i = 0; i < strlen(pcOutputString); i++) {
				    	while(!LL_USART_IsActiveFlag_TXE(xConsole)) {/*taskYIELD();*/}
				    	LL_USART_TransmitData8(xConsole, pcOutputString[i]);
				    }
	            } while( xMoreDataToFollow != pdFALSE );
	            cInputIndex = 0;
            	memset(pcInputString, 0x00, MAX_INPUT_LENGTH);
            	pcOutputString[0] = 0x00;
	        }

            /* All the strings generated by the input command have been sent.
            Processing of the command is complete.  Clear the input string ready
            to receive the next command. */
            while(!LL_USART_IsActiveFlag_TXE(xConsole)) {/*taskYIELD();*/}
			LL_USART_TransmitData8(xConsole, '>');
        }
        else
        {
            /* The if() clause performs the processing after a newline character
            is received.  This else clause performs the processing if any other
            character is received. */

            if (cRxedChar == '\r') {
                /* Ignore carriage returns. */
            } else if (cRxedChar == '\b' || cRxedChar == 0x7F) {
                /* Backspace was pressed.  Erase the last character in the input
                buffer - if there are any. */
                if (cInputIndex > 0) {
                    cInputIndex--;
                    pcInputString[ cInputIndex ] = '\0';
                    for (i = 0; i < strlen(pcBackSpace); i++) {
				    	while(!LL_USART_IsActiveFlag_TXE(xConsole)) {/*taskYIELD();*/}
				    	LL_USART_TransmitData8(xConsole, pcBackSpace[i]);
    				}
                }

            } else {
                /* A character was entered.  It was not a new line, backspace
                or carriage return, so it is accepted as part of the input and
                placed into the input buffer.  When a \n is entered the complete
                string will be passed to the command interpreter. */
                if (cInputIndex < MAX_INPUT_LENGTH) {
                    pcInputString[ cInputIndex ] = cRxedChar;
                    cInputIndex++;
                    while(!LL_USART_IsActiveFlag_TXE(xConsole)) {/*taskYIELD();*/}
    				LL_USART_TransmitData8(xConsole, cRxedChar);
                }
            }
        }
    }
}
			

/*-----------------------------------------------------------*/

static const char* const pcToggleLEDErrorMessage =
  "Error during toggle\r\n\r\n";

/* This function implements the behavior of a command, so must have the correct
prototype. */
static BaseType_t prvToggleLEDCommand(char *pcWriteBuffer,
                                     size_t xWriteBufferLen,
                                     const char *pcCommandString) {
	const char *pcParameter1;
	BaseType_t xParameter1StringLength, xResult;

    /* Obtain the name of the led, and the length of its name, from
    the command string. The name of the led is the first parameter. */
    pcParameter1 = FreeRTOS_CLIGetParameter(
                          /* The command string itself. */
                          pcCommandString,
                          /* Return the first parameter. */
                          1,
                          /* Store the parameter string length. */
                          &xParameter1StringLength
                        );

    /* Terminate the parameter. */
    /* Won't work because const, perhaps copy to temporary buffer? */
    // pcParameter1[ xParameter1StringLength ] = 0x00;

    /* Perform the toggle operation itself. */
    xResult = pdPASS;
    if (pcParameter1[0] == '2') mainTOGGLE_LED2();
    else if (pcParameter1[0] == '3') mainTOGGLE_LED3();
    else xResult = pdFAIL;

    if( xResult != pdPASS )
    {
        /* The toggle was successful.  There is nothing to output. */
        // snprintf( pcWriteBuffer, xWriteBufferLen, "Error during toggle\r\n\r\n" );
    	if (xWriteBufferLen < strlen(pcToggleLEDErrorMessage)) {
    		memcpy(pcWriteBuffer, pcToggleLEDErrorMessage, xWriteBufferLen);
    		return pdTRUE;
    	} else {
    		memcpy(pcWriteBuffer, pcToggleLEDErrorMessage, strlen(pcToggleLEDErrorMessage));
    	}
    }

    /* There is only a single line of output produced in all cases.  pdFALSE is
    returned because there is no more output to be generated. */
    return pdFALSE;
}

/*-----------------------------------------------------------*/

static const char* const pcFreqStartMessage =
  "CPU Clock Frequency: ";
static const char* const pcFreqEndMessage = 
  " Mhz\r\n";

/* This function implements the behavior of a command, so must have the correct
prototype. */
static BaseType_t prvFreqCommand(char *pcWriteBuffer,
                                     size_t xWriteBufferLen,
                                     const char *pcCommandString) {

	char str[5];
    itoa(SystemCoreClock/1000000, str, 10);

    memcpy(pcWriteBuffer, pcFreqStartMessage, strlen(pcFreqStartMessage));
    memcpy(pcWriteBuffer + strlen(pcFreqStartMessage), str, strlen(str));
    memcpy(pcWriteBuffer + strlen(pcFreqStartMessage) + strlen(str), pcFreqEndMessage, strlen(pcFreqEndMessage));

    /* There is only a single line of output produced in all cases.  pdFALSE is
    returned because there is no more output to be generated. */
    return pdFALSE;
}



