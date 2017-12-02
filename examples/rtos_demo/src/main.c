#include <stdio.h>
#include <string.h>

/* STM CUBE files */
#include "stm32f7xx.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_conf.h"

/* FreeRTOS include files. */
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS_CLI.h"
#include "task.h"
#include "semphr.h"
// #include "port.c"

/* Project Files */
#include "board.h"
#include "init.h"
#include "console.h"

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

/*-----------------------------------------------------------*/

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

/* Console UART Handler Type */
static UART_HandleTypeDef usart3;
static RNG_HandleTypeDef rng;

/*-----------------------------------------------------------*/

int main(void) {
	/* Configure the hardware ready to run the demo. */
	vSetupHardware(&usart3, &rng);

	vConsoleInit(&rng);
	xTaskCreate(vCommandConsoleTask, "console", configMINIMAL_STACK_SIZE*4, &usart3, mainCOMMAND_CONSOLE_TASK_PRIORITY, NULL);

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
		HAL_UART_Transmit(&usart3, "Assert Called\r\n", 15, 10);
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
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}




