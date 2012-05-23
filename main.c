/*
 FreeRTOS V7.0.2 - Copyright (C) 2011 Real Time Engineers Ltd.


 ***************************************************************************
 *                                                                       *
 *    FreeRTOS tutorial books are available in pdf and paperback.        *
 *    Complete, revised, and edited pdf reference manuals are also       *
 *    available.                                                         *
 *                                                                       *
 *    Purchasing FreeRTOS documentation will not only help you, by 	      *
 *    ensuring you get running as quickly as possible and with an        *
 *    in-depth knowledge of how to use FreeRTOS, it will also help       *
 *    the FreeRTOS project to continue with its mission of providing     *
 *    professional grade, cross platform, de facto standard solutions    *
 *    for microcontrollers - completely free of charge!                  *
 *                                                                       *
 *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
 *                                                                       *
 *    Thank you for using FreeRTOS, and thank you for your support!      *
 *                                                                       *
 ***************************************************************************


 This file is part of the FreeRTOS distribution.

 FreeRTOS is free software; you can redistribute it and/or modify it under
 the terms of the GNU General Public License (version 2) as published by the
 Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
 >>>NOTE<<< The modification to the GPL is included to allow you to
 distribute a combined work that includes FreeRTOS without being obliged to
 provide the source code for proprietary components outside of the FreeRTOS
 kernel.  FreeRTOS is distributed in the hope that it will be useful, but
 WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 more details. You should have received a copy of the GNU General Public
 License and the FreeRTOS license exception along with FreeRTOS; if not it
 can be viewed here: http://www.freertos.org/a00114.html and also obtained
 by writing to Richard Barry, contact details for whom are available on the
 FreeRTOS WEB site.

 1 tab == 4 spaces!

 http://www.FreeRTOS.org - Documentation, latest information, license and
 contact details.

 http://www.SafeRTOS.com - A version that is certified for use in safety
 critical systems.

 http://www.OpenRTOS.com - Commercial support, development, porting,
 licensing and training services.
 */

/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main. c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.  
 * Each task that does not flash an LED maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles an LED.  Should any task contain an error at any time the LED toggle
 * will stop.
 *
 * The LED flash and communications test tasks do not maintain a count.
 */

/*
 Changes from V1.2.0

 + Changed the baud rate for the serial test from 19200 to 57600.

 Changes from V1.2.3

 + The integer and comtest tasks are now used when the cooperative scheduler
 is being used.  Previously they were only used with the preemptive
 scheduler.

 Changes from V1.2.5

 + Set the baud rate to 38400.  This has a smaller error percentage with an
 8MHz clock (according to the manual).

 Changes from V2.0.0

 + Delay periods are now specified using variables and constants of
 portTickType rather than unsigned long.

 Changes from V2.6.1

 + The IAR and WinAVR AVR ports are now maintained separately.

 Changes from V4.0.5

 + Modified to demonstrate the use of co-routines.

 */

#include <stdlib.h>
#include <string.h>

#ifdef GCC_MEGA_AVR
/* EEPROM routines used only with the WinAVR compiler. */
#include <avr/eeprom.h>
#endif

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"

/* Demo file headers. */
#include "PollQ.h"
#include "integer.h"
#include "serial.h"
#include "comtest.h"
#include "crflash.h"
#include "print.h"
#include "partest.h"
#include "regtest.h"

#include <avr/interrupt.h>
#include <avr/io.h>

#include "oi.h"

/* Priority definitions for most of the tasks in the demo application.  Some
 tasks just use the idle priority. */
#define mainLED_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )
#define mainCOM_TEST_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainQUEUE_POLL_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY			( tskIDLE_PRIORITY + 3 )

/* Baud rate used by the serial port tasks. */
#define mainCOM_TEST_BAUD_RATE			( ( unsigned long ) 38400 )

/* LED used by the serial port tasks.  This is toggled on each character Tx,
 and mainCOM_TEST_LED + 1 is toggles on each character Rx. */
#define mainCOM_TEST_LED				( 4 )

/* LED that is toggled by the check task.  The check task periodically checks
 that all the other tasks are operating without error.  If no errors are found
 the LED is toggled.  If an error is found at any time the LED is never toggles
 again. */
#define mainCHECK_TASK_LED				( 7 )

/* The period between executions of the check task. */
#define mainCHECK_PERIOD				( ( portTickType ) 3000 / portTICK_RATE_MS  )

/* An address in the EEPROM used to count resets.  This is used to check that
 the demo application is not unexpectedly resetting. */
#define mainRESET_COUNT_ADDRESS			( ( void * ) 0x50 )

/* The number of coroutines to create. */
#define mainNUM_FLASH_COROUTINES		( 3 )

/*
 * The task function for the "Check" task.
 */
static void vErrorChecks(void *pvParameters);

/*
 * Checks the unique counts of other tasks to ensure they are still operational.
 * Flashes an LED if everything is okay. 
 */
static void prvCheckOtherTasksAreStillRunning(void);

/*
 * Called on boot to increment a count stored in the EEPROM.  This is used to 
 * ensure the CPU does not reset unexpectedly.
 */
static void prvIncrementResetCount(void);

/*
 * The idle hook is used to scheduler co-routines.
 */
void vApplicationIdleHook(void);

//1st task
static void vLEDTask1(void* pvParameters);
static void vLEDTask2(void* pvParameters);
static void vLEDTask3(void* pvParameters);
static void vLEDTask4(void* pvParameters);

/*-----------------------------------------------------------*/

int main(void) {

	DDRA	= 0xff;
	DDRB	= 0xff;
	DDRC	= 0xf0;
	DDRD 	= 0xff;           /* make the LED pin an output */
	
	//for (;;) {
	//	//LED1On;
//
//		char i;
//		for (i = 0; i < 10; i++) {
//			//_delay_ms(3); /* max is 262.14 ms / F_CPU in MHz */
//			vTaskDelay(10);
//			PORTC ^= 0xff; /* toggle the LED */
//		}	
//		PORTD ^= 0xff; /* toggle the LED */
//
//	}

	xTaskCreate(vLEDTask1, "LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(vLEDTask2, "LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(vLEDTask3, "LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(vLEDTask4, "LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	vTaskStartScheduler();
	return 0;

}
/*-----------------------------------------------------------*/

static void vLEDTask1(void* pvParameters) {
	uint16_t led_state = 0;
	/* The LEDs are updated every 200 ticks, about 200 ms */
	const uint16_t blinkDelay = 1;

	/* Initialize the LEDs */LED1On;

	/* Infinite loop */
	while (1) {

//		LED1On;

		/* Block the task for the defined time */
//		vTaskDelay(blinkDelay);

		
		PORTB ^= 1<< 1; /* toggle the LED */

		/* Block the task for the defined time */
		vTaskDelay(blinkDelay);

	}

}

static void vLEDTask2(void* pvParameters) {
	uint16_t led_state = 0;
	/* The LEDs are updated every 200 ticks, about 200 ms */
	const uint16_t blinkDelay = 10;

	/* Initialize the LEDs */LED1On;

	/* Infinite loop */
	while (1) {

//		LED1On;

		/* Block the task for the defined time */
//		vTaskDelay(blinkDelay);

		
		PORTB ^= 1<< 2; /* toggle the LED */

		/* Block the task for the defined time */
		vTaskDelay(blinkDelay);

	}

}

static void vLEDTask3(void* pvParameters) {
	uint16_t led_state = 0;
	/* The LEDs are updated every 200 ticks, about 200 ms */
	const uint16_t blinkDelay = 100;

	/* Initialize the LEDs */LED1On;

	/* Infinite loop */
	while (1) {

//		LED1On;

		/* Block the task for the defined time */
//		vTaskDelay(blinkDelay);

		
		PORTB ^= 1<< 3; /* toggle the LED */

		/* Block the task for the defined time */
		vTaskDelay(blinkDelay);

	}

}

static void vLEDTask4(void* pvParameters) {
	uint16_t led_state = 0;
	/* The LEDs are updated every 200 ticks, about 200 ms */
	const uint16_t blinkDelay = 1000;

	/* Initialize the LEDs */

	/* Infinite loop */
	while (1) {
		
		PORTB ^= 1 << 4; /* toggle the LED */

		/* Block the task for the defined time */
		vTaskDelay(blinkDelay);

	}

}

static void vErrorChecks(void *pvParameters) {
	static volatile unsigned long ulDummyVariable = 3UL;

	/* The parameters are not used. */
	(void) pvParameters;

	/* Cycle for ever, delaying then checking all the other tasks are still
	 operating without error. */
	for (;;) {
		vTaskDelay(mainCHECK_PERIOD);

		/* Perform a bit of 32bit maths to ensure the registers used by the 
		 integer tasks get some exercise. The result here is not important -
		 see the demo application documentation for more info. */
		ulDummyVariable *= 3;

		prvCheckOtherTasksAreStillRunning();
	}
}
/*-----------------------------------------------------------*/

static void prvCheckOtherTasksAreStillRunning(void) {
	static portBASE_TYPE xErrorHasOccurred = pdFALSE;

	if (xAreIntegerMathsTaskStillRunning() != pdTRUE) {
		xErrorHasOccurred = pdTRUE;
	}

	if (xAreComTestTasksStillRunning() != pdTRUE) {
		xErrorHasOccurred = pdTRUE;
	}

	if (xArePollingQueuesStillRunning() != pdTRUE) {
		xErrorHasOccurred = pdTRUE;
	}

	if (xAreRegTestTasksStillRunning() != pdTRUE) {
		xErrorHasOccurred = pdTRUE;
	}

	if (xErrorHasOccurred == pdFALSE) {
		/* Toggle the LED if everything is okay so we know if an error occurs even if not
		 using console IO. */
		vParTestToggleLED(mainCHECK_TASK_LED);
	}
}
/*-----------------------------------------------------------*/

static void prvIncrementResetCount(void) {
	unsigned char ucCount;

	eeprom_read_block(&ucCount, mainRESET_COUNT_ADDRESS, sizeof(ucCount));
	ucCount++;
	eeprom_write_byte(mainRESET_COUNT_ADDRESS, ucCount);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void) {
	vCoRoutineSchedule();
}

