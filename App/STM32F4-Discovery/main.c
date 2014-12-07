/*
    FreeRTOS V7.5.3 - Copyright (C) 2013 Real Time Engineers Ltd.
    All rights reserved
 */

/*!	\file main.c
 * 	\brief Main file with main functions for MOTOR TCP CLI application 
 */


// includes 

#include "main.h"
#include "spi.h"
#include "W5200.h"
#include "modbus_mk.h"
#include "tcpCLI.h"


#ifdef DEBUG
#include "printf.h"
#endif



///* Priorities for the demo application tasks. */
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1UL )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2UL )
#define mainCREATOR_TASK_PRIORITY			( tskIDLE_PRIORITY + 3UL )
#define mainFLOP_TASK_PRIORITY				( tskIDLE_PRIORITY )


/* The period after which the check timer will expire, in ms, provided no errors
have been reported by any of the standard demo tasks.  ms are converted to the
equivalent in ticks using the portTICK_RATE_MS constant. */
#define mainCHECK_TIMER_PERIOD_MS			( 3000UL / portTICK_RATE_MS )

/* The period at which the check timer will expire, in ms, if an error has been
reported in one of the standard demo tasks.  ms are converted to the equivalent
in ticks using the portTICK_RATE_MS constant. */
#define mainERROR_CHECK_TIMER_PERIOD_MS 	( 200UL / portTICK_RATE_MS )

/* Set mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 1 to create a simple demo.
Set mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY to 0 to create a much more
comprehensive test application.  See the comments at the top of this file, and
the documentation page on the http://www.FreeRTOS.org web site for more
information. */
#define mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY		0

/*-----------------------------------------------------------*/

/** setup hardware for system */ 
static void prvSetupHardware( void );

/*
 * The check timer callback function, as described at the top of this file.
 */
static void prvCheckTimerCallback( xTimerHandle xTimer );

/*
 * Configure the interrupts used to test the interrupt nesting depth as
 * described at the top of this file.
 */
static void prvSetupNestedFPUInterruptsTest( void );

/*
 * Register check tasks, and the tasks used to write over and check the contents
 * of the FPU registers, as described at the top of this file.  The nature of
 * these files necessitates that they are written in an assembly file.
 */
extern void vRegTest1Task( void *pvParameters );
extern void vRegTest2Task( void *pvParameters );
extern void vRegTestClearFlopRegistersToParameterValue( unsigned long ulValue );
extern unsigned long ulRegTestCheckFlopRegistersContainParameterValue( unsigned long ulValue );

/*
 * The task that is synchronised with the button interrupt.  This is done just
 * to demonstrate how to write interrupt service routines, and how to
 * synchronise a task with an interrupt.
 */
static void prvButtonTestTask( void *pvParameters );
static void main_task(void *pvParameters);

/*
 * This file can be used to create either a simple LED flasher example, or a
 * comprehensive test/demo application - depending on the setting of the
 * mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY constant defined above.  If
 * mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY is set to 1, then the following
 * function will create a lot of additional tasks and a software timer.  If
 * mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY is set to 0, then the following
 * function will do nothing.
 */
static void prvOptionallyCreateComprehensveTestApplication( void );

/*-----------------------------------------------------------*/

/* The following two variables are used to communicate the status of the
register check tasks to the check software timer.  If the variables keep
incrementing, then the register check tasks has not discovered any errors.  If
a variable stops incrementing, then an error has been found. */
volatile unsigned long ulRegTest1LoopCounter = 0UL, ulRegTest2LoopCounter = 0UL;

/* The following variables are used to verify that the interrupt nesting depth
is as intended.  ulFPUInterruptNesting is incremented on entry to an interrupt
that uses the FPU, and decremented on exit of the same interrupt.
ulMaxFPUInterruptNesting latches the highest value reached by
ulFPUInterruptNesting.  These variables have no other purpose. */
volatile unsigned long ulFPUInterruptNesting = 0UL, ulMaxFPUInterruptNesting = 0UL;
/* The semaphore used to demonstrate a task being synchronised with an
interrupt. */
static xSemaphoreHandle xTestSemaphore = NULL;

#ifdef DEBUG
#define SWO_BAUD_RATE 230400

void CoreSight_configure(uint32_t SystemCoreClock)
{

  uint32_t SWOPrescaler;

  SWOPrescaler = (SystemCoreClock / SWO_BAUD_RATE ) - 1;

  CoreDebug->DEMCR = 1 << CoreDebug_DEMCR_TRCENA_Pos; /* Enable trace */
  *((volatile unsigned *) 0xE0042004) = 0x00000020;   /* DBGMCU_CR */
  
  
  *((volatile unsigned *) 0xE0040004) = 0x00000001; /* port size -> 1 bit */

  /* Set TPIU register->Selected pinprotocol = 10b: Serial Wire Output - NRZ */
  *((volatile unsigned *) 0xE00400F0) = 0x00000002; /* "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO)*/

  /* Set TPIU -> Async Clock Prescaler Register [bits 0-12] */
  *((volatile unsigned *) 0xE0040010) = SWOPrescaler; /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */

  *((volatile unsigned *) 0xE0040304) = 0x00000100; /* Formatter and Flush Control Register */

  /* ITM Lock Access Register */
  *((volatile unsigned *) 0xE0000FB0) = 0xC5ACCE55; /* ITM Lock Access Register, C5ACCE55 enables more */
                                                    /* write access to Control Register 0xE00 :: 0xFFC */
  *((volatile unsigned *) 0xE0000E80) = 0x00010005; /* ITM Trace Control Register */
  *((volatile unsigned *) 0xE0000E00) = 0x00000001; /* ITM Trace Enable Register. Enabled tracing on stimulus */
                                                    /* ports. One bit per stimulus port. */
  *((volatile unsigned *) 0xE0000E40) = 0x00000001; /* ITM Trace Privilege Register */

  /*  *((volatile unsigned *) 0xE0001000) = 0x400003FE; */ /* DWT_CTRL */

  //  *(volatile unsigned int *)0xE0001000 |= 0x00000001 ;  /* Enable cycle counter*/
  // *(volatile unsigned int *)0xE0001004 = 0;             /* Reset counter */
}

#endif

/*-----------------------------------------------------------*/

int main(void)
{

#ifdef DEBUG
	SystemCoreClockUpdate();

	CoreSight_configure(SystemCoreClock);
	
	/*<! Configure the hardware ready to run the test. */
	t_printf("Starting\n");
#endif
	prvSetupHardware();

	xSmphrUSART = xSemaphoreCreateBinary();
	
	// ============now register CLI commands ===================
	
	FreeRTOS_CLIRegisterCommand( &xMotorCommand );
       	FreeRTOS_CLIRegisterCommand( &xTaskStatsCommand);		

	// create queues 
	QSpd_handle = xQueueCreate(2, sizeof(QueueTelegram));
	QTCP_handle = xQueueCreate(2, sizeof(QueueTelegram));
	

/*------------------added by Matic Knap 24 Jun 2014 ---------------------------------*/



	// echo server task 
	if ( xTaskCreate(tcp_srv_Task, "TCPsrv", configMINIMAL_STACK_SIZE * 10, 
			NULL, mainFLASH_TASK_PRIORITY , &set_macTaskHandle)
			!= pdTRUE) 
	{
		#ifdef DEBUG
		t_printf("Error creating tcp_srv_task\n");
		#endif
		return -1;
	}
	else
	{
		#ifdef DEBUG
		t_printf("Succsessfully created tcp_srv_task\n");
		#endif
			
	}
	
	// run motor task
	if (xTaskCreate(motorControl_task, "motor", configMINIMAL_STACK_SIZE * 10,
		       	NULL, mainFLASH_TASK_PRIORITY , &motorHBHandle)
		!= pdTRUE)
	{
		#ifdef DEBUG
		t_printf("Error creating motor task.\n");
		#endif
		return -1;

	}
	else
	{
		#ifdef DEBUG
		t_printf("Succsessfully created motor task\n");
		#endif
			
	}


	// set motor task 
	if (xTaskCreate(motorHeartBeat_task, "motorHB", configMINIMAL_STACK_SIZE * 5,		       				
			NULL, mainFLASH_TASK_PRIORITY  , &motorHeartBeatHandle)
			!= pdTRUE)
	{
		#ifdef DEBUG
		t_printf("Error creating motorHB task.\n");
		#endif
		return -1;

	}
	else
	{
		#ifdef DEBUG
		t_printf("Succsessfully created motorHB task\n");
		#endif
			
	}

	

/*------------------added by Matic Knap 24 Jun 2014 ---------------------------------*/


	/* The following function will only create more tasks and timers if
	mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY is set to 0 (at the top of this
	file).  See the comments at the top of this file for more information. */
	//prvOptionallyCreateComprehensveTestApplication();

	/* Start the scheduler. */
	vTaskStartScheduler();

	#ifdef DEBUG
	t_printf("\n\nError with scheduler!! .\n\n");
	#endif


	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );
}



/*! 	\fn static void prvSetupHardware(void) 
 *	\brief Sets up hardware
 * 	
 */
void prvSetupHardware( void )
{
	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();

	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	//init_SPIx();
	init_SPIx();	
	
	// init USARTx 
	init_USARTx();

	/*
	 * Create task - initialize Wiznet
	 * It creates task for Wiznet initialization and at end delete it. 
	 * Function : 		init_W5200 from W5200.c file 
	 * Stack size :		5 times minimial stack size 
	 * Task priority :	main flash task priority + 2 
	 * Parameters 	 :	no parameters (NULL)
	 */  
	xTaskCreate(init_W5200, "init_W5200", configMINIMAL_STACK_SIZE, 
			NULL, mainFLASH_TASK_PRIORITY + 2 , NULL);


	
	
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	#if ( mainCREATE_SIMPLE_LED_FLASHER_DEMO_ONLY == 0 )
	{
		/* Just to verify that the interrupt nesting behaves as expected,
		increment ulFPUInterruptNesting on entry, and decrement it on exit. */
		ulFPUInterruptNesting++;

		/* Fill the FPU registers with 0. */
		//vRegTestClearFlopRegistersToParameterValue( 0UL );

		/* Trigger a timer 2 interrupt, which will fill the registers with a
		different value and itself trigger a timer 3 interrupt.  Note that the
		timers are not actually used.  The timer 2 and 3 interrupt vectors are
		just used for convenience. */
		NVIC_SetPendingIRQ( TIM2_IRQn );

		/* Ensure that, after returning from the nested interrupts, all the FPU
		registers contain the value to which they were set by the tick hook
		function. */
		//configASSERT( ulRegTestCheckFlopRegistersContainParameterValue( 0UL ) );

		ulFPUInterruptNesting--;
	}
	#endif
}
/*-----------------------------------------------------------*/

static void prvSetupNestedFPUInterruptsTest( void )
{
NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM2 interrupt in the NVIC.  The timer itself is not used,
	just its interrupt vector to force nesting from software.  TIM2 must have
	a lower priority than TIM3, and both must have priorities above
	configMAX_SYSCALL_INTERRUPT_PRIORITY. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );

	/* Enable the TIM3 interrupt in the NVIC.  The timer itself is not used,
	just its interrupt vector to force nesting from software.  TIM2 must have
	a lower priority than TIM3, and both must have priorities above
	configMAX_SYSCALL_INTERRUPT_PRIORITY. */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY - 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init( &NVIC_InitStructure );
}
/*-----------------------------------------------------------*/

void TIM3_IRQHandler( void )
{
	/* Just to verify that the interrupt nesting behaves as expected, increment
	ulFPUInterruptNesting on entry, and decrement it on exit. */
	ulFPUInterruptNesting++;

	/* This is the highest priority interrupt in the chain of forced nesting
	interrupts, so latch the maximum value reached by ulFPUInterruptNesting.
	This is done purely to allow verification that the nesting depth reaches
	that intended. */
	if( ulFPUInterruptNesting > ulMaxFPUInterruptNesting )
	{
		ulMaxFPUInterruptNesting = ulFPUInterruptNesting;
	}

	/* Fill the FPU registers with 99 to overwrite the values written by
	TIM2_IRQHandler(). */
	//vRegTestClearFlopRegistersToParameterValue( 99UL );

	ulFPUInterruptNesting--;
}
/*-----------------------------------------------------------*/

void TIM2_IRQHandler( void )
{
	/* Just to verify that the interrupt nesting behaves as expected, increment
	ulFPUInterruptNesting on entry, and decrement it on exit. */
	ulFPUInterruptNesting++;

	/* Fill the FPU registers with 1. */
	//vRegTestClearFlopRegistersToParameterValue( 1UL );

	/* Trigger a timer 3 interrupt, which will fill the registers with a
	different value. */
	NVIC_SetPendingIRQ( TIM3_IRQn );

	/* Ensure that, after returning from the nesting interrupt, all the FPU
	registers contain the value to which they were set by this interrupt
	function. */
	//configASSERT( ulRegTestCheckFlopRegistersContainParameterValue( 1UL ) );

	ulFPUInterruptNesting--;
}
/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	/* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
	to 1 in FreeRTOSConfig.h.  It will be called on each iteration of the idle
	task.  It is essential that code added to this hook function never attempts
	to block in any way (for example, call xQueueReceive() with a block time
	specified, or call vTaskDelay()).  If the application makes use of the
	vTaskDelete() API function (as this demo application does) then it is also
	important that vApplicationIdleHook() is permitted to return to its calling
	unction, because it is the responsibility of the idle task to clean up
	memory allocated by the kernel to any task that has since been deleted. */
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
///*-----------------------------------------------------------*/
void assert_failed(uint8_t* file, uint32_t line){}
