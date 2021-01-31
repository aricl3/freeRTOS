/* Standard includes. */
#include <stdlib.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "croutine.h"
#include"list.h"
#include"portable.h"

/* Added Galileo serial support. */
#include "galileo_support.h"

/* Set to 1 to sit in a loop on start up, allowing a debugger to connect to the
application before main() executes. */
#define mainWAIT_FOR_DEBUG_CONNECTION 		0

/* Set mainCREATE_SIMPLE_BLINKY_DEMO_ONLY to one to run the simple blinky demo,
or 0 to run the more comprehensive test and demo application. */
#define mainCREATE_SIMPLE_BLINKY_DEMO_ONLY	1

/*-----------------------------------------------------------*/

/*
 * main_blinky() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 1.
 * main_full() is used when mainCREATE_SIMPLE_BLINKY_DEMO_ONLY is set to 0.
 */
extern void main_blinky( void );
extern void main_temperature( void );


/* Prototypes for functions called from asm start up code. */
int main( void );
void CRT_Init( void );

/*
 * Prototypes for the standard FreeRTOS callback/hook functions implemented
 * within this file.
 */
void vApplicationMallocFailedHook( void );
void vApplicationIdleHook( void );
void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName );
void vApplicationTickHook( void );

/*
 * Perform any hardware/peripheral related initialisation necessary to run the
 * demo.
 */
static void prvSetupHardware( void );
static void prvCalibrateLVTimer( void );

/*
 * If mainWAIT_FOR_DEBUG_CONNECTION is set to 1 then the following function will
 * sit in a loop on start up, allowing a debugger to connect to the application
 * before main() executes.  If mainWAIT_FOR_DEBUG_CONNECTION is not set to 1
 * then the following function does nothing.
 */
static void prvLoopToWaitForDebugConnection( void );

/*
 * Helper functions used when an assert is triggered.  The first periodically
 * displays an assert message, and the second clears the assert message when the
 * function called by the configASSERT() macro is exited.
 */
static void prvDisplayAssertion( const char * pcFile, unsigned long ulLine );
static void prvClearAssertionLine( void );

/*-----------------------------------------------------------*/
//添加
/* -----------------------声明三个任务函数名----------------------------------*/
//第二个测试所需代码
static void vTSK1( void *pvParameters );
static void vTSK2( void *pvParameters );
static void vTSK3( void *pvParameters );

//第三个测试所需代码
static void LEDTest(void *pvParameters);

//第四个测试所需代码
static void TemperatureTest(void *pvParameters);




/*-----------------------------------------------------------*/

/* See http://www.FreeRTOS.org/RTOS_Intel_Quark_Galileo_GCC.html for usage
instructions. */

int main( void )
{
	/* Optionally wait for a debugger to connect. */
	prvLoopToWaitForDebugConnection();
	// Init the UART, GPIO, etc.
	prvSetupHardware();

	 /*The mainCREATE_SIMPLE_BLINKY_DEMO_ONLY setting is described at the top
	of this file.*/
	#if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 1 )
	{
		g_printf_rcc( 3, 2, DEFAULT_SCREEN_COLOR, "Temperature\n" );

/*
		//第一个函数，调用blink()函数
		main_blinky();
*/

/*
		//第二个函数，写三个简单的任务，进行freertos应用程序的开发
		//1、定义任务Handle变量
		xTaskHandle T1,T2,T3;
		//2、创建任务vTSK1
		if(xTaskCreate( vTSK1 , "vTSK1" , 120 , NULL, 2 , &T1 ))
			printf("task1 OK\r\n");
		//创建任务vTSK2
		if(xTaskCreate( vTSK2 , "vTSK2" , 120 , NULL, 2 , &T2 ))
					printf("task2 OK\r\n");
		//创建任务vTSK3
		if(xTaskCreate( vTSK3 , "vTSK3" , 120 , NULL, 2 , &T3 ))
					printf("task3 OK\r\n");
		vTaskStartScheduler(); //3、启动已成功创建的任务
*/

/*
		//设计闪灯程序
		xTaskCreate(LEDTest, (signed portCHAR *) "LED", configMINIMAL_STACK_SIZE*10, NULL, 0, NULL);
		vTaskStartScheduler();
*/
		xTaskCreate(TemperatureTest, (signed portCHAR *) "TEM", configMINIMAL_STACK_SIZE*10, NULL, 0, NULL);
				vTaskStartScheduler();
	}
	#else
	{
		g_printf_rcc( 3, 2, DEFAULT_SCREEN_COLOR, "Running main_full().\n\r" );
  		main_full();
	}
	#endif

	return 0;
}
/*-----------------------------------------------------------*/
//第二个测试所需代码
//三个任务的实现过程
void vTSK1( void *pvParameters )
{
	while(1)
    {
		printf("Task1 ");
		vTaskDelay(300);//如是死循环的任务，必需加入该语句，参数>0即可。
    }
}
void vTSK2( void *pvParameters )
{
	while(1)
    {
		printf("Task2 ");
		vTaskDelay(700);
    }
}
void vTSK3( void *pvParameters )
{
	while(1)
	{
		printf("Task3 ");
		vTaskDelay(3000);
    }
}

//第三个测试所需的任务
static void LEDTest(void *pvParameters)
{
	while(1)
	{
		uint32_t ulLEDStatus;
		ulLEDStatus = ulBlinkLED();
		printf("LED State = %d\r\n", ( int ) ulLEDStatus );
		vTaskDelay(1000);
	}
}

//第四个测试所需的任务

static void TemperatureTest(void *pvParameters)
{
	while(1)
	{
		uint32_t ulLEDStatus;
		ulLEDStatus = DS18B20_ReadTemp();
		printf("Temperature is  %d\r\n", ( int ) ulLEDStatus );
		vTaskDelay(3000);
	}
}


void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h.

	Force an assert. */
	configASSERT( xTaskGetTickCount() == 0 );
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.

	Increase the size of the stack allocated to the offending task.

	Force an assert. */
	configASSERT( pxTask == NULL );
	taskDISABLE_INTERRUPTS();
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
	volatile unsigned long xFreeHeapSpace;

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

static void prvDisplayAssertion( const char * pcFile, unsigned long ulLine )
{
extern void vMilliSecondDelay( uint32_t DelayTime );
const uint32_t ul500ms = 500UL;

	/* Display assertion file and line. Don't use the gated g_printf just in
	the assert was triggered while the gating semaphore was taken.  Always print
	on line 23. */
	UngatedMoveToScreenPosition( 23, 2 );
	printf( ANSI_COLOR_RED );
	printf( "ASSERT: File = %s, Line = %u\n\r", pcFile, ulLine );
	printf( ANSI_COLOR_RESET );
	printf( ANSI_SHOW_CURSOR );
	vMilliSecondDelay( ul500ms );
}
/*-----------------------------------------------------------*/

static void prvClearAssertionLine( void )
{
	UngatedMoveToScreenPosition( 23, 1 );
	printf( ANSI_COLOR_RESET );
	printf( ANSI_CLEAR_LINE );
	printf( ANSI_HIDE_CURSOR );
}
/*-----------------------------------------------------------*/

void vAssertCalled( const char * pcFile, unsigned long ulLine )
{
volatile uint32_t ul = 0;

	( void ) pcFile;
	( void ) ulLine;

	taskENTER_CRITICAL();
	{
		/* Set ul to a non-zero value or press a key to step out of this
		function in order to inspect the location of the assert(). */

		/* Clear any pending key presses. */
		while( ucGalileoGetchar() != 0 )
		{
			/* Nothing to do here - the key press is just discarded. */
		}

		do
		{
		   prvDisplayAssertion(pcFile, ulLine);
		} while ( ( ul == pdFALSE ) && ( ucGalileoGetchar() == 0 ) );

		prvClearAssertionLine();
	}
	taskEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	#if( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY == 0 )
	{
		extern void vTimerPeriodicISRTests( void );

		/* The full demo includes a software timer demo/test that requires
		prodding periodically from the tick interrupt. */
		vTimerPeriodicISRTests();

		/* Call the periodic queue overwrite from ISR demo. */
		vQueueOverwritePeriodicISRDemo();

		/* Call the periodic event group from ISR demo. */
		vPeriodicEventGroupsProcessing();

		/* Call the periodic queue set from ISR demo. */
		vQueueSetAccessQueueSetFromISR();

		/* Use task notifications from an interrupt. */
		xNotifyTaskFromISR();
	}
	#endif
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Initialise the serial port and GPIO. */
	vInitializeGalileoSerialPort( DEBUG_SERIAL_PORT );    //串口初始化函数
	vGalileoInitializeGpioController();                   //＋下面函数共为控制器初始化函数
	vGalileoInitializeLegacyGPIO();                       //在此函数里有控制灯的亮灭

	/* Initialise HPET interrupt(s) */
	#if( ( mainCREATE_SIMPLE_BLINKY_DEMO_ONLY != 1 ) && ( hpetHPET_TIMER_IN_USE != 0 ) )
	{
		portDISABLE_INTERRUPTS();
		vInitializeAllHPETInterrupts();
	}
	#endif

	/* Setup the LED. */
	vGalileoLegacyGPIOInitializationForLED();

	/* Demonstrates how to calibrate LAPIC Timer.  The calibration value
	calculated here may get overwritten when the scheduler starts. */
	prvCalibrateLVTimer();

	/* Print RTOS loaded message. */
	vPrintBanner();
}
/*-----------------------------------------------------------*/

static void prvLoopToWaitForDebugConnection( void )
{
	/* Debug if define = 1. */
	#if( mainWAIT_FOR_DEBUG_CONNECTION == 1 )
	{
	/* When using the debugger, set this value to pdFALSE, and the application
	will sit in a loop at the top of main() to allow the debugger to attached
	before the application starts running.  Once attached, set
	ulExitResetSpinLoop to a non-zero value to leave the loop. */
	volatile uint32_t ulExitResetSpinLoop = pdFALSE;

		/* Must initialize UART before anything will print. */
		vInitializeGalileoSerialPort( DEBUG_SERIAL_PORT );

		/* RTOS loaded message. */
		vPrintBanner();

		/* Output instruction message. */
		MoveToScreenPosition( 3, 1 );
		g_printf( DEFAULT_SCREEN_COLOR );
		g_printf( " Waiting for JTAG connection.\n\n\r" );
		g_printf( ANSI_COLOR_RESET );
		g_printf( " Once connected, either set ulExitResetSpinLoop to a non-zero value,\n\r" );
		g_printf( " or you can [PRESS ANY KEY] to start the debug session.\n\n\r" );
		printf( ANSI_SHOW_CURSOR );

		/* Use the debugger to set the ulExitResetSpinLoop to a non-zero value
		or press a key to exit this loop, and step through the application.  In
		Eclipse, simple hover over the variable to see its value in a pop-over
		box, then edit the value in the pop-over box. */
		do
		{
			portNOP();

		} while( ( ulExitResetSpinLoop == pdFALSE ) && ( ucGalileoGetchar() == 0 ) );
	}
	#endif
}
/*-----------------------------------------------------------*/

void CRT_Init( void )
{
extern uint32_t __bss_start[];
extern uint32_t __bss_end[];
extern uint32_t __data_vma[];
extern uint32_t __data_lma[];
extern uint32_t __data_start[];
extern uint32_t __data_end[];
uint32_t x = 255;
size_t xSize;

	/* Zero out bss. */
	xSize = ( ( size_t ) __bss_end ) - ( ( size_t ) __bss_start );
	memset( ( void * ) __bss_start, 0x00, xSize );

	/* Copy initialised variables. */
	xSize = ( ( size_t ) __data_end ) - ( ( size_t ) __data_start );
	memcpy( ( void * ) __data_vma, __data_lma, xSize );

	/* Ensure no interrupts are pending. */
	do
	{
		portAPIC_EOI = 0;
		x--;
	} while( x > 0 );
}
/*-----------------------------------------------------------*/

static void prvCalibrateLVTimer( void )
{
uint32_t uiInitialTimerCounts, uiCalibratedTimerCounts;

	/* Disable LAPIC Counter. */
	portAPIC_LVT_TIMER = portAPIC_DISABLE;

	/* Calibrate the LV Timer counts to ensure it matches the HPET timer over
	extended periods. */
	uiInitialTimerCounts = ( ( configCPU_CLOCK_HZ >> 4UL ) / configTICK_RATE_HZ );
	uiCalibratedTimerCounts = uiCalibrateTimer( 0, hpetLVTIMER );

	if( uiCalibratedTimerCounts != 0 )
	{
		uiInitialTimerCounts = uiCalibratedTimerCounts;
	}

	/* Set the interrupt frequency. */
	portAPIC_TMRDIV = portAPIC_DIV_16;
	portAPIC_TIMER_INITIAL_COUNT = uiInitialTimerCounts;

	/* Enable LAPIC Counter. */
	portAPIC_LVT_TIMER = portAPIC_TIMER_PERIODIC | portAPIC_TIMER_INT_VECTOR;

	/* Sometimes needed. */
	portAPIC_TMRDIV = portAPIC_DIV_16;
}
