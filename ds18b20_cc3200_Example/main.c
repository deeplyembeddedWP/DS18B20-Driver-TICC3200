// Standard includes.
#include <stdio.h>
#include <stdlib.h>
#include "osi.h"

// Driverlib includes
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "rom.h"
#include "rom_map.h"
#include "uart.h"
#include "prcm.h"
#include "utils.h"

// Common interface includes
#include "common.h"
#include "uart_if.h"
#include "pinmux.h"
#include "gpio_if.h"

// Custom includes
#include "cc3200_ds18b20.h"

//*****************************************************************************
//                      DEBUG MACRO DEFINITIONS
//*****************************************************************************
#define DS18B20_TEST_INIT
#define DS18B20_TEST_READ_ROM_ADDR
#define DS18B20_TEST_POWER_CHECK
#define DS18B20_TEST_WRITE_SCRATCHPAD
#define DS18B20_TEST_READ_SCRATCHPAD
#define DS18B20_TEST_READ_TEMPERATURE

//*****************************************************************************
//                      MACRO DEFINITIONS
//*****************************************************************************
#define SPAWN_TASK_PRIORITY     9
#define OSI_STACK_SIZE          2048

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
#ifndef USE_TIRTOS
/* in case of TI-RTOS don't include startup_*.c in app project */
#if defined(gcc) || defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
#endif

static ds18b20_sram_T ds182b_sram_obj;
static ds18b20_temp_T ds18b2b_temp_obj;
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//                      LOCAL FUNCTION DEFINITIONS
//*****************************************************************************
static void vTestTask( void *pvParameters );
static void BoardInit();


#ifdef USE_FREERTOS
//*****************************************************************************
// FreeRTOS User Hook Functions enabled in FreeRTOSConfig.h
//*****************************************************************************

//*****************************************************************************
//
//! \brief Application defined hook (or callback) function - assert
//!
//! \param[in]  pcFile - Pointer to the File Name
//! \param[in]  ulLine - Line Number
//!
//! \return none
//!
//*****************************************************************************
void
vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    //Handle Assert here
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook( void)
{
    //Handle Idle Hook for Profiling, Power Management etc
}

//*****************************************************************************
//
//! \brief Application defined malloc failed hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationMallocFailedHook()
{
    //Handle Memory Allocation Errors
    while(1)
    {
    }
}

//*****************************************************************************
//
//! \brief Application defined stack overflow hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void vApplicationStackOverflowHook( OsiTaskHandle *pxTask,
                                    signed char *pcTaskName)
{
    //Handle FreeRTOS Stack Overflow
    while(1)
    {
    }
}
#endif //USE_FREERTOS

//******************************************************************************
//
//! First test task
//!
//! \param pvParameters is the parameter passed to the task while creating it.
//!
//!    This Function
//!        1. Receive message from the Queue and display it on the terminal.
//!
//! \return none
//
//******************************************************************************
void vTestTask( void *pvParameters )
{
    unsigned char rom_addr[8];

    ds18b20_Init();

    /* Test Init - Reset Code */
#ifdef DS18B20_TEST_INIT

    if(ds18b20_initialization_proc() == -1)
        UART_PRINT("Init - Reset Failed\r\n");
    else
        UART_PRINT("Init - Reset Passed\r\n");
#endif

    /* Get the ROM address of the ds18b20 */
#ifdef DS18B20_TEST_READ_ROM_ADDR

    if(ds18b20_single_read_ROM_addr(rom_addr) == -1)
        UART_PRINT("Failed to get device address\r\n");
    else
        UART_PRINT("Got Device Address !!!\r\n\r\n");
#endif

    /* Check the power supply mode */
#ifdef DS18B20_TEST_POWER_CHECK
    if(check_power_supply(rom_addr) == EXTERNAL_POWERED)
        UART_PRINT("Power supply check passed !!!\r\n\r\n");
    else
        UART_PRINT("Power supply check invalid\r\n");

#endif

    /* Write the configuration settings to SRAM */
#ifdef DS18B20_TEST_WRITE_SCRATCHPAD
    if(ds18b20_write_scratchpad(0x02, 0x01, 0x7F, rom_addr) == -1)
        UART_PRINT("Failed to write to SRAM\r\n");
    else
        UART_PRINT("Write to SRAM Success !!!\r\n\r\n");
#endif

    /* Read and dump the contents of SRAM */
#ifdef DS18B20_TEST_READ_SCRATCHPAD

    /* Init the sram data structure */
    ds182b_sram_obj.TH_alarm = 0x00;
    ds182b_sram_obj.TL_alarm = 0x00;
    ds182b_sram_obj.config_reg = 0x00;
    ds182b_sram_obj.temp_LSB = 0x00;
    ds182b_sram_obj.temp_MSB = 0x00;

    if(ds18b20_read_scratchpad(rom_addr,&ds182b_sram_obj) == -1)
        UART_PRINT("Failed to read SRAM\r\n");
#endif

    /* Read and dump temperature data from SRAM */
#ifdef DS18B20_TEST_READ_TEMPERATURE

    /* Init the temperature data structure */
    ds18b2b_temp_obj.celsius = 0.0;
    ds18b2b_temp_obj.fahrenheit = 0.0;
    while(1)
    {
        if(ds18b20_read_temperature(rom_addr, &ds18b2b_temp_obj) == -1)
            UART_PRINT("Failed to read temperature data\r\n");
        else
        {
            UART_PRINT("Celcius    = %.3f\r\n",ds18b2b_temp_obj.celsius);
            UART_PRINT("Fahrenheit = %.3f\r\n",ds18b2b_temp_obj.fahrenheit);

            /* One sec delay */
            MAP_UtilsDelay(13000000);
        }
#endif
    }
}



//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
static void
BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs) || defined(gcc)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

//*****************************************************************************
//
//!  main function handling the freertos_demo.
//!
//! \param  None
//!
//! \return none
//
//*****************************************************************************
int main( void )
{
    /* Initialize the board */
    BoardInit();

    /* Initialize peripherals */
    PinMuxConfig();

    /* Initializing the terminal */
    InitTerm();

    /* Clearing the terminal */
    ClearTerm();

    /* Create the Queue Receive task */
    osi_TaskCreate( vTestTask, "TASK1",\
                    OSI_STACK_SIZE, NULL, 1, NULL );

    /* Start the task scheduler */
    osi_start();

    return 0;
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
