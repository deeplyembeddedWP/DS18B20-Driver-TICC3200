/*
 * cc3200_ds18b20.c
 *
 * Created on  : Jan 21, 2018
 *
 * Author      : Vinay Divakar
 * Description : This is a DS18b20 driver for enabling communication between the TICC3200 SimpleLink MCU and the
 *               DS18b20 temperature sensor.
 *
 * Website     : www.deeplyembedded.org
 *
 * Note        : The driver currently supports just one ds18b20 sensor that is powered through external Vcc.
 */

// Standard includes.
#include <stdio.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "hw_types.h"
#include "hw_ints.h"
#include "interrupt.h"
#include "pin.h"
#include "rom.h"
#include "rom_map.h"
#include "gpio.h"
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

/* # Definitions - Debug */
#define DS18B20_DEBUG

/* # Definitions - DS18B20 */
#define DS18B20_DQ_PIN                                  10
#define DS18B20_DQ_HIGH                                 1
#define DS18B20_DQ_LOW                                  0
#define DS18B20_DQ_PRST_WAIT_TIME_US                    60
#define DS18B20_DQ_PRESENCE_TIME_US                     240
#define DS18B20_DQ_RESET_TIME_US                        500
#define DS18B20_WR_ONE_BYTE                             1
#define DS18B20_WR_MULTIPLE_BYTES                       0
#define DS18B20_ROM_ADDR_LEN                            8
#define DS18B20_SRAM_LEN                                9
#define DS18B20_SRAM_WRITE_LEN                          3

/* # Definitions - DS18B20 Commands */
#define DS18B20_READ_ROM_ADDR_CMD                       0x33
#define DS18B20_CONNECT_ADDR_CMD                        0x55
#define DS18B20_START_TCONV_CMD                         0x44
#define DS18B20_READ_SCRATCHPAD_CMD                     0xBE
#define DS18B20_WRITE_SCRATCHPAD_CMD                    0x4E
#define DS18B20_COPY_SCRATCHPAD_CMD                     0x48
#define DS18B20_READ_POWER_SUPPLY_CMD                   0xB4

/* # Definitions - Generic */
#define US_TO_TICKS(x)                                  ((130*x)/10)

/* Static Globals */
static unsigned char uc_dq_pin;
static unsigned int ui_dq_port;

/****************************************************************
 * Function Name : ds18b20_Init
 * Description   : Initialize the master one wire bus
 * Returns       : NONE.
 * Params        : NONE.
 ****************************************************************/
void ds18b20_Init()
{
    /* Configure PIN_01 as a standard open drain pin*/
    MAP_PinTypeGPIO(PIN_01, PIN_MODE_0, false);
    MAP_PinConfigSet(PIN_01, PIN_STRENGTH_6MA, PIN_TYPE_OD);

    /* Get the pin and port address of Pin_01 */
    GPIO_IF_GetPortNPin(DS18B20_DQ_PIN, &ui_dq_port, &uc_dq_pin);
}

/****************************************************************
 * Function Name : dq_switch_state
 * Description   : Switch the master bus to OUTPUT or INPUT
 * Returns       : NONE.
 * Params        : @dq_state - State of the bus
 ****************************************************************/
void dq_switch_state(DQ_PIN_STATE dq_state)
{
    switch(dq_state)
    {
    case DQ_SWITCH_TO_INPUT:
        MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_IN);
        MAP_PinConfigSet(PIN_01, PIN_STRENGTH_2MA|PIN_STRENGTH_6MA, PIN_TYPE_OD_PU);
        break;
    case DQ_SWITCH_TO_OUTPUT:
        MAP_GPIODirModeSet(GPIOA1_BASE, 0x4, GPIO_DIR_MODE_OUT);
        MAP_PinConfigSet(PIN_01, PIN_STRENGTH_2MA|PIN_STRENGTH_6MA, PIN_TYPE_STD);
        break;
    case DQ_INVALID_STATE:
    default:
        UART_PRINT("INVALID STATE\r\n");
        break;
    }
}

/****************************************************************
 * Function Name : ds18b20_send_reset_pulse
 * Description   : Master bus sends a reset pulse
 * Returns       : NONE.
 * Params        : @level_rst_us - pulse timing in us
 ****************************************************************/
void ds18b20_send_reset_pulse(int level_rst_us)
{
    dq_switch_state(DQ_SWITCH_TO_OUTPUT);
    GPIO_IF_Set(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin, DS18B20_DQ_LOW);
    MAP_UtilsDelay(US_TO_TICKS(level_rst_us));
    GPIO_IF_Set(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin, DS18B20_DQ_HIGH);
}

/****************************************************************
 * Function Name : ds18b20_detect_presence
 * Description   : Master detects the ds18b20's presence pulse
 * Returns       : NONE.
 * Params        : NONE.
 ****************************************************************/
int ds18b20_detect_presence()
{
    int ds18b20_prs_time = 0x00;
    int ret = 0;

    dq_switch_state(DQ_SWITCH_TO_INPUT);
    while(GPIO_IF_Get(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin) == DS18B20_DQ_HIGH)
    {
        ds18b20_prs_time++;
        MAP_UtilsDelay(US_TO_TICKS(1));

        if(ds18b20_prs_time > DS18B20_DQ_PRST_WAIT_TIME_US)
        {
            ret = -1;
            break;
        }
    }

    /* Reset the time tracker */
    ds18b20_prs_time = 0x00;

    if(ret == -1)
        return (ret);
    else
    {
        /* Reset the error tracker */
        ret = 0x00;

        while(GPIO_IF_Get(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin) == DS18B20_DQ_LOW)
        {
            ds18b20_prs_time++;
            MAP_UtilsDelay(US_TO_TICKS(1));

            if(ds18b20_prs_time > DS18B20_DQ_PRESENCE_TIME_US)
            {
                ret = -1;
                break;
            }
        }

        if((ds18b20_prs_time >= DS18B20_DQ_PRST_WAIT_TIME_US) &&
                (ds18b20_prs_time <= DS18B20_DQ_PRESENCE_TIME_US))
            ret = -1;
    }
    return (ret);
}

/****************************************************************
 * Function Name : ds18b20_initialization_proc
 * Description   : Initialization process to detect presence of
 *                 any slave devices connected to the bus.
 * Returns       : Returns 0 on Success and -1 on Failure.
 * Params        : NONE.
 ****************************************************************/
int ds18b20_initialization_proc()
{
    int ret = 0x00, retries = 125;

    /* Release bus and configure as input */
    dq_switch_state(DQ_SWITCH_TO_INPUT);

    /* Wait until the bus becomes high... incase if its still low */
    do
    {
        if(--retries == 0)
            return(-1);
        MAP_UtilsDelay(US_TO_TICKS(2));
    }while(GPIO_IF_Get(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin) == DS18B20_DQ_LOW);

    ds18b20_send_reset_pulse(DS18B20_DQ_RESET_TIME_US);
    ret = ds18b20_detect_presence();
    return(ret);
}

/****************************************************************
 * Function Name : ds182b0_write_bit
 * Description   : Writes a Logic bit i.e. '1' or '0'
 * Returns       : NONE.
 * Params        : @ds182b0_wbit - 0x00 or 0x01.
 ****************************************************************/
void ds182b0_write_bit(unsigned char ds182b0_wbit)
{
    if(ds182b0_wbit & 0x01)
    {
        /* Write a Logic bit 1 */
        GPIO_IF_Set(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin, DS18B20_DQ_LOW);
        MAP_UtilsDelay(US_TO_TICKS(10));
        GPIO_IF_Set(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin, DS18B20_DQ_HIGH);
        MAP_UtilsDelay(US_TO_TICKS(55));
    }
    else
    {
        /* Write a Logic bit 0 */
        GPIO_IF_Set(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin, DS18B20_DQ_LOW);
        MAP_UtilsDelay(US_TO_TICKS(65));
        GPIO_IF_Set(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin, DS18B20_DQ_HIGH);
        MAP_UtilsDelay(US_TO_TICKS(5));
    }
}

/****************************************************************
 * Function Name : ds182b0_write_byte
 * Description   : Writes a data byte
 * Returns       : NONE.
 * Params        : @ds182b0_wbyte - Byte to be written
 *                 @flag - Control to write single/multiple bytes
 ****************************************************************/
void ds182b0_write_byte(unsigned char ds182b0_wbyte, unsigned char flag)
{
    unsigned char bit_mask = 0x00;

    if(flag == DS18B20_WR_ONE_BYTE)
        dq_switch_state(DQ_SWITCH_TO_OUTPUT);

    for (bit_mask = 0x01; bit_mask; bit_mask <<= 1)
    {
        ds182b0_write_bit((bit_mask & ds182b0_wbyte)?1:0);
    }
}

/****************************************************************
 * Function Name : ds18b20_write_bytes
 * Description   : Writes a multiple data bytes
 * Returns       : NONE.
 * Params        : @buff_wptr - Pointer to data buffer
 *                 @num - No. of bytes to be written
 ****************************************************************/
void ds18b20_write_bytes(const unsigned char *buff_wptr, int num)
{
    int i = 0x00;

    dq_switch_state(DQ_SWITCH_TO_OUTPUT);

    for (i = 0 ; i < num ; i++)
        ds182b0_write_byte(buff_wptr[i], DS18B20_WR_MULTIPLE_BYTES);
}

/****************************************************************
 * Function Name : ds18b20_read_bit
 * Description   : Reads a Logic bit i.e. '1' or '0'
 * Returns       : NONE.
 * Params        : NONE.
 ****************************************************************/
unsigned char ds18b20_read_bit()
{
    unsigned char dq_sig = 0x00;

    dq_switch_state(DQ_SWITCH_TO_OUTPUT);
    GPIO_IF_Set(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin, DS18B20_DQ_LOW);
    MAP_UtilsDelay(US_TO_TICKS(3));

    dq_switch_state(DQ_SWITCH_TO_INPUT);
    MAP_UtilsDelay(US_TO_TICKS(10));
    dq_sig = GPIO_IF_Get(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin);
    MAP_UtilsDelay(US_TO_TICKS(53));

    return(dq_sig);
}

/****************************************************************
 * Function Name : ds18b20_read_byte
 * Description   : Reads a data byte
 * Returns       : The read data byte from the ds18b20
 * Params        : NONE.
 ****************************************************************/
unsigned char ds18b20_read_byte()
{
    unsigned char bit_mask = 0x00, r_byte = 0x00;

    for (bit_mask = 0x01; bit_mask; bit_mask <<= 1)
    {
        if (ds18b20_read_bit())
            r_byte |= bit_mask;
    }
    return (r_byte);
}

/****************************************************************
 * Function Name : ds18b20_read_bytes
 * Description   : Reads multiple data bytes from the ds18b20
 * Returns       : NONE.
 * Params        : @buff_rptr - Pointer to buffer to store bytes
 *                 @count - No. of bytes to be read
 ****************************************************************/
void ds18b20_read_bytes(unsigned char *buff_rptr, int count)
{
    int i = 0x00;

    for (i = 0 ; i < count ; i++)
        buff_rptr[i] = ds18b20_read_byte();
}

/****************************************************************
 * Function Name : ds18b20_single_read_ROM_addr
 * Description   : Reads the ROM address of the ds18b20.
 * Returns       : Returns -1 on failure and 0 on success.
 * Params        : @rd_rom_addr_cmd - command to read device addr
 *                 @buff_rom_addr_ptr - Buffer to store the dev
 *                 address
 * Note          : Only works when a single device is connected
 *                 to the bus.
 ****************************************************************/
int ds18b20_single_read_ROM_addr(unsigned char *buff_rom_addr_ptr)
{
    int ret = 0x00;
    unsigned char crc = 0x00;

    /* Initialization */
    if(ds18b20_initialization_proc() == -1)
        ret = -1;
    else
    {
        /* Send Read ROM address command (0x33) */
        ds182b0_write_byte(DS18B20_READ_ROM_ADDR_CMD, DS18B20_WR_ONE_BYTE);

        /* Read the DS18B20's 64-bit ROM address */
        ds18b20_read_bytes(buff_rom_addr_ptr, DS18B20_ROM_ADDR_LEN);

        /* Dump the ROM address */
        UART_PRINT("Family Code          = 0x%02x\r\n",buff_rom_addr_ptr[0]);
        UART_PRINT("Serial No.           = 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\r\n",buff_rom_addr_ptr[1],
                   buff_rom_addr_ptr[2], buff_rom_addr_ptr[3],
                   buff_rom_addr_ptr[4], buff_rom_addr_ptr[5],
                   buff_rom_addr_ptr[6]);

        /* Verify CRC */
        if((crc = crc8(buff_rom_addr_ptr, 0x07)) != buff_rom_addr_ptr[7])
            ret = -1;
#ifdef DS18B20_DEBUG
        UART_PRINT("ROM CRC[Received]    = 0x%02x\r\n",buff_rom_addr_ptr[7]);
        UART_PRINT("ROM CRC[Calculated]  = 0x%02x\r\n",crc);
#endif
    }

    return(ret);
}

/****************************************************************
 * Function Name : crc8
 * Description   : Performs data validity check
 * Returns       : CRC value.
 * Params        : @addr - Pointer to address buffer
 *                 @len - Length of the buffer
 ****************************************************************/
unsigned char crc8( unsigned char *addr, unsigned char len)
{
    unsigned char crc = 0, inbyte = 0, i = 0, mix = 0;

    while (len--)
    {
        inbyte = *addr++;
        for (i = 8; i; i--)
        {
            mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }
    }
    return (crc);
}

/****************************************************************
 * Function Name : ds18b20_read_temperature
 * Description   : Reads and loads the temperature data
 * Returns       : (-1) on Failure and (0) on Success
 * Params        : @ds182b0_addr - ds18b20's address
 *                 @ds18b20_temp_ptr - Pointer to store the
 *                 temperature data.
 ****************************************************************/
int ds18b20_read_temperature(const unsigned char *ds182b0_addr,
                             ds18b20_temp_Ptr ds18b20_temp_ptr)
{
    unsigned char data[10], resolution, crc;
    unsigned short raw_data = 0x00;

    /* Perform a reset */
    if(ds18b20_initialization_proc() == -1)
        return (-1);

    /* Send address command */
    ds182b0_write_byte(DS18B20_CONNECT_ADDR_CMD, DS18B20_WR_ONE_BYTE);

    /* Address the ds18b20 */
    ds18b20_write_bytes(ds182b0_addr, DS18B20_ROM_ADDR_LEN);

    /* Send command to start the temperature A-->D conversion */
    ds182b0_write_byte(DS18B20_START_TCONV_CMD, DS18B20_WR_ONE_BYTE);

    /* Hard wait until the conversion has completed, Wouldn't prefer
     * the 750ms delay method. This is only valid for EXT Vcc */
    while(!ds18b20_read_bit());

    /* Perform a reset */
    if(ds18b20_initialization_proc() == -1)
        return (-1);

    /* Send address command */
    ds182b0_write_byte(DS18B20_CONNECT_ADDR_CMD, DS18B20_WR_ONE_BYTE);

    /* Address the ds18b20 */
    ds18b20_write_bytes(ds182b0_addr, DS18B20_ROM_ADDR_LEN);

    /* Send command to read the SRAM */
    ds182b0_write_byte(DS18B20_READ_SCRATCHPAD_CMD, DS18B20_WR_ONE_BYTE);

    /* Read the data bytes from the ScratchPad */
    ds18b20_read_bytes(data, DS18B20_SRAM_LEN);

    /* Verify CRC */
    if((crc = crc8(data, 0x08)) != data[8])
        return (-1);

#ifdef DS18B20_DEBUG
    UART_PRINT("SRAM_T CRC[Received]     = 0x%02x\r\n",data[8]);
    UART_PRINT("SRAM_T CRC[Calculated]   = 0x%02x\r\n",crc);
    UART_PRINT("\r\n");
#endif

    /* Get the raw temperature data */
    raw_data = (data[1] << 8) | data[0];

    /* Check for resolution */
    resolution = data[4] & 0x60;

    /* Set accordingly for conversion,
     * by default, it is 12-bit*/
    if(resolution == 0x00)
        raw_data = raw_data << 3;
    if(resolution == 0x20)
        raw_data = raw_data << 2;
    if(resolution == 0x40)
        raw_data = raw_data << 1;

    /* Convert raw data to actual temperature */
    ds18b20_temp_ptr->celsius = (float)raw_data/16.0;
    ds18b20_temp_ptr->fahrenheit = (ds18b20_temp_ptr->celsius * 1.8) + 32.0;

    return (0);
}

/****************************************************************
 * Function Name : ds18b20_load_eeprom
 * Description   : Loads the TH, TL and config to EEPROM
 * Returns       : (-1) on Failure and (0) on Success
 * Params        : @ds182b0_addr - ds18b20's address
 *                 @parasite - Flag to indicate the power method
 ****************************************************************/
int ds18b20_load_eeprom(const unsigned char *ds182b0_addr,
                        unsigned char parasite)
{
    if(parasite == 0x01)
    {
        /* Code to support parasite needs to be
         * written.*/
        UART_PRINT("Parasite Mode Not Supported !!!");
        return (-1);
    }
    /* Perform a reset */
    if(ds18b20_initialization_proc() == -1)
        return (-1);

    /* Send address command */
    ds182b0_write_byte(DS18B20_CONNECT_ADDR_CMD, DS18B20_WR_ONE_BYTE);

    /* Address the ds18b20 */
    ds18b20_write_bytes(ds182b0_addr, DS18B20_ROM_ADDR_LEN);

    /* Send command to copy the EEPROM */
    ds182b0_write_byte(DS18B20_COPY_SCRATCHPAD_CMD, DS18B20_WR_ONE_BYTE);

    /* Give some time for EEPROM write - 20ms */
    MAP_UtilsDelay(US_TO_TICKS(20000));

    /* Perform a reset */
    if(ds18b20_initialization_proc() == -1)
        return (-1);

    return (0);
}

/****************************************************************
 * Function Name : ds18b20_write_scratchpad
 * Description   : Write the TH, TL and config to SRAM
 * Returns       : (-1) on Failure and (0) on Success
 * Params        : @temp_TH - Higher threshold for temperature
 *                 @temp_TL - Lower threshold for temperature
 *                 @config - Configuration register setting
 *                 @ds182b0_addr - ds18b20's address
 ****************************************************************/
int ds18b20_write_scratchpad(unsigned char temp_TH,
                             unsigned char temp_TL,
                             unsigned char config,
                             const unsigned char *ds182b0_addr)
{
    unsigned char scratch_pad_wr[3];
    scratch_pad_wr[0] = temp_TH;
    scratch_pad_wr[1] = temp_TL;
    scratch_pad_wr[2] = config;

    /* Perform a reset */
    if(ds18b20_initialization_proc() == -1)
        return (-1);

    /* Send address command */
    ds182b0_write_byte(DS18B20_CONNECT_ADDR_CMD, DS18B20_WR_ONE_BYTE);

    /* Address the ds18b20 */
    ds18b20_write_bytes(ds182b0_addr, DS18B20_ROM_ADDR_LEN);

    /* Send command to write the SRAM */
    ds182b0_write_byte(DS18B20_WRITE_SCRATCHPAD_CMD, DS18B20_WR_ONE_BYTE);

    /*Set TH, TL and Configuration in the ds18b20 */
    ds18b20_write_bytes(scratch_pad_wr, DS18B20_SRAM_WRITE_LEN);

    /* Load data to EEPROM */
    if(ds18b20_load_eeprom(ds182b0_addr, 0x00) == -1)
        return (-1);

    return (0);
}

/****************************************************************
 * Function Name : ds18b20_read_scratchpad
 * Description   : Reads the contents on the scratchpad
 * Returns       : (-1) on Failure and (0) on Success
 * Params        : @ds182b0_addr - ds18b20's address
 *                 @ds18b20_sram_ptr - Pointer to sram structure
 ****************************************************************/
int ds18b20_read_scratchpad(const unsigned char *ds182b0_addr,
                            ds18b20_sram_Ptr ds18b20_sram_ptr)
{
    unsigned char read_sram[10], crc;

    /* Perform a reset */
    if(ds18b20_initialization_proc() == -1)
        return (-1);

    /* Send address command */
    ds182b0_write_byte(DS18B20_CONNECT_ADDR_CMD, DS18B20_WR_ONE_BYTE);

    /* Address the ds18b20 */
    ds18b20_write_bytes(ds182b0_addr, DS18B20_ROM_ADDR_LEN);

    /* Send command to read the SRAM */
    ds182b0_write_byte(DS18B20_READ_SCRATCHPAD_CMD, DS18B20_WR_ONE_BYTE);

    /* Read the data bytes from the ScratchPad */
    ds18b20_read_bytes(read_sram, DS18B20_SRAM_LEN);

    /* Verify CRC */
    if((crc = crc8(read_sram, 0x08)) != read_sram[8])
        return (-1);

#ifdef DS18B20_DEBUG
    UART_PRINT("Temperature (LSB)        = 0x%02x\r\n",read_sram[0]);
    UART_PRINT("Temperature (MSB)        = 0x%02x\r\n",read_sram[1]);
    UART_PRINT("Temperature TH           = 0x%02x\r\n",read_sram[2]);
    UART_PRINT("Temperature TL           = 0x%02x\r\n",read_sram[3]);
    UART_PRINT("Configuration Register   = 0x%02x\r\n",read_sram[4]);
    UART_PRINT("SRAM_T CRC[Received]     = 0x%02x\r\n",read_sram[8]);
    UART_PRINT("SRAM_T CRC[Calculated]   = 0x%02x\r\n\r\n",crc);
#endif

    /* Store all the read sram data for the application */
    ds18b20_sram_ptr->TH_alarm = read_sram[2];
    ds18b20_sram_ptr->TL_alarm = read_sram[3];
    ds18b20_sram_ptr->temp_MSB = read_sram[1];
    ds18b20_sram_ptr->temp_LSB = read_sram[0];
    ds18b20_sram_ptr->config_reg = read_sram[4];

    /* Perform a reset */
    if(ds18b20_initialization_proc() == -1)
        return (-1);

    return (0);
}

/****************************************************************
 * Function Name : check_power_supply
 * Description   : Check power supply
 * Returns       : ERROR, PARASITIC OR EXTERNAL
 * Params        : @ds182b0_addr - ds18b20's address
 *
 * IMP Note      : Always run this API once in the beginning to
 *                 ensure proper supply is given to ds18b20.
 ****************************************************************/
DS18B20_POWER_STATE check_power_supply(const unsigned char *ds182b0_addr)
{
    DS18B20_POWER_STATE ds18b20_power;

    /* Perform a reset */
    if(ds18b20_initialization_proc() == -1)
        ds18b20_power = ERROR;
    else
    {

        /* Send address command */
        ds182b0_write_byte(DS18B20_CONNECT_ADDR_CMD, DS18B20_WR_ONE_BYTE);

        /* Address the ds18b20 */
        ds18b20_write_bytes(ds182b0_addr, DS18B20_ROM_ADDR_LEN);

        /* Send command to check the power supply */
        ds182b0_write_byte(DS18B20_READ_POWER_SUPPLY_CMD, DS18B20_WR_ONE_BYTE);

        /* Check the power status by reading a time slot from ds18b20 */
        if(GPIO_IF_Get(DS18B20_DQ_PIN, ui_dq_port, uc_dq_pin) == DS18B20_DQ_LOW)
            ds18b20_power = PARASITIC_POWERED;
        else
            ds18b20_power = EXTERNAL_POWERED;

        if(ds18b20_initialization_proc() == -1)
            ds18b20_power = ERROR;
    }

    return (ds18b20_power);
}
