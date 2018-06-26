/*
 * $ Copyright Broadcom Corporation $
 */

/** @file U8G Library HW Function
 *
 */

#include "u8g_arm.h"
#include "wiced.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_i2c.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
/* #define U8G_I2C_USE_43909_TIMING */
#define RETRIES 5

#ifdef U8G_I2C_USE_43909_TIMING
#define USEC_DELAY_COUNT_1 49
#define USEC_DELAY_COUNT_10 780
#define MSEC_DELAY_COUNT_1 65538
#define MSEC_DELAY_COUNT_FACTOR 53000
#endif

/* This is the number of data bytes that will be sent in one I2C transaction along with 1
 * 1 control byte so the actual number of bytes sent will be this value plus 1. For some hardware
 * there is a limit to this value. For example, for the 20719, this should be set to 64 or less.
 */
#define MAX_WRITE_CHUNK (64)


/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/
uint8_t I2C_address;

/******************************************************
 *               Function Definitions
 ******************************************************/

void u8g_init_wiced_i2c_device(uint8_t address)
{
    I2C_address = address;
}

/* These delay functions were measured with a scope on BCM943909WCD1_3.
 * They may be used for 43909 specific debugging or reference. */
#ifdef U8G_I2C_USE_43909_TIMING
void u8g_Delay(uint16_t number_of_milliseconds)
{
    int counter = 0;

    for (counter = 0; counter < MSEC_DELAY_COUNT_1; ++counter)
    {
        asm("nop");
    }
    if (number_of_milliseconds-- > 1)
    {
        for (counter = 0; counter < MSEC_DELAY_COUNT_FACTOR * number_of_milliseconds; ++counter)
        {
            asm("nop");
        }
    }
}

void u8g_MicroDelay(void)
{
    int counter = 0;

    for (counter = 0; counter < USEC_DELAY_COUNT_1; ++counter)
    {
        asm("nop");
    }
}

void u8g_10MicroDelay(void)
{
    int counter = 0;

    for (counter = 0; counter < USEC_DELAY_COUNT_10; ++counter)
    {
        asm("nop");
    }
}
#else
void u8g_Delay(uint16_t milliseconds)
{
    wiced_rtos_delay_milliseconds(milliseconds,ALLOW_THREAD_TO_SLEEP);
}

void u8g_MicroDelay(void)
{
    wiced_rtos_delay_microseconds(1);
}

void u8g_10MicroDelay(void)
{
    wiced_rtos_delay_microseconds(10);
}
#endif


uint8_t u8g_com_hw_i2c_fn(u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr)
{
    wiced_result_t result;

    /* The control value is either a command byte (0x80) or a series of data bytes (0x40). It
     * must be the first byte sent in every I2C message. */
    static uint8_t control = 0;

    uint8_t  message[MAX_WRITE_CHUNK+1]; /* The message needs to hold 1 control byte plus up to MAX_WRITE_CHUNK bytes */
    uint32_t fullWrites;        /* Number of full I2C write transactions required */
    uint32_t partialWriteSize;  /* Number of bytes needed in final I2C write */
    uint32_t pass;              /* Loop counter to keep track of the pass we are on */
    uint32_t offset = 0;        /* Keep track of where to pull data from the arg_ptr pointer */

    switch(msg)
    {
        /* Initialize I2C controller */
        case U8G_COM_MSG_INIT:
            wiced_hal_i2c_init();
            wiced_hal_i2c_set_speed( I2CM_SPEED_400KHZ );
            break;

        /* Switch between data and command mode */
        case U8G_COM_MSG_ADDRESS:
            /* Command Mode - 0x80 indicates a single command byte will be sent */
            if (arg_val == 0)
            {
                control = 0x80;
            }
            /* Data Mode - 0x40 indicates a series of data bytes will be sent */
            else
            {
                control = 0x40;
            }
            break;

        /* Write single command byte to device */
        case U8G_COM_MSG_WRITE_BYTE:

            /* 1st byte is the control byte, 2nd byte is the command which is passed as arg_val */
            message[0] = control;
            message[1] = arg_val;
            result = wiced_hal_i2c_write(message, 2, I2C_address);
            if (result != WICED_SUCCESS)
            {
                WICED_BT_TRACE( "Single-Byte write failed\n" );
            }
            break;

        /* Write a sequence of data bytes to device */
        case U8G_COM_MSG_WRITE_SEQ:
        case U8G_COM_MSG_WRITE_SEQ_P:

            /* 1st byte is always the control byte */
            message[0] = control;

            /* Need to break data into multiple I2C messages each with length of MAX_WRITE_CHUNK or less*/
            /* The data is pointed to by arg_ptr and the total number of bytes is passed as arg_val */
            fullWrites = arg_val/MAX_WRITE_CHUNK;
            partialWriteSize = arg_val % MAX_WRITE_CHUNK;

            /* Do full I2C write cycles */
            for(pass=0; pass<fullWrites; pass++)
            {
                memcpy(&message[1], (uint8_t *)(arg_ptr)+offset, MAX_WRITE_CHUNK);   /* Copy data to the I2C message starting after the control byte */
                result = wiced_hal_i2c_write(message, MAX_WRITE_CHUNK+1, I2C_address); /* Do the I2C write */
                if (result != WICED_SUCCESS)
                {
                    WICED_BT_TRACE( "Multiple-Byte write failed\n" );
                }
                offset += MAX_WRITE_CHUNK; /* Increment offset for next pass */
            }

            /* Send final I2C message with the left over bytes if necessary */
            if(partialWriteSize != 0)
            {
                memcpy(&message[1], (uint8_t *)(arg_ptr)+offset, partialWriteSize);   /* Copy data to the I2C message starting after the control byte */
                result = wiced_hal_i2c_write(message, partialWriteSize+1, I2C_address); /* Do the I2C write */
                if (result != WICED_SUCCESS)
                {
                    WICED_BT_TRACE( "Multiple-Byte write failed\n" );
                }
            }
            break;
    }
    return 1;
}
