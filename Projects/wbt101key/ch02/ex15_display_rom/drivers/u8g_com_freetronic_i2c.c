#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"

#include "drivers/u8g_com_freetronic_i2c.h"

static uint8_t I2C_address;

void u8g_init_wiced_i2c_device( uint8_t address )
{
    I2C_address = address;
}

uint8_t u8g_com_hw_i2c_fn( u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr )
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


void u8g_Delay( uint16_t milliseconds )
{
    wiced_rtos_delay_milliseconds( milliseconds, KEEP_THREAD_ACTIVE );
}

void u8g_MicroDelay( void )
{
    wiced_rtos_delay_microseconds( 1 );
}

void u8g_10MicroDelay( void )
{
    wiced_rtos_delay_microseconds( 10 );
}

