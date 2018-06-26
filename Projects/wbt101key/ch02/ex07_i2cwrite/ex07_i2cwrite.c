/* Blink LED_1 with a frequency of 2 Hz */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_hal_i2c.h"

/*****************************    Constants   *****************************/
/* Thread will delay for 250ms so that LED frequency will be 500ms = 2 Hz */
#define THREAD_DELAY_IN_MS          (250)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/*****************************    Variables   *****************************/
wiced_thread_t * led_thread;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void led_control( uint32_t arg );

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_bt_stack_init( bt_cback, NULL, NULL );                    // Register BT stack callback
}


/* Callback function for Bluetooth events */
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
        /* BlueTooth stack enabled */
        case BTM_ENABLED_EVT:
            /* Start a thread to control LED blinking */
            led_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    led_thread,                     // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Toggle",                       // Name
                    led_control,                    // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;

        default:
            break;
    }
    return result;
}


/* Thread function to control the LED */
void led_control( uint32_t arg )
{
    /* I2C address and register locations inside the PSoC */
    #define I2C_ADDRESS                 (0x42)
    #define LED_VALUE_REG               (0x04)
    #define LED_CONTROL_REG             (0x05)

    uint8_t i2cWriteBuf[2];                 // I2C write data

    /* Configure I2C block */
    wiced_hal_i2c_init();
    wiced_hal_i2c_set_speed( I2CM_SPEED_400KHZ );

    /* Write the offset and the bit for the shield LEDs to be controlled over I2C */
    i2cWriteBuf[0] = LED_CONTROL_REG;
    i2cWriteBuf[1] = 0x01;
    wiced_hal_i2c_write( i2cWriteBuf , sizeof( i2cWriteBuf ), I2C_ADDRESS );

    /* Set the offset for the LED value register and select the first LED */
    i2cWriteBuf[0] = LED_VALUE_REG;
    i2cWriteBuf[1] = 1;

    for(;;)
    {
        /* Update I2C register */
        wiced_hal_i2c_write( i2cWriteBuf , sizeof( i2cWriteBuf ), I2C_ADDRESS );

        /* Increment to the next LED */
        i2cWriteBuf[1] <<= 1;               // Shift active LED bit left one
        if( i2cWriteBuf[1] > 0x08 )
        {
            i2cWriteBuf[1] = 0x01;          // Reset to the first bit
        }

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
