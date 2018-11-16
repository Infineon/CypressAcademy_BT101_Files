/* Print a value from NVRAM every second */
/* When button 1 is pressed, the value is incremented and stored in NVRAM */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_nvram.h"

/*****************************    Constants   *****************************/
/* Thread will delay for 1000ms */
#define THREAD_DELAY_IN_MS          (1000)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/*****************************    Variables   *****************************/
wiced_thread_t * nvram_read_thread;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void nvram_read_func( uint32_t arg );
void button_cback( void *data, uint8_t port_pin );

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

            wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
            WICED_BT_TRACE( "*** ex06_nvram ***\n\r" );

            /* Configure the Button GPIO as an input with a resistive pull up and interrupt on falling edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_cback, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );

            /* Start a thread to read NVRAM every second */
            nvram_read_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    nvram_read_thread,              // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "NVRAM",                        // Name
                    nvram_read_func,                // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument

            break;

        default:
            break;
    }
    return result;
}


/* Thread function to read the NVRAM */
void nvram_read_func( uint32_t arg )
{
    uint8_t         readCount = 0;
    wiced_result_t  status;
    uint16_t        readBytes;

    for(;;)
    {
        readBytes =  wiced_hal_read_nvram( WICED_NVRAM_VSID_START, sizeof(readCount), &readCount, &status);
        WICED_BT_TRACE( "Value read from NVRAM:  %d, Bytes Read:    %d, Status: 0x%02x\n\r", readCount, readBytes, status );
        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}


/* Interrupt callback function for BUTTON_1 */
void button_cback( void *data, uint8_t port_pin )
{
    static uint8_t count = 0;
    wiced_result_t status;
    uint16_t       writeBytes;

    count ++;
    writeBytes = wiced_hal_write_nvram( WICED_NVRAM_VSID_START, sizeof(count), &count, &status);
    WICED_BT_TRACE( "Value written to NVRAM: %d, Bytes Written: %d, Status: 0x%02x\n\r", count, writeBytes, status );
}

