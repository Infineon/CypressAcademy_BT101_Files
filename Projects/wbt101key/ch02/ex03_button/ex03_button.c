/* Monitor BUTTON_1 every 100ms and set LED_1 based on the button's state */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"

/*****************************    Constants   *****************************/
/* Thread will delay for 100ms between button state checks */
#define THREAD_DELAY_IN_MS         (100)

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

            wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
            WICED_BT_TRACE( "*** ex04_button ***\n\r" );

            /* Start a thread to control LED blinking */
            led_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    led_thread,                     // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Blinky",                       // Name
                    led_control,                    // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;

        default:
            break;
    }
    return result;
}


/* Thread function to monitor BUTTON_1 and update LED_1 */
void led_control( uint32_t arg )
{
    while(1)
    {
        if( 0 == wiced_hal_gpio_get_pin_input_status( WICED_GPIO_PIN_BUTTON_1 ) )
        {
            /* Drive LED HIGH */
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_HIGH );
            WICED_BT_TRACE( "LED_HIGH\n\r" );
        }
        else
        {
            /* Drive LED LOW */
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_LOW );
            WICED_BT_TRACE( "LED_LOW\n\r" );
        }

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
