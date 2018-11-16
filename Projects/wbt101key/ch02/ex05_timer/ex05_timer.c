/* Blink LED_2 with a frequency of 2 Hz */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_timer.h"

/*****************************    Constants   *****************************/
/* Thread will delay for 250ms so that LED frequency will be 500ms = 2 Hz */
#define TIMER_DELAY_IN_MS          (250)

/*****************************    Variables   *****************************/
wiced_timer_t led_timer;

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
            wiced_init_timer(&led_timer, led_control, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER);
            wiced_start_timer(&led_timer, TIMER_DELAY_IN_MS);
            break;
        default:
            break;
    }
    return result;
}


/* Thread function to control the LED */
void led_control( uint32_t arg )
{
    uint32_t led;

    led = wiced_hal_gpio_get_pin_output( WICED_GPIO_PIN_LED_2 );
    wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, ! led );
}
