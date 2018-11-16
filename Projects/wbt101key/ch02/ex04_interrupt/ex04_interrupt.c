/* Use an interrupt on BUTTON_1 to toggle LED_2 whenever the button is pressed */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"

/*****************************    Constants   *****************************/

/*****************************    Variables   *****************************/

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
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
            WICED_BT_TRACE( "*** interrupt exercise ***\n\r" );

            /* Configure the Button GPIO as an input with a resistive pull up and interrupt on falling edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_cback, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );

            break;

        default:
            break;
    }
    return result;
}


/* Interrupt callback function for BUTTON_1 */
void button_cback( void *data, uint8_t port_pin )
{
    /* Clear the GPIO interrupt (this is not strictly needed since it is done automatically for buttons) */
    wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 );

    /* Toggle the LED state */
    wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, ! wiced_hal_gpio_get_pin_output( WICED_GPIO_PIN_LED_2 ) );
    WICED_BT_TRACE( "Toggle\n\r" );
}
