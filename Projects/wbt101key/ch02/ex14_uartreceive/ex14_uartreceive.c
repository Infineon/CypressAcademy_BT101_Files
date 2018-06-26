/* Use a UART interrupt to toggle LED_1 */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"

/*****************************    Constants   *****************************/

/*****************************    Variables   *****************************/

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void rx_cback( void *data );

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

            /* Initialize the UART for input */
            wiced_hal_puart_init( );
            wiced_hal_puart_flow_off( );
            wiced_hal_puart_set_baudrate( 115200 );
            wiced_hal_puart_enable_tx( );
            wiced_hal_puart_print( "*** ex13_uartreceive ***\n\r" );

            /* Enable receive and the interrupt */
            wiced_hal_puart_register_interrupt( rx_cback );

            /* Set watermark level to 1 to receive interrupt up on receiving each byte */
            wiced_hal_puart_set_watermark_level( 1 );
            wiced_hal_puart_enable_rx();

            break;

        default:
            break;
    }
    return result;
}


/* Interrupt callback function for UART */
void rx_cback( void *data )
{
    uint8_t  readbyte;

    /* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt();

    /* Turn LED ON/OFF bases on character received */
    if( readbyte == '1' )
    {
        /* Drive LED HIGH */
        wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_HIGH);
        wiced_hal_puart_print( "LED_ON\n\r" );
    }

    if( readbyte == '0' )
    {
        /* Drive LED LOW */
         wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_LOW);
         wiced_hal_puart_print( "LED_OFF\n\r" );

    }
}
