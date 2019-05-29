#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "GeneratedSource/cycfg.h"


/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t  app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void rx_cback( void *data );

/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Entry point to the application. Initialize transport configuration
*          and register BLE management event callback. The actual application
*          initialization will happen when stack reports that BT device is ready
********************************************************************************/
void application_start(void)
{
    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init( app_bt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}


/**************************************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(wiced_bt_management_evt_t event,
*                                                  wiced_bt_management_evt_data_t *p_event_data)
***********************************************************************************************    ***
* Summary:
*   This is a Bluetooth stack management event handler function to receive events from
*   BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    switch( event )
    {
    case BTM_ENABLED_EVT:						// Bluetooth Controller and Host Stack Enabled

        if( WICED_BT_SUCCESS == p_event_data->enabled.status )
        {
            /* Initialize the UART */
            wiced_hal_puart_init();
            wiced_hal_puart_flow_off();
            wiced_hal_puart_set_baudrate( 115200 );
            wiced_hal_puart_enable_tx();

            /* Enable receive and the interrupt */
            wiced_hal_puart_register_interrupt( rx_cback );

            /* Set watermark level to 1 to receive interrupt up on receiving each byte */
            wiced_hal_puart_set_watermark_level( 1 );
            wiced_hal_puart_enable_rx();

            wiced_hal_puart_print( "**** CYW20819 App Start **** \n\r" );
        }
        break;

    default:
        break;
    }

    return status;
}


/*******************************************************************************
* Function Name: void rx_cback( void *data )
********************************************************************************/
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
        wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_LOW );
        wiced_hal_puart_print( "LED_ON\n\r" );
    }

    if( readbyte == '0' )
    {
        /* Drive LED LOW */
         wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_HIGH );
         wiced_hal_puart_print( "LED_OFF\n\r" );

    }
}
