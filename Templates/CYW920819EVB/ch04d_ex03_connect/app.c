#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "string.h"

#include "GeneratedSource/cycfg.h"

#define MAX_ADV_NAME_LEN				(30)


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_bt_gatt_status_t	app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );
void					uart_rx_callback( void *data );

void					myScanCallback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/


/*******************************************************************************
* Function Name: void application_start( void )
********************************************************************************/
void application_start( void )
{
	#if ((defined WICED_BT_TRACE_ENABLE) || (defined HCI_TRACE_OVER_TRANSPORT))
		/* Select Debug UART setting to see debug traces on the appropriate port */
		wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
	#endif

	WICED_BT_TRACE( "**** CYW20819 App Start **** \r\n" );

	/* Initialize Stack and Register Management Callback */
	wiced_bt_stack_init( app_bt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}


/*******************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(
* 					wiced_bt_management_evt_t event,
* 					wiced_bt_management_evt_data_t *p_event_data )
********************************************************************************/
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
	wiced_result_t status = WICED_BT_SUCCESS;

	switch( event )
	{
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				WICED_BT_TRACE( "Bluetooth Enabled\r\n" );

				/* Use Application Settings dialog to set BT_DEVICE_ADDRESS = random */
				wiced_bt_device_address_t bda;
				wiced_bt_dev_read_local_addr( bda );
				WICED_BT_TRACE( "Local Bluetooth Device Address: [%B]\r\n", bda );

				wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, myScanCallback );

				/* PUART receive not used in first two exercises */
//				wiced_hal_puart_register_interrupt( uart_rx_callback );	// Enable receive interrupts on the PUART
//				wiced_hal_puart_set_watermark_level( 1 );				// Interrupt up on each byte received
//				wiced_hal_puart_enable_rx();
			}
			break;
			
		case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT:
			switch( p_event_data->ble_scan_state_changed )
			{
				case BTM_BLE_SCAN_TYPE_NONE:
					WICED_BT_TRACE( "Scanning stopped.\r\n" );
					break;

				case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
					WICED_BT_TRACE( "High duty scanning.\r\n" );
					break;

				case BTM_BLE_SCAN_TYPE_LOW_DUTY:
					WICED_BT_TRACE( "Low duty scanning.\r\n" );
					break;
			}
			break;

		default:
			WICED_BT_TRACE( "Unhandled Bluetooth Management Event: 0x%x (%d)\n", event, event );
			break;
	}

	return status;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
********************************************************************************/
wiced_bt_gatt_status_t app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	wiced_result_t status = WICED_BT_SUCCESS;

	switch( event )
	{
		case GATT_CONNECTION_STATUS_EVT:
			break;

		case GATT_OPERATION_CPLT_EVT:
			break;

		default:
			WICED_BT_TRACE( "Unhandled GATT Event: 0x%x (%d)\n", event, event );
			break;
	}

	return status;
}


/*******************************************************************************
* Function Name: void uart_rx_callback( void *data )
********************************************************************************/
void uart_rx_callback( void *data )
{
	uint8_t readbyte;

	/* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
	wiced_hal_puart_read( &readbyte );
	wiced_hal_puart_reset_puart_interrupt();

	switch( readbyte )
	{
		case 's':			// Turn on scanning
			break;

		case 'S':			// Turn off scanning
			break;

		case '0':			// LEDs off
		case '1':			// LEDs blue
		case '2':			// LEDs red
		case '3':			// LEDs blue+red
		case '4':			// LEDs green
		case '5':			// LEDs blue+green
		case '6':			// LEDs red+green
		case '7':			// LEDs white
			break;

		default:
			WICED_BT_TRACE( "Unrecognised command\r\n" );
			// No break - fall through and display help

		case '?':			// Help
			WICED_BT_TRACE( "Commands:\r\n" );
			WICED_BT_TRACE( "\t%c\tHelp (this message)\r\n", '?' );
			WICED_BT_TRACE( "\t%c\tStart scanning\r\n", 's' );
			WICED_BT_TRACE( "\t%c\tStop scanning\r\n", 'S' );
			WICED_BT_TRACE( "\t%s\tControl LED\r\n", "0..7" );
			WICED_BT_TRACE( "\r\n" );
			break;
	}
}


/*******************************************************************************
* Function Name: void myScanCallback(
* 					wiced_bt_ble_scan_results_t *p_scan_result,
* 					uint8_t *p_adv_data )
********************************************************************************/
void myScanCallback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data )
{
	uint8_t len;
	uint8_t *p_name = NULL;
	uint8_t *p_service = NULL;

	uint8_t dev_name[MAX_ADV_NAME_LEN];

	p_name = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &len );

	if( p_name && ( len == strlen("key_peri") ) && (memcmp( "key_peri", p_name, len ) == 0) )
	{
		memcpy( dev_name, p_name, len);
		dev_name[len] = 0x00;	/* Null terminate the string */

		WICED_BT_TRACE("Found Device %s with BD Address: [%B]", dev_name, p_scan_result->remote_bd_addr );


		p_service = wiced_bt_ble_check_advertising_data( p_adv_data, BTM_BLE_ADVERT_TYPE_128SRV_COMPLETE, &len );
		if( p_service && len > 0 )
		{
			WICED_BT_TRACE_ARRAY( (uint8_t*)p_service, len, "Service: ");
		}

	}
}
