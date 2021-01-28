#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "string.h"
#include "wiced_memory.h"

#include "cycfg.h"

#define MAX_ADV_NAME_LEN				(28) 	/* Maximum possible name length since flags take 3 bytes and max packet is 31. */

#define SEARCH_DEVICE_NAME "key_peri"			/* Name of device to search for */

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
wiced_bt_gatt_status_t	app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data );
void					uart_rx_callback( void *data );

void myScanCallback( wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data );

void writeLED( uint8_t val );

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
uint16_t connection_id;
uint16_t ledHandle = 0x0009u;

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

				wiced_bt_gatt_register(app_bt_gatt_callback);

				/* PUART receive not used in first two exercises */
				wiced_hal_puart_register_interrupt( uart_rx_callback );	// Enable receive interrupts on the PUART
				wiced_hal_puart_set_watermark_level( 1 );				// Interrupt up on each byte received
				wiced_hal_puart_enable_rx();
			}
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
			status = WICED_ERROR;
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
* Function Name: wiced_bt_gatt_status_t app_bt_gatt_callback(
*					wiced_bt_gatt_evt_t event,
*					wiced_bt_gatt_event_data_t *p_event_data )
********************************************************************************/
wiced_bt_gatt_status_t app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_gatt_connection_status_t *p_conn = &p_event_data->connection_status;

	switch( event )
	{
		case GATT_CONNECTION_STATUS_EVT:
			if(p_conn->connected) /* Connection */
			{
				connection_id = p_conn->conn_id;
				WICED_BT_TRACE( "Connected to: %d\n", connection_id);
			}
			else /* Disconnection */
			{
				WICED_BT_TRACE( "Disconnected from: %d\n", connection_id);
				connection_id = 0;
			}
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
			wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_HIGH_DUTY, WICED_TRUE, myScanCallback );
			break;

		case 'S':			// Turn off scanning
			wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, myScanCallback );
			break;

		case 'd':			// Disconnect
			wiced_bt_gatt_disconnect(connection_id);
			break;

		case '0':			// LEDs off
		case '1':			// LEDs blue
		case '2':			// LEDs red
		case '3':			// LEDs blue+red
		case '4':			// LEDs green
		case '5':			// LEDs blue+green
		case '6':			// LEDs red+green
		case '7':			// LEDs white
			writeLED(readbyte-'0');
			break;

		default:
			WICED_BT_TRACE( "Unrecognized command\r\n" );
			// No break - fall through and display help

		case '?':			// Help
			WICED_BT_TRACE( "Commands:\r\n" );
			WICED_BT_TRACE( "\t%c\tHelp (this message)\r\n", '?' );
			WICED_BT_TRACE( "\t%c\tStart scanning\r\n", 's' );
			WICED_BT_TRACE( "\t%c\tStop scanning\r\n", 'S' );
			WICED_BT_TRACE( "\t%c\tDisconnect\r\n", 'd' );
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

	if( p_name && ( len == strlen(SEARCH_DEVICE_NAME) ) && (memcmp( SEARCH_DEVICE_NAME, p_name, len ) == 0) )
	{
		memcpy( dev_name, p_name, len);
		dev_name[len] = 0x00;	/* Null terminate the string */

		WICED_BT_TRACE("Found Device \"%s\" with BD Address: [%B]\r\n", dev_name, p_scan_result->remote_bd_addr );

		/* Connect and turn off scanning */
		wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE);
		wiced_bt_ble_scan( BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, myScanCallback );
	}
}


/*******************************************************************************
* Function Name: void writeLED( uint8_t val )
********************************************************************************/
void writeLED( uint8_t val )
{
	if((connection_id == 0) || ledHandle == 0 )
	{
		return;
	}

	wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_value_t ) + sizeof(val)-1);
	if ( p_write )
	{
		p_write->handle   = ledHandle;
		p_write->offset   = 0;
		p_write->len      = sizeof(val);
		p_write->auth_req = GATT_AUTH_REQ_NONE;
		memcpy(p_write->value, &val, sizeof(val));

		wiced_bt_gatt_send_write ( connection_id, GATT_WRITE, p_write );

		wiced_bt_free_buffer( p_write );
	 }
}
