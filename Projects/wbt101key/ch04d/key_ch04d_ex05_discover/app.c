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
void					writeLED( uint8_t val );
void					writeCCCD( uint8_t val );

void					startServiceDiscovery( void );
void					startCharacteristicDiscovery( void );
void					startDescriptorDiscovery( void );


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
uint16_t connection_id = 0;
uint16_t ledHandle = 0x0009u;		/* Check this matches your peripheral HDLC_MODUS_RGBLED_VALUE */
uint16_t cccdHandle = 0x000Cu;		/* Check this matches your peripheral HDLD_MODUS_COUNTER_CLIENT_CHAR_CONFIG */

#define __UUID_SERVICE_MODUS                        0x0Au, 0x71u, 0x06u, 0xCAu, 0x27u, 0x68u, 0x44u, 0x8Du, 0xECu, 0x47u, 0x76u, 0x07u, 0x6Eu, 0x82u, 0x91u, 0x79u
#define __UUID_CHARACTERISTIC_MODUS_RGBLED          0x74u, 0x66u, 0xB9u, 0x3Cu, 0x38u, 0x7Du, 0xADu, 0x99u, 0x74u, 0x46u, 0x9Du, 0x81u, 0x95u, 0x05u, 0xAAu, 0x02u
#define __UUID_CHARACTERISTIC_MODUS_COUNTER         0x68u, 0xB9u, 0xECu, 0x13u, 0xEBu, 0x9Fu, 0x7Bu, 0x96u, 0x89u, 0x45u, 0x69u, 0x0Au, 0x63u, 0xB2u, 0xA5u, 0xFCu
#define __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION    0x2902u

static const uint8_t serviceUUID[] = { __UUID_SERVICE_MODUS };
static uint16_t serviceStartHandle = 0x0001;
static uint16_t serviceEndHandle   = 0xFFFF;

typedef struct
{
    uint16_t startHandle;
    uint16_t endHandle;
    uint16_t valHandle;
    uint16_t cccdHandle;
} charHandle_t;

static const uint8_t ledUUID[] = { __UUID_CHARACTERISTIC_MODUS_RGBLED };
static charHandle_t  ledChar;

static const uint8_t counterUUID[] = { __UUID_CHARACTERISTIC_MODUS_COUNTER };
static charHandle_t  counterChar;

#define MAX_CHARS_DISCOVERED (10)
static charHandle_t charHandles[MAX_CHARS_DISCOVERED];
static uint32_t charHandleCount;


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

				wiced_bt_gatt_register( app_bt_gatt_callback );

				/* PUART receive not used in first two exercises */
				wiced_hal_puart_register_interrupt( uart_rx_callback );	// Enable receive interrupts on the PUART
				wiced_hal_puart_set_watermark_level( 1 );				// Interrupt up on each byte received
				wiced_hal_puart_enable_rx();
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
			break;

		case BTM_PAIRING_COMPLETE_EVT:
			break;

		case BTM_ENCRYPTION_STATUS_EVT:
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
			status = WICED_BT_ERROR;
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

		case BTM_BLE_CONNECTION_PARAM_UPDATE:
			break;

		default:
			WICED_BT_TRACE( "Unhandled Bluetooth Management Event: 0x%x (%d)\n", event, event );
			break;
	}

	return status;
}

wiced_bt_gatt_status_t app_bt_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_event_data )
{
	wiced_result_t status = WICED_BT_SUCCESS;
    wiced_bt_gatt_connection_status_t *p_conn = &p_event_data->connection_status;

	switch( event )
	{
		case GATT_CONNECTION_STATUS_EVT:
			if( p_conn->connected )
			{
				WICED_BT_TRACE( "Connected.\r\n" );
				connection_id = p_conn->conn_id;
				wiced_bt_dev_sec_bond(p_conn->bd_addr, p_conn->addr_type, BT_TRANSPORT_LE, 0, NULL );
			}
			else
			{
				WICED_BT_TRACE( "Disconnected.\r\n" );
				connection_id = 0;
			}
			break;

		case GATT_OPERATION_CPLT_EVT:
			// When you get something back from the peripheral... print it out.. and all of its data
			WICED_BT_TRACE("Gatt Event Complete Conn=%d Op=%d status=0x%X Handle=0x%X len=%d",
					p_event_data->operation_complete.conn_id,
					p_event_data->operation_complete.op,
					p_event_data->operation_complete.status,
					p_event_data->operation_complete.response_data.handle,
					p_event_data->operation_complete.response_data.att_value.len);

			if( p_event_data->operation_complete.response_data.att_value.len > 0 )
			{
				WICED_BT_TRACE_ARRAY( p_event_data->operation_complete.response_data.att_value.p_data, p_event_data->operation_complete.response_data.att_value.len, "Data: " );
			}
			else
			{
				WICED_BT_TRACE( "\r\n" );
			}
			break;

		case GATT_DISCOVERY_RESULT_EVT:
			//////////////// Services Discovery /////////////////
			if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_SERVICES_BY_UUID )
	        {
				serviceStartHandle = GATT_DISCOVERY_RESULT_SERVICE_START_HANDLE( p_event_data );
				serviceEndHandle = GATT_DISCOVERY_RESULT_SERVICE_END_HANDLE( p_event_data );
				WICED_BT_TRACE( "Discovered Service Start=%X End=%X\r\n", serviceStartHandle, serviceEndHandle );
	        }


			//////////////// Characteristics Discovery /////////////////
			if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTICS )
			{
				charHandles[charHandleCount].startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle;
				charHandles[charHandleCount].valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data );
				charHandles[charHandleCount].endHandle = serviceEndHandle;

				WICED_BT_TRACE( "Char Handle=0x%X Value Handle=0x%X Len=%d ",
							charHandles[charHandleCount].startHandle,
							charHandles[charHandleCount].valHandle,
							GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data ),
							GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID_LEN( p_event_data ) );

				if( charHandleCount != 0 )
				{
					charHandles[charHandleCount-1].endHandle = charHandles[charHandleCount].endHandle - 1;
				}
				charHandleCount += 1;

				if( charHandleCount > MAX_CHARS_DISCOVERED-1 )
				{
					WICED_BT_TRACE( "This is really bad.. we discovered more characteristics than we can save\r\n" );
				}

				if( memcmp( ledUUID, GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128( p_event_data ), 16 ) == 0 ) // If it is the led Characteristic
				{
					ledChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle; // No macro for this unfortunately
					ledChar.valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data );
				}

				if( memcmp( counterUUID, GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128( p_event_data ),16 ) == 0 ) // If it is the button Characteristic
				{
					counterChar.startHandle = p_event_data->discovery_result.discovery_data.characteristic_declaration.handle; // No macro for this unfortunately
					counterChar.valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE( p_event_data );
				}

				for (int i=0; i<GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID_LEN( p_event_data ); i++ ) // Dump the bytes to the screen
				{
					WICED_BT_TRACE( "%02X ", GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128( p_event_data )[i] );
				}
				WICED_BT_TRACE( "\r\n" );
	        }

			//////////////// Descriptors Discovery /////////////////
			if( p_event_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS )
			{
				WICED_BT_TRACE( "Char Descriptor Handle = %X Len=%d ", GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_VALUE_HANDLE( p_event_data ),
				GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID_LEN( p_event_data ) );

				for( int i=0; i<GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID_LEN( p_event_data ); i++ )
				{
					WICED_BT_TRACE( "%02X ", GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID128( p_event_data )[i] );
				}
				WICED_BT_TRACE( "\r\n" );

				if( GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID16( p_event_data ) == __UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION )
				{
					counterChar.cccdHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_VALUE_HANDLE( p_event_data );
				}
			}
			break;

		case GATT_DISCOVERY_CPLT_EVT:
			// Once all characteristics are discovered... you need to setup the end handles
			if( p_event_data->discovery_complete.disc_type == GATT_DISCOVER_CHARACTERISTICS )
			{
			  for( int i=0; i<charHandleCount; i++ )
			  {
			    if( charHandles[i].startHandle == ledChar.startHandle )
			      ledChar.endHandle = charHandles[i].endHandle;

			    if( charHandles[i].startHandle == counterChar.startHandle )
			       counterChar.endHandle = charHandles[i].endHandle;
			  }
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

		case 'd':
	        wiced_bt_gatt_disconnect( connection_id );
			break;

		case '0':			// LEDs off
		case '1':			// LEDs blue
		case '2':			// LEDs red
		case '3':			// LEDs blue+red
		case '4':			// LEDs green
		case '5':			// LEDs blue+green
		case '6':			// LEDs red+green
		case '7':			// LEDs white
			writeLED( readbyte );
			break;

		case 'n':
			writeCCCD( 0x01 );
			break;

		case 'N':
			writeCCCD( 0x00 );
			break;

		case 'q':
			startServiceDiscovery();
			break;

		case 'w':
			startCharacteristicDiscovery();
			break;

		case 'e':
			startDescriptorDiscovery();
			break;

		default:
			WICED_BT_TRACE( "Unrecognised command\r\n" );
			// No break - fall through and display help

		case '?':			// Help
			WICED_BT_TRACE( "Commands:\r\n" );
			WICED_BT_TRACE( "\t%c\tHelp (this message)\r\n", '?' );
			WICED_BT_TRACE( "\t%c\tStart scanning\r\n", 's' );
			WICED_BT_TRACE( "\t%c\tStop scanning\r\n", 'S' );
			WICED_BT_TRACE( "\t%c\tDisconnect\r\n", 'd' );
			WICED_BT_TRACE( "\t%s\tControl LED\r\n", "0..7" );
			WICED_BT_TRACE( "\t%c\tStart notifications\r\n", 'n' );
			WICED_BT_TRACE( "\t%c\tStop notifications\r\n", 'N' );
			WICED_BT_TRACE( "\t%c\tStart Service Discovery\r\n", 'q' );
			WICED_BT_TRACE( "\t%c\tSttart Characteristic Discovery\r\n", 'w' );
			WICED_BT_TRACE( "\t%c\tStart Descriptor Discovery\r\n", 'e' );
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
	if( connection_id )
	{
		wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_value_t ) + sizeof( val )-1 );

		if ( p_write )
		{
			p_write->handle   = ledHandle;
			p_write->offset   = 0;
			p_write->len      = sizeof( val );
			p_write->auth_req = GATT_AUTH_REQ_NONE;
			memcpy( p_write->value, &val, sizeof( val ) );

			wiced_bt_gatt_status_t status = wiced_bt_gatt_send_write( connection_id, GATT_WRITE, p_write );

			WICED_BT_TRACE( "wiced_bt_gatt_send_write 0x%X\r\n", status );

			wiced_bt_free_buffer( p_write );
		}
	}
}


/*******************************************************************************
* Function Name: void writeCCCD( uint8_t val )
********************************************************************************/
void writeCCCD( uint8_t val )
{
	if( connection_id && cccdHandle )
	{
		wiced_bt_util_set_gatt_client_config_descriptor( connection_id, cccdHandle, val );
	}
}


/*******************************************************************************
* Function Name: void startServiceDiscovery( void )
********************************************************************************/
void startServiceDiscovery( void )
{
    wiced_bt_gatt_discovery_param_t discovery_param;

    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = 1;
    discovery_param.e_handle = 0xFFFF;
    discovery_param.uuid.len = 16;
    memcpy( &discovery_param.uuid.uu.uuid128, serviceUUID, 16 );

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover ( connection_id, GATT_DISCOVER_SERVICES_BY_UUID, &discovery_param );
    WICED_BT_TRACE( "Started Service Discovery 0x%X\r\n", status );
}


/*******************************************************************************
* Function Name: void startCharacteristicDiscovery( void )
********************************************************************************/
void startCharacteristicDiscovery()
{
    charHandleCount = 0;

    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = serviceStartHandle + 1;
    discovery_param.e_handle = serviceEndHandle;

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover( connection_id, GATT_DISCOVER_CHARACTERISTICS, &discovery_param );
    WICED_BT_TRACE( "Start char Discovery 0x%X\r\n", status );
}


/*******************************************************************************
* Function Name: void startDescriptorDiscovery( void )
********************************************************************************/
void startDescriptorDiscovery()
{
	WICED_BT_TRACE( "Counter Start Handle = %X End Handle=%X\r\n", counterChar.valHandle+1, counterChar.endHandle );

	wiced_bt_gatt_discovery_param_t discovery_param;
	memset( &discovery_param, 0, sizeof( discovery_param ) );
	discovery_param.s_handle = counterChar.valHandle+1;
	discovery_param.e_handle = counterChar.endHandle;

	wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover( connection_id, GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS, &discovery_param );
	WICED_BT_TRACE( "Start char descriptor discovery 0x%X\r\n", status );
}
