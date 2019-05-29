#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_aclk.h"
#include "wiced_hal_nvram.h"

#include "GeneratedSource/cycfg.h"
#include "GeneratedSource/cycfg_gatt_db.h"


/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
 
/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/* PWM configuration defines */
#define PWM_FREQUENCY			(1000)

/* PWM range values */
#define PWM_MAX					(0xFFFF)
#define PWM_BONDING_INIT		(PWM_MAX-999)
#define PWM_BONDED_INIT			(PWM_MAX-99)

/* PWM compare values for blinking, always on, always off */
#define PWM_BONDING_TOGGLE		(PWM_MAX-500)
#define PWM_BONDED_TOGGLE		(PWM_MAX-50)
#define PWM_ALWAYS_ON_TOGGLE	(PWM_BONDING_INIT)
#define PWM_ALWAYS_OFF_TOGGLE	(PWM_MAX)

/* NVRAM VSID allocation */
#define VSID_HOSTINFO			(WICED_NVRAM_VSID_START)
#define VSID_REMOTE_KEY			(WICED_NVRAM_VSID_START+1)
#define VSID_LOCAL_KEY			(WICED_NVRAM_VSID_START+2)


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t	app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );

wiced_bt_gatt_status_t			app_gatt_get_value( wiced_bt_gatt_attribute_request_t *p_attr );
wiced_bt_gatt_status_t			app_gatt_set_value( wiced_bt_gatt_attribute_request_t *p_attr );

void							app_set_advertisement_data( void );

void							button_cback( void *data, uint8_t port_pin );
void							rx_cback( void *data );



/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
uint16_t connection_id = 0;

uint16_t bonded = WICED_FALSE;		// State of the peripheral - bonded or bonding

struct hostinfo						// Remember the central and the notification status
{
	BD_ADDR	remote_addr;
	uint8_t	cccd[2];
} hostinfo;


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

    wiced_bt_device_link_keys_t link_keys;
    wiced_result_t key_read_status;
    uint16_t bytes;
    uint16_t counter;

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

			    /* Load the address resolution DB with the keys stored in the NVRAM */
			    /* If no client has bonded previously, then this read will fail */
			    memset( &link_keys, 0, sizeof(wiced_bt_device_link_keys_t) );
			    wiced_hal_read_nvram( VSID_REMOTE_KEY, sizeof(wiced_bt_device_link_keys_t), (uint8_t*)&link_keys, &key_read_status);
			    if( key_read_status == WICED_BT_SUCCESS )
			    {
			    	key_read_status = wiced_bt_dev_add_device_to_address_resolution_db ( &link_keys );
			        WICED_BT_TRACE("\tRead paired keys from NVSRAM and add to address resolution %B result:%d \r\n", &link_keys, key_read_status );
			        bonded = WICED_TRUE; /* We have bonding information already, so don't go into bonding mode */
			    }

				/* Configure the GATT database and advertise for connections */
				wiced_bt_gatt_register( app_gatt_callback );
				wiced_bt_gatt_db_init( gatt_database, gatt_database_len );

				/* Enable/disable pairing */
				wiced_bt_set_pairable_mode( WICED_TRUE, WICED_FALSE );

				/* Start the PWM in the LED always off state */
				wiced_hal_aclk_enable( PWM_FREQUENCY, ACLK1, ACLK_FREQ_24_MHZ );
				wiced_hal_pwm_start( PWM0, PMU_CLK, PWM_ALWAYS_OFF_TOGGLE, PWM_BONDING_INIT, 0 );

				/* Enable receive interrupts on the PUART */
			    wiced_hal_puart_register_interrupt( rx_cback );
			    /* Set watermark level to 1 to receive interrupt up on receiving each byte */
			    wiced_hal_puart_set_watermark_level( 1 );
			    wiced_hal_puart_enable_rx();

				/* Configure the button to trigger an interrupt when pressed */
				wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );
				wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_cback, 0 );

				/* Create the packet and begin advertising */
				app_set_advertisement_data();
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
			}
			else
			{
				WICED_BT_TRACE( "Bluetooth stack failure\r\n" );
			}
			break;

		case BTM_PASSKEY_NOTIFICATION_EVT:
			WICED_BT_TRACE("\r\n********************\r\n" );
			WICED_BT_TRACE("* PASSKEY = %06d *", p_event_data->user_passkey_notification.passkey );
			WICED_BT_TRACE("\r\n********************\r\n\n" );
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: 		// IO capabilities request
			p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_DISPLAY_ONLY;
			break;

		case BTM_PAIRING_COMPLETE_EVT: 						// Pairing Complete event
		    WICED_BT_TRACE( "Pairing Complete %d.\n\r", p_event_data->pairing_complete.pairing_complete_info.ble.reason );

	        if ( p_event_data->pairing_complete.pairing_complete_info.ble.reason == WICED_BT_SUCCESS ) /* Bonding successful */
	        {
	        	/* Write to NVRAM */
	        	wiced_hal_write_nvram( VSID_HOSTINFO, sizeof(hostinfo), (uint8_t*)&hostinfo, &status );
	        	WICED_BT_TRACE("\tBonding info save to NVRAM: %B\n\r", &hostinfo);
	        	bonded = WICED_TRUE; // remember that the device is now bonded
	        }
			break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
	        WICED_BT_TRACE( "Encryption Status event: bd ( %B ) res %d\n\r", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result );

	        /* Connection has been encrypted and we are already bonded meaning that we have correct/paired device restore values in the database */
	    	if( WICED_TRUE == bonded )
	    	{
	    		wiced_hal_read_nvram( VSID_HOSTINFO, sizeof(hostinfo), (uint8_t*)&hostinfo, &(p_event_data->encryption_status.result) );
	    		/* Set CCCD value from the value that was previously saved in the NVRAM */
	    		app_modus_counter_client_char_config[0] = hostinfo.cccd[0];
	    		app_modus_counter_client_char_config[1] = hostinfo.cccd[1];
	    		WICED_BT_TRACE("\tRestored existing bonded device info from NVRAM %B result: %d \n\r", hostinfo.remote_addr);
	    	}
			break;

		case BTM_SECURITY_REQUEST_EVT: 						// Security access
	    	if( WICED_FALSE == bonded )
	    	{
	    		wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
	    		status = WICED_BT_SUCCESS;
	    	}
	    	else
	    	{
	    		WICED_BT_TRACE("Security Request Denied - not in bonding mode\n");
	    		status = WICED_BT_FAILED_ON_SECURITY;
	    	}
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys with app
			WICED_BT_TRACE( "Paired Device Key Update\n\r");
			wiced_hal_write_nvram ( VSID_REMOTE_KEY, sizeof( wiced_bt_device_link_keys_t ), (uint8_t*)&(p_event_data->paired_device_link_keys_update), &status );
			WICED_BT_TRACE( "\tKeys save to NVRAM %B result: %d \n\r", (uint8_t*)&(p_event_data->paired_device_link_keys_update), status );
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 		// Retrieval saved link keys
	        WICED_BT_TRACE( "Paired Device Link Request Keys Event for device %B\n\r",&(p_event_data->paired_device_link_keys_request ) );

	        /* If the status from read_nvram is not SUCCESS, the stack will generate keys and will then call BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that they can be stored */
	        wiced_hal_read_nvram( VSID_REMOTE_KEY, sizeof(wiced_bt_device_link_keys_t), (uint8_t *) &(p_event_data->paired_device_link_keys_request), &status );
	        WICED_BT_TRACE( "\tKeys read from NVRAM %B result: %d \n\r", &(p_event_data->paired_device_link_keys_request), status );
	        break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
	        WICED_BT_TRACE( "Local Identity Key Update\n\r" );
	        bytes = wiced_hal_write_nvram ( VSID_LOCAL_KEY, sizeof( wiced_bt_local_identity_keys_t ), (uint8_t*)&(p_event_data->local_identity_keys_update), &status );

	        WICED_BT_TRACE( "\tlocal keys save to NVRAM:\n\r" );
	        for( counter = 0; counter<bytes;counter++ )
	        {
	           WICED_BT_TRACE( "%02X ", p_event_data->local_identity_keys_update.local_key_data[counter] );
	           if( counter % 16 == 0 )
	           {
	               WICED_BT_TRACE( "\n\r" );
	           }
	        }
	        WICED_BT_TRACE( "result: %d \n\r", status );
	        break;

		case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 			// Read keys from NVRAM
	        WICED_BT_TRACE( "Local Identity Key Request\n\r" );
	        /* If the status from read_nvram is not SUCCESS, the stack will generate keys and will then call BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT so that they can be stored */
	        bytes = wiced_hal_read_nvram( VSID_LOCAL_KEY, sizeof(wiced_bt_local_identity_keys_t), (uint8_t *)&(p_event_data->local_identity_keys_request), &status );

	        WICED_BT_TRACE( "\tlocal keys read from NVRAM:\n\r" );
	        for( counter = 0; counter<bytes;counter++ )
	        {
	            WICED_BT_TRACE( "%02X ", p_event_data->local_identity_keys_request.local_key_data[counter] );
	            if( counter % 16 == 0 )
	            {
	                WICED_BT_TRACE( "\n\r" );
	            }
	        }
	        WICED_BT_TRACE( "result: %d \n\r", status );
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
			WICED_BT_TRACE( "Advertising state = %d\r\n", p_event_data->ble_advert_state_changed );

		    switch( p_event_data->ble_advert_state_changed )
		    {
		    	case BTM_BLE_ADVERT_OFF:
		     		wiced_hal_pwm_change_values( PWM0, connection_id ? PWM_ALWAYS_ON_TOGGLE : PWM_ALWAYS_OFF_TOGGLE, PWM_BONDING_INIT );
		     		break;

				case BTM_BLE_ADVERT_UNDIRECTED_LOW:
				case BTM_BLE_ADVERT_UNDIRECTED_HIGH:
					if( bonded )
						wiced_hal_pwm_change_values( PWM0, PWM_BONDED_TOGGLE, PWM_BONDED_INIT );
					else
						wiced_hal_pwm_change_values( PWM0, PWM_BONDING_TOGGLE, PWM_BONDING_INIT );
					break;

				default:
					wiced_hal_pwm_change_values( PWM0, PWM_ALWAYS_OFF_TOGGLE, PWM_BONDING_INIT );
					break;
			}
			break;

		default:
			WICED_BT_TRACE( "Unhandled Bluetooth Management Event: 0x%x (%d)\r\n", event, event );
			break;
    }

    return status;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_gatt_callback(
* 					wiced_bt_gatt_evt_t event,
* 					wiced_bt_gatt_event_data_t *p_data )
********************************************************************************/
wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;

    wiced_bt_gatt_connection_status_t *p_conn = &p_data->connection_status;
    wiced_bt_gatt_attribute_request_t *p_attr = &p_data->attribute_request;

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:					// Remote device initiates connect/disconnect
            if( p_conn->connected )
			{
				WICED_BT_TRACE( "GATT_CONNECTION_STATUS_EVT: Connect BDA %B, Connection ID %d\r\n",p_conn->bd_addr, p_conn->conn_id );
				
				/* Handle the connection */
				connection_id = p_conn->conn_id;

				// Save the remote bd_addr into hostinfo because, at this point, we know that is good data
	            memcpy( hostinfo.remote_addr, p_conn->bd_addr, sizeof( BD_ADDR ) );
			}
			else
			{
				// Device has disconnected
				WICED_BT_TRACE( "GATT_CONNECTION_STATUS_EVT: Disconnect BDA %B, Connection ID %d, Reason=%d\r\n", p_conn->bd_addr, p_conn->conn_id, p_conn->reason );
				
				/* Handle the disconnection */
				connection_id = 0;

	            /* Reset the CCCD value so that on a reconnect CCCD will be off */
	            memset( &hostinfo.remote_addr, 0, sizeof( BD_ADDR ) );
	            app_modus_counter_client_char_config[0] = 0;
	            app_modus_counter_client_char_config[1] = 0;

				/* Restart the advertisements */
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
			}
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:					// Remote device initiates a GATT read/write
			switch( p_attr->request_type )
			{
				case GATTS_REQ_TYPE_READ:
					result = app_gatt_get_value( p_attr );
					break;

				case GATTS_REQ_TYPE_WRITE:
					result = app_gatt_set_value( p_attr );
					break;
            }
            break;

        default:
            WICED_BT_TRACE( "Unhandled GATT Event: 0x%x (%d)\r\n", event, event );
            break;
    }

    return result;
}


/*******************************************************************************
* Function Name: void app_set_advertisement_data( void )
********************************************************************************/
void app_set_advertisement_data( void )
{
    wiced_bt_ble_advert_elem_t adv_elem[2] = { 0 };
    uint8_t adv_flag = BTM_BLE_GENERAL_DISCOVERABLE_FLAG | BTM_BLE_BREDR_NOT_SUPPORTED;
    uint8_t num_elem = 0;

    /* Advertisement Element for Flags */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_FLAG;
    adv_elem[num_elem].len = sizeof( uint8_t );
    adv_elem[num_elem].p_data = &adv_flag;
    num_elem++;

    /* Advertisement Element for Name */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
    adv_elem[num_elem].len = app_gap_device_name_len;
    adv_elem[num_elem].p_data = app_gap_device_name;
    num_elem++;

    /* Set Raw Advertisement Data */
    wiced_bt_ble_set_raw_advertisement_data( num_elem, adv_elem );
}


/*******************************************************************************
* Function Name: app_gatt_get_value(
* 					wiced_bt_gatt_attribute_request_t *p_attr )
********************************************************************************/
wiced_bt_gatt_status_t app_gatt_get_value( wiced_bt_gatt_attribute_request_t *p_attr )
{
	uint16_t attr_handle = 	p_attr->data.handle;
	uint8_t  *p_val = 		p_attr->data.read_req.p_val;
	uint16_t *p_len = 		p_attr->data.read_req.p_val_len;

    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Detected a matching handle in the external lookup table
            if (app_gatt_db_ext_attr_tbl[i].cur_len <= *p_len)
            {
                // Value fits within the supplied buffer; copy over the value
                *p_len = app_gatt_db_ext_attr_tbl[i].cur_len;
                memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data, app_gatt_db_ext_attr_tbl[i].cur_len);
                res = WICED_BT_GATT_SUCCESS;

                // TODO: Add code for any action required when this attribute is read
                switch ( attr_handle )
                {
//					case handle:
//						break;
                }
            }
            else
            {
                // Value to read will not fit within the buffer
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        // TODO: Add code to read value using handles not contained within external lookup table
        // This can apply when the option is enabled to not generate initial value arrays.
        // If the value for the current handle is successfully read then set the result using:
        // res = WICED_BT_GATT_SUCCESS;
        switch ( attr_handle )
        {
			default:
				// The read operation was not performed for the indicated handle
				WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\r\n", attr_handle);
				res = WICED_BT_GATT_READ_NOT_PERMIT;
				break;
        }
    }

    return res;
}

/*******************************************************************************
* Function Name: app_gatt_set_value(
*					wiced_bt_gatt_attribute_request_t *p_attr )
********************************************************************************/
wiced_bt_gatt_status_t app_gatt_set_value( wiced_bt_gatt_attribute_request_t *p_attr )
{
	uint16_t attr_handle = 	p_attr->data.handle;
	uint8_t  *p_val = 		p_attr->data.write_req.p_val;
	uint16_t len = 			p_attr->data.write_req.val_len;

    int i = 0;
    wiced_bool_t isHandleInTable = WICED_FALSE;
    wiced_bool_t validLen = WICED_FALSE;
    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            isHandleInTable = WICED_TRUE;
            // Verify that size constraints have been met
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);
            if (validLen)
            {
                // Value fits within the supplied buffer; copy over the value
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
                res = WICED_BT_GATT_SUCCESS;

                // TODO: Add code for any action required when this attribute is written
                // For example you may need to write the value into NVRAM if it needs to be persistent
                switch ( attr_handle )
                {
                	case HDLD_MODUS_COUNTER_CLIENT_CHAR_CONFIG:
                		if ( len != 2 )
                		{
                			return WICED_BT_GATT_INVALID_ATTR_LEN;
                		}
                		hostinfo.cccd[0] = p_val[0];
                		hostinfo.cccd[1] = p_val[1];

                		/* Save value to NVRAM */
                		wiced_result_t temp_result;
                		wiced_hal_write_nvram( VSID_HOSTINFO, sizeof(hostinfo), (uint8_t*)&hostinfo, &temp_result );
                		WICED_BT_TRACE( "\t\tWrite CCCD value to NVRAM\n\r" );
                }
            }
            else
            {
                // Value to write does not meet size constraints
                res = WICED_BT_GATT_INVALID_ATTR_LEN;
            }
            break;
        }
    }

    if (!isHandleInTable)
    {
        // TODO: Add code to write value using handles not contained within external lookup table
        // This can apply when the option is enabled to not generate initial value arrays.
        // If the value for the current handle is successfully written then set the result using:
        // res = WICED_BT_GATT_SUCCESS;
        switch ( attr_handle )
        {
			default:
				// The write operation was not performed for the indicated handle
				WICED_BT_TRACE("Write Request to Invalid Handle: 0x%x\r\n", attr_handle);
				res = WICED_BT_GATT_WRITE_NOT_PERMIT;
				break;
        }
    }

    return res;
}

/*******************************************************************************
* Function Name: void button_cback( void *data, uint8_t port_pin )
********************************************************************************/
void button_cback( void *data, uint8_t port_pin )
{
    app_modus_counter[0]++;

    if( connection_id )
    {
    	if( app_modus_counter_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION )
    	{
    		wiced_bt_gatt_send_notification(
				connection_id,
				HDLC_MODUS_COUNTER_VALUE,
				app_modus_counter_len,
				app_modus_counter );
    	}
    }

    /* Clear the GPIO interrupt */
    wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 );
}


/*******************************************************************************
* Function Name: void rx_cback( void *data )
********************************************************************************/
void rx_cback( void *data )
{
	uint8_t readbyte;
    wiced_result_t                  result;
    wiced_bt_device_link_keys_t     keys;
    wiced_bt_local_identity_keys_t  local_keys;
    BD_ADDR                         bonded_address;

    /* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt();

    /* Remove bonding info if the user sends 'e' */
    if( readbyte == 'e' )
    {
		/* Put into bonding mode  */
		bonded = WICED_FALSE;
		wiced_hal_pwm_change_values( PWM0, PWM_BONDING_TOGGLE, PWM_BONDING_INIT );

		/* Remove from the bonded device list */
		wiced_hal_read_nvram( VSID_HOSTINFO, sizeof(bonded_address), (uint8_t*)&bonded_address, &result );
		wiced_bt_dev_delete_bonded_device(bonded_address);
		WICED_BT_TRACE( "Remove host %B from bonded device list \n\r", bonded_address );
		WICED_BT_TRACE( "Bonding information removed\n\r" );

		/* Remove device from address resolution database */
		wiced_hal_read_nvram( VSID_REMOTE_KEY, sizeof(wiced_bt_device_link_keys_t), (uint8_t*)&keys, &result);
		wiced_bt_dev_remove_device_from_address_resolution_db ( &keys );

		/* Remove bonding information from NVRAM */
		/* Remove bonding information from NVRAM */
		wiced_hal_delete_nvram(VSID_HOSTINFO, &result);
		wiced_hal_delete_nvram(VSID_REMOTE_KEY, &result);
    }
}

