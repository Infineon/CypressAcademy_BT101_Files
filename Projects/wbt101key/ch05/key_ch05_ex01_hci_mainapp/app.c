#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"

#include "GeneratedSource/cycfg.h"
#include "GeneratedSource/cycfg_gatt_db.h"

#include "wiced_transport.h"
#include "hci_control_api.h"

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
 
/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/* PWM configuration defines */
#define PWM_FREQUENCY	(1000)

/* PWM range values */
#define PWM_MAX			(0xFFFF)
#define PWM_INIT		(PWM_MAX-999)

/* PWM compare values for blinking, always on, always off */
#define PWM_TOGGLE		(PWM_MAX-100)
#define PWM_ALWAYS_ON	(PWM_INIT)
#define PWM_ALWAYS_OFF	(PWM_MAX)


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t	app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );

wiced_bt_gatt_status_t			app_gatt_get_value( wiced_bt_gatt_attribute_request_t *p_attr );
wiced_bt_gatt_status_t			app_gatt_set_value( wiced_bt_gatt_attribute_request_t *p_attr );

void							app_set_advertisement_data( void );

void							button_cback( void *data, uint8_t port_pin );

uint32_t hci_control_process_rx_cmd( uint8_t* p_data, uint32_t len );


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
uint8_t mfr_data[] = { 0x31, 0x01, 0x00 };

static wiced_transport_buffer_pool_t* transport_pool = NULL;

/*******************************************************************
 * Transport Configuration
 ******************************************************************/
#define TRANS_UART_BUFFER_SIZE  1024
#define TRANS_UART_BUFFER_COUNT 2

const wiced_transport_cfg_t  transport_cfg =
{
    .type                = WICED_TRANSPORT_UART,		/**< Wiced transport type. */
    .cfg.uart_cfg        =
    {
	.mode = WICED_TRANSPORT_UART_HCI_MODE,		/**<  UART mode, HCI or Raw */
	.baud_rate = HCI_UART_DEFAULT_BAUD			/**<  UART baud rate */
    },
    .rx_buff_pool_cfg    =
    {
    	.buffer_size = TRANS_UART_BUFFER_SIZE,		/**<  Rx Buffer Size */
	.buffer_count = TRANS_UART_BUFFER_COUNT		/**<  Rx Buffer Count */
    },
    .p_status_handler    = NULL,				/**< Wiced transport status handler.*/
    .p_data_handler      = hci_control_process_rx_cmd,	/**< Wiced transport receive data handler. */
    .p_tx_complete_cback = NULL				/**< Wiced transport tx complete callback. */
};


/*******************************************************************************
* Function Name: void application_start( void )
********************************************************************************/
void application_start( void )
{
	/* Initialize the transport configuration */
	    wiced_transport_init( &transport_cfg );

	    /* Initialize Transport Buffer Pool */
	    transport_pool = wiced_transport_create_buffer_pool ( TRANS_UART_BUFFER_SIZE,
	     TRANS_UART_BUFFER_COUNT );

	#if ((defined WICED_BT_TRACE_ENABLE) || (defined HCI_TRACE_OVER_TRANSPORT))
        /* Select Debug UART setting to see debug traces on the appropriate port */
        wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    #endif

    wiced_bt_trace_enable( );
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
    uint8_t temp;

    switch( event )
    {
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				WICED_BT_TRACE( "Bluetooth Enabled\r\n" );

				/* Configure the button to trigger an interrupt when pressed */
				wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );
				wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_cback, 0 );

				/* Use Application Settings dialog to set BT_DEVICE_ADDRESS = random */
				wiced_bt_device_address_t bda;
				wiced_bt_dev_read_local_addr( bda );
				WICED_BT_TRACE( "Local Bluetooth Device Address: [%B]\r\n", bda );

				/* TODO: Configure the GATT database and advertise for connections */

				

				/* TODO: Enable/disable pairing */

				
				
				/* Create the packet and begin advertising */
				app_set_advertisement_data();
				//wiced_bt_start_advertisements( BTM_BLE_ADVERT_NONCONN_HIGH, 0, NULL );
			}
			else
			{
				WICED_BT_TRACE( "Bluetooth Disabled\r\n" );
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: 		// IO capabilities request
			break;

		case BTM_PAIRING_COMPLETE_EVT: 						// Pairing Complete event
			break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
			break;

		case BTM_SECURITY_REQUEST_EVT: 						// Security access
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys with app
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 		// Retrieval saved link keys
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
			break;

		case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 			// Read keys from NVRAM
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
			WICED_BT_TRACE( "Advertising state = %d\r\n", p_event_data->ble_advert_state_changed );
			if( p_event_data->ble_advert_state_changed == 0) /* Advertising stopped */
			{
				temp = 0;
			}
			else /* Advertising started */
			{
				 temp = 1;
			}
			wiced_transport_send_data(HCI_CONTROL_LE_COMMAND_ADVERTISE, &temp, 1 );
			break;

		default:
			WICED_BT_TRACE( "Unhandled Bluetooth Management Event: 0x%x (%d)\n", event, event );
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
				
				/* TODO: Handle the connection */
			}
			else
			{
				// Device has disconnected
				WICED_BT_TRACE( "GATT_CONNECTION_STATUS_EVT: Disconnect BDA %B, Connection ID %d, Reason=%d\r\n", p_conn->bd_addr, p_conn->conn_id, p_conn->reason );
				
				/* TODO: Handle the disconnection */

				/* Restart the advertisements */
				//wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
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
    wiced_bt_ble_advert_elem_t adv_elem[3] = { 0 };
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

    /* Advertisement Element for Manufacturer */
    adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_MANUFACTURER;
    adv_elem[num_elem].len = sizeof( mfr_data );
    adv_elem[num_elem].p_data = mfr_data;
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
            WICED_BT_TRACE("Read Request to Invalid Handle: 0x%x\n", attr_handle);
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
            WICED_BT_TRACE("Write Request to Invalid Handle: 0x%x\n", attr_handle);
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
	mfr_data[2]++;
	app_set_advertisement_data();
}


/* Handle Command Received over Transport */
uint32_t hci_control_process_rx_cmd( uint8_t* p_data, uint32_t len )
{
    uint8_t  status = 0;
    uint8_t  cmd_status = HCI_CONTROL_STATUS_SUCCESS;
    uint16_t opcode = 0;
    uint8_t* p_payload_data = NULL;

    WICED_BT_TRACE("hci_control_process_rx_cmd : Data Length '%d'\n", len);

    // At least 4 bytes are expected in WICED Header
    if ((NULL == p_data) || (len < 4))
    {
        WICED_BT_TRACE("Invalid Parameters\n");
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    else
    {
        // Extract OpCode and Payload data from little-endian byte array
        opcode = (uint16_t)( ((p_data)[0] | ((p_data)[1] << 8)) );
        p_payload_data = &p_data[sizeof(uint16_t)*2];

        // TODO: Process received HCI Command based on its Opcode
        // (see 'hci_control_api.h' for additional details)
        switch (opcode)
        {
        case HCI_CONTROL_LE_COMMAND_ADVERTISE:
        	if(p_payload_data[0]== 0)
        	{
        		wiced_bt_start_advertisements( BTM_BLE_ADVERT_OFF, 0, NULL );
        	}
        	else
        	{
        		wiced_bt_start_advertisements( BTM_BLE_ADVERT_NONCONN_HIGH, 0, NULL );
        	}
        	break;
        default:
            // HCI Control Group was not handled
            cmd_status = HCI_CONTROL_STATUS_UNKNOWN_GROUP;
            wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status,
sizeof(cmd_status));
            break;
        }
    }

    // When operating in WICED_TRANSPORT_UART_HCI_MODE or WICED_TRANSPORT_SPI,
    // application has to free buffer in which data was received
    wiced_transport_free_buffer( p_data );
    p_data = NULL;

    return status;
}
