/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the BLE template for the
* ModusToolbox 101 course.
*
*******************************************************************************
* (c) 2019-2020, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*******************************************************************************/
#include "cybsp.h"
#include "cyhal.h"
#include "cy_retarget_io.h"

#include <FreeRTOS.h>
#include <task.h>
#include "cybt_platform_config.h"
#include "wiced_bt_stack.h"

#include "util_functions.h"

/* Include header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#define TASK_STACK_SIZE (4096u)
#define	TASK_PRIORITY 	(5u)

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t	app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );

static wiced_bt_gatt_status_t	app_gatt_get_value( wiced_bt_gatt_read_t *p_data );
static wiced_bt_gatt_status_t	app_gatt_set_value( wiced_bt_gatt_write_t *p_data );

static void 					ble_address_print(wiced_bt_device_address_t bdadr);

/* Button callback function declaration and counter task function declaration */
static void counter_task(void * arg);
void button_cback(void *handler_arg, cyhal_gpio_irq_event_t event);


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
/* BT device stack configuration settings */
const cybt_platform_config_t bt_platform_cfg_settings =
{
	.hci_config =
	{
		.hci_transport = CYBT_HCI_UART,

		.hci =
		{
			.hci_uart =
			{
				.uart_tx_pin = CYBSP_BT_UART_TX,
				.uart_rx_pin = CYBSP_BT_UART_RX,
				.uart_rts_pin = CYBSP_BT_UART_RTS,
				.uart_cts_pin = CYBSP_BT_UART_CTS,

				.baud_rate_for_fw_download = 115200,
				.baud_rate_for_feature     = 115200,

				.data_bits = 8,
				.stop_bits = 1,
				.parity = CYHAL_UART_PARITY_NONE,
				.flow_control = WICED_TRUE
			}
		}
	},

    .controller_config =
    {
        .bt_power_pin      = CYBSP_BT_POWER,
		.sleep_mode =
		{
			#if (bt_0_power_0_ENABLED == 1)     /* BT Power control is enabled in the LPA */
				.sleep_mode_enabled   = CYCFG_BT_LP_ENABLED,
				.device_wakeup_pin    = CYCFG_BT_DEV_WAKE_GPIO,
				.host_wakeup_pin      = CYCFG_BT_HOST_WAKE_GPIO,
				.device_wake_polarity = CYCFG_BT_DEV_WAKE_POLARITY,
				.host_wake_polarity   = CYCFG_BT_HOST_WAKE_IRQ_EVENT
			#else                               /* BT Power control is disabled in the LPA, default to BSP's low power configuration */
				.sleep_mode_enabled   = WICED_TRUE,
				.device_wakeup_pin    = CYBSP_BT_DEVICE_WAKE,
				.host_wakeup_pin      = CYBSP_BT_HOST_WAKE,
				.device_wake_polarity = CYBT_WAKE_ACTIVE_LOW,
				.host_wake_polarity   = CYBT_WAKE_ACTIVE_LOW
			#endif
		}
    },

	.task_mem_pool_size    = 2048
};

/* Global variable for connection ID */
uint16_t connection_id = 0;

/* Global variable for counter task handle */
TaskHandle_t CounterTaskHandle = NULL;


/*******************************************************************
 * Function Implementations
 ******************************************************************/


/*******************************************************************************
* Function Name: int main( void )
********************************************************************************/
int main(void)
{
    cy_rslt_t result ;

    /* Initialize the board support package */
    result = cybsp_init() ;
    CY_ASSERT(CY_RSLT_SUCCESS == result) ;

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,\
                        CY_RETARGET_IO_BAUDRATE);

    /* Initialize LED Pin */
    cyhal_gpio_init(CYBSP_USER_LED,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Configure CYBSP_USER_BTN for a falling edge interrupt */
    cyhal_gpio_init(CYBSP_USER_BTN,CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, button_cback, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, 3, true);

    printf("**********Application Start*****************\n");

    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&bt_platform_cfg_settings);

    /* Initialize stack and register the callback function */
    result=wiced_bt_stack_init (app_bt_management_callback, &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if( WICED_BT_SUCCESS == result)
    {
        printf("Bluetooth Stack Initialization Successful\n");
    }
    else
    {
        printf("Bluetooth Stack Initialization failed!!\n");
    }

    /* Start task to handle Counter notifications */
    xTaskCreate (counter_task,
    "CounterTask",
    TASK_STACK_SIZE,
    NULL,
    TASK_PRIORITY,
    &CounterTaskHandle);

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler() ;

    /* Should never get here */
    CY_ASSERT(0) ;
}


/*******************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(
* 					wiced_bt_management_evt_t event,
* 					wiced_bt_management_evt_data_t *p_event_data )
********************************************************************************/
static wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t status = WICED_BT_SUCCESS;


    printf("Bluetooth Management Event: 0x%x %s\n", event, btm_event_name(event));

    switch( event )
    {
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				printf( "Bluetooth Enabled\n" );

				/* Generate a random local Bluetooth Device Address and print it out */
				wiced_bt_device_address_t bda = {0};
				cyhal_trng_t trng_obj;
				cyhal_trng_init(&trng_obj);
				bda[0] = (uint8_t) cyhal_trng_generate(&trng_obj);
				bda[1] = (uint8_t) cyhal_trng_generate(&trng_obj);
				bda[2] = (uint8_t) cyhal_trng_generate(&trng_obj);
				bda[3] = (uint8_t) cyhal_trng_generate(&trng_obj);
				bda[4] = (uint8_t) cyhal_trng_generate(&trng_obj);
				bda[5] = (uint8_t) cyhal_trng_generate(&trng_obj);
				cyhal_trng_free(&trng_obj);
				wiced_bt_set_local_bdaddr( bda, BLE_ADDR_RANDOM);
				wiced_bt_dev_read_local_addr( bda );
				printf( "Local Bluetooth Device Address: ");
				ble_address_print(bda);
				printf( "\n");

				/* Register GATT callback */
				wiced_bt_gatt_register( app_gatt_callback );

			    printf("GATT event Handler registration status: %s \n",gatt_status_name(status));

			    /* Initialize the GATT database */
				wiced_bt_gatt_db_init( gatt_database, gatt_database_len, NULL );

			    printf("GATT database initialization status: %s \n",gatt_status_name(status));

				/* Enable/disable pairing */
			    wiced_bt_set_pairable_mode( WICED_TRUE, WICED_FALSE );

				/* Create the packet and begin advertising */
			    wiced_bt_ble_set_raw_advertisement_data(CY_BT_ADV_PACKET_DATA_SIZE,cy_bt_adv_packet_data);
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
			}
			else
			{
				printf( "Bluetooth Disabled\n" );
			}
			break;

		case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT: 		// IO capabilities request
			p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_MITM_BOND;
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
			p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
			p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
			break;

		case BTM_PAIRING_COMPLETE_EVT: 						// Pairing Complete event
			break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
			break;

		case BTM_SECURITY_REQUEST_EVT: 						// Security access
			wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr, WICED_BT_SUCCESS );
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys with app
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 		// Retrieval saved link keys
			break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to NVRAM
			break;

		case  BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 			// Read keys from NVRAM
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
            printf("Advertisement State Change: %s\n", btm_advert_mode_name(p_event_data->ble_advert_state_changed));
			break;

		default:
			break;
    }

    return status;
}


/*******************************************************************************
* Function Name: wiced_bt_gatt_status_t app_gatt_callback(
* 					wiced_bt_gatt_evt_t event,
* 					wiced_bt_gatt_event_data_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data )
{
    wiced_bt_gatt_status_t result = WICED_SUCCESS;

    wiced_bt_gatt_connection_status_t *p_conn = &p_data->connection_status;
    wiced_bt_gatt_attribute_request_t *p_attr = &p_data->attribute_request;

    switch( event )
    {
        case GATT_CONNECTION_STATUS_EVT:					// Remote device initiates connect/disconnect
            if( p_conn->connected )
			{
            	printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
            	ble_address_print(p_conn->bd_addr);
            	printf("Connection ID %d\n", p_conn->conn_id );

				/* Handle the connection */
            	connection_id = p_conn->conn_id;
			}
			else
			{
				// Device has disconnected
	            printf("Disconnected : BDA " );
	            ble_address_print(p_conn->bd_addr);
	            printf("Connection ID '%d', Reason '%s'\n", p_conn->conn_id, gatt_disconn_reason_name(p_conn->reason) );

				/* Handle the disconnection */
	            connection_id = 0;

				/* Restart the advertisements */
				wiced_bt_start_advertisements( BTM_BLE_ADVERT_UNDIRECTED_HIGH, 0, NULL );
			}
            break;

        case GATT_ATTRIBUTE_REQUEST_EVT:					// Remote device initiates a GATT read/write
			switch( p_attr->request_type )
			{
				case GATTS_REQ_TYPE_READ:
					result = app_gatt_get_value( &(p_attr->data.read_req) );
					break;

				case GATTS_REQ_TYPE_WRITE:
					result = app_gatt_set_value( &(p_attr->data.write_req) );
					break;
            }
            break;

        default:
        	printf( "Unhandled GATT Event: 0x%x (%d)\n", event, event );
            break;
    }

    return result;
}


/*******************************************************************************
* Function Name: app_gatt_get_value(
* 					wiced_bt_gatt_read_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t	app_gatt_get_value( wiced_bt_gatt_read_t *p_data )
{
	uint16_t attr_handle = 	p_data->handle;
	uint8_t  *p_val = 		p_data->p_val;
	uint16_t *p_len = 		p_data->p_val_len;
	uint16_t  offset =		p_data->offset;

	int i = 0;
	int len_to_copy;

    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
    	// Search for a matching handle in the external lookup table
    	if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            /* Start by assuming we will copy entire value */
    		len_to_copy = app_gatt_db_ext_attr_tbl[i].cur_len;

    		/* Offset is beyond the end of the actual data length, nothing to do*/
    		if ( offset >= len_to_copy)
    		{
    			return WICED_BT_GATT_INVALID_OFFSET;
    		}

    		/* Only need to copy from offset to the end */
    		len_to_copy = len_to_copy - offset;

    		/* Determine if there is enough space to copy the entire value.
    		 * If not, only copy as much as will fit. */
            if (len_to_copy > *p_len)
            {
            	len_to_copy = *p_len;
            }

			/* Tell the stack how much will be copied to the buffer and then do the copy */
			*p_len = len_to_copy;
			memcpy(p_val, app_gatt_db_ext_attr_tbl[i].p_data + offset, len_to_copy);
			res = WICED_BT_GATT_SUCCESS;

			// Add case for any action required when this attribute is read
			switch ( attr_handle )
			{
				case HDLC_BT101_LED_VALUE:
					printf( "LED is %s\r\n", app_bt101_led[0] ? "ON" : "OFF" );
					break;
			}
			break; /* break out of for loop once matching handle is found */
       }
    }
    return res;
}


/*******************************************************************************
* Function Name: app_gatt_set_value(
*					wiced_bt_gatt_write_t *p_data )
********************************************************************************/
static wiced_bt_gatt_status_t	app_gatt_set_value( wiced_bt_gatt_write_t *p_data )
{
	uint16_t attr_handle = 	p_data->handle;
	uint8_t  *p_val = 		p_data->p_val;
	uint16_t len = 			p_data->val_len;

	int i = 0;
    wiced_bool_t validLen = WICED_FALSE;

    wiced_bt_gatt_status_t res = WICED_BT_GATT_INVALID_HANDLE;

    // Check for a matching handle entry and find is max available size
    for (i = 0; i < app_gatt_db_ext_attr_tbl_size; i++)
    {
        if (app_gatt_db_ext_attr_tbl[i].handle == attr_handle)
        {
            // Detected a matching handle in external lookup table
            // Verify that size constraints have been met
            validLen = (app_gatt_db_ext_attr_tbl[i].max_len >= len);
            if (validLen)
            {
                // Value fits within the supplied buffer; copy over the value
                app_gatt_db_ext_attr_tbl[i].cur_len = len;
                memcpy(app_gatt_db_ext_attr_tbl[i].p_data, p_val, len);
    			res = WICED_BT_GATT_SUCCESS;

                // Action required when this attribute is written
                // Print message when notifications are enabled/disabled
                switch ( attr_handle )
                {
					case HDLC_BT101_LED_VALUE:
						cyhal_gpio_write(CYBSP_USER_LED, app_bt101_led[0] == 0 );
						printf( "Turn the LED %s\r\n", app_bt101_led[0] ? "ON" : "OFF" );
						break;
					case HDLD_BT101_COUNTER_CLIENT_CHAR_CONFIG:
					    printf("Setting notify (0x%02x, 0x%02x)\n", p_val[0], p_val[1]);
					    break;
                }
            }
            else /* Length of data will not fit */
            {
                // Value to write does not meet size constraints
                return WICED_BT_GATT_INVALID_ATTR_LEN;
            }
			break; /* break out of for loop once matching handle is found */
        }
    }
    return res;
}


/**************************************************************************************************
* Function Name: ble_address_print()
***************************************************************************************************
* Summary:
*   This is the utility function that prints the address of the Bluetooth device
*
* Parameters:
*   wiced_bt_device_address_t bdadr                : Bluetooth address
*
* Return:
*  void
*
**************************************************************************************************/
static void ble_address_print(wiced_bt_device_address_t bdadr)
{
    for(uint8_t i=0;i<BD_ADDR_LEN;i++)
    {
        printf("%02X:",bdadr[i]);
    }
}


/* Counter task to send a notification */
static void counter_task(void * arg)
{
    /* Notification values received from ISR */
    uint32_t ulNotificationValue;
    while(true)
    {
       /* Wait for the button ISR */
       ulNotificationValue = ulTaskNotifyTake(pdFALSE, portMAX_DELAY);

       /* If button was pressed increment value and check to see if a
        * BLE notification should be sent. If this value is not 1, then
        * it was not a button press (most likely a timeout) that caused
        * the event so we don't want to send a BLE notification. */
        if (ulNotificationValue == 1)
        {
            if( connection_id ) /* Check if we have an active
                                   connection */
            {
                /* Check to see if the client has asked for
                   notifications */
                 if( app_bt101_counter_client_char_config[0] &
                 GATT_CLIENT_CONFIG_NOTIFICATION )
                 {
                     printf( "Notifying counter change (%d)\n",
                         app_bt101_counter[0] );
                     wiced_bt_gatt_send_notification(
                         connection_id,
                         HDLC_BT101_COUNTER_VALUE,
                         app_bt101_counter_len,
                         app_bt101_counter );
                 }
             }
         }

         else
         {
                /* The call to ulTaskNotifyTake() timed out. */
        }
    }
}


/* Button callback function */
void button_cback(void *handler_arg, cyhal_gpio_irq_event_t event)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /* Increment button counter */
    app_bt101_counter[0]++;

    /* Notify the counter task that the button was pressed */
    vTaskNotifyGiveFromISR( CounterTaskHandle,
                            &xHigherPriorityTaskWoken );

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context
       Switch should be performed to ensure the interrupt returns
       directly to the highest priority task.  The macro used for this
       purpose is dependent on the port in use and may be called
       portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/* [] END OF FILE */
