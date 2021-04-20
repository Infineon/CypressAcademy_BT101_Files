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

#include "util_functions.h"

#include "cy_em_eeprom.h"

/* FreeRTOS */
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

/* btstack */
#include "wiced_bt_stack.h"

/* Include header files from BT configurator */
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cycfg_gatt_db.h"

/*******************************************************************
 * Macros to assist development of the exercises
 ******************************************************************/
#define	COUNTER_TASK_PRIORITY 	(5u)
#define COUNTER_TASK_STACK_SIZE (4096u)

#define UART_TASK_PRIORITY 		(5u)
#define UART_TASK_STACK_SIZE 	(4096u)

#define PWM_BONDING_FREQUENCY	(1)
#define PWM_BONDED_FREQUENCY	(5)
#define PWM_OFF_DUTY     		(100)
#define PWM_TOGGLE_DUTY			(50)
#define PWM_ON_DUTY      		(0)

/* LE Key Size */
#define MAX_KEY_SIZE (0x10)

/* Logical Start of Emulated EEPROM and location of structure elements. */
/* Sizeof can't be used because of padding in the structure */
#define LOGICAL_EEPROM_START    (0u)
#define EEPROM_LOCAL_BDA		((void *)&(bondinfo.local_bda) - (void *)&bondinfo)
#define EEPROM_REMOTE_BDA		((void *)&(bondinfo.remote_bda) - (void *)&bondinfo)
#define EEPROM_CCCD				((void *)&(bondinfo.cccd) - (void *)&bondinfo)
#define EEPROM_IDENTITY_KEYS	((void *)&(bondinfo.identity_keys) - (void *)&bondinfo)
#define EEPROM_LINK_KEYS		((void *)&(bondinfo.link_keys) - (void *)&bondinfo)

/* EEPROM Configuration details. */
#define EEPROM_SIZE				(sizeof(bondinfo))
#define SIMPLE_MODE				(0u)
#define WEAR_LEVELLING_FACTOR   (2u)
#define REDUNDANT_COPY          (1u)
#define BLOCKING_WRITE          (1u)

/* Set the macro FLASH_REGION_TO_USE to either USER_FLASH or
 * EMULATED_EEPROM_FLASH to specify the region of the flash used for
 * emulated EEPROM.
 */
#define EMEEPROM_APPLICATION_FLASH              (0u)
#define EMEEPROM_AUXILIARTY_FLASH   			(1u)
#define FLASH_REGION_TO_USE     				EMEEPROM_AUXILIARTY_FLASH


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static wiced_bt_gatt_status_t	app_gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data );

static wiced_bt_gatt_status_t	app_gatt_get_value( wiced_bt_gatt_read_t *p_data );
static wiced_bt_gatt_status_t	app_gatt_set_value( wiced_bt_gatt_write_t *p_data );

static void 					print_ble_address(wiced_bt_device_address_t bdadr);

/* Button callback function declaration and counter task function declaration */
static void button_cback(void *handler_arg, cyhal_gpio_irq_event_t event);
static void counter_task(void * arg);

static void rx_cback(void *handler_arg, cyhal_uart_event_t event); /* Callback for data received from UART */
static void uart_task(void *pvParameters);

static void print_array(void * to_print, uint16_t len);

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/

/* Global variable for connection ID */
uint16_t connection_id = 0;

/* Global variables for  task handles */
TaskHandle_t CounterTaskHandle = NULL;
TaskHandle_t UartTaskHandle = NULL;

/* Queue Handle */
QueueHandle_t xUARTQueue = 0;

cyhal_pwm_t pwm_obj;

bool bonded = WICED_FALSE;		// State of the peripheral - bonded or bonding

/* Structure to store info that goes into EEPROM - it holds the remote BDA, CCCD value, remote keys and local keys */
struct bondinfo
{
	wiced_bt_device_address_t		local_bda;		/* BD address of local host - randomly generated on 1st power up */
	wiced_bt_device_address_t		remote_bda;		/* BD address of remote */
	uint8_t							cccd[2];
	wiced_bt_local_identity_keys_t 	identity_keys;
	wiced_bt_device_link_keys_t 	link_keys;
}  bondinfo;

/* EmEEPROM storage and configuration setup */
#if (EMEEPROM_AUXILIARTY_FLASH == FLASH_REGION_TO_USE)
CY_SECTION(".cy_em_eeprom")
#endif /* #if(FLASH_REGION_TO_USE) */
CY_ALIGN(CY_EM_EEPROM_FLASH_SIZEOF_ROW)
const uint8_t EepromStorage[CY_EM_EEPROM_GET_PHYSICAL_SIZE(EEPROM_SIZE, SIMPLE_MODE, WEAR_LEVELLING_FACTOR, REDUNDANT_COPY)] = {0u};

cy_stc_eeprom_config_t Em_EEPROM_config =
{
	.eepromSize = 			EEPROM_SIZE,
	.simpleMode = 			SIMPLE_MODE,
	.wearLevelingFactor = 	WEAR_LEVELLING_FACTOR,
	.redundantCopy = 		REDUNDANT_COPY,
	.blockingWrite = 		BLOCKING_WRITE,
	.userFlashStartAddr =	(uint32_t)&(EepromStorage[0u]),
};

cy_stc_eeprom_context_t Em_EEPROM_context;
cy_en_em_eeprom_status_t eepromReturnValue;

/* All zero arrays to test against the stored address and device link keys */
const uint8_t zero_bda[sizeof(wiced_bt_device_address_t)] = {0};
const uint8_t zero_key[sizeof(wiced_bt_device_sec_keys_t)] = {0};


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
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,\
                        CY_RETARGET_IO_BAUDRATE);

    /* Register a callback function and set it to fire for any received UART characters */
    cyhal_uart_register_callback(&cy_retarget_io_uart_obj, rx_cback, NULL);
    cyhal_uart_enable_event(&cy_retarget_io_uart_obj, CYHAL_UART_IRQ_RX_NOT_EMPTY , 3, TRUE);

	/* Initialize LED Pin */
    cyhal_gpio_init(CYBSP_USER_LED,CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);

    /* Initialize PWM for connection status LED */
    cyhal_pwm_init(&pwm_obj, CYBSP_USER_LED2, NULL);
    cyhal_pwm_set_duty_cycle(&pwm_obj, PWM_OFF_DUTY, PWM_BONDING_FREQUENCY);
    cyhal_pwm_start(&pwm_obj);

    /* Configure CYBSP_USER_BTN for a falling edge interrupt */
    cyhal_gpio_init(CYBSP_USER_BTN, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLUP, CYBSP_BTN_OFF);
    cyhal_gpio_register_callback(CYBSP_USER_BTN, button_cback, NULL);
    cyhal_gpio_enable_event(CYBSP_USER_BTN, CYHAL_GPIO_IRQ_FALL, 3, true);

    printf("**********Application Start*****************\n");

    /* Initialize the EMEEPROM, read the contents into the bondinfo structure and print the stored values. */
    eepromReturnValue = Cy_Em_EEPROM_Init(&Em_EEPROM_config, &Em_EEPROM_context);
    if(CY_EM_EEPROM_SUCCESS != eepromReturnValue)
    {
    	printf("Error initializing EMEEPROM: %d\n", eepromReturnValue);
		CY_ASSERT(0);
    }
    /* Clear out the bondinfo structure */
	memset( &bondinfo, 0, sizeof(bondinfo) );

	/* Read contents of EEPROM */
	eepromReturnValue = Cy_Em_EEPROM_Read(LOGICAL_EEPROM_START, &(bondinfo), sizeof(bondinfo), &Em_EEPROM_context);
    if(CY_EM_EEPROM_SUCCESS != eepromReturnValue)
    {
    	printf("EEPROM Read Error: %d\n",eepromReturnValue);
    }

	printf("Contents of EEPROM: \n");
	printf("Remote BDA: ");
	print_array(&bondinfo.remote_bda, sizeof(bondinfo.remote_bda));
	printf("CCCD: ");
	print_array(&bondinfo.cccd, sizeof(bondinfo.cccd));
	printf("Identity Keys: ");
	print_array(&bondinfo.identity_keys, sizeof(bondinfo.identity_keys));
	printf("Link Keys: ");
	print_array(&bondinfo.link_keys, sizeof(bondinfo.link_keys));


    /* Configure platform specific settings for the BT device */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

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

    /* Create a queue capable of containing 10 unsigned integer values.
     this is used for communicating between UART ISR and the UART task */
    xUARTQueue = xQueueCreate( 10, sizeof(uint8_t) );

    /* Start task to handle Counter notifications */
    xTaskCreate (counter_task,
    		"CounterTask",
			COUNTER_TASK_STACK_SIZE,
			NULL,
			COUNTER_TASK_PRIORITY,
			&CounterTaskHandle);

    /* Start task to handle UART input */
    xTaskCreate (uart_task,
    		"UartTask",
			UART_TASK_STACK_SIZE,
			NULL,
			UART_TASK_PRIORITY,
			&UartTaskHandle);


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

				/* If the local BDA is all zeros, generate a random address and store it to EEPROM.
				 * This is only done at first power up after programming. It will not be changed until
				 * the device is re-programmed. */
				if(0 == memcmp( &(bondinfo.local_bda), zero_bda, sizeof(zero_bda)))
				{
					cyhal_trng_t trng_obj;
					cyhal_trng_init(&trng_obj);
					bondinfo.local_bda[0] = (uint8_t) cyhal_trng_generate(&trng_obj);
					bondinfo.local_bda[1] = (uint8_t) cyhal_trng_generate(&trng_obj);
					bondinfo.local_bda[2] = (uint8_t) cyhal_trng_generate(&trng_obj);
					bondinfo.local_bda[3] = (uint8_t) cyhal_trng_generate(&trng_obj);
					bondinfo.local_bda[4] = (uint8_t) cyhal_trng_generate(&trng_obj);
					bondinfo.local_bda[5] = (uint8_t) cyhal_trng_generate(&trng_obj);
					cyhal_trng_free(&trng_obj);

		        	eepromReturnValue = Cy_Em_EEPROM_Write(EEPROM_LOCAL_BDA, &(bondinfo.local_bda), sizeof(bondinfo.local_bda), &Em_EEPROM_context);
					if(CY_EM_EEPROM_SUCCESS == eepromReturnValue)
					{
			        	printf("Local BDA saved to EERPROM\n");
					}
					else
					{
						printf("EEROM Write Error: %d\n", eepromReturnValue);
					}
				}
				/* Set the local BDA and print it out */
				wiced_bt_set_local_bdaddr( bondinfo.local_bda, BLE_ADDR_RANDOM);
				printf( "Local Bluetooth Device Address: ");
				print_ble_address(bondinfo.local_bda);
				printf( "\n");

				/* If a bonded device was previously stored, copy in the keys and add them to the address resolution database */
				if(0 != memcmp(&(bondinfo.link_keys.key_data), zero_key, sizeof(zero_key)))
				{
					wiced_bt_dev_add_device_to_address_resolution_db ( &bondinfo.link_keys );
			        printf("Found bonding info for BDA ");
			        print_ble_address(bondinfo.link_keys.bd_addr);
			        printf("\r");
			        bonded = WICED_TRUE; /* We have bonding information already, so don't go into bonding mode */
			    }

				/* Register GATT callback */
				wiced_bt_gatt_register( app_gatt_callback );
			    printf("GATT event Handler registration status: %s \n",gatt_status_name(status));

			    /* Initialize the GATT database */
			    wiced_bt_gatt_db_init( gatt_database, gatt_database_len, NULL );
			    printf("GATT database initiliazation status: %s \n",gatt_status_name(status));

				/* Enable/disable pairing */
			    wiced_bt_set_pairable_mode( WICED_TRUE, WICED_FALSE ); /* Enable pairing */

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
			p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC | BTM_LE_KEY_PID;
			p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
			break;

		case BTM_PAIRING_COMPLETE_EVT: 						// Pairing Complete event
		    printf( "Pairing Complete %d.\n", p_event_data->pairing_complete.pairing_complete_info.ble.reason );

	        if ( p_event_data->pairing_complete.pairing_complete_info.ble.reason == WICED_BT_SUCCESS ) /* Bonding successful */
	        {
	        	/* Write remote bda to to EEPROM */
	        	memcpy(&bondinfo.remote_bda, &(p_event_data->pairing_complete.bd_addr), sizeof(wiced_bt_device_address_t));
	        	eepromReturnValue = Cy_Em_EEPROM_Write(EEPROM_REMOTE_BDA, &(p_event_data->pairing_complete.bd_addr), sizeof(p_event_data->pairing_complete.bd_addr), &Em_EEPROM_context);
				if(CY_EM_EEPROM_SUCCESS == eepromReturnValue)
				{
		        	printf("Bonding info save to EERPROM for BDA ");
		        	print_ble_address(p_event_data->pairing_complete.bd_addr);
		        	printf(" result: %d\n", eepromReturnValue);
		        	bonded = WICED_TRUE; // remember that the device is now bonded
				}
				else
				{
					printf("EEROM Write Error: %d\n", eepromReturnValue);
				}
	        }
	        break;

		case BTM_ENCRYPTION_STATUS_EVT: 						// Encryption Status Event
	        printf( "Encryption Status event: Remote BDA ");
	        print_ble_address(p_event_data->encryption_status.bd_addr);
	        printf(" res %d\n", p_event_data->encryption_status.result );

	        /* Connection has been encrypted and we are already bonded meaning that we have correct/paired device restore values in the EEPROM */
	        if( WICED_TRUE == bonded )
	    	{
	    		/* Set CCCD value from the value that was previously saved in the EEPROM */
	    		app_bt101_counter_client_char_config[0] = bondinfo.cccd[0];
	    		app_bt101_counter_client_char_config[1] = bondinfo.cccd[1];
	    		printf("Restored existing CCCD info from EEPROM\n");
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
	    		printf("Security Request Denied - not in bonding mode\n");
	    		status = WICED_BT_FAILED_ON_SECURITY;
	    	}
			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT: 			// Save link keys with app
			printf( "Paired Device Key Update\n");
			memcpy(&bondinfo.link_keys, &(p_event_data->paired_device_link_keys_update), sizeof( wiced_bt_device_link_keys_t));
			eepromReturnValue = Cy_Em_EEPROM_Write(EEPROM_LINK_KEYS, &(p_event_data->paired_device_link_keys_update), sizeof( wiced_bt_device_link_keys_t), &Em_EEPROM_context);
			if(CY_EM_EEPROM_SUCCESS == eepromReturnValue)
			{
				printf( "Keys saved to EEPROM for BDA ");
				print_ble_address(bondinfo.link_keys.bd_addr);
				printf(" result: %d:", eepromReturnValue );
				print_array(&(p_event_data->paired_device_link_keys_update), sizeof( wiced_bt_device_link_keys_t ));
			}
			else
			{
				printf("EEROM Write Error: %d\n", eepromReturnValue);
			}

			break;

		case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT: 		// Attempt to retrieve saved link keys
	        /* If the key is all 0's, we must return an error to cause the stack to generate keys. After generating keys
	         * the stack will call BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that the keys can be stored */
			if(0 == memcmp(&(bondinfo.link_keys.key_data), zero_key, sizeof(zero_key)))
			{
				status = WICED_BT_ERROR;
				printf("New link keys need to be generated by the stack\n");
			}
			else
			{
				memcpy(&(p_event_data->paired_device_link_keys_request), &(bondinfo.link_keys), sizeof(wiced_bt_device_link_keys_t));
				printf("Link keys are available in the database");
				print_array(&(bondinfo.link_keys), sizeof(wiced_bt_device_link_keys_t));
			}
	        break;

		case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT: 				// Save keys to EEPROM
			printf( "Local Identity Key Update\n" );
			memcpy(&bondinfo.identity_keys, &(p_event_data->local_identity_keys_update), sizeof(sizeof( wiced_bt_local_identity_keys_t)));
			eepromReturnValue = Cy_Em_EEPROM_Write(EEPROM_IDENTITY_KEYS, &(p_event_data->local_identity_keys_update), sizeof( wiced_bt_local_identity_keys_t), &Em_EEPROM_context);
			if(CY_EM_EEPROM_SUCCESS == eepromReturnValue)
			{
				printf( "Local identity Keys saved to EEPROM, result: %d:", eepromReturnValue);
				print_array(&(p_event_data->local_identity_keys_update), sizeof( wiced_bt_local_identity_keys_t));
			}
			else
			{
				printf("EEROM Write Error: %d\n", eepromReturnValue);
			}
	        break;

		case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT: 			// Read keys from NVRAM
			printf( "Local Identity Key Request\n" );
	        /* If the key type is 0, we must return an error to cause the stack to generate keys and then call
	         * BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that the keys can be stored */
			if(0 == bondinfo.identity_keys.key_type_mask)
			{
				status = WICED_ERROR;
				printf("New identity keys need to be generated by the stack.\n");
			}
			else
			{
				memcpy(&(p_event_data->local_identity_keys_request), &(bondinfo.identity_keys), sizeof(wiced_bt_local_identity_keys_t));
				printf("Identity keys are available in the database.\n");

		        printf( "Local identity keys read from EEPROM: \n" );
		        print_array(&bondinfo.identity_keys, sizeof( wiced_bt_local_identity_keys_t));
			}

			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT: 					// Scan State Change
			break;

		case BTM_BLE_ADVERT_STATE_CHANGED_EVT:					// Advertising State Change
            printf("Advertisement State Change: %s\n", btm_advert_mode_name(p_event_data->ble_advert_state_changed));

            /* Set PWM to correct state based on advertising and connection */
            switch( p_event_data->ble_advert_state_changed )
			{
				case BTM_BLE_ADVERT_OFF:
					if(connection_id)
					{
						cyhal_pwm_set_duty_cycle(&pwm_obj, PWM_ON_DUTY, PWM_BONDING_FREQUENCY);
					}
					else
					{
						cyhal_pwm_set_duty_cycle(&pwm_obj, PWM_OFF_DUTY, PWM_BONDING_FREQUENCY);
					}
					break;

				default: /* Advertising is on */
					if( bonded )
					{
						cyhal_pwm_set_duty_cycle( &pwm_obj, PWM_TOGGLE_DUTY, PWM_BONDED_FREQUENCY );
					}
					else
					{
						cyhal_pwm_set_duty_cycle( &pwm_obj, PWM_TOGGLE_DUTY, PWM_BONDING_FREQUENCY);
						break;
					}
			}
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
        case GATT_CONNECTION_STATUS_EVT:					/* Remote device initiates connect/disconnect */
            if( p_conn->connected )
			{
            	printf("GATT_CONNECTION_STATUS_EVT: Connect BDA ");
            	print_ble_address(p_conn->bd_addr);
            	printf("Connection ID %d\n", p_conn->conn_id );

				/* Handle the connection */
            	connection_id = p_conn->conn_id;

				/* Save the remote bd_addr into hostinfo because, at this point, we know that is good data */
	            memcpy( bondinfo.link_keys.bd_addr, p_conn->bd_addr, sizeof( wiced_bt_device_address_t ) );
			}
			else
			{
				// Device has disconnected
	            printf("Disconnected : BDA " );
	            print_ble_address(p_conn->bd_addr);
	            printf("Connection ID '%d', Reason '%s'\n", p_conn->conn_id, gatt_disconn_reason_name(p_conn->reason) );

				/* Handle the disconnection */
	            connection_id = 0;

	            /* Reset the CCCD value so that on a reconnect CCCD will be off */
	            memset( &bondinfo.link_keys.bd_addr, 0, sizeof( wiced_bt_device_address_t ) );
	            app_bt101_counter_client_char_config[0] = 0;
	            app_bt101_counter_client_char_config[1] = 0;

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
				/* Nothing to do here */
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

                // Add case for action required when this attribute is written
                // Add case to print message when notifications are enabled/disabled
                // For example you may need to write the value into EERPOM if it needs to be persistent
                switch ( attr_handle )
                {
					case HDLC_BT101_LED_VALUE:
						cyhal_gpio_write(CYBSP_USER_LED, app_bt101_led[0] == 0 );
						printf( "Turn the LED %s\r\n", app_bt101_led[0] ? "ON" : "OFF" );
						break;
					case HDLD_BT101_COUNTER_CLIENT_CHAR_CONFIG:
                		printf("Setting notify (0x%02x, 0x%02x)\n", p_val[0], p_val[1]);
                   		if ( len != 2 ) /* Check that exactly 2 bytes were sent since the CCCD is always 2 bytes */
						{
							return WICED_BT_GATT_INVALID_ATTR_LEN;
						}

                    		/* Save value to EEPROM */
                   		bondinfo.cccd[0] = p_val[0];
						bondinfo.cccd[1] = p_val[1];
						eepromReturnValue = Cy_Em_EEPROM_Write(EEPROM_CCCD, &(bondinfo.cccd), sizeof(bondinfo.cccd), &Em_EEPROM_context);
						if(CY_EM_EEPROM_SUCCESS != eepromReturnValue)
						{
							printf("EEROM Write Error: %d\n", eepromReturnValue);
						}

                    	printf( "Write CCCD value to EEPROM\n" );
    					break;
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


/* Counter task to send a notification */
/**************************************************************************************************
* Function Name: counter_task(void * arg)
***************************************************************************************************
* Summary:
*   This is the utility function that sends a notification if there is a connection and the client
*   has asked for notifications to be sent. It is unlocked by the button ISR.
*
* Parameters:
*   none
*
* Return:
*   void
*
**************************************************************************************************/
static void counter_task(void * arg)
{
	/* Notification values received from ISR */
	uint32_t ulNotificationValue;

	while(true)
	{
		/* Wait for the button ISR */
		ulNotificationValue = ulTaskNotifyTake( pdFALSE, portMAX_DELAY );

		/* If button was pressed increment value and check to see if a
		 * BLE notification should be sent. If this value is not 1, then
		 * it was not a button press (most likely a timeout) that caused
		 * the event so we don't want to send a BLE notification. */
		if (ulNotificationValue == 1)
		{

			if( connection_id ) /* Check if we have an active connection */
			{
				/* Check to see if the client has asked for notifications */
				if( app_bt101_counter_client_char_config[0] & GATT_CLIENT_CONFIG_NOTIFICATION )
				{
					printf( "Notifying counter change (%d)\n", app_bt101_counter[0] );
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
/**************************************************************************************************
* Function Name: button_cback(void *handler_arg, cyhal_gpio_irq_event_t event)
***************************************************************************************************/
static void button_cback(void *handler_arg, cyhal_gpio_irq_event_t event)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Increment button counter */
	app_bt101_counter[0]++;

	/* Notify the counter task that the button was pressed */
	vTaskNotifyGiveFromISR( CounterTaskHandle, &xHigherPriorityTaskWoken );

	/* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
	should be performed to ensure the interrupt returns directly to the highest
	priority task.  The macro used for this purpose is dependent on the port in
	use and may be called portEND_SWITCHING_ISR(). */
	portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


/*******************************************************************************
* Function Name: rx_cback()
********************************************************************************
*
* Summary:
*   This function gets a character from the UART and sends it to the UART
*   task for processing
*
* Parameters:
*   void *handler_arg:                 Not used
*   cyhal_uart_event_t event:          Not used
*
* Return:
*   None
*
*******************************************************************************/
void rx_cback(void *handler_arg, cyhal_uart_event_t event)
{
    (void) handler_arg;
    uint8_t readbyte;
	cy_rslt_t status;

    /* Read one byte from the buffer with a 100ms timeout */
    status = cyhal_uart_getc(&cy_retarget_io_uart_obj , &readbyte, 100);

    /* If a character was received,send it to the UART task */
	if(CY_RSLT_SUCCESS == status)
	{
    	xQueueSendFromISR( xUARTQueue, &readbyte, NULL);
	}
}


/*******************************************************************************
* Function Name: uart_task()
********************************************************************************
*
* Summary:
*   This function runs the UART task which processes the received commands via
*   Terminal.
*
* Parameters:
*   void *pvParameters                 Not used
*
* Return:
*   None
*
*******************************************************************************/
static void uart_task(void *pvParameters)
{
    uint8_t readbyte;
    for(;;)
    {
		/* Wait for a character to be sent from the UART ISR */
        if(pdPASS == xQueueReceive( xUARTQueue, &(readbyte), portMAX_DELAY))
        {
            switch (readbyte)
			{
            	case 'e':
					printf( "Removing bonded device info\n");

					/* Put into bonding mode  */
					bonded = WICED_FALSE;
					cyhal_pwm_set_duty_cycle(&pwm_obj, PWM_TOGGLE_DUTY, PWM_BONDING_FREQUENCY);

					/* Remove from the bonded device list */
					wiced_bt_dev_delete_bonded_device(bondinfo.remote_bda);
					printf( "Removed host from bonded device list: ");
					print_ble_address(bondinfo.remote_bda);
					printf( "\n");

					/* Remove device from address resolution database */
					wiced_bt_dev_remove_device_from_address_resolution_db (&(bondinfo.link_keys));
					printf( "Removed device from address resolution database\n");

					/* Remove bonding information from EERPOM (keep the local host BDA intact) */
					memset( &bondinfo, 0, sizeof(bondinfo) );
					eepromReturnValue = Cy_Em_EEPROM_Write(EEPROM_REMOTE_BDA, &(bondinfo.remote_bda), (sizeof(bondinfo) - sizeof(bondinfo.local_bda)), &Em_EEPROM_context);
					if(CY_EM_EEPROM_SUCCESS == eepromReturnValue)
					{
						printf( "Erased EEPROM\n");
					}
					else
					{
						printf("EEROM Write Error: %d\n", eepromReturnValue);
					}
					break;
            	default:
            		printf( "Invalid input");
            		break;
			}
        }
    }
}


/**************************************************************************************************
* Function Name: print_ble_address(wiced_bt_device_address_t bdadr)
***************************************************************************************************
* Summary:
*   This is a utility function that prints the address of the Bluetooth device
*
* Parameters:
*   wiced_bt_device_address_t bdadr                : Bluetooth address
*
* Return:
*  void
*
**************************************************************************************************/
static void print_ble_address(wiced_bt_device_address_t bdadr)
{
    for(uint8_t i=0;i<BD_ADDR_LEN;i++)
    {
        printf("%02X:",bdadr[i]);
    }
}


/*******************************************************************************
* Function Name: void print_array( void* to_print, uint16_t len )
********************************************************************************
* Summary:
*   This is a utility function that prints the specified number of values from memory
*
* Parameters:
*   void* to_print            	: Pointer to the location to print
*   uint16_t					: Number of bytes to print
*
* Return:
*  void
*
********************************************************************************/
static void print_array(void * to_print, uint16_t len)
{
	uint16_t counter;

	for( counter = 0; counter<len;counter++ )
	{
	   if( counter % 16 == 0 )
	   {
		   printf( "\n" );
	   }
		printf( "%02X ", *((uint8_t *)(to_print + counter)) );
	}
	printf( "\n" );

}
/* [] END OF FILE */
