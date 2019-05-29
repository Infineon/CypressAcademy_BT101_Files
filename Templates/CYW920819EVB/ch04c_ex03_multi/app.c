#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_beacon.h"
#include "wiced_rtos.h"
#include "wiced_timer.h"
#include "GeneratedSource/cycfg.h"


/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)


/* Allocate the multi-advertising instance numbers */
#define BEACON_EDDYSTONE_URL		1
#define BEACON_EDDYSTONE_TLM		2
#define BEACON_EDDYSTONE_UID		3

/* This one byte will insert .com at the end of a URL in a URL frame. */
#define DOT_COM 0x07

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t	app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void					app_set_advertisement_data( void );

void					timer_cback( uint32_t data );


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/

wiced_timer_t led_timer;							// 0.1s timer

uint8_t url_packet[WICED_BT_BEACON_ADV_DATA_MAX];	// URL advertising packet

wiced_bt_ble_multi_adv_params_t adv_parameters =
{
    .adv_int_min = BTM_BLE_ADVERT_INTERVAL_MIN,
    .adv_int_max = BTM_BLE_ADVERT_INTERVAL_MAX,
    .adv_type = MULTI_ADVERT_NONCONNECTABLE_EVENT,
    .channel_map = BTM_BLE_ADVERT_CHNL_37 | BTM_BLE_ADVERT_CHNL_38 | BTM_BLE_ADVERT_CHNL_39,
    .adv_filter_policy = BTM_BLE_ADVERT_FILTER_WHITELIST_CONNECTION_REQ_WHITELIST_SCAN_REQ,
    .adv_tx_power = MULTI_ADV_TX_POWER_MAX,
    .peer_bd_addr = {0},
    .peer_addr_type = BLE_ADDR_PUBLIC,
    .own_bd_addr = {0},
    .own_addr_type = BLE_ADDR_PUBLIC
};


/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Entry point to the application. Initialize transport configuration
*          and register BLE management event callback. The actual application
*          initialization will happen when stack reports that BT device is ready
********************************************************************************/
void application_start(void)
{
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    WICED_BT_TRACE( "**** CYW20819 App Start **** \r\n" );

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
        	WICED_BT_TRACE( "Bluetooth Enabled\r\n" );
        	
            /* Use Application Settings dialog to set BT_DEVICE_ADDRESS = random */
        	wiced_bt_device_address_t bda;
        	wiced_bt_dev_read_local_addr( bda );
        	WICED_BT_TRACE( "Local Bluetooth Device Address: [%B]\r\n", bda );

        	/* Create the packet and begin advertising */
        	app_set_advertisement_data();

        	/* Start 1s timer */
        	wiced_init_timer( &led_timer, timer_cback, 0, WICED_MILLI_SECONDS_PERIODIC_TIMER );
        	wiced_start_timer( &led_timer, SLEEP_100MS );
        }
        break;

    default:
        break;
    }

    return status;
}


/*******************************************************************************
* Function Name: void app_set_advertisement_data( void )
********************************************************************************/
void app_set_advertisement_data( void )
{
	uint8_t packet_len;

    uint8_t url[] = {'c', 'y', 'p', 'r', 'e', 's', 's', DOT_COM, 0x00}; /* Name for cypress.com with null termination for the string added */

    /* Set up a URL packet with max power, and implicit "http://www." prefix */
    wiced_bt_eddystone_set_data_for_url( adv_parameters.adv_tx_power, EDDYSTONE_URL_SCHEME_0, url, url_packet, &packet_len );
    wiced_set_multi_advertisement_data( url_packet, packet_len, BEACON_EDDYSTONE_URL );
    wiced_set_multi_advertisement_params( BEACON_EDDYSTONE_URL, &adv_parameters );
    wiced_start_multi_advertisements( MULTI_ADVERT_START, BEACON_EDDYSTONE_URL );

    /* Set up a TLM packet with the number of seconds and battery voltage, temperature, advert count = 0 */


	/* Set up a UID packet with ranging_data = 0, and the uid_namespace and uid_instance values declared above */


}


/*******************************************************************************
* Function Name: void timer_cback( uint32_t *data )
********************************************************************************/
void timer_cback( uint32_t data )
{
	static uint32_t tenths = 0;
	uint8_t packet_len;

	/* Increment the tenths */


	/* Reuse two lines of code from app_set_advertisement_data() to re-generate the packet and re-set the advertising data */


}
