#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "string.h"
#include "app.h"
#include "company_ids.h"
#include "device_table.h"
#include "print_functions.h"

#include "GeneratedSource/cycfg.h"

/*******************************************************************
 * Constant Definitions
 ******************************************************************/
#define TYPE_TIMEOUT            2

/*******************************************************************
 * Static Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t 	app_bt_management_callback		( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                  	uart_rx_callback       			( void *data );
static void 					timer_function					(uint32_t arg);
static void 					newAdv							(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);
static void                  	change_scan_state      			(wiced_bt_ble_scan_type_t new_state);

/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
static wiced_thread_t *timer_thread;
static wiced_bt_ble_scan_type_t scan_state = BTM_BLE_SCAN_TYPE_NONE;
static wiced_bool_t print = WICED_TRUE;
static wiced_bool_t old_print;
static wiced_bool_t filter = WICED_FALSE;
static uint32_t timer = 0;
static uint32_t type_time = 0;
// 0: Scan screen
// 1: Multi-line table
// 2: Filter recent packets
static uint32_t screen_number = 0;


/*******************************************************************
 * Function Definitions
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

	WICED_BT_TRACE( "**** CYW20819 App Start **** \n" );

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
	wiced_bt_device_address_t bda = {0};

	switch( event )
	{
		case BTM_ENABLED_EVT:								// Bluetooth Controller and Host Stack Enabled
			if( WICED_BT_SUCCESS == p_event_data->enabled.status )
			{
				WICED_BT_TRACE( "Bluetooth Enabled\n" );

				/* Use Application Settings dialog to set BT_DEVICE_ADDRESS = random */
				wiced_bt_dev_read_local_addr( bda );
				WICED_BT_TRACE( "Local Bluetooth Device Address: [%B]\n", bda );

				/* PUART receive not used in first two exercises */
				wiced_hal_puart_register_interrupt( uart_rx_callback );	// Enable receive interrupts on the PUART
				wiced_hal_puart_set_watermark_level( 1 );				// Interrupt up on each byte received
				wiced_hal_puart_enable_rx();

	            /* Enable scanning */
	            timer_thread = wiced_rtos_create_thread();
	            wiced_rtos_init_thread(timer_thread,4,"timer",timer_function,1000,0);
	            change_scan_state(BTM_BLE_SCAN_TYPE_HIGH_DUTY);
			}
			break;

		case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
			break;

		case BTM_BLE_SCAN_STATE_CHANGED_EVT:
			switch( p_event_data->ble_scan_state_changed )
			{
				case BTM_BLE_SCAN_TYPE_NONE:
					WICED_BT_TRACE( "Scanning stopped.\n" );
					break;

				case BTM_BLE_SCAN_TYPE_HIGH_DUTY:
					WICED_BT_TRACE( "High duty scanning.\n" );
					break;

				case BTM_BLE_SCAN_TYPE_LOW_DUTY:
					WICED_BT_TRACE( "Low duty scanning.\n" );
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
* Table Creation Helper Functions used in the UART RX callback
********************************************************************************/
void start_singleline_table()
{
    if(dt_getNumDevices() == 0)
    {
        WICED_BT_TRACE("No devices.\n");
        return;
    }
    if(dt_getNumDevices() > PAGE_SIZE_S)
    {
        old_print = print;
        print = WICED_FALSE;
        screen_number = 3;
        printDeviceTableOneLine();
    }
    else
    {
        screen_number = 0;
        wiced_hal_puart_print( "\n" );
        printDeviceTableOneLine();
        wiced_hal_puart_print( "\n" );
    }
}

void start_multiline_table()
{
    if(dt_getNumDevices() == 0)
    {
        WICED_BT_TRACE("No devices.\n");
        return;
    }
        old_print = print;
    print = WICED_FALSE;
    screen_number = 1;
    printDeviceTableMultiLine();
}

void start_beacon_table()
{
    if(dt_getNumBeacons() == 0)
    {
        WICED_BT_TRACE("No beacons.\n");
        return;
    }
    if(dt_getNumBeacons() > PAGE_SIZE_S)
    {
            old_print = print;
        print = WICED_FALSE;
        screen_number = 4;
        printBeaconTable();
    }
    else
    {
        screen_number = 0;
        wiced_hal_puart_print( "\n" );
        printBeaconTable();
        wiced_hal_puart_print( "\n" );
    }
}

void start_recent_table()
{
    if(dt_getNumDevices() == 0)
    {
        WICED_BT_TRACE("No data.\n");
        return;
    }
        old_print = print;
    print = WICED_FALSE;
    screen_number = 2;
    printRecentFilterData();
}


/*******************************************************************************
* Function Name: void uart_rx_callback( void *data )
********************************************************************************/
void uart_rx_callback( void *data )
{
    uint8_t  readbyte;
    uint32_t focus;

    /* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt();

    if(screen_number == 0)
    {
        /* Process the read character */
        switch (readbyte)
        {
        case 's':
            /* Turn scanning on */
            change_scan_state(BTM_BLE_SCAN_TYPE_HIGH_DUTY);
            break;
        case 'S':
            /* Turn scanning off */
            change_scan_state(BTM_BLE_SCAN_TYPE_NONE);
            break;
        case 't':
            /* Dump table */
            start_singleline_table();
            break;
        case 'b':
            /* Dump beacon table */
            start_beacon_table();
            break;
        case 'm':
            /* Dump multiline table */
            start_multiline_table();
            break;
        case 'r':
            /* Dump recent filtered packets */
            start_recent_table();
            break;
        case 'R':
            /* Dump most recent filtered packet */
            if(dt_getNumDevices() == 0)
            {
                WICED_BT_TRACE("No data.\n");
                break;
            }
            printMostRecentFilterData();
            break;
        case 'f':
            /* Enabled filter */
            if(filter == WICED_FALSE)
                wiced_hal_puart_print( "Filter Enabled\n" );
            filter = WICED_TRUE;
            break;
        case 'F':
            /* Disable filter */
            if(filter == WICED_TRUE)
                wiced_hal_puart_print( "Filter Disabled\n" );
            filter = WICED_FALSE;
            break;
        case 'p':
            /* Enable printing */
            if(print == WICED_FALSE)
                wiced_hal_puart_print( "Printing Enabled\n" );
            print = WICED_TRUE;
            break;
        case 'P':
            /* Disable printing */
            if(print == WICED_TRUE)
                wiced_hal_puart_print( "Printing Disabled\n" );
            print = WICED_FALSE;
            break;
        case 'd':
            /* Clear device data */
            timer = 0;
            dt_reset();
            wiced_hal_puart_print( "All device data cleared\n" );
            clear_terminal();
            break;
        case 'c':
            /* Clear the terminal */
            clear_terminal();
            break;
        case '?':
            /* Print help */
            wiced_hal_puart_print( "\n" );
            wiced_hal_puart_print( "+-------- Available Commands --------+\n" );
            wiced_hal_puart_print( "|  s/S    Enable/Disable Scanning    |\n" );
            wiced_hal_puart_print( "|  p/P    Enable/Disable Printing    |\n" );
            wiced_hal_puart_print( "|  f/F    Enable/Disable Filter      |\n" );
            wiced_hal_puart_print( "|  r      Dump Recent Filter Packets |\n" );
            wiced_hal_puart_print( "|  R      Print Most Recent Packet   |\n" );
            wiced_hal_puart_print( "|  t      Dump Single-line Table     |\n" );
            wiced_hal_puart_print( "|  b      Dump Beacon Table          |\n" );
            wiced_hal_puart_print( "|  m      Dump Multiline Table       |\n" );
            wiced_hal_puart_print( "|  d      Delete All Device Data     |\n" );
            wiced_hal_puart_print( "|  c      Clear Screen               |\n" );
            wiced_hal_puart_print( "|  ?      Print Commands             |\n" );
            wiced_hal_puart_print( "+------------------------------------+\n" );
            wiced_hal_puart_print( "\n" );
            wiced_hal_puart_print( "+----------- Changing the filter -----------+\n" );
            wiced_hal_puart_print( "| 1. Find the device's index in the table.  |\n" );
            wiced_hal_puart_print( "| 2. Type the index with less than two      |\n" );
            wiced_hal_puart_print( "|    seconds between each key press.        |\n" );
            wiced_hal_puart_print( "| 3. To change the filter, wait at least    |\n" );
            wiced_hal_puart_print( "|    two seconds and type the new index.    |\n" );
            wiced_hal_puart_print( "+-------------------------------------------+\n" );
            wiced_hal_puart_print( "\n" );
            break;
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        case '9':
            /* Set filter number */
            if(timer - type_time > TYPE_TIMEOUT)
                dt_setFocus(readbyte-'0');
            else
                dt_setFocus(10*dt_getFocus()+(readbyte-'0'));
            type_time = timer;
            break;
        }
    }
    else
    {
        /* Process the read character */
        switch (readbyte)
        {
        /* Change table pages */
        case '>':
        {
            switch (screen_number)
            {
            case 1:
                incrementPageNum_m();
                printDeviceTableMultiLine();
                break;
            case 2:
                incrementPageNum_r();
                printRecentFilterData();
                break;
            case 3:
                incrementPageNum_s();
                printDeviceTableOneLine();
                break;
            case 4:
                incrementPageNum_b();
                printBeaconTable();
                break;
            }
            break;
        }
        case '<':
        {
            switch (screen_number)
            {
            case 1:
                decrementPageNum_m();
                printDeviceTableMultiLine();
                break;
            case 2:
                decrementPageNum_r();
                printRecentFilterData();
                break;
            case 3:
                decrementPageNum_s();
                printDeviceTableOneLine();
                break;
            case 4:
                decrementPageNum_b();
                printBeaconTable();
                break;
            }
            break;
        }
        case 't':
            /* Dump table */
            print = old_print;
            start_singleline_table();
            break;
        case 'b':
            /* Dump beacon table */
            print = old_print;
            start_beacon_table();
            break;
        case 'm':
            /* Dump multiline table */
            print = old_print;
            start_multiline_table();
            break;
        case 'r':
            /* Dump recent filtered packets */
            print = old_print;
            start_recent_table();
            break;
        case 0x1B:
        case 'c':
            /* Clear the terminal and exit the table */
            screen_number = 0;
            print = old_print;
            clear_terminal();
            reset_tables();
            break;
        case '?':
            /* Print help */
            wiced_hal_puart_print( "\n" );
            wiced_hal_puart_print( "+------- Available Commands -------+\n" );
            wiced_hal_puart_print( "|  <    Increment Page             |\n" );
            wiced_hal_puart_print( "|  >    Decrement Page             |\n" );
            wiced_hal_puart_print( "|  ESC  Exit Table                 |\n" );
            wiced_hal_puart_print( "|  r    Dump Recent Filter Packets |\n" );
            wiced_hal_puart_print( "|  t    Dump Single-line Table     |\n" );
            wiced_hal_puart_print( "|  b    Dump Beacon Table          |\n" );
            wiced_hal_puart_print( "|  m    Dump Multiline Table       |\n" );
            wiced_hal_puart_print( "|  ?    Print Commands             |\n" );
            wiced_hal_puart_print( "+----------------------------------+\n" );
            wiced_hal_puart_print( "\n" );
            break;
        }
    }
}


/*******************************************************************************
* Function Name: void timer_function(uint32_t arg)
********************************************************************************/
void timer_function(uint32_t arg)
{
    WICED_BT_TRACE("Started timer thread\n");
    while(1)
    {
        timer++;
        wiced_rtos_delay_milliseconds(1000, ALLOW_THREAD_TO_SLEEP);
    }
}


/* This function is executed in the BTM_ENABLED_EVT management callback */
static void change_scan_state(wiced_bt_ble_scan_type_t new_state)
{
    if(new_state != scan_state)
    {
        wiced_bt_ble_scan(new_state,FALSE,newAdv);
        scan_state = new_state;
    }
}


/* This is the scan result callback */
void newAdv(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    (void)dt_addDevice(p_scan_result,p_adv_data,timer);
}


/*******************************************************************************
* Functions to return global variable values
********************************************************************************/
wiced_bool_t printing_enabled(void)
{
    return print;
}

wiced_bool_t filter_enabled(void)
{
    return filter;
}

uint32_t current_time(void)
{
    return timer;
}
