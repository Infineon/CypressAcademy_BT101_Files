/*******************************************************************
 * Imports
 ******************************************************************/
#include "sparcommon.h"
#include "hci_control_api.h"

#include "wiced.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_platform.h"
#include "wiced_hal_pspi.h"
#include "wiced_hal_puart.h"
#include "wiced_rtos.h"
#include "wiced_transport.h"

#include "company_ids.h"
#include "device_table.h"
#include "print_functions.h"

/*******************************************************************
 * Constant Definitions
 ******************************************************************/
#define TRANS_UART_BUFFER_SIZE  1024
#define TRANS_UART_BUFFER_COUNT 2
#define TYPE_TIMEOUT            2

/*******************************************************************
 * Variable Definitions
 ******************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
// Transport pool for sending RFCOMM data to host
static wiced_transport_buffer_pool_t* transport_pool = NULL;
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
 * Function Prototypes
 ******************************************************************/
static void                  change_scan_state      (wiced_bt_ble_scan_type_t new_state);
static wiced_bt_dev_status_t advscanner_management_callback    ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static uint32_t              hci_control_process_rx_cmd        ( uint8_t* p_data, uint32_t len );
static void                  rx_cback                          ( void *data );
#ifdef HCI_TRACE_OVER_TRANSPORT
static void                  advscanner_trace_callback         ( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
#endif

/*******************************************************************
 * Macro Definitions
 ******************************************************************/
// Macro to extract uint16_t from little-endian byte array
#define LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(byte_array) \
        (uint16_t)( ((byte_array)[0] | ((byte_array)[1] << 8)) )

/*******************************************************************
 * Transport Configuration
 ******************************************************************/
wiced_transport_cfg_t transport_cfg =
{
        WICED_TRANSPORT_UART,              /**< Wiced transport type. */
        {
                WICED_TRANSPORT_UART_HCI_MODE, /**<  UART mode, HCI or Raw */
                HCI_UART_DEFAULT_BAUD          /**<  UART baud rate */
        },
        {
                TRANS_UART_BUFFER_SIZE,        /**<  Rx Buffer Size */
                TRANS_UART_BUFFER_COUNT        /**<  Rx Buffer Count */
        },
        NULL,                              /**< Wiced transport status handler.*/
        hci_control_process_rx_cmd,        /**< Wiced transport receive data handler. */
        NULL                               /**< Wiced transport tx complete callback. */
};

/*******************************************************************
 * Function Definitions
 ******************************************************************/
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

static wiced_thread_t *timer_thread;
void timer_function(uint32_t arg)
{
    WICED_BT_TRACE("Started timer thread\n\r");
    while(1)
    {
        timer++;
        wiced_rtos_delay_milliseconds(1000,ALLOW_THREAD_TO_SLEEP);
    }
}

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
void application_start(void)
{
    /* Initialize the transport configuration */
    wiced_transport_init( &transport_cfg );

    /* Initialize Transport Buffer Pool */
    transport_pool = wiced_transport_create_buffer_pool ( TRANS_UART_BUFFER_SIZE, TRANS_UART_BUFFER_COUNT );

#if ((defined WICED_BT_TRACE_ENABLE) || (defined HCI_TRACE_OVER_TRANSPORT))
    /* Set the Debug UART as WICED_ROUTE_DEBUG_NONE to get rid of prints */
    //  wiced_set_debug_uart( WICED_ROUTE_DEBUG_NONE );

    /* Set Debug UART as WICED_ROUTE_DEBUG_TO_PUART to see debug traces on Peripheral UART (PUART) */
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );

    /* Set the Debug UART as WICED_ROUTE_DEBUG_TO_WICED_UART to send debug strings over the WICED debug interface */
    //wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_WICED_UART );
#endif

    /* Initialize Bluetooth Controller and Host Stack */
    wiced_bt_stack_init(advscanner_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

void newAdv(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    (void)dt_addDevice(p_scan_result,p_adv_data,timer);
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

/* Bluetooth Management Event Handler */
wiced_bt_dev_status_t advscanner_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_dev_status_t status = WICED_BT_SUCCESS;
    wiced_bt_device_address_t bda = { 0 };
    wiced_bt_dev_ble_pairing_info_t *p_ble_info = NULL;
    wiced_bt_ble_advert_mode_t *p_adv_mode = NULL;

    switch (event)
    {
    case BTM_ENABLED_EVT:
        /* Bluetooth Controller and Host Stack Enabled */

#ifdef HCI_TRACE_OVER_TRANSPORT
        // There is a virtual HCI interface between upper layers of the stack and
        // the controller portion of the chip with lower layers of the BT stack.
        // Register with the stack to receive all HCI commands, events and data.
        wiced_bt_dev_register_hci_trace(advscanner_trace_callback);
#endif

        WICED_BT_TRACE("Bluetooth Enabled (%s)\n\r",
                ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Local Bluetooth Address: [%B]\n\r", bda);

            /* Enables scanning */
            timer_thread = wiced_rtos_create_thread();
            wiced_rtos_init_thread(timer_thread,4,"timer",timer_function,1000,0);
            change_scan_state(BTM_BLE_SCAN_TYPE_HIGH_DUTY);
        }
        /* Initialize the UART for input */
        wiced_hal_puart_init( );
        wiced_hal_puart_flow_off( );
        wiced_hal_puart_set_baudrate( 115200 );

        /* Enable receive and the interrupt */
        wiced_hal_puart_register_interrupt( rx_cback );

        /* Set watermark level to 1 to receive interrupt up on receiving each byte */
        wiced_hal_puart_set_watermark_level( 1 );
        wiced_hal_puart_enable_rx();

        break;
    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        WICED_BT_TRACE("Bluetooth Disabled\n\r");
        break;
    case BTM_SECURITY_REQUEST_EVT:
        /* Security Request */
        WICED_BT_TRACE("Security Request\n\r");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Request for Pairing IO Capabilities (BLE) */
        WICED_BT_TRACE("BLE Pairing IO Capabilities Request\n\r");
        /* No IO Capabilities on this Platform */
        p_event_data->pairing_io_capabilities_ble_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
        p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_BOND|BTM_LE_AUTH_REQ_MITM;
        p_event_data->pairing_io_capabilities_ble_request.max_key_size = 0x10;
        p_event_data->pairing_io_capabilities_ble_request.init_keys = 0;
        p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID;
        break;
    case BTM_PAIRING_COMPLETE_EVT:
        /* Pairing is Complete */
        p_ble_info = &p_event_data->pairing_complete.pairing_complete_info.ble;
        WICED_BT_TRACE("Pairing Complete %d.\n\r", p_ble_info->reason);
        break;
    case BTM_ENCRYPTION_STATUS_EVT:
        /* Encryption Status Change */
        WICED_BT_TRACE("Encryption Status event: bd ( %B ) res %d\n\r", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        WICED_BT_TRACE("Paired Device Link Request Keys Event\n\r");
        /* Device/app-specific TODO: HANDLE PAIRED DEVICE LINK REQUEST KEY - retrieve from NVRAM, etc */
#if 0
        if (advscanner_read_link_keys( &p_event_data->paired_device_link_keys_request ))
        {
            WICED_BT_TRACE("Key Retrieval Success\n\r");
        }
        else
#endif
            /* Until key retrieval implemented above, just fail the request - will cause re-pairing */
        {
            WICED_BT_TRACE("Key Retrieval Failure\n\r");
            status = WICED_BT_ERROR;
        }
        break;
    case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
        /*
         * Request local identity key (get local_identity_keys from NV memory).
         * If successful, return WICED_BT_SUCCESS.
         * Event data: #wiced_bt_local_identity_keys_t
         */
        WICED_BT_TRACE("Local identity key requested\n\r");
        break;
    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        /* Scan State Changed */
        if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_NONE)
            WICED_BT_TRACE("Scanning Disabled\n\r");
        else if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_LOW_DUTY)
            WICED_BT_TRACE("Scanning Enabled (Low Duty)\n\r");
        else if(p_event_data->ble_scan_state_changed == BTM_BLE_SCAN_TYPE_HIGH_DUTY)
            WICED_BT_TRACE("Scanning Enabled (High Duty)\n\r");
        break;
    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
        /* Advertisement State Changed */
        p_adv_mode = &p_event_data->ble_advert_state_changed;
        WICED_BT_TRACE("Advertisement State Change: %d\n\r", *p_adv_mode);
        break;
    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* Pairing request, TODO: handle confirmation of numeric compare here if desired */
        WICED_BT_TRACE("numeric_value: %d\n\r", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;
    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\n\r", event, event);
        break;
    }

    return status;
}

void start_singleline_table()
{
    if(dt_getNumDevices() == 0)
    {
        WICED_BT_TRACE("No devices.\n\r");
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
        wiced_hal_puart_print( "\n\r" );
        printDeviceTableOneLine();
        wiced_hal_puart_print( "\n\r" );
    }
}
void start_multiline_table()
{
    if(dt_getNumDevices() == 0)
    {
        WICED_BT_TRACE("No devices.\n\r");
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
        WICED_BT_TRACE("No beacons.\n\r");
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
        wiced_hal_puart_print( "\n\r" );
        printBeaconTable();
        wiced_hal_puart_print( "\n\r" );
    }
}
void start_recent_table()
{
    if(dt_getNumDevices() == 0)
    {
        WICED_BT_TRACE("No data.\n\r");
        return;
    }
        old_print = print;
    print = WICED_FALSE;
    screen_number = 2;
    printRecentFilterData();
}

//TODO: Add key to print out most recent packet
/* Interrupt callback function for UART */
void rx_cback( void *data )
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
                WICED_BT_TRACE("No data.\n\r");
                break;
            }
            printMostRecentFilterData();
            break;
        case 'f':
            /* Enabled filter */
            if(filter == WICED_FALSE)
                wiced_hal_puart_print( "Filter Enabled\n\r" );
            filter = WICED_TRUE;
            break;
        case 'F':
            /* Disable filter */
            if(filter == WICED_TRUE)
                wiced_hal_puart_print( "Filter Disabled\n\r" );
            filter = WICED_FALSE;
            break;
        case 'p':
            /* Enable printing */
            if(print == WICED_FALSE)
                wiced_hal_puart_print( "Printing Enabled\n\r" );
            print = WICED_TRUE;
            break;
        case 'P':
            /* Disable printing */
            if(print == WICED_TRUE)
                wiced_hal_puart_print( "Printing Disabled\n\r" );
            print = WICED_FALSE;
            break;
        case 'd':
            /* Clear device data */
            timer = 0;
            dt_reset();
            wiced_hal_puart_print( "All device data cleared\n\r" );
            clear_terminal();
            break;
        case 'c':
            /* Clear the terminal */
            clear_terminal();
            break;
        case '?':
            /* Print help */
            wiced_hal_puart_print( "\n\r" );
            wiced_hal_puart_print( "+-------- Available Commands --------+\n\r" );
            wiced_hal_puart_print( "|  s/S    Enable/Disable Scanning    |\n\r" );
            wiced_hal_puart_print( "|  p/P    Enable/Disable Printing    |\n\r" );
            wiced_hal_puart_print( "|  f/F    Enable/Disable Filter      |\n\r" );
            wiced_hal_puart_print( "|  r      Dump Recent Filter Packets |\n\r" );
            wiced_hal_puart_print( "|  R      Print Most Recent Packet   |\n\r" );
            wiced_hal_puart_print( "|  t      Dump Single-line Table     |\n\r" );
            wiced_hal_puart_print( "|  b      Dump Beacon Table          |\n\r" );
            wiced_hal_puart_print( "|  m      Dump Multiline Table       |\n\r" );
            wiced_hal_puart_print( "|  d      Delete All Device Data     |\n\r" );
            wiced_hal_puart_print( "|  c      Clear Screen               |\n\r" );
            wiced_hal_puart_print( "|  ?      Print Commands             |\n\r" );
            wiced_hal_puart_print( "+------------------------------------+\n\r" );
            wiced_hal_puart_print( "\n\r" );
            wiced_hal_puart_print( "+----------- Changing the filter -----------+\n\r" );
            wiced_hal_puart_print( "| 1. Find the device's index in the table.  |\n\r" );
            wiced_hal_puart_print( "| 2. Type the index with less than two      |\n\r" );
            wiced_hal_puart_print( "|    seconds between each key press.        |\n\r" );
            wiced_hal_puart_print( "| 3. To change the filter, wait at least    |\n\r" );
            wiced_hal_puart_print( "|    two seconds and type the new index.    |\n\r" );
            wiced_hal_puart_print( "+-------------------------------------------+\n\r" );
            wiced_hal_puart_print( "\n\r" );
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
            wiced_hal_puart_print( "\n\r" );
            wiced_hal_puart_print( "+------- Available Commands -------+\n\r" );
            wiced_hal_puart_print( "|  <    Increment Page             |\n\r" );
            wiced_hal_puart_print( "|  >    Decrement Page             |\n\r" );
            wiced_hal_puart_print( "|  ESC  Exit Table                 |\n\r" );
            wiced_hal_puart_print( "|  r    Dump Recent Filter Packets |\n\r" );
            wiced_hal_puart_print( "|  t    Dump Single-line Table     |\n\r" );
            wiced_hal_puart_print( "|  b    Dump Beacon Table          |\n\r" );
            wiced_hal_puart_print( "|  m    Dump Multiline Table       |\n\r" );
            wiced_hal_puart_print( "|  ?    Print Commands             |\n\r" );
            wiced_hal_puart_print( "+----------------------------------+\n\r" );
            wiced_hal_puart_print( "\n\r" );
            break;
        }
    }
}

/* Handle Command Received over Transport */
uint32_t hci_control_process_rx_cmd( uint8_t* p_data, uint32_t len )
{
    uint8_t status = 0;
    uint8_t cmd_status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t opcode = 0;
    uint8_t* p_payload_data = NULL;
    uint8_t payload_length = 0;

    WICED_BT_TRACE("hci_control_process_rx_cmd : Data Length '%d'\n\r", len);

    // At least 4 bytes are expected in WICED Header
    if ((NULL == p_data) || (len < 4))
    {
        WICED_BT_TRACE("Invalid Parameters\n\r");
        status = HCI_CONTROL_STATUS_INVALID_ARGS;
    }
    else
    {
        // Extract OpCode and Payload Length from little-endian byte array
        opcode = LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(p_data);
        payload_length = LITTLE_ENDIAN_BYTE_ARRAY_TO_UINT16(&p_data[sizeof(uint16_t)]);
        p_payload_data = &p_data[sizeof(uint16_t)*2];

        // TODO : Process received HCI Command based on its Control Group
        // (see 'hci_control_api.h' for additional details)
        switch ( HCI_CONTROL_GROUP(opcode) )
        {
        default:
            // HCI Control Group was not handled
            cmd_status = HCI_CONTROL_STATUS_UNKNOWN_GROUP;
            wiced_transport_send_data(HCI_CONTROL_EVENT_COMMAND_STATUS, &cmd_status, sizeof(cmd_status));
            break;
        }
    }

    // When operating in WICED_TRANSPORT_UART_HCI_MODE or WICED_TRANSPORT_SPI,
    // application has to free buffer in which data was received
    wiced_transport_free_buffer( p_data );
    p_data = NULL;

    return status;
}

#ifdef HCI_TRACE_OVER_TRANSPORT
/* Handle Sending of Trace over the Transport */
void advscanner_trace_callback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    wiced_transport_send_hci_trace( transport_pool, type, length, p_data );
}
#endif
