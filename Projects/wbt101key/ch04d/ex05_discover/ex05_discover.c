
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_hal_nvram.h"
#include "wiced_hal_gpio.h"
#include "wiced_bt_app_hal_common.h"
#include "wiced_hal_platform.h"
#include "wiced_hal_wdog.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_app_common.h"
#include "sparcommon.h"
#include "string.h"
#include "hci_control_api.h"
#include "wiced_transport.h"
#include "wiced_hal_pspi.h"
#include "wiced_bt_cfg.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_gatt_util.h"
#include "wiced_memory.h"

/*******************************************************************
 * Constant Definitions
 ******************************************************************/
#define TRANS_UART_BUFFER_SIZE  1024
#define TRANS_UART_BUFFER_COUNT 2

/*******************************************************************
 * Variable Definitions
 ******************************************************************/
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];
// Local Device Name in device configuration
// Transport pool for sending RFCOMM data to host
static wiced_transport_buffer_pool_t* transport_pool = NULL;

static uint16_t conn_id=0; // Hold the connection id of the Peripheral .. should be 0 when there is no connection

static const uint8_t serviceUUID[] = {0x01, 0xee, 0x1f, 0x93, 0x07, 0xfc, 0x40, 0x99, 0x99, 0xbc, 0x55, 0x96, 0xec, 0x3a, 0xb9, 0x5b};
static uint16_t serviceStartHandle=1;
static uint16_t serviceEndHandle=0xFFFF;

typedef struct {
    uint16_t startHandle;
    uint16_t endHandle;
    uint16_t valHandle;
    uint16_t cccdHandle;
} charHandle_t;

#define MAX_CHARS_DISCOVERED (10)
static charHandle_t charHandles[MAX_CHARS_DISCOVERED];
static uint32_t charHandleCount;

static const uint8_t ledUUID[] = {0xfd, 0x01, 0xe4, 0x3b, 0xd7, 0x56, 0x4b, 0x2f, 0x98, 0x19, 0xde, 0x07, 0x56, 0x54, 0x32, 0x5d};
static charHandle_t ledChar;

static const uint8_t buttonUUID[] = {0xb1, 0xfa, 0x18, 0x1a, 0xb6, 0xaa, 0x44, 0x40, 0xbf, 0x6f, 0x57, 0xc2, 0x54, 0xb2, 0x33, 0xea};
static charHandle_t buttonChar;


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static void                  ex05_discover_app_init               ( void );
static wiced_bt_dev_status_t ex05_discover_management_callback    ( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
static void                  ex05_discover_reset_device           ( void );
static uint32_t              hci_control_process_rx_cmd          ( uint8_t* p_data, uint32_t len );
#ifdef HCI_TRACE_OVER_TRANSPORT
static void                  ex05_discover_trace_callback         ( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data );
#endif
static void newAdvCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data);
static void                  rx_cback                          ( void *data );
static wiced_bt_gatt_status_t gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data);



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
    wiced_bt_stack_init(ex05_discover_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

/*
 * This function is executed in the BTM_ENABLED_EVT management callback.
 */
void ex05_discover_app_init(void)
{
    /* Initialize Application */
    wiced_bt_app_init();

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);

    /* Initialize the UART for input */
     wiced_hal_puart_init( );
     wiced_hal_puart_flow_off( );
     wiced_hal_puart_set_baudrate( 115200 );
     /* Enable receive and the interrupt */
     wiced_hal_puart_register_interrupt( rx_cback );
     /* Set watermark level to 1 to receive interrupt up on receiving each byte */
     wiced_hal_puart_set_watermark_level( 1 );
     wiced_hal_puart_enable_rx();



}

/* TODO: This function should be called when the device needs to be reset */
void ex05_discover_reset_device( void )
{
    /* TODO: Clear any additional persistent values used by the application from NVRAM */

    // Reset the device
    wiced_hal_wdog_reset_system( );
}

// This function is called when an advertising packet is received.
void newAdvCallback(wiced_bt_ble_scan_results_t *p_scan_result, uint8_t *p_adv_data)
{
    uint8_t len;

    uint8_t *findName = wiced_bt_ble_check_advertising_data ( p_adv_data, BTM_BLE_ADVERT_TYPE_NAME_COMPLETE, &len);
    if(len > 0) /* Found a Name in the advertising packet */
    {
        if((strncmp(findName,"key_pair",len) == 0) || (strncmp(findName,"key_manu",len) == 0)) /* Found name of pairing example */
        {
            WICED_BT_TRACE("Connecting to Device Name with BD Address: [%B]\r\n",p_scan_result->remote_bd_addr);
            wiced_bt_gatt_le_connect(p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BLE_CONN_MODE_HIGH_DUTY, WICED_TRUE);
            wiced_bt_dev_sec_bond(p_scan_result->remote_bd_addr, p_scan_result->ble_addr_type, BT_TRANSPORT_LE, 0, NULL);
            wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE, WICED_TRUE, newAdvCallback);
        }
    }
}

/* Bluetooth Management Event Handler */
wiced_bt_dev_status_t ex05_discover_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
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
        wiced_bt_dev_register_hci_trace(ex05_discover_trace_callback);
#endif

        WICED_BT_TRACE("Bluetooth Enabled (%s)\r\n",
                ((WICED_BT_SUCCESS == p_event_data->enabled.status) ? "success" : "failure"));

        wiced_bt_gatt_register( gatt_callback );

        if (WICED_BT_SUCCESS == p_event_data->enabled.status)
        {
            /* Bluetooth is enabled */
            wiced_bt_dev_read_local_addr(bda);
            WICED_BT_TRACE("Local Bluetooth Address: [%B]\r\n", bda);

            /* Perform application-specific initialization */
            ex05_discover_app_init();
        }


        break;
    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        WICED_BT_TRACE("Bluetooth Disabled\r\n");
        break;
    case BTM_SECURITY_REQUEST_EVT:
        /* Security Request */
        WICED_BT_TRACE("Security Request\r\n");
        wiced_bt_ble_security_grant(p_event_data->security_request.bd_addr, WICED_BT_SUCCESS);
        break;
    case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
        /* Request for Pairing IO Capabilities (BLE) */
        WICED_BT_TRACE("BLE Pairing IO Capabilities Request\r\n");
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
        WICED_BT_TRACE("Pairing Complete %d.\r\n", p_ble_info->reason);
        break;
    case BTM_ENCRYPTION_STATUS_EVT:
        /* Encryption Status Change */
        WICED_BT_TRACE("Encryption Status event: bd ( %B ) res %d\n", p_event_data->encryption_status.bd_addr, p_event_data->encryption_status.result);
        break;
    case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* Paired Device Link Keys Request */
        WICED_BT_TRACE("Paired Device Link Request Keys Event\r\n");
        /* Device/app-specific TODO: HANDLE PAIRED DEVICE LINK REQUEST KEY - retrieve from NVRAM, etc */
#if 0
        if (ex05_discover_read_link_keys( &p_event_data->paired_device_link_keys_request ))
        {
            WICED_BT_TRACE("Key Retrieval Success\r\n");
        }
        else
#endif
        /* Until key retrieval implemented above, just fail the request - will cause re-pairing */
        {
            WICED_BT_TRACE("Key Retrieval Failure\r\n");
            status = WICED_BT_ERROR;
        }
        break;
    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:

        break;
    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* Pairing request, TODO: handle confirmation of numeric compare here if desired */
        WICED_BT_TRACE("numeric_value: %d\n", p_event_data->user_confirmation_request.numeric_value);
        wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS , p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_BLE_SCAN_STATE_CHANGED_EVT:
        WICED_BT_TRACE("Scan State Changed\r\n");
        break;

    default:
        WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x (%d)\r\n", event, event);
        break;
    }

    return status;
}

//
//////// readLed
//
wiced_bt_gatt_status_t readLed()
{
    if(conn_id == 0 || ledChar.valHandle == 0)
        return WICED_ERROR;
    wiced_bt_gatt_status_t     status;
    status = wiced_bt_util_send_gatt_read_by_handle(conn_id,ledChar.valHandle);
    return status;
}
// writeLED is a function to send either a 1 or a 0 to the LED Characteristic
// This function will check and see if there is a connection and we know the handle of the LED
// It will then setup a write... and then write
void writeLed(uint8_t val)
{
    if(conn_id == 0 || ledChar.valHandle == 0)
        return;


    wiced_bt_gatt_value_t *p_write = ( wiced_bt_gatt_value_t* )wiced_bt_get_buffer( sizeof( wiced_bt_gatt_value_t ));
    if ( p_write )
    {
        p_write->handle   = ledChar.valHandle;
        p_write->offset   = 0;
        p_write->len      = 1;
        p_write->auth_req = GATT_AUTH_REQ_NONE;
        p_write->value[0] = val;

        wiced_bt_gatt_status_t status = wiced_bt_gatt_send_write ( conn_id, GATT_WRITE, p_write );

        WICED_BT_TRACE("wiced_bt_gatt_send_write 0x%X\r\n", status);

        wiced_bt_free_buffer( p_write );
    }

}

//////////////////////////// Service Discovery Functions ////////////////////////////////

void startServiceDiscovery()
{
    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = 1;
    discovery_param.e_handle = 0xFFFF;
    discovery_param.uuid.len = 16;
    memcpy(&discovery_param.uuid.uu.uuid128,serviceUUID,16);

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover (conn_id, GATT_DISCOVER_SERVICES_BY_UUID, &discovery_param);
    WICED_BT_TRACE("Started Service Discovery 0x%X\r\n",status);
}

// This function starts the discovery of the characteristics associated with the WICED101 Service
void startCharacteristicDiscovery()
{
    charHandleCount = 0;

    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = serviceStartHandle + 1;
    discovery_param.e_handle = serviceEndHandle;

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover (conn_id,     GATT_DISCOVER_CHARACTERISTICS,   &discovery_param);
    WICED_BT_TRACE("Start char Discovery 0x%X\r\n",status);
}

// This function starts the discovery of the Descriptors associated with the Button Characteristic
void startCharDescriptorDiscovery()
{

    WICED_BT_TRACE("Button Start Handle = %X End Handle=%X\r\n",buttonChar.valHandle+1,buttonChar.endHandle);

    wiced_bt_gatt_discovery_param_t discovery_param;
    memset( &discovery_param, 0, sizeof( discovery_param ) );
    discovery_param.s_handle = buttonChar.valHandle+1;
    discovery_param.e_handle = buttonChar.endHandle;

    wiced_bt_gatt_status_t status = wiced_bt_gatt_send_discover (conn_id,     GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS,   &discovery_param);
    WICED_BT_TRACE("Start char descriptor discovery 0x%X\r\n",status);

}


// This function is called when gatt events occur.

wiced_bt_gatt_status_t gatt_callback( wiced_bt_gatt_evt_t event, wiced_bt_gatt_event_data_t *p_data)
{
    wiced_bt_gatt_status_t result = WICED_BT_SUCCESS;

    switch( event )
    {
    case GATT_CONNECTION_STATUS_EVT:
        if ( p_data->connection_status.connected )
        {
            conn_id =  p_data->connection_status.conn_id;
            WICED_BT_TRACE("Connection ID=%d\r\n",conn_id);
        }
        else
        {
            WICED_BT_TRACE(("Disconnected\r\n"));
            conn_id = 0;
        }
        break;

    case GATT_OPERATION_CPLT_EVT:

        // When you get something back from the peripheral... print it out.. and all of its data

        WICED_BT_TRACE("Gatt Event Complete Conn=%d Op=%d status=0x%X Handle=0x%X len=%d Data=",
                p_data->operation_complete.conn_id,
                p_data->operation_complete.op,
                p_data->operation_complete.status,
                p_data->operation_complete.response_data.handle,
                p_data->operation_complete.response_data.att_value.len);

        for(int i=0;i<p_data->operation_complete.response_data.att_value.len;i++)
        {
            WICED_BT_TRACE("%02X ",p_data->operation_complete.response_data.att_value.p_data[i]);
        }
        WICED_BT_TRACE("\r\n");

        break;

    case GATT_DISCOVERY_CPLT_EVT:

        WICED_BT_TRACE("Discovery Complete Type=0x%02Xd\r\n",p_data->discovery_complete.disc_type);

        // Once all characteristics are discovered... you need to setup the end handles
        if(p_data->discovery_complete.disc_type == GATT_DISCOVER_CHARACTERISTICS)
        {
            for(int i=0;i<charHandleCount;i++)
            {
                if(charHandles[i].startHandle == ledChar.startHandle)
                    ledChar.endHandle = charHandles[i].endHandle;

                if(charHandles[i].startHandle == buttonChar.startHandle)
                    buttonChar.endHandle = charHandles[i].endHandle;

                WICED_BT_TRACE("Char Handle = 0x%X end=0x%X\r\n",charHandles[i].startHandle,charHandles[i].endHandle);
            }
        }


        break;
    case GATT_DISCOVERY_RESULT_EVT:

        //////////////// Services Discovery /////////////////
        if(p_data->discovery_result.discovery_type == GATT_DISCOVER_SERVICES_BY_UUID)
        {
            serviceStartHandle = GATT_DISCOVERY_RESULT_SERVICE_START_HANDLE(p_data);
            serviceEndHandle = GATT_DISCOVERY_RESULT_SERVICE_END_HANDLE(p_data);
            WICED_BT_TRACE("Discovered Service Start=%X End=%X\r\n",serviceStartHandle,serviceEndHandle);
        }

        //////////////// Characteristics Discovery /////////////////
        if(p_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTICS)
        {

            charHandles[charHandleCount].startHandle = p_data->discovery_result.discovery_data.characteristic_declaration.handle;
            charHandles[charHandleCount].valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data);
            charHandles[charHandleCount].endHandle = serviceEndHandle;

            WICED_BT_TRACE("Char Handle=0x%X Value Handle=0x%X Len=%d ",
                               charHandles[charHandleCount].startHandle,
                               charHandles[charHandleCount].valHandle,
                               GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data),
                               GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID_LEN(p_data)
                               );

            if(charHandleCount != 0)
            {
                charHandles[charHandleCount-1].endHandle = charHandles[charHandleCount].endHandle-1;
            }
            charHandleCount += 1;

            if(charHandleCount > MAX_CHARS_DISCOVERED-1)
            {
                WICED_BT_TRACE("This is really bad.. we discovered more characteristics than we can save\r\n");
                ASSERT(1==0);
            }

            if(memcmp(ledUUID,GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128(p_data),16) == 0) // If it is the led Characteristic
            {
                ledChar.startHandle = p_data->discovery_result.discovery_data.characteristic_declaration.handle; // No macro for this unfortunately
                ledChar.valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data);
            }

            if(memcmp(buttonUUID,GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128(p_data),16) == 0) // If it is the button Characteristic
            {
                buttonChar.startHandle = p_data->discovery_result.discovery_data.characteristic_declaration.handle; // No macro for this unfortunately
                buttonChar.valHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_VALUE_HANDLE(p_data);
            }

            for(int i=0;i<GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID_LEN(p_data);i++) // Dump the bytes to the screen
            {
                WICED_BT_TRACE("%02X ",GATT_DISCOVERY_RESULT_CHARACTERISTIC_UUID128(p_data)[i]);

            }
            WICED_BT_TRACE("\r\n");

        }

        //////////////// Descriptors Discovery /////////////////
        if(p_data->discovery_result.discovery_type == GATT_DISCOVER_CHARACTERISTIC_DESCRIPTORS)
        {

            WICED_BT_TRACE("Char Descriptor Handle = %X Len=%d ",GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_VALUE_HANDLE(p_data),
                    GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID_LEN(p_data));

            for(int i=0;i<GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID_LEN(p_data);i++)
            {
                WICED_BT_TRACE("%02X ",GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID128(p_data)[i]);
            }
            WICED_BT_TRACE("\r\n");

            if(GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_UUID16(p_data) == UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION)
            {
                buttonChar.cccdHandle = GATT_DISCOVERY_RESULT_CHARACTERISTIC_DESCRIPTOR_VALUE_HANDLE(p_data);
            }
        }

        break;

    default:
        WICED_BT_TRACE(("Unknown GATT Event %d\r\n",event));
        break;
    }

    return result;
}


/* Handle Command Received over Transport */
uint32_t hci_control_process_rx_cmd( uint8_t* p_data, uint32_t len )
{
    uint8_t status = 0;
    uint8_t cmd_status = HCI_CONTROL_STATUS_SUCCESS;
    uint8_t opcode = 0;
    uint8_t* p_payload_data = NULL;
    uint8_t payload_length = 0;

    WICED_BT_TRACE("hci_control_process_rx_cmd : Data Length '%d'\n", len);

    // At least 4 bytes are expected in WICED Header
    if ((NULL == p_data) || (len < 4))
    {
        WICED_BT_TRACE("Invalid Parameters\n");
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
void ex05_discover_trace_callback( wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data )
{
    wiced_transport_send_hci_trace( transport_pool, type, length, p_data );
}
#endif


void rx_cback( void *data )
{
    uint8_t  readbyte;
    uint32_t focus;

    /* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt();

    switch (readbyte)
    {

    case 's':
        WICED_BT_TRACE( "Scan On\n\r" );
        wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_HIGH_DUTY,FALSE,newAdvCallback);
        break;
    case 'S':
        WICED_BT_TRACE( "Scan Off\n\r" );
        wiced_bt_ble_scan(BTM_BLE_SCAN_TYPE_NONE,FALSE,newAdvCallback);
        break;

    case '0':
        WICED_BT_TRACE("LED Off\r\n");
        writeLed(0);
        break;

    case '1':
        WICED_BT_TRACE("LED On\r\n");
        writeLed(1);
        break;

    case 'l':
        if(conn_id > 0 || ledChar.valHandle > 0)
            wiced_bt_util_send_gatt_read_by_handle(conn_id,ledChar.valHandle);
        break;

    case 'n':
        WICED_BT_TRACE("CCCD On %X\r\n",buttonChar.cccdHandle);
        if(conn_id > 0 || buttonChar.cccdHandle > 0)
            wiced_bt_util_set_gatt_client_config_descriptor(conn_id,buttonChar.cccdHandle,1);

        break;

    case 'N':
        WICED_BT_TRACE("CCCD Off\r\n");
        if(conn_id > 0 || buttonChar.cccdHandle > 0)
            wiced_bt_util_set_gatt_client_config_descriptor(conn_id,buttonChar.cccdHandle,0);
        break;

    case 'd':
        wiced_bt_gatt_disconnect(conn_id);
        break;

    case 'q':
        startServiceDiscovery();
        break;

    case 'w':
        startCharacteristicDiscovery();
        break;

    case 'e':
        startCharDescriptorDiscovery();
        break;

    case '?':
        /* Print help */
        WICED_BT_TRACE( "\n\r" );
        WICED_BT_TRACE( "+------- Available Commands -------+\n\r" );
        WICED_BT_TRACE( "|  s    Turn on Scanning           |\n\r" );
        WICED_BT_TRACE( "|  S    Turn off Scanning          |\n\r" );
        WICED_BT_TRACE( "|  0    LED Off                    |\n\r" );
        WICED_BT_TRACE( "|  1    LED On                     |\n\r" );
        WICED_BT_TRACE( "|  l    Read Current LED State     |\n\r" );

        WICED_BT_TRACE( "|  n    CCCD On                    |\n\r" );

        WICED_BT_TRACE( "|  N    CCCD Off                   |\n\r" );
        WICED_BT_TRACE( "|  d    disconnect                 |\n\r" );
        WICED_BT_TRACE( "|  q    Discover Service Handle    |\n\r" );
        WICED_BT_TRACE( "|  w    Discover Char Handles      |\n\r" );
        WICED_BT_TRACE( "|  e    Discover CCCD Handles      |\n\r" );

        WICED_BT_TRACE( "|  ?    Print Commands             |\n\r" );
        WICED_BT_TRACE( "+----------------------------------+\n\r" );
        WICED_BT_TRACE( "\n\r" );
        break;
    }
}
