/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
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
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 * SPP Application for 20XXX devices.
 *
 * SPP application uses SPP profile library to establish, terminate, send and receive SPP
 * data over BR/EDR. This sample supports single a single SPP connection.
 *
 * Following compilation flags are important for testing
 *  HCI_TRACE_OVER_TRANSPORT - configures HCI traces to be routed to the WICED HCI interface
 *  WICED_BT_TRACE_ENABLE    - enables WICED_BT_TRACEs.  You can also modify makefile.mk to build
 *                             with _debug version of the library
 *  SEND_DATA_ON_INTERRUPT   - if defined, the app will send 1Meg of data on application button push
 *  SEND_DATA_ON_TIMEOUT     - if enabled, the app will send 4 bytes every second while session is up
 *  LOOPBACK_DATA            - if enabled, the app sends back received data
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download the application to the WICED board
 * 2. Use standard terminal emulation application such as Term Term to open the WICED Peripheral UART, use
 *    baud rate of 115200.
 * 3. Us the computer's 'Add a Bluetooth Device' menu to pair with spp app. That should create an incoming
 *    and outgoing COM ports on your computer, see 'More Bluetooth options'
 * 4. Use application such as Term Term to open the outgoing COM port.
 * 5. By default the spp application sends data on a timer to the peer application.
 * 6. Type any keys on the terminal of the outgoing COM port, the spp application will receive the keys.
 * 7. Press the application button on the WICED board to send 1 MB data to the Windows application.
 *
 * Features demonstrated
 *  - Use of SPP library
 *
 *  Note: This snippet app does not support WICED HCI Control and may use transport only for tracing.
 *  If you route traces to WICED HCI UART, use ClientControl app with baud rate equal to that
 *  set in the wiced_transport_cfg_t structure below (currently set at HCI_UART_DEFAULT_BAUD, i.e. 3 Mbps).
 */

#include "sparcommon.h"
#include "wiced.h"
#include "wiced_gki.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sdp.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_hal_nvram.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_spp.h"
#include "wiced_hci.h"
#include "wiced_timer.h"
#include "wiced_transport.h"
#include "wiced_platform.h"
#include "wiced_memory.h"
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_rfcomm.h"


#define HCI_TRACE_OVER_TRANSPORT            1   // If defined HCI traces are send over transport/WICED HCI interface
//#define SEND_DATA_ON_INTERRUPT              1   // If defined application button causes 1Meg of data to be sent
//#define SEND_DATA_ON_TIMEOUT              1   // If defined application sends 4 bytes of data every second
//#define LOOPBACK_DATA                     1   // If defined application loops back received data

#define WICED_EIR_BUF_MAX_SIZE              264
#define SPP_NVRAM_ID                        WICED_NVRAM_VSID_START

/* Max TX packet to be sent over SPP */
#define MAX_TX_BUFFER                       1017
#define TRANS_MAX_BUFFERS                   10
#define TRANS_UART_BUFFER_SIZE              1024
#define SPP_MAX_PAYLOAD                     1007

#if SEND_DATA_ON_INTERRUPT
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"

#define APP_TOTAL_DATA_TO_SEND             1000000
#define BUTTON_GPIO                         WICED_P30
int     app_send_offset = 0;
uint8_t app_send_buffer[SPP_MAX_PAYLOAD];
#endif

#ifdef SEND_DATA_ON_TIMEOUT
wiced_timer_t spp_app_timer;
void app_timeout(uint32_t count);
#endif

/*****************************************************************************
**  Structures
*****************************************************************************/
#if defined (CYW20706A2)
#define SPP_RFCOMM_SCN               2
#else
#define SPP_RFCOMM_SCN               1
#endif


static void         spp_connection_up_callback(uint16_t handle, uint8_t* bda);
static void         spp_connection_down_callback(uint16_t handle);
static wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len);

wiced_bt_spp_reg_t spp_reg =
{
    SPP_RFCOMM_SCN,                     /* RFCOMM service channel number for SPP connection */
    MAX_TX_BUFFER,                      /* RFCOMM MTU for SPP connection */
    spp_connection_up_callback,         /* SPP connection established */
    NULL,                               /* SPP connection establishment failed, not used because this app never initiates connection */
    NULL,                               /* SPP service not found, not used because this app never initiates connection */
    spp_connection_down_callback,       /* SPP connection disconnected */
    spp_rx_data_callback,               /* Data packet received */
};

wiced_transport_buffer_pool_t*  host_trans_pool;
uint16_t                        spp_handle;
wiced_timer_t                   app_tx_timer;

const uint8_t app_sdp_db[] = // Define SDP database
{
    SDP_ATTR_SEQUENCE_2(142),
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes
        SDP_ATTR_RECORD_HANDLE(0x10003),                                    // 8 bytes
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_SERIAL_PORT),                      // 8
        SDP_ATTR_RFCOMM_PROTOCOL_DESC_LIST( SPP_RFCOMM_SCN ),               // 17 bytes
        SDP_ATTR_BROWSE_LIST,                                               // 8
        SDP_ATTR_PROFILE_DESC_LIST(UUID_SERVCLASS_SERIAL_PORT, 0x0102),     // 13 byte
        SDP_ATTR_SERVICE_NAME(10),                                          // 15
        'S', 'P', 'P', ' ', 'S', 'E', 'R', 'V', 'E', 'R',

    // Device ID service
    SDP_ATTR_SEQUENCE_1(69),                                                // 2 bytes, length of the record
        SDP_ATTR_RECORD_HANDLE(0x10002),                                    // 8 byte
        SDP_ATTR_CLASS_ID(UUID_SERVCLASS_PNP_INFORMATION),                  // 8
        SDP_ATTR_PROTOCOL_DESC_LIST(1),                                     // 18
        SDP_ATTR_UINT2(ATTR_ID_SPECIFICATION_ID, 0x103),                    // 6
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID, 0x0f),                            // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_ID, 0x0401),                         // 6
        SDP_ATTR_UINT2(ATTR_ID_PRODUCT_VERSION, 0x0001),                    // 6
        SDP_ATTR_BOOLEAN(ATTR_ID_PRIMARY_RECORD, 0x01),                     // 5
        SDP_ATTR_UINT2(ATTR_ID_VENDOR_ID_SOURCE, DI_VENDOR_ID_SOURCE_BTSIG) // 6
};

// Length of the SDP database
const uint16_t app_sdp_db_len = sizeof(app_sdp_db);

uint8_t pincode[4] = { 0x30, 0x30, 0x30, 0x30 };

extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;
extern const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS];

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
const wiced_transport_cfg_t transport_cfg =
{
    .type = WICED_TRANSPORT_UART,
    .cfg =
    {
        .uart_cfg =
        {
            .mode = WICED_TRANSPORT_UART_HCI_MODE,
            .baud_rate =  HCI_UART_DEFAULT_BAUD
        },
    },
    .rx_buff_pool_cfg =
    {
        .buffer_size  = TRANS_UART_BUFFER_SIZE,
        .buffer_count = 1
    },
    .p_status_handler    = NULL,
    .p_data_handler      = NULL,
    .p_tx_complete_cback = NULL
};
#endif

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
static wiced_bt_dev_status_t app_management_callback (wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data);
static void                  app_write_eir(void);
static int                   app_write_nvram(int nvram_id, int data_len, void *p_data);
static int                   app_read_nvram(int nvram_id, void *p_data, int data_len);

#if SEND_DATA_ON_INTERRUPT
static void                  app_tx_ack_timeout(uint32_t param);
static void                  app_interrupt_handler(void *data, uint8_t port_pin);
#endif
#ifdef HCI_TRACE_OVER_TRANSPORT
static void                  app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data);
#endif


extern void     wiced_bt_trace_array( const char *string, const uint8_t* array, const uint16_t len );
#if defined (CYW20706A2)
extern BOOL32 wiced_hal_puart_select_uart_pads(UINT8 rxdPin, UINT8 txdPin, UINT8 ctsPin, UINT8 rtsPin);
extern wiced_result_t wiced_bt_app_init( void );
#endif
/*******************************************************************
 * Function Definitions
 ******************************************************************/

/*
 * Entry point to the application. Set device configuration and start BT
 * stack initialization.  The actual application initialization will happen
 * when stack reports that BT device is ready
 */
APPLICATION_START()
{
    wiced_result_t result;

#if defined WICED_BT_TRACE_ENABLE || defined HCI_TRACE_OVER_TRANSPORT
    wiced_transport_init(&transport_cfg);

    // create special pool for sending data to the MCU
    host_trans_pool = wiced_transport_create_buffer_pool(TRANS_UART_BUFFER_SIZE, TRANS_MAX_BUFFERS);

    // Set the debug uart as WICED_ROUTE_DEBUG_NONE to get rid of prints
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_NONE);

    // Set to PUART to see traces on peripheral uart(puart)
    wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
#if defined (CYW20706A2)
    wiced_hal_puart_select_uart_pads( WICED_PUART_RXD, WICED_PUART_TXD, 0, 0);
#endif

    // Set to HCI to see traces on HCI uart - default if no call to wiced_set_debug_uart()
    // wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_HCI_UART );

    // Use WICED_ROUTE_DEBUG_TO_WICED_UART to send formatted debug strings over the WICED
    // HCI debug interface to be parsed by ClientControl/BtSpy.
    // Note: WICED HCI must be configured to use this - see wiced_trasnport_init(), must
    // be called with wiced_transport_cfg_t.wiced_tranport_data_handler_t callback present
    // wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_WICED_UART);
#endif

    WICED_BT_TRACE("APP Start\n");

    /* Initialize Stack and Register Management Callback */
    // Register call back and configuration with stack
    wiced_bt_stack_init(app_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools);
}

/*
 * SPP application initialization is executed after BT stack initialization is completed.
 */
void application_init(void)
{
    wiced_bt_gatt_status_t gatt_status;
    wiced_result_t         result;

#if defined (CYW20706A2)
    /* Initialize wiced app */
    wiced_bt_app_init();
#endif

#if SEND_DATA_ON_INTERRUPT
#if !defined (CYW20706A2) && !defined (CYW43012C0)
    /* Configure the button available on the platform */
    wiced_platform_register_button_callback( WICED_PLATFORM_BUTTON_1, app_interrupt_handler, NULL, WICED_PLATFORM_BUTTON_RISING_EDGE);
#elif defined (CYW43012C0)
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_PIN_BUTTON, app_interrupt_handler, NULL);
    wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON, WICED_GPIO_BUTTON_SETTINGS, GPIO_PIN_OUTPUT_LOW);
#else
    wiced_hal_gpio_configure_pin( WICED_GPIO_BUTTON, WICED_GPIO_BUTTON_SETTINGS( GPIO_EN_INT_RISING_EDGE ), WICED_GPIO_BUTTON_DEFAULT_STATE );
    wiced_hal_gpio_register_pin_for_interrupt(WICED_GPIO_BUTTON, app_interrupt_handler, NULL);
#endif

    // init timer that we will use for the rx data flow control.
    wiced_init_timer(&app_tx_timer, app_tx_ack_timeout, 0, WICED_MILLI_SECONDS_TIMER);
#endif

    app_write_eir();

#if defined (CYW20706A2)
    // Initialize RFCOMM.  We will not be using application buffer pool and will rely on the
    // stack pools configured in the wiced_bt_cfg.c
    wiced_bt_rfcomm_init(MAX_TX_BUFFER, 1);
#endif

    // Initialize SPP library
    wiced_bt_spp_startup(&spp_reg);

#ifdef HCI_TRACE_OVER_TRANSPORT
    // There is a virtual HCI interface between upper layers of the stack and
    // the controller portion of the chip with lower layers of the BT stack.
    // Register with the stack to receive all HCI commands, events and data.
    wiced_bt_dev_register_hci_trace(app_trace_callback);
#endif
    /* create SDP records */
    wiced_bt_sdp_db_init((uint8_t *)app_sdp_db, sizeof(app_sdp_db));

    /* Allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_TRUE, 0);


    // This application will always configure device connectable and discoverable
    wiced_bt_dev_set_discoverability(BTM_GENERAL_DISCOVERABLE,
        wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_interval,
        wiced_bt_cfg_settings.br_edr_scan_cfg.inquiry_scan_window);

    wiced_bt_dev_set_connectability(BTM_CONNECTABLE,
        wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_interval,
        wiced_bt_cfg_settings.br_edr_scan_cfg.page_scan_window);


#if SEND_DATA_ON_TIMEOUT
    /* Starting the app timers, seconds timer and the ms timer  */
    if (wiced_init_timer(&spp_app_timer, app_timeout, 0, WICED_SECONDS_PERIODIC_TIMER) == WICED_SUCCESS)
    {
        wiced_start_timer(&spp_app_timer, 1);
    }
#endif
}

/*
 *  Management callback receives various notifications from the stack
 */
wiced_result_t app_management_callback(wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t                      result = WICED_BT_SUCCESS;
    wiced_bt_dev_status_t               dev_status;
    wiced_bt_dev_pairing_info_t*        p_pairing_info;
    wiced_bt_dev_encryption_status_t*   p_encryption_status;
    int                                 bytes_written, bytes_read;
    wiced_bt_power_mgmt_notification_t* p_power_mgmt_notification;

    WICED_BT_TRACE("app_management_callback %d\n", event);

    switch(event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:
        application_init();
        //WICED_BT_TRACE("Free mem:%d", cfa_mm_MemFreeBytes());
        break;

    case BTM_DISABLED_EVT:
        break;

    case BTM_PIN_REQUEST_EVT:
        WICED_BT_TRACE("remote address= %B\n", p_event_data->pin_request.bd_addr);
        wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,result/*WICED_BT_SUCCESS*/,4, &pincode[0]);
        break;

    case BTM_USER_CONFIRMATION_REQUEST_EVT:
        /* This application always confirms peer's attempt to pair */
        wiced_bt_dev_confirm_req_reply (WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
        break;

    case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
        /* This application supports only Just Works pairing */
        WICED_BT_TRACE("BTM_PAIRING_IO_CAPABILITIES_REQUEST_EVT bda %B\n", p_event_data->pairing_io_capabilities_br_edr_request.bd_addr);
        p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap   = BTM_IO_CAPABILITIES_NONE;
        p_event_data->pairing_io_capabilities_br_edr_request.auth_req       = BTM_AUTH_SINGLE_PROFILE_GENERAL_BONDING_NO;
        break;

    case BTM_PAIRING_COMPLETE_EVT:
        p_pairing_info = &p_event_data->pairing_complete.pairing_complete_info;
        WICED_BT_TRACE("Pairing Complete: %d\n", p_pairing_info->br_edr.status);
        result = WICED_BT_USE_DEFAULT_SECURITY;
        break;

    case BTM_ENCRYPTION_STATUS_EVT:
        p_encryption_status = &p_event_data->encryption_status;
        WICED_BT_TRACE("Encryption Status Event: bd (%B) res %d\n", p_encryption_status->bd_addr, p_encryption_status->result);
        break;

    case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
        /* This application supports a single paired host, we can save keys under the same NVRAM ID overwriting previous pairing if any */
        app_write_nvram(SPP_NVRAM_ID, sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update);
        break;

    case  BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
        /* read existing key from the NVRAM  */
        if (app_read_nvram(SPP_NVRAM_ID, &p_event_data->paired_device_link_keys_request, sizeof(wiced_bt_device_link_keys_t)) != 0)
        {
            result = WICED_BT_SUCCESS;
        }
        else
        {
            result = WICED_BT_ERROR;
            WICED_BT_TRACE("Key retrieval failure\n");
        }
        break;

    case BTM_POWER_MANAGEMENT_STATUS_EVT:
        p_power_mgmt_notification = &p_event_data->power_mgmt_notification;
        WICED_BT_TRACE("Power mgmt status event: bd (%B) status:%d hci_status:%d\n", p_power_mgmt_notification->bd_addr, \
                p_power_mgmt_notification->status, p_power_mgmt_notification->hci_status);
        break;

    default:
        result = WICED_BT_USE_DEFAULT_SECURITY;
        break;
    }
    return result;
}


/*
 *  Prepare extended inquiry response data.  Current version publishes device name and 16bit
 *  SPP service.
 */
void app_write_eir(void)
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t length;
    uint16_t eir_length;

    pBuf = (uint8_t *)wiced_bt_get_buffer(WICED_EIR_BUF_MAX_SIZE);
    WICED_BT_TRACE("hci_control_write_eir %x\n", pBuf);

    if (!pBuf)
    {
        WICED_BT_TRACE("app_write_eir %x\n", pBuf);
        return;
    }

    p = pBuf;

    length = strlen((char *)wiced_bt_cfg_settings.device_name);

    *p++ = length + 1;
    *p++ = BT_EIR_COMPLETE_LOCAL_NAME_TYPE;        // EIR type full name
    memcpy(p, wiced_bt_cfg_settings.device_name, length);
    p += length;

    *p++ = 2 + 1;                                   // Length of 16 bit services
    *p++ = BT_EIR_COMPLETE_16BITS_UUID_TYPE;        // 0x03 EIR type full list of 16 bit service UUIDs
    *p++ = UUID_SERVCLASS_SERIAL_PORT & 0xff;
    *p++ = (UUID_SERVCLASS_SERIAL_PORT >> 8) & 0xff;

    *p++ = 0;                                       // end of EIR Data is 0

    eir_length = (uint16_t) (p - pBuf);

    // print EIR data
    wiced_bt_trace_array("EIR :", pBuf, MIN(p-pBuf, 100));
    wiced_bt_dev_write_eir(pBuf, eir_length);

    return;
}

/*
 * The function invoked on timeout of app seconds timer.
 */
#if SEND_DATA_ON_TIMEOUT
void app_timeout(uint32_t count)
{
    static uint32_t timer_count = 0;
    timer_count++;
    WICED_BT_TRACE("app_timeout: %d\n", timer_count);
    if (spp_handle != 0)
    {
        wiced_bt_spp_send_session_data(spp_handle, (uint8_t *)&timer_count, sizeof(uint32_t));
    }
}
#endif

/*
 * SPP connection up callback
 */
void spp_connection_up_callback(uint16_t handle, uint8_t* bda)
{
    WICED_BT_TRACE("%s handle:%d address:%B\n", __FUNCTION__, handle, bda);
    spp_handle = handle;
}

/*
 * SPP connection down callback
 */
void spp_connection_down_callback(uint16_t handle)
{
    WICED_BT_TRACE("%s handle:%d\n", __FUNCTION__, handle);
    spp_handle = 0;
}

/*
 * Process data received over EA session.  Return TRUE if we were able to allocate buffer to
 * deliver to the host.
 */
wiced_bool_t spp_rx_data_callback(uint16_t handle, uint8_t* p_data, uint32_t data_len)
{
    int i;
//    wiced_bt_buffer_statistics_t buffer_stats[4];

//    wiced_bt_get_buffer_usage (buffer_stats, sizeof(buffer_stats));

//    WICED_BT_TRACE("0:%d/%d 1:%d/%d 2:%d/%d 3:%d/%d\n", buffer_stats[0].current_allocated_count, buffer_stats[0].max_allocated_count,
//                   buffer_stats[1].current_allocated_count, buffer_stats[1].max_allocated_count,
//                   buffer_stats[2].current_allocated_count, buffer_stats[2].max_allocated_count,
//                   buffer_stats[3].current_allocated_count, buffer_stats[3].max_allocated_count);

//    wiced_result_t wiced_bt_get_buffer_usage (&buffer_stats, sizeof(buffer_stats));

    WICED_BT_TRACE("%s handle:%d len:%d %02x-%02x\n", __FUNCTION__, handle, data_len, p_data[0], p_data[data_len - 1]);

#if LOOPBACK_DATA
    wiced_bt_spp_send_session_data(handle, p_data, data_len);
#endif
    return WICED_TRUE;
}

/*
 * Write NVRAM function is called to store information in the NVRAM.
 */
int app_write_nvram(int nvram_id, int data_len, void *p_data)
{
    wiced_result_t  result;
    int             bytes_written = wiced_hal_write_nvram(nvram_id, data_len, (uint8_t*)p_data, &result);

    WICED_BT_TRACE("NVRAM ID:%d written :%d bytes result:%d\n", nvram_id, bytes_written, result);
    return (bytes_written);
}

/*
 * Read data from the NVRAM and return in the passed buffer
 */
int app_read_nvram(int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;
    wiced_result_t  result;

    if (data_len >= sizeof(wiced_bt_device_link_keys_t))
    {
        read_bytes = wiced_hal_read_nvram(nvram_id, sizeof(wiced_bt_device_link_keys_t), p_data, &result);
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d result:%d\n", nvram_id, sizeof(wiced_bt_device_link_keys_t), read_bytes, result);
    }
    return (read_bytes);
}

#if SEND_DATA_ON_INTERRUPT
/*
 * Test function which sends as much data as possible.
 */
void app_send_data(void)
{
    int i;

    while ((spp_handle != 0) && wiced_bt_spp_can_send_more_data(spp_handle) && (app_send_offset != APP_TOTAL_DATA_TO_SEND))
    {
        int bytes_to_send = app_send_offset + SPP_MAX_PAYLOAD < APP_TOTAL_DATA_TO_SEND ? SPP_MAX_PAYLOAD : APP_TOTAL_DATA_TO_SEND - app_send_offset;
        for (i = 0; i < bytes_to_send; i++)
        {
            app_send_buffer[i] = app_send_offset + i;
        }
        wiced_bt_spp_send_session_data(spp_handle, app_send_buffer, bytes_to_send);
        app_send_offset += bytes_to_send;
    }
    // Check if we were able to send everything
    if (app_send_offset < APP_TOTAL_DATA_TO_SEND)
    {
        wiced_start_timer(&app_tx_timer, 100);
    }
    else
    {
        app_send_offset = 0;
    }
}

/*
 * Test function which start sending data.
 */
void app_interrupt_handler(void *data, uint8_t port_pin)
{
    int i;

    WICED_BT_TRACE("gpio_interrupt_handler pin:%d send_offset:%d\n", port_pin, app_send_offset);

     /* Get the status of interrupt on P# */
    if (wiced_hal_gpio_get_pin_interrupt_status(BUTTON_GPIO))
    {
        /* Clear the GPIO interrupt */
        wiced_hal_gpio_clear_pin_interrupt_status(BUTTON_GPIO);
    }
    // If we are already sending data, do nothing
    if (app_send_offset != 0)
        return;

    app_send_data();
}

/*
 * The timeout function is periodically called while we are sending big amount of data
 */
void app_tx_ack_timeout(uint32_t param)
{
    app_send_data();
}
#endif


#ifdef HCI_TRACE_OVER_TRANSPORT
/*
 *  Pass protocol traces up over the transport
 */
void app_trace_callback(wiced_bt_hci_trace_type_t type, uint16_t length, uint8_t* p_data)
{
    wiced_transport_send_hci_trace(host_trans_pool, type, length, p_data);
}
#endif
