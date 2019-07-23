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
 * Runtime Bluetooth stack configuration parameters
 *
 */

#include "wiced_bt_dev.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"

/* Null-Terminated Local Device Name */
uint8_t BT_LOCAL_NAME[] = { 's','p','p',' ','t','e','s','t','\0' };
const uint16_t BT_LOCAL_NAME_CAPACITY = sizeof(BT_LOCAL_NAME);


/*******************************************************************
 * wiced_bt core stack configuration
 ******************************************************************/
const wiced_bt_cfg_settings_t wiced_bt_cfg_settings =
{
    (uint8_t*)BT_LOCAL_NAME,                                                /**< Local device name (NULL terminated) */
    {0x00, 0x00, 0x00},                                                     /**< Local device class */
    BTM_SEC_IN_AUTHENTICATE | BTM_SEC_OUT_AUTHENTICATE | BTM_SEC_ENCRYPT,   /**< Security requirements mask (BTM_SEC_NONE, or combination of BTM_SEC_IN_AUTHENTICATE, BTM_SEC_OUT_AUTHENTICATE, BTM_SEC_ENCRYPT */
    3,
    /* BR/EDR Scan Configuration */
    {
        BTM_SCAN_TYPE_STANDARD,                                             /**< Inquiry Scan Type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_INTERVAL,                         /**< Inquiry Scan Interval (0 to use default) */
        WICED_BT_CFG_DEFAULT_INQUIRY_SCAN_WINDOW,                           /**< Inquiry Scan Window (0 to use default) */

        BTM_SCAN_TYPE_STANDARD,                                             /**< Page Scan Type (BTM_SCAN_TYPE_STANDARD or BTM_SCAN_TYPE_INTERLACED) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_INTERVAL,                            /**< Page Scan Interval (0 to use default) */
        WICED_BT_CFG_DEFAULT_PAGE_SCAN_WINDOW,                              /**< Page Scan Window (0 to use default) */
    },

    /* BLE scan settings  */
    {
        BTM_BLE_SCAN_MODE_ACTIVE,                                   /**< BLE scan mode ( BTM_BLE_SCAN_MODE_PASSIVE, BTM_BLE_SCAN_MODE_ACTIVE, or BTM_BLE_SCAN_MODE_NONE ) */

        /* Advertisement scan configuration */
        96,                 /**< High duty scan interval */
        48,                 /**< High duty scan window */
        30,                                                         /**< High duty scan duration in seconds ( 0 for infinite ) */

        2048,                /**< Low duty scan interval  */
        48,                  /**< Low duty scan window */
        30,                                                         /**< Low duty scan duration in seconds ( 0 for infinite ) */

        /* Connection scan configuration */
        96,            /**< High duty cycle connection scan interval */
        48,            /**< High duty cycle connection scan window */
        30,                                                         /**< High duty cycle connection duration in seconds ( 0 for infinite ) */

        2048,           /**< Low duty cycle connection scan interval */
        48,             /**< Low duty cycle connection scan window */
        30,                                                         /**< Low duty cycle connection duration in seconds ( 0 for infinite ) */

        /* Connection configuration */
        WICED_BT_CFG_DEFAULT_CONN_MIN_INTERVAL,                     /**< Minimum connection interval */
        WICED_BT_CFG_DEFAULT_CONN_MAX_INTERVAL,                     /**< Maximum connection interval */
        WICED_BT_CFG_DEFAULT_CONN_LATENCY,                          /**< Connection latency */
        WICED_BT_CFG_DEFAULT_CONN_SUPERVISION_TIMEOUT,              /**< Connection link supervision timeout */
    },

    /* BLE advertisement settings */
    {
        BTM_BLE_ADVERT_CHNL_37 |                                    /**< Advertising channel map ( mask of BTM_BLE_ADVERT_CHNL_37, BTM_BLE_ADVERT_CHNL_38, BTM_BLE_ADVERT_CHNL_39 ) */
        BTM_BLE_ADVERT_CHNL_38 |
        BTM_BLE_ADVERT_CHNL_39,

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MIN_INTERVAL,            /**< High duty undirected connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_ADV_MAX_INTERVAL,            /**< High duty undirected connectable maximum advertising interval */
        30,                                                         /**< High duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MIN_INTERVAL,             /**< Low duty undirected connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_ADV_MAX_INTERVAL,             /**< Low duty undirected connectable maximum advertising interval */
        60,                                                         /**< Low duty undirected connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MIN_INTERVAL,   /**< High duty directed connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_DIRECTED_ADV_MAX_INTERVAL,   /**< High duty directed connectable maximum advertising interval */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MIN_INTERVAL,    /**< Low duty directed connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_DIRECTED_ADV_MAX_INTERVAL,    /**< Low duty directed connectable maximum advertising interval */
        30,                                                         /**< Low duty directed connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MIN_INTERVAL,    /**< High duty non-connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_HIGH_DUTY_NONCONN_ADV_MAX_INTERVAL,    /**< High duty non-connectable maximum advertising interval */
        30,                                                         /**< High duty non-connectable advertising duration in seconds ( 0 for infinite ) */

        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MIN_INTERVAL,     /**< Low duty non-connectable minimum advertising interval */
        WICED_BT_CFG_DEFAULT_LOW_DUTY_NONCONN_ADV_MAX_INTERVAL,     /**< Low duty non-connectable maximum advertising interval */
        0                                                           /**< Low duty non-connectable advertising duration in seconds ( 0 for infinite ) */
    },

    /* GATT Configuration */
    {
        APPEARANCE_GENERIC_COMPUTER,                                /**< GATT appearance ( see gatt_appearance_e ) */
        3,                                                          /**< Client config: maximum number of servers that local client can connect to  */
        1,                                                          /**< Server config: maximum number of remote clients connections allowed by the local */
        512
    },

    /* RFCOMM Configuration */
    {
        2,                                                          /**< Maximum Number of simultaneous RFCOMM ports */
        2,                                                          /**< Maximum Number of simultaneous RFCOMM connections */
    },

    /* Application managed l2cap protocol configuration */
    {
        0,                                                          /**< Maximum number of application-managed l2cap links (BR/EDR and LE) */
        /* BR EDR l2cap configuration */
        0,                                                          /**< Maximum number of application-managed BR/EDR PSMs */
        0,                                                          /**< Maximum number of application-managed BR/EDR channels  */
        /* LE L2cap connection-oriented channels configuration */
        0,                                                          /**< Maximum number of application-managed LE PSMs */
        0,                                                          /**< Maximum number of application-managed LE channels */
    },

    /* Audio/Video Distribution configuration */
    {
        0,                                                          /**< Maximum simultaneous audio/video links */
    },

    /* Audio/Video Remote Control configuration */
    {
        0,                                                          /**< Mask of local roles supported (AVRC_CONN_INITIATOR|AVRC_CONN_ACCEPTOR) */
        0                                                           /**< Maximum simultaneous remote control links */
    },

    /* LE Address Resolution DB size  */
    5,                                                               /**< LE Address Resolution DB settings - effective only for pre 4.2 controller*/
#ifdef CYW20706A2
    517,                                                              /**< Maximum MTU size for GATT connections, should be between 23 and (max_attr_len + 5 )*/
    12
#else
    /* Maximum number of buffer pools */
    6,                                                              /**< Maximum number of buffer pools in p_btm_cfg_buf_pools and by wiced_create_pool */

    /* Interval of  random address refreshing */
    WICED_BT_CFG_DEFAULT_RANDOM_ADDRESS_NEVER_CHANGE              /**< Interval of  random address refreshing - secs */
#endif
};

/*******************************************************************
 * wiced_bt_stack buffer pool configuration
 *
 * Configure buffer pools used by the stack
 *
 * Pools must be ordered in increasing buf_size.
 * If a pools runs out of buffers, the next pool will be used.
 ******************************************************************/

const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS] =
{
/*  { buf_size, buf_count, }, */
    { 64,       16,        }, /* Small Buffer Pool */
#ifdef CYW20819A1
    { 360,      10,         }, /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 1056,     3   },      /* Large Buffer Pool  (used for HCI ACL messages) */
#else
    { 360,      40,         }, /* Medium Buffer Pool (used for HCI & RFCOMM control messages, min recommended size is 360) */
    { 1056,     6,         }, /* Large Buffer Pool  (used for HCI ACL messages) */
#endif
    { 1056,     1,         }, /* Extra Large Buffer Pool (used for SDP Discovery) */

};
