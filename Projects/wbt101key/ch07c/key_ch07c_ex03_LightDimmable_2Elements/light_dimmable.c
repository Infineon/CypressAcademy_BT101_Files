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
 * This demo application shows a simple implementation of a dimmable light.
 * The app is based on the snip/mesh/mesh_light_lightness sample which
 * implements BLE Mesh Light Lightness Server model. Because Light Lightness
 * Server model extends Generic OnOff and Generic Level, the dimmable
 * light can be controlled by a Switch (Generic OnOff Client), a Dimmer
 * (Generic Level Client), or by an application which implements Light
 * Lightness Client.  The WICED Mesh Models library takes care of the
 * translation of the OnOff and Level messages and the only messages
 * that the application layer needs to process is those of the Light
 * Lightness Model.
 *
 * Features demonstrated
 *  - LED usage on the EVK
 *  - Processing of the Light Lightness messages
 *
 * To demonstrate the app, work through the following steps.
 * 1. Build and download the application (to the WICED board)
 * 2. Build and download a controlling application (to another WICED board)
 *    (for example apps/demo/dimmer project)
 * 3. Use Mesh Client or Client Control to provision a light bulb and a dimmer
 * 4. Configure dimmer to control the light bulb.
 *    (note that the bulb and the dimmer were provisioned in the same group,
 *    the dimmer will be automatically configured to send messages to the group
 *    and this step can be skipped.
 * 5. Push/release the application button on the dimmer.  The LED on the light
 *    side should turn on.
 * 6. Push/release the application button on the dimmer.  The LED on the light
 *    side should turn off.
 * 7. Push and hold the application button on the dimmer.  The LED on the light
 *    side should gradually go from off to on within 4 seconds.
 * 8. Push and hold the application button on the dimmer.  The LED on the light
 *    side should gradually go from on to off within 4 seconds.
 * 9. Try pushing and holding button for less than 4 seconds, and all other
 *    combinations.
 *
 */
#include "wiced_bt_ble.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_mesh_models.h"
#include "wiced_bt_trace.h"
#include "wiced_bt_mesh_app.h"
#include "led_control.h"

#ifdef HCI_CONTROL
#include "wiced_transport.h"
#include "hci_control_api.h"
#endif

#include "wiced_bt_cfg.h"
extern wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/******************************************************
 *          Constants
 ******************************************************/
#define MESH_PID                0x311F
#define MESH_VID                0x0001
#define MESH_FWID               0x311F000101010001
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static void mesh_app_attention(uint8_t element_idx, uint8_t time);
static void mesh_app_message_handler(uint8_t element_idx, uint16_t event, void *p_data);
static void mesh_app_process_set_level(uint8_t element_idx, wiced_bt_mesh_light_lightness_status_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/
uint32_t global_element_index = 0;

uint8_t mesh_appearance[WICED_BT_MESH_PROPERTY_LEN_DEVICE_APPEARANCE]             = { BIT16_TO_8(APPEARANCE_LIGHT_CEILING) };
uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME]        = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]            = { 'A', '1', '9', 0 };
uint8_t mesh_prop_fw_version[WICED_BT_MESH_PROPERTY_LEN_DEVICE_FIRMWARE_REVISION] = { '0', '6', '.', '0', '2', '.', '0', '5' }; // this is overwritten during init
uint8_t mesh_system_id[8]                                                         = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_USER_PROPERTY_SERVER,
    WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_SERVER,
};
#define MESH_APP_NUM_MODELS_RED  (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

wiced_bt_mesh_core_config_property_t mesh_element1_properties[] =
{
    {
        .id          = WICED_BT_MESH_PROPERTY_DEVICE_FIRMWARE_REVISION,
        .type        = WICED_BT_MESH_PROPERTY_TYPE_USER,
        .user_access = WICED_BT_MESH_PROPERTY_ID_READABLE,
        .max_len     = WICED_BT_MESH_PROPERTY_LEN_DEVICE_FIRMWARE_REVISION,
        .value       = mesh_prop_fw_version
    },
};
#define MESH_APP_NUM_PROPERTIES (sizeof(mesh_element1_properties) / sizeof(wiced_bt_mesh_core_config_property_t))
#define MESH_LIGHT_LIGHTNESS_SERVER_ELEMENT_INDEX 0

wiced_bt_mesh_core_config_model_t   mesh_element2_models[] =
{
    WICED_BT_MESH_MODEL_LIGHT_LIGHTNESS_SERVER,
};
#define MESH_APP_NUM_MODELS_GREEN  (sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t))


wiced_bt_mesh_core_config_element_t mesh_elements[] =
{
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = MESH_APP_NUM_PROPERTIES,                      // Number of properties in the array models
        .properties = mesh_element1_properties,                         // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = MESH_APP_NUM_MODELS_RED,                          // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
    {
        .location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
        .default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
        .onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
        .default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_min = 1,                                                 // Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .range_max = 0xffff,                                            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
        .move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
        .properties_num = 0,                                            // Number of properties in the array models
        .properties = NULL,                                             // Array of properties in the element.
        .sensors_num = 0,                                               // Number of sensors in the sensor array
        .sensors = NULL,                                                // Array of sensors of that element
        .models_num = MESH_APP_NUM_MODELS_GREEN,                        // Number of models in the array models
        .models = mesh_element2_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
};

wiced_bt_mesh_core_config_t  mesh_config =
{
    .company_id         = MESH_COMPANY_ID_CYPRESS,                  // Company identifier assigned by the Bluetooth SIG
    .product_id         = MESH_PID,                                 // Vendor-assigned product identifier
    .vendor_id          = MESH_VID,                                 // Vendor-assigned product version identifier
    .replay_cache_size  = MESH_CACHE_REPLAY_SIZE,                   // Number of replay protection entries, i.e. maximum number of mesh devices that can send application messages to this device.
    .features           = WICED_BT_MESH_CORE_FEATURE_BIT_FRIEND | WICED_BT_MESH_CORE_FEATURE_BIT_RELAY | WICED_BT_MESH_CORE_FEATURE_BIT_GATT_PROXY_SERVER,   // In Friend mode support friend, relay
    .friend_cfg         =                                           // Configuration of the Friend Feature(Receive Window in Ms, messages cache)
    {
        .receive_window        = 200,                               // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len         = 300                                // Length of the buffer for the cache
    },
    .low_power          =                                           // Configuration of the Low Power Feature
    {
        .rssi_factor           = 0,                                 // contribution of the RSSI measured by the Friend node used in Friend Offer Delay calculations.
        .receive_window_factor = 0,                                 // contribution of the supported Receive Window used in Friend Offer Delay calculations.
        .min_cache_size_log    = 0,                                 // minimum number of messages that the Friend node can store in its Friend Cache.
        .receive_delay         = 0,                                 // Receive delay in 1 ms units to be requested by the Low Power node.
        .poll_timeout          = 0                                  // Poll timeout in 100ms units to be requested by the Low Power node.
    },
    .gatt_client_only          = WICED_FALSE,                       // Can connect to mesh over GATT or ADV
    .elements_num  = (uint8_t)(sizeof(mesh_elements) / sizeof(mesh_elements[0])),   // number of elements on this device
    .elements      = mesh_elements                                  // Array of elements for this device
};

/*
 * Mesh application library will call into application functions if provided by the application.
 */
wiced_bt_mesh_app_func_table_t wiced_bt_mesh_app_func_table =
{
    mesh_app_init,          // application initialization
    NULL,                   // Default SDK platform button processing
    NULL,                   // GATT connection status
    mesh_app_attention,     // attention processing
    NULL,                   // notify period set
    NULL,                   // WICED HCI command
    NULL,                   // LPN sleep
    NULL                    // factory reset
};

uint8_t last_known_brightness[2] = {0};
uint8_t attention_brightness = 0;
uint8_t attention_time = 0;

wiced_timer_t attention_timer;
static void attention_timer_cb(TIMER_PARAM_TYPE arg);

/******************************************************
 *               Function Definitions
 ******************************************************/
void mesh_app_init(wiced_bool_t is_provisioned)
{
#if 0
    extern uint8_t wiced_bt_mesh_model_trace_enabled;
    wiced_bt_mesh_model_trace_enabled = WICED_TRUE;
#endif
	wiced_bt_cfg_settings.device_name = (uint8_t *)"2 Elements Key";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_LIGHT_CEILING;
    mesh_prop_fw_version[0] = 0x30 + (WICED_SDK_MAJOR_VER / 10);
    mesh_prop_fw_version[1] = 0x30 + (WICED_SDK_MAJOR_VER % 10);
    mesh_prop_fw_version[2] = 0x30 + (WICED_SDK_MINOR_VER / 10);
    mesh_prop_fw_version[3] = 0x30 + (WICED_SDK_MINOR_VER % 10);
    mesh_prop_fw_version[4] = 0x30 + (WICED_SDK_REV_NUMBER / 10);
    mesh_prop_fw_version[5] = 0x30 + (WICED_SDK_REV_NUMBER % 10);
    mesh_prop_fw_version[6] = 0x30 + (WICED_SDK_BUILD_NUMBER / 10);
    mesh_prop_fw_version[7] = 0x30 + (WICED_SDK_BUILD_NUMBER % 10);
	
	 // Adv Data is fixed. Spec allows to put URI, Name, Appearance and Tx Power in the Scan Response Data.
    if (!is_provisioned)
    {
        wiced_bt_ble_advert_elem_t  adv_elem[3];
        uint8_t                     buf[2];
        uint8_t                     num_elem = 0;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_NAME_COMPLETE;
        adv_elem[num_elem].len         = (uint16_t)strlen((const char*)wiced_bt_cfg_settings.device_name);
        adv_elem[num_elem].p_data      = wiced_bt_cfg_settings.device_name;
        num_elem++;

        adv_elem[num_elem].advert_type = BTM_BLE_ADVERT_TYPE_APPEARANCE;
        adv_elem[num_elem].len         = 2;
        buf[0]                         = (uint8_t)wiced_bt_cfg_settings.gatt_cfg.appearance;
        buf[1]                         = (uint8_t)(wiced_bt_cfg_settings.gatt_cfg.appearance >> 8);
        adv_elem[num_elem].p_data      = buf;
        num_elem++;

        wiced_bt_mesh_set_raw_scan_response_data(num_elem, adv_elem);
    }

    led_control_init();

    wiced_init_timer(&attention_timer, attention_timer_cb, global_element_index, WICED_SECONDS_PERIODIC_TIMER);

    // Initialize Light Lightness Server and register a callback which will be executed when it is time to change the brightness of the bulb
    wiced_bt_mesh_model_light_lightness_server_init(RED, mesh_app_message_handler, is_provisioned);
    wiced_bt_mesh_model_light_lightness_server_init(GREEN, mesh_app_message_handler, is_provisioned);

    // Initialize the Property Server.  We do not need to be notified when Property is set, because our only property is readonly
    wiced_bt_mesh_model_property_server_init(MESH_LIGHT_LIGHTNESS_SERVER_ELEMENT_INDEX, NULL, is_provisioned);
}

/*
 * Mesh library requests to alert user for "time" seconds.
 */
void mesh_app_attention(uint8_t element_idx, uint8_t time)
{
    WICED_BT_TRACE("dimmable light attention:%d sec\n", time);

    // If time is zero, stop alerting and restore the last known brightness
    if (time == 0)
    {
        wiced_stop_timer(&attention_timer);
        led_control_set_brighness_level(last_known_brightness[element_idx], element_idx);
        return;
    }
    global_element_index = element_idx;
    wiced_start_timer(&attention_timer, 1);
    attention_time = time;
    attention_brightness = (last_known_brightness[element_idx] != 0) ? 0 : 100;
    led_control_set_brighness_level(attention_brightness, element_idx);
}

/*
 * Attention timer callback is executed every second while user needs to be alerted.
 * Just switch brightness between 0 and 100%
 */
void attention_timer_cb(TIMER_PARAM_TYPE arg)
{
    WICED_BT_TRACE("dimmable light attention timeout:%d\n", attention_time);

    if (--attention_time == 0)
    {
        wiced_stop_timer(&attention_timer);
        led_control_set_brighness_level(last_known_brightness[global_element_index], (uint8_t)arg);
        return;
    }
    attention_brightness = (attention_brightness == 0) ? 100 : 0;
    led_control_set_brighness_level(attention_brightness, (uint8_t)arg);
}

/*
 * Process event received from the models library.
 */
void mesh_app_message_handler(uint8_t element_idx, uint16_t event, void *p_data)
{
    switch (event)
    {
    case WICED_BT_MESH_LIGHT_LIGHTNESS_SET:
        mesh_app_process_set_level(element_idx, (wiced_bt_mesh_light_lightness_status_t *)p_data);
        break;

    default:
        WICED_BT_TRACE("dimmable light unknown msg:%d\n", event);
        break;
    }
}

/*
 * Command from the level client is received to set the new level
 */
void mesh_app_process_set_level(uint8_t element_idx, wiced_bt_mesh_light_lightness_status_t *p_status)
{
    WICED_BT_TRACE("mesh light srv set level element:%d present actual:%d linear:%d remaining_time:%d\n",
        element_idx, p_status->lightness_actual_present, p_status->lightness_linear_present, p_status->remaining_time);

    last_known_brightness[element_idx] = (uint8_t)((uint32_t)p_status->lightness_actual_present * 100 / 65535);
    led_control_set_brighness_level(last_known_brightness[element_idx], element_idx);

    // If we were alerting user, stop it.
    wiced_stop_timer(&attention_timer);
}
