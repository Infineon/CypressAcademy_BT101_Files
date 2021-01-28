/*
* Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
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
 * This application supports factory reset via five fast power off cycles:
 *   turn off device in less than 5 seconds after power on and do it five times.
 * It increments the power on counter (persistent in the NVRAM) on each power on
 *   and does factory reset when counter reaches 5. It resets that counter when
 *   device stays in the power on state for longer than 5 seconds.
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
#include "wiced_hal_nvram.h"
#include "led_control.h"
#include "wiced_timer.h"

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
#define MESH_VID                0x0002
#define MESH_CACHE_REPLAY_SIZE  0x0008

/******************************************************
 *          Structures
 ******************************************************/

/******************************************************
 *          Function Prototypes
 ******************************************************/
static void mesh_app_init(wiced_bool_t is_provisioned);
static void mesh_app_message_handler(uint8_t element_idx, uint16_t event, void *p_data);
static void mesh_app_process_set_hsl(uint8_t element_idx, wiced_bt_mesh_light_hsl_status_data_t *p_data);

/******************************************************
 *          Variables Definitions
 ******************************************************/

uint8_t mesh_mfr_name[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MANUFACTURER_NAME]        = { 'C', 'y', 'p', 'r', 'e', 's', 's', 0 };
uint8_t mesh_model_num[WICED_BT_MESH_PROPERTY_LEN_DEVICE_MODEL_NUMBER]            = { 'A', '1', '9', 0 };
uint8_t mesh_prop_fw_version[WICED_BT_MESH_PROPERTY_LEN_DEVICE_FIRMWARE_REVISION] = { '0', '6', '.', '0', '2', '.', '0', '5' }; // this is overwritten during init
uint8_t mesh_system_id[8]                                                         = { 0xbb, 0xb8, 0xa1, 0x80, 0x5f, 0x9f, 0x91, 0x71 };

wiced_bt_mesh_core_config_model_t   mesh_element1_models[] =
{
    WICED_BT_MESH_DEVICE,
    WICED_BT_MESH_MODEL_USER_PROPERTY_SERVER,
    WICED_BT_MESH_MODEL_LIGHT_HSL_SERVER,
};
#define MESH_APP_NUM_MODELS1 (sizeof(mesh_element1_models) / sizeof(wiced_bt_mesh_core_config_model_t))

wiced_bt_mesh_core_config_model_t   mesh_element2_models[] =
{
    WICED_BT_MESH_MODEL_LIGHT_HSL_HUE_SERVER,
};
#define MESH_APP_NUM_MODELS2 (sizeof(mesh_element2_models) / sizeof(wiced_bt_mesh_core_config_model_t))

wiced_bt_mesh_core_config_model_t   mesh_element3_models[] =
{
    WICED_BT_MESH_MODEL_LIGHT_HSL_SATURATION_SERVER,
};
#define MESH_APP_NUM_MODELS3 (sizeof(mesh_element3_models) / sizeof(wiced_bt_mesh_core_config_model_t))

#define MESH_LIGHT_HSL_SERVER_ELEMENT_INDEX             0
#define MESH_LIGHT_HSL_HUE_SERVER_ELEMENT_INDEX         1
#define MESH_LIGHT_HSL_SATURATION_SERVER_ELEMENT_INDEX  2

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


#define MESH_LIGHT_HSL_SERVER_ELEMENT_INDEX   0

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
        .models_num = MESH_APP_NUM_MODELS1,                              // Number of models in the array models
        .models = mesh_element1_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
    },
	{
		.location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
		.default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
		.onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
		.default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
		.range_min = 1,                            						// Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
		.range_max = 0xffff,                   				            // Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
		.move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
		.properties_num = 0,                                            // Number of properties in the array models
		.properties = NULL,                                             // Array of properties in the element.
		.sensors_num = 0,                                               // Number of sensors in the sensor array
		.sensors = NULL,                                                // Array of sensors of that element
		.models_num = MESH_APP_NUM_MODELS2,						        // Number of models in the array models
		.models = mesh_element2_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
	},
	{
		.location = MESH_ELEM_LOC_MAIN,                                 // location description as defined in the GATT Bluetooth Namespace Descriptors section of the Bluetooth SIG Assigned Numbers
		.default_transition_time = MESH_DEFAULT_TRANSITION_TIME_IN_MS,  // Default transition time for models of the element in milliseconds
		.onpowerup_state = WICED_BT_MESH_ON_POWER_UP_STATE_RESTORE,     // Default element behavior on power up
		.default_level = 0,                                             // Default value of the variable controlled on this element (for example power, lightness, temperature, hue...)
		.range_min = 1,                     							// Minimum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
		.range_max = 0xffff,                     						// Maximum value of the variable controlled on this element (for example power, lightness, temperature, hue...)
		.move_rollover = 0,                                             // If true when level gets to range_max during move operation, it switches to min, otherwise move stops.
		.properties_num = 0,                                            // Number of properties in the array models
		.properties = NULL,                                             // Array of properties in the element.
		.sensors_num = 0,                                               // Number of sensors in the sensor array
		.sensors = NULL,                                                // Array of sensors of that element
		.models_num = MESH_APP_NUM_MODELS3,  						    // Number of models in the array models
		.models = mesh_element3_models,                                 // Array of models located in that element. Model data is defined by structure wiced_bt_mesh_core_config_model_t
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
        .receive_window        = 20,                                // Receive Window value in milliseconds supported by the Friend node.
        .cache_buf_len         = 300,                               // Length of the buffer for the cache
        .max_lpn_num           = 4                                  // Max number of Low Power Nodes with established friendship. Must be > 0 if Friend feature is supported.
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
    NULL,			        // attention processing
    NULL,                   // notify period set
    NULL,                   // WICED HCI command
    NULL,                   // LPN sleep
    NULL                    // factory reset
};

uint8_t last_known_brightness = 0;
uint8_t attention_brightness = 0;
uint8_t attention_time = 0;

wiced_timer_t attention_timer;
static void attention_timer_cb(TIMER_PARAM_TYPE arg);

/******************************************************
 *               Function Definitions
 ******************************************************/

// Time in power on state to clear resets counter
#define MESH_APP_FAST_POWER_OFF_TIMEOUT_IN_SECONDS  5
// Number of fast power off to do factory reset
#define MESH_APP_FAST_POWER_OFF_NUM                 5
// Resets counter timer
wiced_timer_t   mesh_app_fast_power_off_timer = { 0 };

// It is called if 5 seconds passed since start - then delete reset NVRAM ID to count power ons(resets) frmom 0
void mesh_app_fast_power_off_timer_cb(uint32_t arg)
{
    wiced_result_t  result;
    uint16_t        id = mesh_application_get_nvram_id_app_start();
    WICED_BT_TRACE("reset: timeout\n");
    wiced_hal_delete_nvram(id, &result);
}

// Does factory reset on fifth fast power off(reset).
void mesh_app_fast_power_off_execute(void)
{
    wiced_result_t  result;
    uint16_t        id;
    uint8_t         cnt;

    // Get the first usable by application NVRAM Identifier
    id = mesh_application_get_nvram_id_app_start();
    // read counter from the NVRAM and increment it. If it doesn't exist then reset counter
    if (sizeof(cnt) != wiced_hal_read_nvram(id, sizeof(cnt), &cnt, &result))
    {
        WICED_BT_TRACE("reset: read_nvram failed. result:%x\n", result);
        cnt = 1;
    }
    else
        cnt++;
    // If counter has reached 5 then delete counter NVRAM ID and do factory reset
    if (cnt >= MESH_APP_FAST_POWER_OFF_NUM)
    {
        WICED_BT_TRACE("reset: user requested factory reset\n");
        wiced_hal_delete_nvram(id, &result);
        mesh_application_factory_reset();
    }
    // Write counter to NVRAM and start 5 sec timer to reset it if it expires
    else if (sizeof(cnt) != wiced_hal_write_nvram(id, sizeof(cnt), &cnt, &result))
    {
        WICED_BT_TRACE("reset: write_nvram failed. result:%x\n", result);
    }
    else if (wiced_init_timer(&mesh_app_fast_power_off_timer, mesh_app_fast_power_off_timer_cb, 0, WICED_SECONDS_TIMER) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("reset: init_timer failed\n");
    }
    else if (wiced_start_timer(&mesh_app_fast_power_off_timer, MESH_APP_FAST_POWER_OFF_TIMEOUT_IN_SECONDS) != WICED_SUCCESS)
    {
        WICED_BT_TRACE("reset: init_timer failed\n");
    }
    else
    {
        WICED_BT_TRACE("reset: cnt:%d\n", cnt);
    }
}

void mesh_app_init(wiced_bool_t is_provisioned)
{
#if 0
    // Set Debug trace level for mesh_models_lib and mesh_provisioner_lib
    wiced_bt_mesh_models_set_trace_level(WICED_BT_MESH_CORE_TRACE_INFO);
#endif
#if 0
    // Set Debug trace level for all modules but Info level for CORE_AES_CCM module
    wiced_bt_mesh_core_set_trace_level(WICED_BT_MESH_CORE_TRACE_FID_ALL, WICED_BT_MESH_CORE_TRACE_DEBUG);
    wiced_bt_mesh_core_set_trace_level(WICED_BT_MESH_CORE_TRACE_FID_CORE_AES_CCM, WICED_BT_MESH_CORE_TRACE_INFO);
#endif
    // Do factory reset on fifth power on(reset) in the provisioned state.
    // User can do it to request factory reset.
    if (is_provisioned)
        mesh_app_fast_power_off_execute();

    wiced_bt_cfg_settings.device_name = (uint8_t *)"Dimmable Light";
    wiced_bt_cfg_settings.gatt_cfg.appearance = APPEARANCE_LIGHT_CEILING;

    mesh_prop_fw_version[0] = 0x30 + (WICED_SDK_MAJOR_VER / 10);
    mesh_prop_fw_version[1] = 0x30 + (WICED_SDK_MAJOR_VER % 10);
    mesh_prop_fw_version[2] = 0x30 + (WICED_SDK_MINOR_VER / 10);
    mesh_prop_fw_version[3] = 0x30 + (WICED_SDK_MINOR_VER % 10);
    mesh_prop_fw_version[4] = 0x30 + (WICED_SDK_REV_NUMBER / 10);
    mesh_prop_fw_version[5] = 0x30 + (WICED_SDK_REV_NUMBER % 10);
    // convert 12 bits of BUILD_NUMMBER to two base64 characters big endian
    mesh_prop_fw_version[6] = wiced_bt_mesh_base64_encode_6bits((uint8_t)(WICED_SDK_BUILD_NUMBER >> 6) & 0x3f);
    mesh_prop_fw_version[7] = wiced_bt_mesh_base64_encode_6bits((uint8_t)WICED_SDK_BUILD_NUMBER & 0x3f);

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
    led_control_init(LED_CONTROL_TYPE_COLOR);

    // Initialize Light Lightness Server and register a callback which will be executed when it is time to change the brightness of the bulb
    wiced_bt_mesh_model_light_hsl_server_init(MESH_LIGHT_HSL_SERVER_ELEMENT_INDEX, mesh_app_message_handler, is_provisioned);

    // Initialize the Property Server.  We do not need to be notified when Property is set, because our only property is readonly
    wiced_bt_mesh_model_property_server_init(MESH_LIGHT_HSL_SERVER_ELEMENT_INDEX, NULL, is_provisioned);
}

/*
 * Process event received from the models library.
 */
void mesh_app_message_handler(uint8_t element_idx, uint16_t event, void *p_data)
{
    switch (event)
    {
    case WICED_BT_MESH_LIGHT_HSL_SET:
        mesh_app_process_set_hsl(element_idx, (wiced_bt_mesh_light_hsl_status_data_t *)p_data);
        break;

    default:
        WICED_BT_TRACE("dimmable light unknown msg:%d\n", event);
        break;
    }
}

/*
 * Command from the level client is received to set the new level
 */
void mesh_app_process_set_hsl(uint8_t element_idx, wiced_bt_mesh_light_hsl_status_data_t *p_status)
{
   WICED_BT_TRACE("mesh HSL srv set level element:%d hue:%d saturation: %d ligthness: %d remaining_time:%d\n",
		element_idx, p_status->present.hue, p_status->present.saturation, p_status->present.lightness, p_status->remaining_time);

	led_control_set_brighness_level(p_status->present.hue, p_status->present.saturation, p_status->present.lightness);
}
