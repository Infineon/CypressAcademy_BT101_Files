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

#include "wiced_platform.h"
#include "spar_utils.h"

extern wiced_platform_gpio_t platform_gpio_pins[];
extern wiced_platform_led_config_t platform_led[];
extern wiced_platform_button_config_t platform_button[];
extern wiced_platform_gpio_config_t platform_gpio[];
extern size_t led_count;
extern size_t button_count;
extern size_t gpio_count;

extern void wiced_app_hal_init(void );
/* utility functions */

/**
 *  \brief Provide utility function for application to register for cb upon button interrupt
 *
 *  \param [in] button select a button from wiced_platform_button_number_t
 *  \param [in] userfn Provide the call back function
 *  \param [in] userdata Data to be provided with the call back
 *  \param [in] trigger_edge To configure interrupt on rising/falling/dual edge
 *
 *  \return none
 *
 */
void wiced_platform_register_button_callback(wiced_platform_button_number_t button, void (*userfn)(void*, UINT8), void* userdata,
                wiced_platform_button_interrupt_edge_t trigger_edge)
{
    if(button < button_count)
    {
        wiced_hal_gpio_register_pin_for_interrupt(*platform_button[button].gpio, userfn, userdata);
        wiced_hal_gpio_configure_pin(*platform_button[button].gpio, (platform_button[button].config | trigger_edge), platform_button[button].default_state);
    }
}

/**
 *  \brief Return state of the pin when button is pressed
 *
 *  \param [in] button select a button from wiced_platform_button_number_t
 *
 *  \return button pressed value
 *
 */
uint32_t wiced_platform_get_button_pressed_value(wiced_platform_button_number_t button)
{
	return platform_button[button].button_pressed_value;
}

/**
 *  \brief Initialize all the required pins and configure their functionality
 *
 *  \return none
 *
 */
void wiced_platform_init(void)
{
    uint32_t i = 0;

    wiced_app_hal_init();
    /* Configure pins available on the platform with the chosen functionality */
    for (i = 0; i < PLATFORM_GPIO_MAX_PINS; i++)
    {
        wiced_hal_gpio_select_function(platform_gpio_pins[i].gpio_pin, platform_gpio_pins[i].functionality);
    }

    /* Initialize LEDs and turn off by default */
    for (i = 0; i < led_count; i++)
    {
        wiced_hal_gpio_configure_pin(*platform_led[i].gpio, platform_led[i].config, platform_led[i].default_state);
    }

    /* Initialize buttons with the default configuration */
    for (i = 0; i < button_count; i++)
    {
        wiced_hal_gpio_configure_pin(*platform_button[i].gpio, (platform_button[i].config), platform_button[i].default_state);
    }

    /* Initialize GPIOs with the default configuration */
    for (i = 0; i < gpio_count; i++)
    {
        wiced_hal_gpio_configure_pin(*platform_gpio[i].gpio, (platform_gpio[i].config), platform_gpio[i].default_state);
    }

    /* disable watchdog, set up SWD, wait for attach if ENABLE_DEBUG */
    SETUP_APP_FOR_DEBUG_IF_DEBUG_ENABLED();
    BUSY_WAIT_TILL_MANUAL_CONTINUE_IF_DEBUG_ENABLED();
}
