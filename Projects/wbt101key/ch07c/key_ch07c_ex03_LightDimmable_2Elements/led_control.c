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
 * Led control functionality
 *
 */

#include "sparcommon.h"

#include "wiced_hal_pwm.h"
#include "wiced_hal_gpio.h"
#include "wiced_hal_aclk.h"
#include "wiced_bt_trace.h"
#include "wiced_platform.h"
#include "led_control.h"

/******************************************************************************
 *                                Constants
 ******************************************************************************/
#define PWM_CHANNELR         PWM0
#define PWM_CHANNELG         PWM1


#define PWM_INP_CLK_IN_HZ   (512*1000)
#define PWM_FREQ_IN_HZ      (10000)

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/
#if ( !defined(CYW20719B1) && !defined(CYW20819A1) )
#define WICED_GPIO_PIN_LED_2 1
#endif
wiced_bt_gpio_numbers_t led_pin_r = LED_RED;
wiced_bt_gpio_numbers_t led_pin_g = LED_GREEN;

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
void led_control_init(void)
{
    pwm_config_t pwm_config;

    /* configure PWM */
#ifdef CYW20719B1
    wiced_hal_pwm_configure_pin(led_pin_r, PWM_CHANNELR);
    wiced_hal_pwm_configure_pin(led_pin_g, PWM_CHANNELG);
#endif

#ifdef CYW20819A1
    wiced_hal_gpio_select_function(LED_RED,   WICED_PWM0);
    wiced_hal_gpio_select_function(LED_GREEN, WICED_PWM1);

#endif
    wiced_hal_aclk_enable(PWM_INP_CLK_IN_HZ, ACLK1, ACLK_FREQ_24_MHZ);
    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, 0, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_start(PWM_CHANNELR, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);
    wiced_hal_pwm_start(PWM_CHANNELG, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);

}

void led_control_set_brighness_level(uint8_t brightness_level, uint8_t element_idx)
{
    pwm_config_t pwm_config;

    WICED_BT_TRACE("set brightness:%d\n", brightness_level);

    // ToDo.  For some reason, setting brightness to 100% does not work well on 20719B1 platform. For now just use 99% instead of 100.
    if (brightness_level == 100)
        brightness_level = 99;

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, brightness_level, PWM_FREQ_IN_HZ, &pwm_config);

    switch(element_idx)
    {
    case 0:
        wiced_hal_pwm_change_values(PWM_CHANNELR, pwm_config.toggle_count, pwm_config.init_count);
    	break;
    case 1:
        wiced_hal_pwm_change_values(PWM_CHANNELG, pwm_config.toggle_count, pwm_config.init_count);
    	break;
    }

}

