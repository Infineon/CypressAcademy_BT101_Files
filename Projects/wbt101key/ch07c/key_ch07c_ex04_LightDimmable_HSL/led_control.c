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
#define PWM_CHANNELB         PWM2

#define PWM_INP_CLK_IN_HZ   (512*1000)
#define PWM_FREQ_IN_HZ      (10000)

/******************************************************************************
 *                          Function Declarations
 ******************************************************************************/
//float Hue_2_RGB( float v1, float v2, float vH ); //Function Hue_2_RGB
void HSL_to_RGB(uint16_t hue, uint16_t sat, uint16_t light, uint8_t* r, uint8_t* g, uint8_t* b);

/******************************************************************************
 *                                Variables Definitions
 ******************************************************************************/

wiced_bt_gpio_numbers_t led_pin_r = LED_RED;
wiced_bt_gpio_numbers_t led_pin_g = LED_GREEN;
wiced_bt_gpio_numbers_t led_pin_b = LED_BLUE;

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
    wiced_hal_pwm_configure_pin(led_pin_b, PWM_CHANNELB);
#endif

#ifdef CYW20819A1
    wiced_hal_gpio_select_function(LED_RED,   WICED_PWM0);
    wiced_hal_gpio_select_function(LED_GREEN, WICED_PWM1);
    wiced_hal_gpio_select_function(LED_BLUE,  WICED_PWM2);
#endif
    wiced_hal_aclk_enable(PWM_INP_CLK_IN_HZ, ACLK1, ACLK_FREQ_24_MHZ);
    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, 0, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_start(PWM_CHANNELR, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);
    wiced_hal_pwm_start(PWM_CHANNELG, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);
    wiced_hal_pwm_start(PWM_CHANNELB, PMU_CLK, pwm_config.toggle_count, pwm_config.init_count, 1);

}

void led_control_set_brightness_level(uint16_t hue, uint16_t saturation, uint16_t lightness)
{
    pwm_config_t pwm_config;

    uint8_t PWMR = 0;
    uint8_t PWMG = 0;
    uint8_t PWMB = 0;

    HSL_to_RGB(hue, saturation, lightness, &PWMR, &PWMG, &PWMB);

    WICED_BT_TRACE("R:%d G:%d B:%d\n", PWMR, PWMG, PWMB);

    // Setting brightness to 100% does not work well on 20719B1 platform. For now just use 99% instead of 100.
    if (PWMR == 100)
    	PWMR = 99;
    if (PWMG == 100)
    	PWMG = 99;
    if (PWMB == 100)
    	PWMB = 99;

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, PWMR, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_change_values(PWM_CHANNELR, pwm_config.toggle_count, pwm_config.init_count);

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, PWMG, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_change_values(PWM_CHANNELG, pwm_config.toggle_count, pwm_config.init_count);

    wiced_hal_pwm_get_params(PWM_INP_CLK_IN_HZ, PWMB, PWM_FREQ_IN_HZ, &pwm_config);
    wiced_hal_pwm_change_values(PWM_CHANNELB, pwm_config.toggle_count, pwm_config.init_count);
}

/* Convert HSL values to RGB values */
/* Inputs: hue (0-360), sat (0-100), and lightness (0-100) */
/* Outputs: r (0-100), g (0-100), b (0-100) */
void HSL_to_RGB(uint16_t hue, uint16_t sat, uint16_t light, uint8_t* r, uint8_t* g, uint8_t* b)
{
	uint16_t v;

    /* Formulas expect input in the range of 0-255 so need to convert
	   they input ranges which are H: 0-360, S: 0-100, L: 0-100 */
    hue = (hue*255)/360;
    sat = (sat*255)/100;
    light = (light*255)/100;

    v = (light < 128) ? (light * (256 + sat)) >> 8 :
          (((light + sat) << 8) - light * sat) >> 8;
    if (v <= 0) {
        *r = *g = *b = 0;
    } else {
        int m;
        int sextant;
        int fract, vsf, mid1, mid2;

        m = light + light - v;
        hue *= 6;
        sextant = hue >> 8;
        fract = hue - (sextant << 8);
        vsf = v * fract * (v - m) / v >> 8;
        mid1 = m + vsf;
        mid2 = v - vsf;

        // Convert output range of 0-255 to 0-100
        v = (v*100)/255;
        m = (m*100)/255;
        mid1 = (mid1*100)/255;
        mid2 = (mid2*100)/255;

        switch (sextant) {
           case 0: *r = v; *g = mid1; *b = m; break;
           case 1: *r = mid2; *g = v; *b = m; break;
           case 2: *r = m; *g = v; *b = mid1; break;
           case 3: *r = m; *g = mid2; *b = v; break;
           case 4: *r = mid1; *g = m; *b = v; break;
           case 5: *r = v; *g = m; *b = mid2; break;
        }
    }
}

//float Hue_2_RGB( float v1, float v2, float vH ) //Function Hue_2_RGB
//{
//	if ( vH < 0 )
//		{
//			vH += 1;
//		}
//	if ( vH > 1 )
//		{
//			vH -= 1;
//		}
//	if (( 6 * vH ) < 1 )
//	{
//		return ( v1 + ( v2 - v1 ) * 6 * vH );
//	}
//	if (( 2 * vH ) < 1 )
//	{
//		return ( v2 );
//	}
//	if (( 3 * vH ) < 2 )
//	{
//		return ( v1 + ( v2 - v1 ) * ( ( 2/3 ) - vH ) * 6 );
//	}
//	return ( v1 );
//}


