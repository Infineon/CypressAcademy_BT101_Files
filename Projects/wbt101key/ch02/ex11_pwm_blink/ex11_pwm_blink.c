/*
 * Use a PWM to control on/off time of a blinking LED
 * It will be OFF for 800ms, then ON for 200ms
 * so we will need a period of 1000ms and a duty cycle of 20%
 */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_aclk.h"

/*****************************    Constants   *****************************/
/* The PWM starts at the init value and counts up to 0xFFFF. It switches state at the toggle value */
/* These values give us at a period of 1000 and a duty cycle of 20% */
/* A clock of 1 kHz and a period of 1000 means a 1 second period */
#define PWM_MAX                     (0xFFFF)
#define PWM_INIT                    (PWM_MAX-999)
#define PWM_TOGGLE                  (PWM_MAX-800)

/* Frequency is in Hz */
#define CLK_FREQ                    (1000)

/*****************************    Variables   *****************************/

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

/*****************************    Functions   *****************************/
/*  Main application. This just starts the BT stack and provides the callback function.
 *  The actual application initialization will happen when stack reports that BT device is ready. */
APPLICATION_START( )
{
    wiced_bt_stack_init( bt_cback, NULL, NULL );                    // Register BT stack callback
}


/* Callback function for Bluetooth events */
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t result = WICED_SUCCESS;

    switch( event )
    {
        /* BlueTooth stack enabled */
        case BTM_ENABLED_EVT:

            /* Set up the clock, pin and PWM */
            wiced_hal_aclk_enable( CLK_FREQ, ACLK1, ACLK_FREQ_24_MHZ );
            wiced_hal_pwm_configure_pin( WICED_GPIO_PIN_LED_2, PWM1 );
            wiced_hal_pwm_start( PWM1, PMU_CLK, PWM_TOGGLE, PWM_INIT, 0 );

            break;

        default:
            break;
    }
    return result;
}
