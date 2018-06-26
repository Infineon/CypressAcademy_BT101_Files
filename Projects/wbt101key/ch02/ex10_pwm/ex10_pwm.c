/* Change a PWM duty cycle to change brightness of an LED */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_hal_pwm.h"

/*****************************    Constants   *****************************/
/* Thread will delay for 10ms */
#define THREAD_DELAY_IN_MS          (10)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/* The PWM starts at the init value and counts up to 0xFFFF. Then it wraps back around to the init value
 * The output of the PWM starts low and switches high at the toggle value */
/* These values will cause the PWM to count from (0xFFFF - 99) to 0xFFFF (i.e. period = 100)
 * with a duty cycle of 0xFFFF - 49 (i.e. a 50% duty cycle). */
#define PWM_MAX                     (0xFFFF)
#define PWM_INIT                    (PWM_MAX-99)
#define PWM_TOGGLE                  (PWM_MAX-50)

/*****************************    Variables   *****************************/
wiced_thread_t * led_thread;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void led_control( uint32_t arg );

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

            /* Configure and start the PWM */
            wiced_hal_pwm_configure_pin( WICED_GPIO_PIN_LED_2, PWM1 );
            wiced_hal_pwm_start( PWM1, LHL_CLK, PWM_TOGGLE, PWM_INIT, 0 );

            /* Start a thread to control the PWM */
            led_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    led_thread,                     // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Blinky",                       // Name
                    led_control,                    // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;

        default:
            break;
    }
    return result;
}


/* Thread function to control the LED */
void led_control( uint32_t arg )
{
    uint16_t pwmInit   = PWM_INIT;
    uint16_t pwmToggle = PWM_TOGGLE;

    for(;;)
    {
        pwmToggle++; /* Increase duty cycle by 1% (1 count out of 100) */
        if( pwmToggle == PWM_MAX ) /* Reset to 0% duty cycle once we reach 100% */
        {
            pwmToggle = PWM_INIT;
        }
        wiced_hal_pwm_change_values( PWM1, pwmToggle, pwmInit );

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
