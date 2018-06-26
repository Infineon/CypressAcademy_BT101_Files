/* Blink LED_1 with a frequency of 2 Hz */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"

/*****************************    Constants   *****************************/
/* Thread will delay for 250ms so that LED frequency will be 500ms = 2 Hz */
/* Comment out the following line to see what happens without the mutex */
#define USE_MUTEX

#define THREAD_DELAY_IN_MS           (250)
#define THREAD2_DELAY_IN_MS          (200)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/*****************************    Variables   *****************************/
#ifdef USE_MUTEX
wiced_mutex_t*  led_mutex;
#endif
wiced_thread_t * led_thread;
wiced_thread_t * led_thread2;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void led_control( uint32_t arg );
void led_control2( uint32_t arg );

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
            /* Setup Mutex */
             #ifdef USE_MUTEX
             led_mutex = wiced_rtos_create_mutex();
             wiced_rtos_init_mutex(led_mutex);
             #endif
            /* Start a thread to control LED blinking with button 1 */
            led_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    led_thread,                     // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Blinky",                       // Name
                    led_control,                    // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            /* Start a second thread to control LED blinking with button 2 */
            led_thread2 = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    led_thread2,                    // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Blinky2",                      // Name
                    led_control2,                   // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );
            break;

        default:
            break;
    }
    return result;
}


/* Thread function to control the LED with button 1 */
void led_control( uint32_t arg )
{
    uint32_t led;

    for(;;)
    {
        #ifdef USE_MUTEX
        wiced_rtos_lock_mutex(led_mutex);
        #endif
        /* Loop while button is pressed */
        while(0 == wiced_hal_gpio_get_pin_input_status( WICED_GPIO_PIN_BUTTON_1 ))
        {
            led = wiced_hal_gpio_get_pin_output( WICED_GPIO_PIN_LED_1 );
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, ! led );

            /* Send the thread to sleep for a period of time */
            wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
        }
        #ifdef USE_MUTEX
        wiced_rtos_unlock_mutex(led_mutex);
        #endif
        /* Yield control when button is not pressed */
        wiced_rtos_delay_milliseconds( 1, ALLOW_THREAD_TO_SLEEP );
    }
}


/* Thread function to control the LED with button 2 */
void led_control2( uint32_t arg )
{
    uint32_t led;

    for(;;)
    {
        #ifdef USE_MUTEX
        wiced_rtos_lock_mutex(led_mutex);
        #endif
        /* Loop while button is pressed */
        while(0 == wiced_hal_gpio_get_pin_input_status( WICED_GPIO_PIN_BUTTON_2 ))
        {
            led = wiced_hal_gpio_get_pin_output( WICED_GPIO_PIN_LED_1 );
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, ! led );

            /* Send the thread to sleep for a period of time */
            wiced_rtos_delay_milliseconds( THREAD2_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
        }
        #ifdef USE_MUTEX
        wiced_rtos_unlock_mutex(led_mutex);
        #endif
        /* Yield control when button is not pressed */
        wiced_rtos_delay_milliseconds( 1, ALLOW_THREAD_TO_SLEEP );
    }
}
