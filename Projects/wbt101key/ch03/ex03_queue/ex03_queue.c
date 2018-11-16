/* Use a semaphore to control button press to LED toggle */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"

/*****************************    Constants   *****************************/
/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Parameters for the Queue */
#define MESSAGE_SIZE        (4)
#define QUEUE_LENGTH        (10)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/*****************************    Variables   *****************************/
wiced_thread_t * led_thread;
wiced_queue_t * button_queue;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void gpio_interrupt_callback(void *data, uint8_t port_pin);
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
            /* Setup the semaphore */
            button_queue = wiced_rtos_create_queue();
            wiced_rtos_init_queue(button_queue, "queue", MESSAGE_SIZE, QUEUE_LENGTH);

            /* Configure the Button GPIO as an input with a resistive pull up and interrupt on rising edge */
            wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, gpio_interrupt_callback, NULL );
            wiced_hal_gpio_configure_pin( WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE), GPIO_PIN_OUTPUT_HIGH );

            /* Start a thread to control LED blinking */
            led_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    led_thread,                     // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "LED Toggle",                   // Name
                    led_control,                    // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;

        default:
            break;
    }
    return result;
}


void gpio_interrupt_callback(void *data, uint8_t port_pin)
{
   /* Number of times to blink the LED */
    static uint32_t numBlinks = 0;

    /* Clear the gpio interrupt */
   wiced_hal_gpio_clear_pin_interrupt_status( WICED_GPIO_PIN_BUTTON_1 );

   /* Increment number of blinks and push to queue */
   numBlinks++;
   wiced_rtos_push_to_queue( button_queue,  &numBlinks, WICED_NO_WAIT );
}


/* Thread function to control the LED */
void led_control( uint32_t arg )
{
    uint32_t loop;
    uint32_t blinkValue;

    while(1)
    {
        /* Wait for button press semaphore */
        wiced_rtos_pop_from_queue( button_queue, &blinkValue, WICED_WAIT_FOREVER );

        /* Blink the number of times indicated by the queue */
        for( loop = 0; loop < blinkValue; loop++)
        {
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_HIGH );
            wiced_rtos_delay_milliseconds( 250, ALLOW_THREAD_TO_SLEEP );
            wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_2, GPIO_PIN_OUTPUT_LOW );
            wiced_rtos_delay_milliseconds( 250, ALLOW_THREAD_TO_SLEEP );
        }
        /* Wait 1 second between button presses */
        wiced_rtos_delay_milliseconds( 1000, ALLOW_THREAD_TO_SLEEP );
    }
}
