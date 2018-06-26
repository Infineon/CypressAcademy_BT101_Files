/* Measure Ambient Light Sensor voltage and send to UART every 500ms */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_adc.h"

/*****************************    Constants   *****************************/
/* ADC pin for Light Sensor */
#define ADC_CHANNEL                 (ADC_INPUT_P10)

/* Thread will delay for 500ms */
#define THREAD_DELAY_IN_MS          (500)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/*****************************    Variables   *****************************/
wiced_thread_t * adc_thread;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void adc_control( uint32_t arg );

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

            wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
            WICED_BT_TRACE( "*** ex11_adc ***\n\r" );

            /* initialize the ADC */
            wiced_hal_adc_init();

            /* Start a thread to control LED blinking */
            adc_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    adc_thread,                     // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "ADC",                          // Name
                    adc_control,                    // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;
        default:
            break;
    }
    return result;
}


/* Thread function to read ADC and copy results to PUART */
void adc_control( uint32_t arg )
{
    int16_t raw_val = 0;
    uint32_t voltage_val = 0;

    for(;;)
    {
        /* measure the voltage(raw count and voltage values) from the specified ADC channel */
        raw_val = wiced_hal_adc_read_raw_sample( ADC_CHANNEL );
        voltage_val = wiced_hal_adc_read_voltage( ADC_CHANNEL );

        /* Print results to the UART */
        WICED_BT_TRACE( "Raw value (counts): %d\t", raw_val );
        WICED_BT_TRACE( "Voltage (in mV): %d\r\n", voltage_val );

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
