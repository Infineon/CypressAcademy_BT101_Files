#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_rtos.h"
#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "rtc.h"
#include "sparcommon.h"
#include "wiced_hal_puart.h"

#include <stdio.h>

/*****************************   Constants   *****************************/
/* Thread will delay for 100ms */
#define THREAD_DELAY_IN_MS          (250)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/************************   Function Definitions   ************************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void rtc_thread( uint32_t arg );
void rx_cback( void *data );

/*****************************    Variables   *****************************/
wiced_thread_t * rtc_thread_handle;

enum {DISPLAY_TIME, SET_TIME} mode;
uint8_t timeVal[6];
uint8_t position = 0;

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
APPLICATION_START()
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    // Register call back with stack
    wiced_bt_stack_init( bt_cback, NULL, NULL );
}

wiced_result_t bt_cback(
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t result = WICED_SUCCESS;

    switch (event)
    {
    // Bluetooth  stack enabled
    case BTM_ENABLED_EVT:

        /* Configure UART */
        wiced_hal_puart_init( );
        wiced_hal_puart_flow_off( );
        wiced_hal_puart_set_baudrate( 115200 );
        wiced_hal_puart_enable_tx( );
        /* Enable receive and the interrupt */
        wiced_hal_puart_register_interrupt( rx_cback );
        /* Set watermark level to 1 to receive interrupt up on receiving each byte */
        wiced_hal_puart_set_watermark_level( 1 );
        wiced_hal_puart_enable_rx();

        /* Start a thread to handle the RTC */
        rtc_thread_handle = wiced_rtos_create_thread();       // Get memory for the thread handle
        wiced_rtos_init_thread(
                rtc_thread_handle,              // Thread handle
                PRIORITY_MEDIUM,                // Priority
                "RTC",                          // Name
                rtc_thread,                     // Function
                THREAD_STACK_MIN_SIZE,          // Stack
                NULL                            // Function argument
        );

        /* TODO: configure clock/oscillator and initialize RTC */

        break;

    default:
        break;
    }

    return result;
}


/* Thread function to read time data from RTC and display to the UART */
void rtc_thread( uint32_t arg )
{
    RtcTime current_time;
    mode = DISPLAY_TIME;

    char msg[20];
    uint8_t i;

    /* This string will clear the screen and set the cursor to the top left corner */
    uint8_t clearScreen[] = {0x1B, '[', '2', 'J', 0x1B, '[', 'H', 0x00};

    while(1)
    {
        switch(mode)
        {
        case DISPLAY_TIME:
            /* TODO Get the current time from the RTC and save to the "current_time" structure */

            /* Clear the screen and print the time */
            wiced_hal_puart_print(clearScreen);
            wiced_hal_puart_print("Current Time: ");
            snprintf(msg, sizeof(msg), "%02d:%02d:%02d",current_time.hour, current_time.minute, current_time.second);
            wiced_hal_puart_print(msg);
            wiced_hal_puart_print("\n\r");
            wiced_hal_puart_print("Press \"s\" to set the time");
            break;
        case SET_TIME:
            wiced_hal_puart_print(clearScreen);
            wiced_hal_puart_print("Enter 2 Digits each for Hour, Minute, and Second: ");
            for(i=0; i<position; i++) /* Display values entered */
            {
                snprintf(msg, sizeof(msg), "%d",timeVal[i]);
                wiced_hal_puart_print(msg);
            }
            if(position > 5) /* We have a full time value entered */
            {
                position = 0;
                current_time.hour   = (timeVal[0] * 10) + timeVal[1];
                current_time.minute = (timeVal[2] * 10) + timeVal[3];
                current_time.second = (timeVal[4] * 10) + timeVal[5];

                /* TODO: Set the RTC time to the value in the "current_time" structure */

                mode = DISPLAY_TIME;
            }
            break;
        }
        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}


/* Interrupt callback function for UART */
void rx_cback( void *data )
{
    uint8_t  readbyte;

    /* Read one byte from the buffer and (unlike GPIO) reset the interrupt */
    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt();

    if( readbyte == 's' || readbyte == 'S' )
    {
        mode = SET_TIME;
        position = 0;
    }
    if( readbyte >= '0' && readbyte <= '9') /* Collect numeric time values entered */
    {
        if(position < 6)
        {
            timeVal[position] = readbyte-'0'; /* Convert ASCII to a number */
            position++;
        }
    }
}
