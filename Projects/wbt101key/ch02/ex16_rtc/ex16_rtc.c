#include "wiced_bt_dev.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"

#include "wiced_hal_gpio.h"
#include "wiced_platform.h"
#include "wiced_timer.h"

#include "u8g_arm.h"
#include "rtc.h"
#include "sparcommon.h"

#include <stdio.h>

/*****************************   Constants   *****************************/
/* Thread will delay for 100ms */
#define THREAD_DELAY_IN_MS          (100)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/* I2C addresses of the display */
#define I2C_OLED_ADDRESS            (0x3C)

/************************   Function Definitions   ************************/
static wiced_result_t hal_gpio_app_management_cback(
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data
);
void button1_cback( void *data, uint8_t port_pin );
void button2_cback( void *data, uint8_t port_pin );
void display( uint32_t arg );

/*****************************    Variables   *****************************/
wiced_thread_t * display_thread;


/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
APPLICATION_START()
{
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);

    WICED_BT_TRACE("**** hal_rtc_app **** \n\r");

    // Register call back with stack
    wiced_bt_stack_init(hal_gpio_app_management_cback, NULL, NULL);
}

wiced_result_t hal_gpio_app_management_cback(
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data
){
    wiced_result_t result = WICED_SUCCESS;

    WICED_BT_TRACE("hal_gpio_app_management_cback %d\n\r", event);

    switch (event)
    {
    // Bluetooth  stack enabled
    case BTM_ENABLED_EVT:
        WICED_BT_TRACE("**** BTM_ENABLED_EVT **** \n\r");

        /* Configure Button 1 for interrupt */
        wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button1_cback, NULL );
        wiced_hal_gpio_configure_pin(
                WICED_GPIO_PIN_BUTTON_1,
                ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH
        );

        /* Configure Button 2 for interrupt */
        wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_2, button2_cback, NULL );
        wiced_hal_gpio_configure_pin(
                WICED_GPIO_PIN_BUTTON_2,
                ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH
        );
        WICED_BT_TRACE("\n\rUse MB1 to switch between changing minutes/hours/days/months/years\n\r");
        WICED_BT_TRACE("Use MB2 to increment the value\n\r\n\r");

        WICED_BT_TRACE("Changing minute value...\n\r");

        /* Start a thread to read data from PSoC and display on the OLED */
        display_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
        wiced_rtos_init_thread(
                display_thread,                 // Thread handle
                PRIORITY_MEDIUM,                // Priority
                "Display",                      // Name
                display,                        // Function
                THREAD_STACK_MIN_SIZE,          // Stack
                NULL                            // Function argument
        );

        rtcConfig.rtcRefClock = RTC_REF_CLOCK_SRC_32KHZ;
        rtcConfig.oscillatorFrequencykHz = RTC_REF_CLOCK_SRC_32KHZ;
        rtc_init();

        break;

    default:
        break;
    }

    return result;
}

/*
 * Callback for Button 2.
 * Switches between which value (hour, month, day, etc.) is being changed)
 */
uint8_t b2_var = 0;
void button2_cback( void *data, uint8_t port_pin )
{
    RtcTime current_time;
    rtc_getRTCTime(&current_time);

    switch(b2_var){
    case 0:
        current_time.minute++;
        break;
    case 1:
        current_time.hour++;
        break;
    case 2:
        current_time.day++;
        break;
    case 3:
        current_time.month++;
        break;
    case 4:
        current_time.year++;
        break;
    }

    rtc_setRTCTime(&current_time);
}

/*
 * Callback for Button 1.
 * Increments the value that is currently being changed.
 */
void button1_cback( void *data, uint8_t port_pin )
{
    b2_var = (b2_var + 1) % 5;
    switch(b2_var){
    case 0:
        WICED_BT_TRACE("Changing minute value...\n\r");
        break;
    case 1:
        WICED_BT_TRACE("Changing hour value...\n\r");
        break;
    case 2:
        WICED_BT_TRACE("Changing day value...\n\r");
        break;
    case 3:
        WICED_BT_TRACE("Changing month value...\n\r");
        break;
    case 4:
        WICED_BT_TRACE("Changing year value...\n\r");
        break;
    }
}

/* Thread function to read time data from RTC and display to the OLED */
void display( uint32_t arg )
{
    // Buffer for the string to print
    char msg[11];
    RtcTime current_time;

    /* Initialize the OLED */
    u8g_t u8g;
    u8g_init_wiced_i2c_device(I2C_OLED_ADDRESS);
    u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, (u8g_com_fnptr)u8g_com_hw_i2c_fn);

    for(;;)
    {
        /* Get latest time data from the RTC */
        rtc_getRTCTime(&current_time);

        /* Display data on the OLED */
        u8g_FirstPage( &u8g );
        do
        {
            u8g_DrawFrame( &u8g, 0, 0, 127, 64 );

            u8g_SetFont( &u8g, u8g_font_unifont );
            u8g_SetFontPosTop( &u8g );

            snprintf(msg, 9, "%02d:%02d:%02d", current_time.hour, current_time.minute, current_time.second);
            u8g_DrawStr( &u8g, 31, 18, msg );

            snprintf(msg, 11, "%02d/%02d/%04d", current_time.month+1, current_time.day, current_time.year);
            u8g_DrawStr( &u8g, 24, 33, msg );

        } while( u8g_NextPage( &u8g ) );

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
