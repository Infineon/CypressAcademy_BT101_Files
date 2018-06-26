/* Display environmental data on the OLED */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_rtos.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_i2c.h"

#include "u8g_arm.h"



/*****************************    Constants   *****************************/
/* Thread will delay for 250ms */
#define THREAD_DELAY_IN_MS          (250)

/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/* I2C addresses of the display and PSoC */
#define I2C_OLED_ADDRESS            (0x3C)
#define I2C_PSOC_ADDRESS            (0x42)

/* I2C offset for environment data in the PSoC */
#define PSOC_ENVDATA_ADDR           (7)

/* Data structure for information returned from PSoC I2C */
typedef struct
{
    float temperature;
    float humidity;
    float light;
    float voltage;
} psoc_envdata_t;

/* Floating point conversion macros */
#define FABS(f)                     ((f<0.0)?-f:f)
#define INTEGER(f)                  ((int)f)
#define FRACTION(f)                 ((int)((FABS(f)-INTEGER(FABS(f)))*10))


/*****************************    Variables   *****************************/
wiced_thread_t * display_thread;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void display( uint32_t arg );

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

            WICED_BT_TRACE("*** ex15_oled ***\r\n");

            /* Start a thread to read data from PSoC and display on the OLED */
            display_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    display_thread,                 // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Display",                      // Name
                    display,                        // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;

        default:
            break;
    }
    return result;
}

/* Simple floating point to string conversion routine */
void ftostr( char *msg, float f )
{
    int digit = INTEGER( f );

    if( digit >= 100000 || digit < -100000 )
    {
        *msg++ = 'N';
        *msg++ = 'a';
        *msg++ = 'N';
        *msg++ = '\0';

        return;
    }

    if( digit < 0 )
    {
        *msg++ = '-';
        digit = -digit;
    }

    if( digit / 10000 )
    {
        *msg++ = '0' + ( ( digit / 10000 ) % 10 );
    }

    if( digit / 1000 )
    {
        *msg++ = '0' + ( ( digit / 1000 ) % 10 );
    }

    if( digit / 100 )
    {
        *msg++ = '0' + ( ( digit / 100 ) % 10 );
    }

    if( digit / 10 )
    {
        *msg++ = '0' + ( ( digit / 10 ) % 10 );
    }

    *msg++ = '0' + ( digit % 10 );
    *msg++ = '.';

    *msg++ = '0' + FRACTION( f );
    *msg = 0;
}


/* Thread function to read environment data from PSoC and display to the OLED */
void display( uint32_t arg )
{
    u8g_t u8g;

    char msg[30];                           // Buffer for the string to print

    uint8_t wbuf[] = { PSOC_ENVDATA_ADDR }; // Write buffer initialized with PSoC environment data offset
    psoc_envdata_t rbuf;                    // Receive buffer (4 floats)

    /* Initialize the OLED */
    u8g_init_wiced_i2c_device(I2C_OLED_ADDRESS);
    u8g_InitComFn(&u8g, &u8g_dev_ssd1306_128x64_i2c, (u8g_com_fnptr)u8g_com_hw_i2c_fn);

    /* Write the offset address for the environment data to the PSoC */
    wiced_hal_i2c_write( wbuf, sizeof( wbuf ), I2C_PSOC_ADDRESS );

    for(;;)
    {
        /* Get latest environment data from the PSoC */
        wiced_hal_i2c_read( (char *)&rbuf, sizeof( psoc_envdata_t ), I2C_PSOC_ADDRESS );

        /* Display data on the OLED */
        u8g_FirstPage( &u8g );
        do
        {
            u8g_DrawFrame( &u8g, 0, 0, 127, 64 );

            u8g_SetFont( &u8g, u8g_font_unifont );
            u8g_SetFontPosTop( &u8g );

            u8g_DrawStr( &u8g, 5, 5, "Temp" );
            ftostr( msg, rbuf.temperature );
            u8g_DrawStr( &u8g, 80, 5, msg );

            u8g_DrawStr( &u8g, 5, 25, "Humidity" );
            ftostr( msg, rbuf.humidity );
            u8g_DrawStr( &u8g, 80, 25, msg );

            u8g_SetFont( &u8g, u8g_font_courR14 );
            u8g_SetFontPosTop( &u8g );

            u8g_DrawStr( &u8g, 5, 45, "Light" );
            ftostr( msg, rbuf.light );
            u8g_DrawStr( &u8g, 80, 45, msg );

        } while( u8g_NextPage( &u8g ) );

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
