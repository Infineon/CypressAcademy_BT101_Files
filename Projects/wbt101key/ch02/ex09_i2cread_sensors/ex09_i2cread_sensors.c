/* Read sensors on the PSoC every 100ms and display to the PUART */

#include "wiced.h"
#include "wiced_platform.h"
#include "sparcommon.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"

/*****************************    Constants   *****************************/
/* Useful macros for thread priorities */
#define PRIORITY_HIGH               (3)
#define PRIORITY_MEDIUM             (5)
#define PRIORITY_LOW                (7)

/* Sensible stack size for most threads */
#define THREAD_STACK_MIN_SIZE       (500)

/*****************************    Variables   *****************************/
wiced_thread_t * i2c_thread;


/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void i2c_read( uint32_t arg );

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

            /* Start a thread to read button values */
            i2c_thread = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    i2c_thread,                     // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Buttons",                      // Name
                    i2c_read,                       // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;

        default:
            break;
    }
    return result;
}


/* Thread function to read button values from PSoC */
void i2c_read( uint32_t arg )
{
    /* Thread will delay so that sensor values are read every 100ms */
    #define THREAD_DELAY_IN_MS          (100)

    /* Address of the I2C slave on the shield and analog sensor register starting location */
    #define I2C_ADDRESS   (0x42)
    #define SENSOR_REG    (0x07)

    /* Convert float into integer-dot-integer values */
    #define FABS(f)     ((f<0.0)?-f:f)
    #define INTEGER(f)  ((int)f)
    #define FRACTION(f) ((int)((FABS(f)-INTEGER(FABS(f)))*10))

    char i2cOffsetBuf;                 // 1 byte I2C buffer to set the offset

    /* Structure to hold sensor data read from I2C */
    struct
    {
        float temperature;
        float humidity;
        float light;
        float pot;
    } __attribute__((packed)) i2cReadBuf;   // I2C read data

    /* Configure I2C block */
     wiced_hal_i2c_init();
     wiced_hal_i2c_set_speed( I2CM_SPEED_400KHZ );

     /* Write the offset to allow reading of the button register */
     i2cOffsetBuf = SENSOR_REG;
     wiced_hal_i2c_write( &i2cOffsetBuf , sizeof( i2cOffsetBuf ), I2C_ADDRESS );

    for(;;)
    {
        /* Read the sensor data */
        wiced_hal_i2c_read( (char *) &i2cReadBuf , sizeof( i2cReadBuf ), I2C_ADDRESS );

        /* Print out sensor values */
        WICED_BT_TRACE( "Temperature: %3d.%1d C  ",  INTEGER(i2cReadBuf.temperature), FRACTION(i2cReadBuf.temperature) );
        WICED_BT_TRACE( "Humidity: %3d.%1d %%  ",    INTEGER(i2cReadBuf.humidity),    FRACTION(i2cReadBuf.humidity) );
        WICED_BT_TRACE( "Light: %5d.%1d lux  ",      INTEGER(i2cReadBuf.light),       FRACTION(i2cReadBuf.light) );
        WICED_BT_TRACE( "Voltage: %d.%1d V\n\r",     INTEGER(i2cReadBuf.pot),         FRACTION(i2cReadBuf.pot) );

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
