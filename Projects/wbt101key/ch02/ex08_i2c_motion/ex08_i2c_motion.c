/* Read motion sensor acceleration data and display to the PUART */

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
wiced_thread_t * sensor_thread_handle;

/*****************************    Function Prototypes   *******************/
wiced_result_t bt_cback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );
void sensor_thread( uint32_t arg );

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

            /* Start a thread to read motion sensor values */
            sensor_thread_handle = wiced_rtos_create_thread();       // Get memory for the thread handle
            wiced_rtos_init_thread(
                    sensor_thread_handle,           // Thread handle
                    PRIORITY_MEDIUM,                // Priority
                    "Buttons",                      // Name
                    sensor_thread,                  // Function
                    THREAD_STACK_MIN_SIZE,          // Stack
                    NULL );                         // Function argument
            break;

        default:
            break;
    }
    return result;
}


/* Thread function to read button values from PSoC */
void sensor_thread( uint32_t arg )
{
    /* Thread will delay so that sensor values are read every 100ms */
    #define THREAD_DELAY_IN_MS          (250)

    /* Motion sensor registers and settings */
    #define ACCEL_ADDRESS     (0x6A) /* This is 0xD4 shifted right by 1 */
    #define ACCEL_CONFIG_REG  (0x20)
    #define ACCEL_CONFIG_VAL  (0x40)

    uint8_t accelDataReg = 0x28;

    /* Structure to hold sensor data read from I2C */
    struct
    {
        int16_t ax;
        int16_t ay;
        int16_t az;
    } __attribute__((packed)) accelData;   // I2C read data


    /* TODO: Initialize I2C and set speed to 400kHz */
     wiced_hal_i2c_init();
     wiced_hal_i2c_set_speed( I2CM_SPEED_400KHZ );

     /* TODO: Write to the configuration register */
     /*       You need to send 2 bytes - first the register location and then the register value */
     uint8_t data[] = {ACCEL_CONFIG_REG, ACCEL_CONFIG_VAL};
     wiced_hal_i2c_write( (uint8_t *)data , sizeof( data ), ACCEL_ADDRESS );

     while(1)
     {
        /* TODO: Read the sensor data */
        /*       You need to send a write to the data register followed by a read of 6 bytes into the accelData structure */
        wiced_hal_i2c_write((uint8_t *)&accelDataReg, sizeof(accelDataReg), ACCEL_ADDRESS);
        wiced_hal_i2c_read ((uint8_t *)&accelData,    sizeof(accelData),    ACCEL_ADDRESS);

        WICED_BT_TRACE("Ax=%6d       Ay=%6d       Az=%6d\n\r",accelData.ax,accelData.ay,accelData.az);

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( THREAD_DELAY_IN_MS, ALLOW_THREAD_TO_SLEEP );
    }
}
