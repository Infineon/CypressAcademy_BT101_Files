#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_rtc.h"
#include "GeneratedSource/cycfg.h"


/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/* Ring buffer for UART receive characters */
#define RING_SIZE	(20)

static uint8_t ring_buffer[RING_SIZE];
static int ring_first = 0;
static int ring_last = 0;

/* Escape character and strings for screen/cursor control */
#define ESCAPE		(0x1B)

static char CLEAR[] = { ESCAPE, '[', '2', 'J', '\0' };
static char HOME[] =  { ESCAPE, '[', 'H', '\0' };


/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t  app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void app_task(uint32_t );
void rx_cback( void* );
uint32_t getDateTimeEntry( void );
void ring_init();
int ring_push( uint8_t ch );
int ring_pop( uint8_t* p_ch );


/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary:  point to the application. Initialize transport configuration
*          and register BLE management event callback. The actual application
*          initialization will happen when stack reports that BT device is ready
********************************************************************************/
void application_start(void)
{
    /* Initialize Stack and Register Management Callback */
    wiced_bt_stack_init( app_bt_management_callback, &wiced_bt_cfg_settings, wiced_bt_cfg_buf_pools );
}


/**************************************************************************************************
* Function Name: wiced_bt_dev_status_t app_bt_management_callback(wiced_bt_management_evt_t event,
*                                                  wiced_bt_management_evt_data_t *p_event_data)
***********************************************************************************************    ***
* Summary:
*   This is a Bluetooth stack management event handler function to receive events from
*   BLE stack and process as per the application.
*
* Parameters:
*   wiced_bt_management_evt_t event             : BLE event code of one byte length
*   wiced_bt_management_evt_data_t *p_event_data: Pointer to BLE management event structures
*
* Return:
*  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
*
***********************************************************************************************/
wiced_result_t app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    switch( event )
    {
    case BTM_ENABLED_EVT:						// Bluetooth Controller and Host Stack Enabled

        if( WICED_BT_SUCCESS == p_event_data->enabled.status )
        {
			/* Turn on the UART for rx and tx */
		    wiced_hal_puart_init( );
		    wiced_hal_puart_flow_off( );
		    wiced_hal_puart_set_baudrate( 115200 );
		    wiced_hal_puart_enable_tx( );

		    /* Enable receive and the interrupt */
		    wiced_hal_puart_register_interrupt( rx_cback );

		    /* Set watermark level to 1 to receive interrupt up on receiving each byte */
		    wiced_hal_puart_set_watermark_level( 1 );
		    wiced_hal_puart_enable_rx();

		    /* TODO: Initialize the RTC */


			
			/* The stack is safely up - create a thread to test out peripherals */
			wiced_thread_t* peripheral_test_thread = wiced_rtos_create_thread();

			wiced_rtos_init_thread(
					peripheral_test_thread,		// Thread handle
					4,                			// Medium Priority
					"App Task",					// Name
					app_task,					// Function
					1024,						// Stack space for the app_task function to use
					NULL );						// Function argument (not used)
        }
        break;

    default:
        break;
    }

    return status;
}


/*******************************************************************************
* Function Name: uint32_t getDateTimeEntry( void )
********************************************************************************/
uint32_t getDateTimeEntry( void )
{
	uint8_t v1, v2;

	/* Get the first of two chars from UART */
	while( ! ring_pop( &v1 ) )
        wiced_rtos_delay_milliseconds( SLEEP_10MS, ALLOW_THREAD_TO_SLEEP );
	wiced_hal_puart_write( v1 );						// Echo

	/* Get the second of two chars from UART */
	while( ! ring_pop( &v2 ) )
        wiced_rtos_delay_milliseconds( SLEEP_10MS, ALLOW_THREAD_TO_SLEEP );
	wiced_hal_puart_write( v2 );						// Echo

	return ( ( ( v1 - '0' ) * 10 ) + ( v2 - '0' ) );	// Convert to a number
}


/*******************************************************************************
* Function Name: void rx_cback( void* arg )
*
* Push a received character into the ring buffer.
********************************************************************************/
void rx_cback( void* arg )
{
	uint8_t readbyte;

    wiced_hal_puart_read( &readbyte );
    wiced_hal_puart_reset_puart_interrupt();

    /* Push the character into the ring buffer */
    if( ! ring_push( readbyte ) )
    	wiced_hal_puart_print( "\r\n\nFULL\r\n" );

}


/*******************************************************************************
* Function Name: void ring_init()
********************************************************************************/
void ring_init()
{
	/* Prepare the two buffer pointers */
	ring_first = 0;
	ring_last = 0;
}


/*******************************************************************************
* Function Name: int ring_push( uint8_t ch )
*
* Push a character into the ring buffer.
* 0 = buffer full, 1 = success
********************************************************************************/
int ring_push( uint8_t ch )
{
	if( ring_first == ( ring_last + 1 ) % RING_SIZE )
	{
		return 0;
	}

	ring_buffer[ring_first] = ch;
	ring_last++;
	ring_last %= RING_SIZE;

	return 1;
}


/*******************************************************************************
* Function Name: int ring_pop( uint8_t* p_ch )
*
* Extract a character from the ring buffer.
* 0 = buffer empty, 1 = success
********************************************************************************/
int ring_pop( uint8_t* p_ch )
{
	if( ring_first == ring_last )
	{
		return 0;
	}

	*p_ch = ring_buffer[ring_first];
	ring_first++;
	ring_first %= RING_SIZE;

	return 1;
}


/*******************************************************************************
* Function Name: void app_task(uint32_t)
********************************************************************************/
void app_task( uint32_t arg )
{
	char timeBuf[25];
	wiced_rtc_time_t rtcTime;
	uint32_t entry;

	/* Start by reading the time/date - then BREAK */
	while( 1 )
	{
		/* Prompt for time/date */
	    wiced_hal_puart_print( CLEAR );
	    wiced_hal_puart_print( HOME );
		wiced_hal_puart_print( "hh:mm yy:mm:dd" );
	    wiced_hal_puart_print( HOME );

	    /* Hour */
		entry = getDateTimeEntry();
		if( entry > 23 )
			continue;
		rtcTime.hour = entry;
		wiced_hal_puart_write( ':' );

		/* Minute */
		entry = getDateTimeEntry();
		if( entry > 59 )
			continue;
		rtcTime.minute = entry;
		wiced_hal_puart_write( ' ' );

		/* Always clear the seconds */
		rtcTime.second = 0;

		/* Year */
		entry = getDateTimeEntry();
		if( entry > 99 )
			continue;
		rtcTime.year = 2000 + entry;		// Year is assumed 20xx
		wiced_hal_puart_write( ':' );

		/* Month */
		entry = getDateTimeEntry();
		if( entry > 11 )
			continue;
		rtcTime.month = entry - 1;			// Month is 0..11 so subtract 1
		wiced_hal_puart_write( ':' );

		/* Day */
		entry = getDateTimeEntry();			// Day is 0..30 so subtract 1
		if( entry > 31 )
			continue;
		rtcTime.day = entry;

		/* TODO: Set the reference time/date */

		
        /* EXIT this loop and start the clock running */
        break;
	}

    wiced_hal_puart_print( "\r\n\nDate and time entered" );
    wiced_rtos_delay_milliseconds( SLEEP_1000MS, ALLOW_THREAD_TO_SLEEP );

    wiced_hal_puart_print( CLEAR );

    while( 1 )
    {
        wiced_hal_puart_print( HOME );

    	/* TODO: Read the time, convert to a string, and print */


        /* Send the thread to sleep for a specified number of milliseconds */
        wiced_rtos_delay_milliseconds( SLEEP_250MS, ALLOW_THREAD_TO_SLEEP );
    }
}

