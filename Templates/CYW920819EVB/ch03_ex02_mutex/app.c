#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "GeneratedSource/cycfg.h"


/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t  app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void Fast_LED_task( uint32_t );
void Slow_LED_task( uint32_t );


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
wiced_thread_t * Fast_LED_thread;
wiced_thread_t * Slow_LED_thread;

/* TODO: Declare pointer to a mutex */



/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Entry point to the application. Initialize transport configuration
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
			/* TODO: Create and initialize the mutex */
	
			Fast_LED_thread = wiced_rtos_create_thread();
			wiced_rtos_init_thread(
					Fast_LED_thread,			// Thread handle
					4,                			// Medium Priority 
					"Fast LED Task",			// Name
					Fast_LED_task,				// Function
					1024,						// Stack space for the app_task function to use
					NULL );						// Function argument (not used)
					
			Slow_LED_thread = wiced_rtos_create_thread();
			wiced_rtos_init_thread(
					Slow_LED_thread,			// Thread handle
					4,                			// Medium Priority 
					"Slow LED Task",			// Name
					Slow_LED_task,				// Function
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
* Function Name: void Fast_LED_task(uint32_t)
********************************************************************************/
void Fast_LED_task( uint32_t arg )
{
    while(1)
    {
        /* TODO: Lock Mutex before checking if button is pressed to make sure you have access to the LED */

		
        /* Blink LED only while button is pressed */
        while( 0 == wiced_hal_gpio_get_pin_input_status( WICED_GPIO_PIN_BUTTON_1 ) )
		{
			if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output( WICED_GPIO_PIN_LED_1 ) )
			{
				wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW );
			}
			else
			{
				wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH );
			}

            /* Send the thread to sleep for a period of time */
            wiced_rtos_delay_milliseconds( SLEEP_100MS, ALLOW_THREAD_TO_SLEEP );
        }

        /* TODO: Unlock Mutex when button is not pressed to allow other thread to have access */

		
        /* Yield control when button is not pressed */
        wiced_rtos_delay_milliseconds( 1, ALLOW_THREAD_TO_SLEEP );
	}
}


/*******************************************************************************
* Function Name: void Slow_LED_task(uint32_t)
********************************************************************************/
void Slow_LED_task( uint32_t arg )
{
	while( 1 )
	{
        /* TODO: Lock Mutex to make sure you have access to the LED */

		
        /* Blink LED at a fixed rate */
    	if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output( WICED_GPIO_PIN_LED_1 ) )
    	{
    		wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW );
    	}
    	else
    	{
    		wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH );
    	}

        /* Send the thread to sleep for a period of time */
        wiced_rtos_delay_milliseconds( SLEEP_250MS, ALLOW_THREAD_TO_SLEEP );

        /* TODO: Unlock Mutex when done with the LED to allow other thread to have access */
		
		
	}
}


