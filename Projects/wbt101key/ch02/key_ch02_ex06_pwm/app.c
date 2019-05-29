#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_hal_pwm.h"
#include "GeneratedSource/cycfg.h"


/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/* The PWM starts at the init value and counts up to 0xFFFF. Then it wraps back around to the init value
 * The output of the PWM starts low and switches high at the toggle value */
/* These values will cause the PWM to count from (0xFFFF - 99) to 0xFFFF (i.e. period = 100)
 * with a duty cycle of 0xFFFF - 49 (i.e. a 50% duty cycle). */
#define PWM_MAX			(0xFFFF)
#define PWM_INIT		(PWM_MAX-99)
#define PWM_TOGGLE		(PWM_MAX-50)

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t  app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void app_task(uint32_t );


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
			/* Configure and start the PWM */
	        wiced_hal_pwm_start( PWM0, LHL_CLK, PWM_TOGGLE, PWM_INIT, 0 );
			
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
* Function Name: void app_task(uint32_t)
********************************************************************************/
void app_task( uint32_t arg )
{
    uint16_t pwmToggle = PWM_TOGGLE;

    while( 1 )
    {
		pwmToggle++;					/* Increase duty cycle by 1% (1 count out of 100) */
		if( pwmToggle == PWM_MAX )		/* Reset to 0% duty cycle once we reach 100% */
		{
			pwmToggle = PWM_INIT;
		}
		wiced_hal_pwm_change_values( PWM0, pwmToggle, PWM_INIT );

		/* Send the thread to sleep for a period of time */
		wiced_rtos_delay_milliseconds( SLEEP_10MS, ALLOW_THREAD_TO_SLEEP );
	}
}
