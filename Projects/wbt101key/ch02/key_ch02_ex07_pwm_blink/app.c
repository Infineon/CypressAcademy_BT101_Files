#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "wiced_hal_pwm.h"
#include "wiced_hal_aclk.h"
#include "GeneratedSource/cycfg.h"


/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/* The PWM starts at the init value and counts up to 0xFFFF. Then it wraps back around to the init value
 * The output of the PWM starts low and switches high at the toggle value */
/* These values will cause the PWM to count from (0xFFFF - 999) to 0xFFFF (i.e. period = 1000)
 * with a duty cycle of 0xFFFF - 100 (i.e. a 10% duty cycle). */
#define PWM_MAX			(0xFFFF)
#define PWM_INIT		(PWM_MAX-999)
#define PWM_TOGGLE		(PWM_MAX-100)

#define PWM_FREQUENCY	(1000)

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
			wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_LED_1, GPIO_OUTPUT_ENABLE, GPIO_PIN_OUTPUT_LOW);
			wiced_hal_gpio_select_function(WICED_GPIO_PIN_LED_1, WICED_PWM1);
            wiced_hal_aclk_enable( PWM_FREQUENCY, ACLK1, ACLK_FREQ_24_MHZ );
	        wiced_hal_pwm_start( PWM1, PMU_CLK, PWM_TOGGLE, PWM_INIT, 0 );
         }
        break;

    default:
        break;
    }

    return status;
}


