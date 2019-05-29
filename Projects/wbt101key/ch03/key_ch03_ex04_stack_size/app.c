#include "app_bt_cfg.h"
#include "sparcommon.h"
#include "wiced_bt_dev.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_bt_stack.h"
#include "wiced_rtos.h"
#include "tx_api.h"

/* Convenient defines for thread sleep times */
#define SLEEP_10MS		(10)
#define SLEEP_100MS		(100)
#define SLEEP_250MS		(250)
#define SLEEP_1000MS	(1000)

/*******************************************************************
 * Function Prototypes
 ******************************************************************/
wiced_bt_dev_status_t  app_bt_management_callback( wiced_bt_management_evt_t event, wiced_bt_management_evt_data_t *p_event_data );

void LED_task(uint32_t );

void button_cback( void *data, uint8_t port_pin );

uint32_t maxStackUsage(TX_THREAD *thread);


/*******************************************************************
 * Global/Static Variables
 ******************************************************************/
wiced_thread_t * LED_control_thread;
wiced_semaphore_t * button_semaphore;


/*******************************************************************************
* Function Name: void application_start(void)
********************************************************************************
* Summary: Entry point to the application. Initialize transport configuration
*          and register BLE management event callback. The actual application
*          initialization will happen when stack reports that BT device is ready
********************************************************************************/
void application_start(void)
{
    	wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    
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
            /* Setup the semaphore */
            button_semaphore = wiced_rtos_create_semaphore();
            wiced_rtos_init_semaphore( button_semaphore );

			/* Configure the button to trigger an interrupt when pressed */
        	wiced_hal_gpio_configure_pin(WICED_GPIO_PIN_BUTTON_1, ( GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_FALLING_EDGE ), GPIO_PIN_OUTPUT_HIGH );
        	wiced_hal_gpio_register_pin_for_interrupt( WICED_GPIO_PIN_BUTTON_1, button_cback, 0 );
			
			LED_control_thread = wiced_rtos_create_thread();
			wiced_rtos_init_thread(
					LED_control_thread,			// Thread handle
					4,                			// Medium Priority
					"LED Task",					// Name
					LED_task,					// Function
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
* Function Name: void button_cback( void *data, uint8_t port_pin )
********************************************************************************/
void button_cback( void *data, uint8_t port_pin )
{
   /* Clear the gpio interrupt */
   wiced_hal_gpio_clear_pin_interrupt_status( port_pin );

   /* Set the semaphore */
   wiced_rtos_set_semaphore( button_semaphore );
}


/*******************************************************************************
* Function Name: void LED_task(uint32_t)
********************************************************************************/
void LED_task( uint32_t arg )
{
    uint32_t size;

	while( 1 )
    {
        wiced_rtos_get_semaphore(button_semaphore, WICED_WAIT_FOREVER);

    	if( GPIO_PIN_OUTPUT_HIGH == wiced_hal_gpio_get_pin_output( WICED_GPIO_PIN_LED_1 ) )
    	{
    		wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_LOW );
    	}
    	else
    	{
    		wiced_hal_gpio_set_pin_output( WICED_GPIO_PIN_LED_1, GPIO_PIN_OUTPUT_HIGH );
    	}

        /* Get max stack usage and print */
    	size = maxStackUsage((TX_THREAD*)LED_control_thread);
    	WICED_BT_TRACE("Max LED Thread Stack Size:%d\n\r",size);
    }
}

/* This function will find the max amount of stack a thread has used */
/* This is ThreadX specific code so it will only work for ThreadX RTOS */
uint32_t maxStackUsage(TX_THREAD *thread)
{
    uint8_t *end =   thread->tx_thread_stack_end;
    uint8_t *start = thread->tx_thread_stack_start;
    while(start < end)
    {
        if(*start != 0xEF)
        {
            return end-start;
        }
        start++;
    }
    return 0;
}
