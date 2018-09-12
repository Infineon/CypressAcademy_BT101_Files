#include "u8g.h"
#include "wiced.h"
#include "wiced_hal_i2c.h"
#include "wiced_bt_trace.h"

/*
 * This is the number of data bytes that will be sent in one I2C transaction
 * along with 1 1 control byte so the actual number of bytes sent will be this
 * value plus 1. For some hardware there is a limit to this value. For example,
 *  for the 20719, this should be set to 64 or less.
 */
#define MAX_WRITE_CHUNK (64)

void    u8g_init_wiced_i2c_device( uint8_t address);
uint8_t u8g_com_hw_i2c_fn( u8g_t *u8g, uint8_t msg, uint8_t arg_val, void *arg_ptr );

/* Un-define the hard-coded MIPI / SPI definitions in u8g.h */
#undef u8g_Delay
#undef u8g_MicroDelay
#undef u8g_10MicroDelay
