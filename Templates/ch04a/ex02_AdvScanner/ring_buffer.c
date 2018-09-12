#include <stdio.h>
#include "wiced.h"
#include "wiced_bt_trace.h"

#include "AdvScanner.h"
#include "print_functions.h"
#include "device_table.h"
#include "ring_buffer.h"

#define RING_BUFFER_SIZE (21)

static ring_struct_t ring_buffer[RING_BUFFER_SIZE];
static uint8_t ring_start = 0;
static uint8_t ring_end = 0;
static uint8_t ring_size = 0;

uint8_t rb_size()
{
    return ring_size;
}

void rb_insert(uint8_t *data, int8_t rssi, uint32_t time_stamp)
{
    ring_start = (ring_start - 1 + RING_BUFFER_SIZE)%RING_BUFFER_SIZE;
    if(ring_start < 0)
        ring_start+=RING_BUFFER_SIZE;
    memcpy(ring_buffer[ring_start].data, data, 31);
    ring_buffer[ring_start].rssi = rssi;
    ring_buffer[ring_start].time_stamp = time_stamp;
    if(ring_end == ring_start)
        ring_end = (ring_end - 1 + RING_BUFFER_SIZE)%RING_BUFFER_SIZE;
    if(ring_size < RING_BUFFER_SIZE)
        ring_size++;
}

void rb_reset()
{
    ring_start = 0;
    ring_end = 0;
    ring_size = 0;
}

#define LINE "+--------------------------------------------------------------------------------------------------------------"
void rb_print_num(uint8_t offset, uint8_t max)
{
    uint32_t len;
    uint8_t index;

    if(ring_size == 0)
    {
        WICED_BT_TRACE("\n\rNo recent data\n\r");
        //WICED_BT_TRACE("---------- End Data ----------\n\r");
        return;
    }
    WICED_BT_TRACE("\n\r");
    WICED_BT_TRACE("------+------%s\n\r", LINE);
    WICED_BT_TRACE("      |      | Data\n\r");
    WICED_BT_TRACE(" Seen | RSSI %s\n\r", LINE);
    WICED_BT_TRACE("      |      | Decoded Data\n\r");
    WICED_BT_TRACE("------+------%s\n\r", LINE);
    for(int i = 0; i < max && (index = i + offset) < ring_size && index < RING_BUFFER_SIZE-1; i++)
    {
        uint8_t *data = ring_buffer[(index+ring_start)%RING_BUFFER_SIZE].data;
        uint32_t time_stamp = ring_buffer[(index+ring_start)%RING_BUFFER_SIZE].time_stamp;
        int8_t rssi = ring_buffer[(index+ring_start)%RING_BUFFER_SIZE].rssi;

        uint32_t time_dif = current_time() - time_stamp;
        if(time_dif < 60){
            WICED_BT_TRACE("  %2ds | ", time_dif);
        }else if(time_dif < 3600){
            time_dif/=60;
            WICED_BT_TRACE("  %2dm | ", time_dif);
        }else if(time_dif < 86400){
            time_dif/=3600;
            WICED_BT_TRACE("  %2dh | ", time_dif);
        }else if(time_dif < 864000){
            time_dif/=86400;
            WICED_BT_TRACE("   %dd | ", time_dif);
        }else{
            WICED_BT_TRACE("  >9d | ");
        }

        WICED_BT_TRACE("%4d | ", rssi);

        int length = dt_advGetLength(data);
        for(int j=0; j < length && j < 31; j++)
        {
            WICED_BT_TRACE("%02X ",data[j]);
        }
        WICED_BT_TRACE("\n\r");
        WICED_BT_TRACE("      |      %s\n\r", LINE);
        len = dt_advGetLength(data);
        blePrintAdvPacketData(data,len,"      |      | ");
        WICED_BT_TRACE("------+------%s\n\r", LINE);
    }
}

void rb_print()
{
    rb_print_num(0, RING_BUFFER_SIZE);
}
