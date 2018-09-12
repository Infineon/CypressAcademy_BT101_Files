#include <stdio.h>
#include "wiced.h"
#include "wiced_bt_trace.h"

#include "AdvScanner.h"
#include "company_ids.h"
#include "decode_functions.h"
#include "device_table.h"
#include "ring_buffer.h"

static scan_device_t devices[MAX_DEVICES];
static uint32_t numDevices;
static uint32_t numBeacons;
static uint32_t focusDevice = 0;

scan_device_t *dt_getTable()
{
    return devices;
}

uint32_t dt_getNumDevices()
{
    return numDevices;
}

uint32_t dt_getNumBeacons()
{
    return numBeacons;
}

uint32_t dt_getFocus()
{
    return focusDevice;
}

void dt_setFocus(uint32_t new_focus)
{
    if(new_focus != focusDevice)
        rb_reset();
    if(new_focus >= 0 && new_focus < numDevices){
        focusDevice = new_focus;
        WICED_BT_TRACE( "Filtering on device %d\n\r", new_focus);
    }
    else
    {
        focusDevice = 0;
        WICED_BT_TRACE( "Invalid index: %d\n\r", new_focus);
    }
}

void dt_reset()
{
    dt_setFocus(0);
    numDevices = 0;
    numBeacons = 0;
    rb_reset();
}

scan_device_t *dt_findDevice(wiced_bt_device_address_t *bdaddr)
{
    for(uint32_t i=0;i<numDevices;i++)
    {
        if(memcmp(&devices[i].remote_bd_addr,bdaddr,sizeof(wiced_bt_device_address_t)) == 0)
            return &devices[i];
    }
    return 0;
}

scan_device_t *dt_addDevice(wiced_bt_ble_scan_results_t *scanDev, uint8_t *advData, uint32_t time)
{
    uint32_t length = dt_advGetLength(advData);

    /* If the length of the data is zero, something went wrong */
    if(length == 0)
    {
        return NULL;
    }

    scan_device_t *myDev;
    myDev = dt_findDevice(&scanDev->remote_bd_addr);

    if(!myDev)
    {
        if(numDevices < MAX_DEVICES)
        {
            myDev = &devices[numDevices++];
            if(
                    isEddystone(advData) == WICED_TRUE ||
                    is_iBeacon(advData) == WICED_TRUE ||
                    isCypress(advData) == WICED_TRUE
            )
                numBeacons++;
            if(printing_enabled() == WICED_TRUE && filter_enabled() == WICED_FALSE && numDevices == MAX_DEVICES)
                WICED_BT_TRACE("Table is now full, so new devices will replace the oldest device\n\r");
        }
        else
        {
            uint32_t oldest_time;
            int oldest_index, sub_beacon;
            for(int i = 0; i < MAX_DEVICES; i++)
            {
                myDev = &devices[i];
                if(i == 0 || myDev->time_stamp < oldest_time)
                {
                    if(
                            isEddystone(myDev->data) == WICED_TRUE ||
                            is_iBeacon(myDev->data) == WICED_TRUE ||
                            isCypress(myDev->data) == WICED_TRUE
                    )
                        sub_beacon = 1;
                    else
                        sub_beacon = 0;
                    oldest_time = myDev->time_stamp;
                    oldest_index = i;
                }
            }
            numBeacons-=sub_beacon;
            if(
                    isEddystone(advData) == WICED_TRUE ||
                    is_iBeacon(advData) == WICED_TRUE ||
                    isCypress(advData) == WICED_TRUE
            )
                numBeacons++;

            myDev = &devices[oldest_index];
        }
        memcpy(&myDev->remote_bd_addr,&scanDev->remote_bd_addr,sizeof(wiced_bt_device_address_t));
        myDev->flag = scanDev->flag;
        myDev->ble_addr_type = scanDev->ble_addr_type;
        if(printing_enabled() == WICED_TRUE && filter_enabled() == WICED_FALSE)
        {
            WICED_BT_TRACE("Adding new device %B[Scan Type %d]: ",scanDev->remote_bd_addr,scanDev->ble_evt_type);
            for(int i=0;i<length;i++)
                WICED_BT_TRACE("%02X ", advData[i]);
            WICED_BT_TRACE("\n\r");
        }
    }
    else if(myDev == &devices[focusDevice])
    {
        if(printing_enabled() == WICED_TRUE && filter_enabled() == WICED_TRUE){
            WICED_BT_TRACE("Updating filtered device: ");
            for(int i=0;i<length;i++)
                WICED_BT_TRACE("%02X ", advData[i]);
            WICED_BT_TRACE("\n\r");
        }
        rb_insert(advData, myDev->rssi, time);
    }

    myDev->rssi = scanDev->rssi;
    myDev->time_stamp = time;
    memcpy(myDev->data,advData,31);

    return myDev;
}

uint32_t dt_advGetLength(uint8_t *p_adv_data)
{
    uint32_t length = 0;

    for(int i=0; p_adv_data[i] && length<31; i += 1+p_adv_data[i])
    {
        length += p_adv_data[i] + 1;
    }
    return length;
}
