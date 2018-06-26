#include "wiced.h"
#include "device_table.h"
#include "wiced_bt_trace.h"
#include "company_ids.h"
#include "AdvScanner.h"
#include <stdio.h>

#define MAX_DEVICES (64)
#define RECENT_PACKET_NUM (16)
#define RING_BUFFER_SIZE (17)

void bleDecodeAdInfo(uint8_t *bytes);
void bleFormat128bitUUID(uint8_t *bytes);
void bleFormat32bitUUID(uint8_t *bytes);
void bleFormat16bitUUID(uint8_t *bytes);
void blePrintAdvPacketData(uint8_t *data,int len,char *pad);

static scan_device_t devices[MAX_DEVICES];
static uint32_t numDevices;

static uint32_t focusDevice = 0;
static uint8_t ring_buffer[31][RING_BUFFER_SIZE];
static uint8_t ring_start = 0;
static uint8_t ring_end = 0;
static uint8_t ring_size = 0;

void rb_insert(uint8_t *data)
{
    memcpy(ring_buffer[ring_end], data, 31);
    ring_end = (ring_end+1)%RING_BUFFER_SIZE;
    if(ring_end == ring_start)
        ring_start = (ring_start+1)%RING_BUFFER_SIZE;
    if(ring_size < RING_BUFFER_SIZE)
        ring_size++;
}

void rb_reset()
{
    ring_start = 0;
    ring_end = 0;
    ring_size = 0;
}

uint32_t dt_getNumDevices(){
    return numDevices;
}
uint32_t dt_getFocus(){
    return focusDevice;
}
void dt_setFocus(uint32_t new_focus){
    if(new_focus != focusDevice)
        rb_reset();
    if(new_focus >= 0 && new_focus < numDevices)
        focusDevice = new_focus;
}
void dt_reset()
{
    dt_setFocus(0);
    numDevices = 0;
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
    /* If the length of the data is zero, something went wrong */
    if(dt_advGetLength(advData) == 0)
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
            if(printing_enabled() == WICED_TRUE && filter_enabled() == WICED_FALSE && numDevices == MAX_DEVICES)
                WICED_BT_TRACE("Table is now full, so new devices will replace the oldest device\n\r");
        }
        else
        {
            uint32_t oldest_time;
            int oldest_index;
            for(int i = 0; i < MAX_DEVICES; i++)
            {
                myDev = &devices[i];
                if(i == 0 || myDev->time_stamp < oldest_time)
                {
                    oldest_time = myDev->time_stamp;
                    oldest_index = i;
                }
            }

            myDev = &devices[oldest_index];
        }
        memcpy(&myDev->remote_bd_addr,&scanDev->remote_bd_addr,sizeof(wiced_bt_device_address_t));
        myDev->flag = scanDev->flag;
        myDev->ble_addr_type = scanDev->ble_addr_type;
        myDev->rssi = scanDev->rssi;
        if(printing_enabled() == WICED_TRUE && filter_enabled() == WICED_FALSE)
            WICED_BT_TRACE("Adding new device %B Scan Type %d\n\r",scanDev->remote_bd_addr,scanDev->ble_evt_type);
    }
    else if(myDev == &devices[focusDevice])
    {
        if(printing_enabled() == WICED_TRUE && filter_enabled() == WICED_TRUE)
            WICED_BT_TRACE("Updating device %B Scan Type %d\n\r",scanDev->remote_bd_addr,scanDev->ble_evt_type);
        rb_insert(advData);
    }

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

void dt_printDeviceOneLine(scan_device_t *device, uint32_t index)
{
    uint32_t nameLength;
    uint32_t length;
    length = dt_advGetLength(device->data);

    WICED_BT_TRACE("%02d | %B| %02d          | ", index, device->remote_bd_addr, (int)length);

    uint8_t name_len;
    uint8_t *name_start = wiced_bt_ble_check_advertising_data(device->data,BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,&name_len);
    if(!name_start)
    {
        uint8_t *name_start = wiced_bt_ble_check_advertising_data(device->data,BTM_BLE_ADVERT_TYPE_NAME_SHORT,&name_len);
    }


    if(name_start)
    {
        for(int i=0;i<((name_len>10)?10:name_len);i++)
        {
            WICED_BT_TRACE("%c",name_start[i]);
        }
        for(int i=name_len ; i<10;i++)
        {
            WICED_BT_TRACE(" ");
        }
        WICED_BT_TRACE(" | ");
    }
    else
    {
        WICED_BT_TRACE("           | ");
    }

    if(0)
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");

    // iBeacon prefix: 02 01 06 1A FF 4C 00 02
    if(
            device->data[0] == 0x02 &&
            device->data[1] == 0x01 &&
            device->data[4] == 0xFF &&
            device->data[5] == 0x4C &&
            device->data[6] == 0x00 &&
            device->data[7] == 0x02)
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");

    // Cypress Company Prefix: 02 01 06 1A FF 01 31
    if(
            device->data[4] == 0xFF &&
            device->data[5] == 0x01 &&
            device->data[6] == 0x31)
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");

    // Print time since last seen
    uint32_t time_dif = current_time() - device->time_stamp;
    if(time_dif >= 60)
    {
        time_dif/=60;
        if(time_dif >= 60)
        {
            time_dif/=60;
            if(time_dif >= 24)
            {
                time_dif/=24;
                if(time_dif > 9)
                    WICED_BT_TRACE(">9d ago   | ");
                else
                    WICED_BT_TRACE(" %dd ago   | ", time_dif);
            }
            else
                WICED_BT_TRACE("%02dh ago   | ", time_dif);
        }
        else
            WICED_BT_TRACE("%02dm ago   | ", time_dif);
    }
    else
        WICED_BT_TRACE("%02ds ago   | ", time_dif);


    for(int i=0;i<length;i++)
    {
        WICED_BT_TRACE("%02X ",device->data[i]);

    }
    WICED_BT_TRACE("\n\r");

    if(length>31)
    {
        WICED_BT_TRACE("Length Error\n\r");
    }
}

#define FIRST_BAR "---+-------------------+-------------+------------+---+---+---+-----------+"
#define LONG_BAR "--------------------------------------------------------------------------------------------------------------"
#define SHORT_BAR "---------------------------------------------------------------------------------------------"
void dt_printDeviceTableOneLine()
{
    WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, SHORT_BAR);
    WICED_BT_TRACE("#  | Address           | Data Length | Name       | E | I | C | Last Seen | Data\n\r");
    WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, SHORT_BAR);
    for(int i=0;i<numDevices;i++)
    {
        dt_printDeviceOneLine(&devices[i], i);
    }

}

#define PAD "   |                   |             |            |   |   |   |           | "
void dt_printDeviceTableMultiLine()
{
    uint32_t len;

    WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, LONG_BAR);
    WICED_BT_TRACE("%sData\n\r", PAD);
    WICED_BT_TRACE("#  | Address           | Data Length | Name       | E | I | C | Last Seen +%s\n\r", LONG_BAR);
    WICED_BT_TRACE("%sDecoded Data\n\r", PAD);
    for(int i=0;i<numDevices;i++)
    {
        WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, LONG_BAR);
        len = dt_advGetLength(devices[i].data);
        dt_printDeviceOneLine(&devices[i], i);
        WICED_BT_TRACE("   |                   |             |            |   |   |   |           +%s\n\r", LONG_BAR);
        blePrintAdvPacketData(devices[i].data,len,PAD);

    }
}

void dt_printRecentFilterData()
{
    uint32_t len;

    WICED_BT_TRACE( "\n\r" );
    WICED_BT_TRACE("----- Recent Filter Data -----\n\r");
    WICED_BT_TRACE( "\n\r" );
    WICED_BT_TRACE("----------- Device -----------\n\r");
    WICED_BT_TRACE( "\n\r" );
    WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, SHORT_BAR);
    WICED_BT_TRACE("#  | Address           | Data Length | Name       | E | I | C | Last Seen | Data\n\r");
    WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, SHORT_BAR);
    dt_printDeviceOneLine(&devices[focusDevice], focusDevice);
    WICED_BT_TRACE("%s%s\n\r", FIRST_BAR, SHORT_BAR);
    WICED_BT_TRACE( "\n\r" );
    WICED_BT_TRACE("------------ Data ------------\n\r");
    if(ring_size == 0)
        WICED_BT_TRACE("No recent data\n\r");
    for(int i = 0; i < ring_size && i < RECENT_PACKET_NUM; i++)
    {
        if(i != 0)
            WICED_BT_TRACE("\n\r");
        uint8_t *data = ring_buffer[(i+ring_start)%RING_BUFFER_SIZE];
        int length = dt_advGetLength(data);
        for(int j=0; j < length && j < 31; j++)
        {
            WICED_BT_TRACE("%02X ",data[j]);

        }
        WICED_BT_TRACE("\n\r");
        len = dt_advGetLength(data);
        blePrintAdvPacketData(data,len,"");
    }
    WICED_BT_TRACE("---------- End Data ----------\n\r");
    WICED_BT_TRACE( "\n\r" );
}

// print each field on one line
// len f# name data

void blePrintAdvPacketData(uint8_t *data,int len,char *pad)
{
    int index=0;

    uint8_t *adInfoPacket;

    while(index<len)
    {
        adInfoPacket = &data[index];
        index += data[index] + 1;

        WICED_BT_TRACE(pad);
        bleDecodeAdInfo(adInfoPacket);
        WICED_BT_TRACE("\n\r");
    }

}


struct advDecode_t {
    uint8_t code;
    char *name;
    void (*fcn)(uint8_t *bytes);
};

void bleAdInfoDecodeName(uint8_t *bytes)
{
    for(int i = 0; i < bytes[0]; i++){
        WICED_BT_TRACE("%c",bytes[1+i]);
    }
}

void bleAdInfoDecodeUnknown(char *buff, uint8_t *bytes)
{
    WICED_BT_TRACE("Len:%02X Type:%02X",bytes[0],bytes[1]);
}


void bleAdInfoDecodeFlags(uint8_t *bytes)
{
    int count = 0;

    WICED_BT_TRACE("%02X ",bytes[2]);

    if(bytes[2] & 0x01)
        WICED_BT_TRACE("LE Ltd Discoverable ");

    if(bytes[2] & 0x02)
        WICED_BT_TRACE("LE General Discoverable ");

    if(bytes[2] & 0x04)
        WICED_BT_TRACE("BR/EDR Not Supported ");

    if(bytes[2] & 0x08)
        WICED_BT_TRACE("BR/EDR Controller ");

    if(bytes[2] & 0x10)
        WICED_BT_TRACE("BR/EDR Host ");

}

void bleAdInfoDumpBytesOffset(uint8_t *bytes,uint32_t offset)
{
    int i;
    for(i=offset;i<bytes[0]-1;i++)
    {
        WICED_BT_TRACE("%02X ",bytes[2+i]);
    }

}

void bleAdInfoDumpBytes(uint8_t *bytes)
{
    int i;
    for(i=0;i<bytes[0]-1;i++)
    {
        WICED_BT_TRACE("%02X ",bytes[2+i]);
    }

}

void bleAdInfoDecodeMfgData( uint8_t *bytes)
{
    uint16_t mfg;
    mfg = bytes[2] | bytes[3]<<8;
    WICED_BT_TRACE("%s ",getCompanyName(mfg));
    if(mfg == 0x004C && bytes[4] == 0x02)
    {
        WICED_BT_TRACE("iBeacon ");
        bleAdInfoDumpBytesOffset(bytes,3);
    }
    else
    {
        bleAdInfoDumpBytesOffset(bytes,2);
    }
}

void bleAdInfoDecode16bitServiceUUID( uint8_t *bytes)
{
    bleFormat16bitUUID(&bytes[2]);
}

void bleAdInfoDecode32bitServiceUUID( uint8_t *bytes)
{
    bleFormat128bitUUID(&bytes[2]);
}


void bleAdInfoDecode128bitServiceUUID( uint8_t *bytes)
{
    bleFormat128bitUUID(&bytes[2]);
}
void bleAdInfoDecodePublicAddress(uint8_t *bytes)
{
    bleAdInfoDumpBytes(bytes);
}

// https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile
struct advDecode_t advDecodeArray[] = {
        {0x01, "Flags",bleAdInfoDecodeFlags},
        {0x02, "16-bit Service UUID",bleAdInfoDecode16bitServiceUUID},
        {0x03, "16-bit Service UUID",bleAdInfoDecode16bitServiceUUID},
        {0x04, "32-bit Service UUID",bleAdInfoDecode32bitServiceUUID},
        {0x05, "32-bit Service UUID",bleAdInfoDecode32bitServiceUUID},
        {0x06, "128-bit Service UUIDs", bleAdInfoDecode128bitServiceUUID},
        {0x07, "128-bit Service UUIDs", bleAdInfoDecode128bitServiceUUID},
        {0x08, "Short Name", bleAdInfoDecodeName},
        {0x09, "Complete Name", bleAdInfoDecodeName},
        //0xXX:-127 to +127dBm
        //Note: when the TX Power Level tag is not present,
        //the TX power level of the packet is unknown.
        {0x0A, "Tx Power Level", bleAdInfoDumpBytes},
        {0x0D, "Device Class", bleAdInfoDumpBytes},
        {0x0E, "Pairing Hash C", bleAdInfoDumpBytes},
        {0x0E, "Pairing Hash C-192", bleAdInfoDumpBytes},
        {0x0F, "Pairing Randomizer R", bleAdInfoDumpBytes},
        {0x0F, "Pairing Randomizer R-192", bleAdInfoDumpBytes},
        {0x10, "Device ID", bleAdInfoDumpBytes},
        {0x10, "Security Manager TK Value", bleAdInfoDumpBytes},
        {0x11, "Security Manager Out of Band Flags", bleAdInfoDumpBytes},
        {0x12, "Slave Connection Interval Range", bleAdInfoDumpBytes},
        {0x14, "16-bit Service Solicitation UUIDs", bleAdInfoDecode16bitServiceUUID},
        {0x15, "128-bit Service Solicitation UUIDs", bleAdInfoDecode128bitServiceUUID},
        {0x16, "Service Data", bleAdInfoDumpBytes},
        {0x16, "16-bit Service Data UUID", bleAdInfoDumpBytes},
        {0x17, "Public Target Address", bleAdInfoDumpBytes},
        {0x18, "Random Target Address", bleAdInfoDumpBytes},
        {0x19, "Appearance", bleAdInfoDumpBytes},
        {0x1A, "Advertising Interval", bleAdInfoDumpBytes},
        {0x1B, "LE Bluetooth Device Address", bleAdInfoDumpBytes},
        {0x1C, "LE Role", bleAdInfoDumpBytes},
        {0x1D, "Simple Pairing Hash C-256", bleAdInfoDumpBytes},
        {0x1E, "Simple Pairing Randomizer R-256", bleAdInfoDumpBytes},
        {0x1F, "32-bit Service Solitication UUIDs",bleAdInfoDecode32bitServiceUUID},
        {0x20, "32-bit Service Data UUID",bleAdInfoDecode32bitServiceUUID},
        {0x21, "128-bit Service Data UUID",bleAdInfoDecode128bitServiceUUID},
        {0x22, "LE Secure Connections Confirmation Value",bleAdInfoDumpBytes},
        {0x23, "LE Secure Connections Random Value",bleAdInfoDumpBytes},
        {0x24, "URI",bleAdInfoDumpBytes},
        {0x25, "Indoor Positioning",bleAdInfoDumpBytes},
        {0x26, "Transport Discovery Data",bleAdInfoDumpBytes},
        {0x27, "LE Supported Features",bleAdInfoDumpBytes},
        {0x28, "Channel Map Update Indication",bleAdInfoDumpBytes},
        {0x29, "PB-ADV",bleAdInfoDumpBytes},
        {0x3D, "3D Information Data",bleAdInfoDumpBytes},
        {0xFF, "MFG Data", bleAdInfoDecodeMfgData}
};


// Iterate over the whole ADV Pack and print out each field as a row
void bleDecodeAdInfo(uint8_t *bytes)
{
    int numElements = sizeof(advDecodeArray)/sizeof(struct advDecode_t);
    int i;

    for(i = 0; i<numElements; i++)
    {
        if(bytes[1] == advDecodeArray[i].code)
        {
            WICED_BT_TRACE("%s ",advDecodeArray[i].name );
            (*advDecodeArray[i].fcn)(bytes);
            return;
        }
    }
    WICED_BT_TRACE("Unknown");
    return;


}


void bleFormat128bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE("%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X",
            bytes[15],bytes[14],bytes[13],bytes[12],
            bytes[11],bytes[10],
            bytes[9],bytes[8],
            bytes[7],bytes[6],
            bytes[5],bytes[4],bytes[3],bytes[2],bytes[1],bytes[0]);

}

void bleFormat32bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE("%02X%02X%02X%02X",
            bytes[3],bytes[2],bytes[1],bytes[0]);

}

void bleFormat16bitUUID(uint8_t *bytes)
{
    WICED_BT_TRACE("%02X%02X",
            bytes[1],bytes[0]);

}

