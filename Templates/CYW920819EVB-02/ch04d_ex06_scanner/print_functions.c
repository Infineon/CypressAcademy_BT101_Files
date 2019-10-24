/* ----- Imports ----- */
#include <app.h>
#include "wiced.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"

#include "company_ids.h"
#include "decode_functions.h"
#include "device_table.h"
#include "print_functions.h"
#include "ring_buffer.h"


/* ----- Definitions ----- */
#define HEADER    "  #  |      Address      | Len |    Name    | E | I | C | Seen | RSSI "
#define PAD       "     |                   |     |            |   |   |   |      |      | "
#define FIRST_BAR "-----+-------------------+-----+------------+---+---+---+------+------+"
#define EXTEND    "-----------------"
#define SHORT_BAR "---------------------------------------------------------------------------------------------"


/* ----- Function Prototypes ----- */
void blePrintAdvPacketData(uint8_t *data,int len,char *pad);


/* ----- Variable declarations ----- */
uint8_t page_num_m = 0;
uint8_t page_num_r = 0;
uint8_t page_num_s = 0;
uint8_t page_num_b = 0;


/* Clear the terminal and reset the cursor position */
#define ASCII_ESC 27
void clear_terminal()
{
    // Esc[2J clears the screen
    WICED_BT_TRACE( "%c[2J", ASCII_ESC );

    // Esc[H moves the cursor to the top left corner
    WICED_BT_TRACE( "%c[H", ASCII_ESC );
}

/* Increment/Decrement multiline table page */
void incrementPageNum_m()
{
    uint8_t max_pages = dt_getNumDevices()/PAGE_SIZE_M;
    if(dt_getNumDevices() % PAGE_SIZE_M)
        max_pages++;
    if(page_num_m < max_pages - 1)
        page_num_m++;
}
void decrementPageNum_m()
{
    if(page_num_m > 0)
        page_num_m--;
}

/* Increment/Decrement recent packet table page */
void incrementPageNum_r()
{
    uint8_t max_pages = 0;
    for(int i = 0; i < PAGE_NUM_F; i++)
        if(rb_size() >= i*PAGE_SIZE_F)
            max_pages++;
    if(page_num_r < max_pages - 1)
        page_num_r++;
}
void decrementPageNum_r()
{
    if(page_num_r > 0)
        page_num_r--;
}

/* Increment/Decrement single-line table page */
void incrementPageNum_s()
{
    uint8_t max_pages = dt_getNumDevices()/PAGE_SIZE_S;
    if(dt_getNumDevices() % PAGE_SIZE_S)
        max_pages++;
    if(page_num_s < max_pages - 1)
        page_num_s++;
}
void decrementPageNum_s()
{
    if(page_num_s > 0)
        page_num_s--;
}

/* Increment/Decrement beacon table page */
void incrementPageNum_b()
{
    uint8_t max_pages = dt_getNumBeacons()/PAGE_SIZE_S;
    if(dt_getNumBeacons() % PAGE_SIZE_S)
        max_pages++;
    if(page_num_b < max_pages - 1)
        page_num_b++;
}
void decrementPageNum_b()
{
    if(page_num_b > 0)
        page_num_b--;
}

/* Reset table page numbers */
void reset_tables()
{
    page_num_r = 0;
    page_num_m = 0;
    page_num_s = 0;
    page_num_b = 0;
}

/* Print unprocessed device data */
void printDeviceOneLine(scan_device_t *device, uint32_t index, wiced_bool_t extra_data)
{
    /* Print the index, address, and data length */
    uint32_t length;
    length = dt_advGetLength(device->data);
    WICED_BT_TRACE("%4d | %B| %02d  | ", index, device->remote_bd_addr, (int)length);

    /* Print the name */
    uint8_t name_len;
    uint8_t *name_start = wiced_bt_ble_check_advertising_data(device->data,BTM_BLE_ADVERT_TYPE_NAME_COMPLETE,&name_len);
    if(!name_start)
        name_start = wiced_bt_ble_check_advertising_data(device->data,BTM_BLE_ADVERT_TYPE_NAME_SHORT,&name_len);

    if(name_start)
    {
        for(int i=0;i<((name_len>10)?10:name_len);i++)
            WICED_BT_TRACE("%c",name_start[i]);
        for(int i=name_len ; i<10;i++)
            WICED_BT_TRACE(" ");
        WICED_BT_TRACE(" | ");
    }
    else
        WICED_BT_TRACE("           | ");

    /* Check for Eddystone, iBeacon, and Cypress devices */
    if(isEddystone(device->data) == WICED_TRUE)
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");

    if(is_iBeacon(device->data) == WICED_TRUE)
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");

    if(isCypress(device->data) == WICED_TRUE)
        WICED_BT_TRACE("X | ");
    else
        WICED_BT_TRACE("  | ");


    if(extra_data == WICED_FALSE)
        WICED_BT_TRACE("  ");

    // Print the time since the device was last seen
    uint32_t time_dif = current_time() - device->time_stamp;
    if(time_dif < 60){
        WICED_BT_TRACE(" %2ds", time_dif);
    }else if(time_dif < 3600){
        time_dif/=60;
        WICED_BT_TRACE(" %2dm", time_dif);
    }else if(time_dif < 86400){
        time_dif/=3600;
        WICED_BT_TRACE(" %2dh", time_dif);
    }else if(time_dif < 864000){
        time_dif/=86400;
        WICED_BT_TRACE("  %dd", time_dif);
    }else{
        WICED_BT_TRACE(" >9d");
    }

    if(extra_data == WICED_FALSE)
    {
        WICED_BT_TRACE("\n");
        return;
    }

    WICED_BT_TRACE(" | ");

    // Print the RSSI of the device
    WICED_BT_TRACE("%4d | ", device->rssi);

    // Print the device data
    for(int i = 0; i < length && i < 31; i++)
        WICED_BT_TRACE("%02X ", device->data[i]);
    WICED_BT_TRACE("\n");

    if(length>31)
        WICED_BT_TRACE("Length Error\n");
}

/* Print a table of unprocessed device data */
void printDeviceTableOneLine()
{
    // Clear the screen
    clear_terminal();

    // Retrieve a pointer to the table
    scan_device_t *devices = dt_getTable();

    // Print table header
    WICED_BT_TRACE("%s%s\n", FIRST_BAR, SHORT_BAR);
    WICED_BT_TRACE("%s| Data\n", HEADER);
    WICED_BT_TRACE("%s%s\n", FIRST_BAR, SHORT_BAR);

    //Print device data
    uint32_t index;
    for(int i = 0; i < PAGE_SIZE_S && (index = i+PAGE_SIZE_S*page_num_s) < dt_getNumDevices(); i++)
        printDeviceOneLine(&devices[index], index, WICED_TRUE);

    //Print page numbers
    uint8_t max_pages = dt_getNumDevices()/PAGE_SIZE_S;
    if(dt_getNumDevices() % PAGE_SIZE_S)
        max_pages++;
    WICED_BT_TRACE("\nPage %d out of %d\n", page_num_s + 1, max_pages);
}

void printBeaconTable()
{
    // Clear the screen
    clear_terminal();

    // Retrieve a pointer to the table
    scan_device_t *devices = dt_getTable();

    // Print table header
    WICED_BT_TRACE("%s%s\n", FIRST_BAR, SHORT_BAR);
    WICED_BT_TRACE("%s| Data\n", HEADER);
    WICED_BT_TRACE("%s%s\n", FIRST_BAR, SHORT_BAR);

    //Print device data
    uint32_t index;
    int found_beacons = 0;
    int i = 0;
    while(i < dt_getNumDevices() && found_beacons < (page_num_b + 1) * PAGE_SIZE_S)
    {
        if(
                isEddystone(devices[i].data) == WICED_TRUE ||
                is_iBeacon(devices[i].data) == WICED_TRUE ||
                isCypress(devices[i].data) == WICED_TRUE
        ){
            if(found_beacons >= page_num_b * PAGE_SIZE_S)
                printDeviceOneLine(&devices[i], i, WICED_TRUE);
            found_beacons++;
        }
        i++;
    }

    //Print page numbers
    uint8_t max_pages = dt_getNumBeacons()/PAGE_SIZE_S;
    if(dt_getNumBeacons() % PAGE_SIZE_S)
        max_pages++;
    WICED_BT_TRACE("\nPage %d out of %d\n", page_num_b + 1, max_pages);
}

/* Print a table of both processed and unprocessed device data */
void printDeviceTableMultiLine()
{
    // Clear the screen
    clear_terminal();

    // Retrieve a pointer to the table
    scan_device_t *devices = dt_getTable();
    uint32_t len, index;

    // Print the table header
    WICED_BT_TRACE("%s%s%s\n", FIRST_BAR, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("%sData\n", PAD);
    WICED_BT_TRACE("%s+%s%s\n", HEADER, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("%sDecoded Data\n", PAD);

    // Print device data
    char *pad = "     |                   |     |            |   |   |   |      |      +";
    for(int i = 0; i < PAGE_SIZE_M && (index = i+PAGE_SIZE_M*page_num_m) < dt_getNumDevices(); i++)
    {
        WICED_BT_TRACE("%s%s%s\n", FIRST_BAR, SHORT_BAR, EXTEND);
        len = dt_advGetLength(devices[index].data);
        printDeviceOneLine(&devices[index], index, WICED_TRUE);
        WICED_BT_TRACE("%s%s%s\n", pad, SHORT_BAR, EXTEND);
        blePrintAdvPacketData(devices[index].data,len,PAD);
    }

    //Print page numbers
    uint8_t max_pages = dt_getNumDevices()/PAGE_SIZE_M;
    if(dt_getNumDevices() % PAGE_SIZE_M)
        max_pages++;
    WICED_BT_TRACE("Page %d out of %d\n", page_num_m + 1, max_pages);
}

/* Print the sixteen most recent advertising packets of the filtered device */
void printRecentFilterData()
{
    // Clear the screen
    clear_terminal();

    //Retrieve a pointer to the filtered device
    scan_device_t *devices = dt_getTable();
    uint32_t focusDevice = dt_getFocus();

    // Table headers
    WICED_BT_TRACE( "\n" );
    WICED_BT_TRACE("----- Recent Filter Data -----\n");
    WICED_BT_TRACE( "\n" );
    //WICED_BT_TRACE("----------- Device -----------\n");
    //WICED_BT_TRACE( "\n" );
    WICED_BT_TRACE("-----+-------------------+-----+------------+---+---+---+-----------\n");
    WICED_BT_TRACE("  #  |      Address      | Len |    Name    | E | I | C | Last Seen\n");
    WICED_BT_TRACE("-----+-------------------+-----+------------+---+---+---+-----------\n");

    // Print unprocessed device data
    printDeviceOneLine(&devices[focusDevice], focusDevice, WICED_FALSE);
    WICED_BT_TRACE("-----+-------------------+-----+------------+---+---+---+-----------\n");
    //WICED_BT_TRACE( "\n" );
    //WICED_BT_TRACE("------------ Data ------------\n");

    // Print the 20 most recent advertising packets (pages of 5)
    rb_print_num(page_num_r*PAGE_SIZE_F, PAGE_SIZE_F);

    WICED_BT_TRACE( "\n" );

    //Print page numbers
    uint8_t max_pages = 0;
    for(int i = 0; i < PAGE_NUM_F; i++)
        if(rb_size() >= i*PAGE_SIZE_F)
            max_pages++;
    WICED_BT_TRACE("Page %d out of %d\n", page_num_r + 1, max_pages);
}

void printMostRecentFilterData()
{
    WICED_BT_TRACE("\n");

    // Print the table header
    WICED_BT_TRACE("%s%s%s\n", FIRST_BAR, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("%sData\n", PAD);
    WICED_BT_TRACE("%s+%s%s\n", HEADER, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("%sDecoded Data\n", PAD);

    uint32_t len;
    scan_device_t *devices = dt_getTable();
    uint32_t focusDevice = dt_getFocus();
    char *pad = "     |                   |     |            |   |   |   |      |      +";

    // Print device data
    WICED_BT_TRACE("%s%s%s\n", FIRST_BAR, SHORT_BAR, EXTEND);
    len = dt_advGetLength(devices[focusDevice].data);
    printDeviceOneLine(&devices[focusDevice], focusDevice, WICED_TRUE);
    WICED_BT_TRACE("%s%s%s\n", pad, SHORT_BAR, EXTEND);
    blePrintAdvPacketData(devices[focusDevice].data,len,PAD);

    WICED_BT_TRACE("%s%s%s\n", FIRST_BAR, SHORT_BAR, EXTEND);
    WICED_BT_TRACE("\n");
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
        WICED_BT_TRACE("\n");
    }
}
