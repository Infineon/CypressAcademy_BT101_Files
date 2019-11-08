#pragma once
#include "device_table.h"

#define PAGE_SIZE_M (6)
#define PAGE_SIZE_F (5)
#define PAGE_NUM_F  (4)
#define PAGE_SIZE_S (40)

void blePrintAdvPacketData(uint8_t *data,int len,char *pad);
void clear_terminal();
void printDeviceOneLine(scan_device_t *device, uint32_t index, wiced_bool_t extra_data);
void printDeviceTableOneLine();
void printBeaconTable();
void printDeviceTableMultiLine();
void printRecentFilterData();
void printMostRecentFilterData();

void decrementPageNum_m();
void incrementPageNum_m();
void decrementPageNum_r();
void incrementPageNum_r();
void decrementPageNum_s();
void incrementPageNum_s();
void decrementPageNum_b();
void incrementPageNum_b();
void reset_tables();
