#pragma ONCE

#include "wiced.h"

void spp_start();
void spp_tx_data(uint8_t* p_data, uint32_t data_len);
