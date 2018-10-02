#pragma once

void bleDecodeAdInfo(uint8_t *bytes);
wiced_bool_t isEddystone(uint8_t *data);
wiced_bool_t is_iBeacon(uint8_t *data);
wiced_bool_t isCypress(uint8_t *data);
