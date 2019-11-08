/*******************************************************************************
* File Name: cycfg_routing.h
*
* Description:
* Establishes all necessary connections between hardware elements.
* This file was automatically generated and should not be modified.
* Device Configurator: 2.0.0.1483
* Device Support Library (../../../wiced_btsdk/dev-kit/baselib/20819A1): 2.0.0.2684
*
********************************************************************************
* Copyright 2017-2019 Cypress Semiconductor Corporation
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
********************************************************************************/

#if !defined(CYCFG_ROUTING_H)
#define CYCFG_ROUTING_H

#if defined(__cplusplus)
extern "C" {
#endif

#include "cycfg_notices.h"
void init_cycfg_routing(void);
#define init_cycfg_connectivity() init_cycfg_routing()
#define ioss_0_pin_11_AUX UNKNOWN
#define ioss_0_pin_17_AUX UNKNOWN
#define ioss_0_pin_27_AUX UNKNOWN
#define ioss_0_pin_28_AUX UNKNOWN
#define ioss_0_pin_29_AUX UNKNOWN
#define ioss_0_pin_32_AUX UNKNOWN
#define ioss_0_pin_37_AUX UNKNOWN
#define ioss_0_pin_6_AUX UNKNOWN
#define ioss_0_pin_8_AUX UNKNOWN
#define ioss_0_pin_9_AUX UNKNOWN

#define CYBSP_A0_aux_0_TRIGGER_OUT ADC_INPUT_P8
#define CYBSP_THERM_TEMP_SENSE_aux_0_TRIGGER_OUT CYBSP_A0_aux_0_TRIGGER_OUT
#define adc_0_channel_0_TRIGGER_IN WICED_GPIO
#define i2c_0_scl_0_TRIGGER_IN WICED_I2C_1_SCL
#define i2c_0_sda_0_TRIGGER_IN WICED_I2C_1_SDA
#define pwm_1_pwm_0_TRIGGER_IN WICED_PWM1
#define spi_1_clk_0_TRIGGER_IN WICED_SPI_2_CLK
#define spi_1_cs_0_TRIGGER_IN WICED_SPI_2_CS
#define spi_1_miso_0_TRIGGER_IN WICED_SPI_2_MISO
#define spi_1_mosi_0_TRIGGER_IN WICED_SPI_2_MOSI
#define uart_1_rxd_0_TRIGGER_IN WICED_UART_2_RXD
#define uart_1_txd_0_TRIGGER_IN WICED_UART_2_TXD

#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_ROUTING_H */
