/*******************************************************************************
* File Name: cycfg_pins.h
*
* Description:
* Pin configuration
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

#if !defined(CYCFG_PINS_H)
#define CYCFG_PINS_H

#include "cycfg_notices.h"
#include "wiced_platform.h"
#include "cycfg_routing.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define adc_0_ENABLED 1U
#define bluetooth_0_ENABLED 1U
#define i2c_0_ENABLED 1U
#define ioss_0_ENABLED 1U
#define WICED_GET_PIN_FOR_LED(idx) (*(platform_led[(idx)].gpio))
#define WICED_GET_PIN_FOR_BUTTON(idx) (*(platform_button[(idx)].gpio))
#define WICED_GET_PIN_FOR_IO(idx) (*(platform_gpio[(idx)].gpio))
#define CYBSP_D2_ENABLED 1U
#define SW3_ENABLED CYBSP_D2_ENABLED
#define CYBSP_D2 WICED_P00
#define SW3 CYBSP_D2
#define CYBSP_A2_ENABLED 1U
#define CYBSP_A2 WICED_P10
#define CYBSP_RSVD9_ENABLED 1U
#define SPI_CS_ENABLED CYBSP_RSVD9_ENABLED
#define CYBSP_RSVD9 WICED_P11
#define SPI_CS CYBSP_RSVD9
#define CYBSP_A4_ENABLED 1U
#define CYBSP_A4 WICED_P12
#define CYBSP_A5_ENABLED 1U
#define CYBSP_A5 WICED_P13
#define CYBSP_D8_ENABLED 1U
#define CYBSP_D8 WICED_P14
#define CYBSP_D10_ENABLED 1U
#define CYBSP_D10 WICED_P15
#define CYBSP_D12_ENABLED 1U
#define SPI_MISO_ENABLED CYBSP_D12_ENABLED
#define CYBSP_D12 WICED_P17
#define SPI_MISO CYBSP_D12
#define CYBSP_RST_ENABLED 1U
#define CYBSP_RST WICED_P01
#define LED2_ENABLED 1U
#define LED2 WICED_P26
#define LED1_ENABLED 1U
#define LED1 WICED_P27
#define I2C_SCL_ENABLED 1U
#define I2C_SCL WICED_P28
#define I2C_SDA_ENABLED 1U
#define I2C_SDA WICED_P29
#define CYBSP_D4_ENABLED 1U
#define CYBSP_D4 WICED_P02
#define CYBSP_D1_ENABLED 1U
#define UART_TX_ENABLED CYBSP_D1_ENABLED
#define CYBSP_D1 WICED_P32
#define UART_TX CYBSP_D1
#define CYBSP_D0_ENABLED 1U
#define UART_RX_ENABLED CYBSP_D0_ENABLED
#define CYBSP_D0 WICED_P37
#define UART_RX CYBSP_D0
#define CYBSP_D5_ENABLED 1U
#define CYBSP_D5 WICED_P03
#define CYBSP_D6_ENABLED 1U
#define CYBSP_D6 WICED_P04
#define CYBSP_D7_ENABLED 1U
#define CYBSP_D7 WICED_P05
#define CYBSP_D11_ENABLED 1U
#define SPI_MOSI_ENABLED CYBSP_D11_ENABLED
#define CYBSP_D11 WICED_P06
#define SPI_MOSI CYBSP_D11
#define CYBSP_A0_ENABLED 1U
#define CYBSP_THERM_TEMP_SENSE_ENABLED CYBSP_A0_ENABLED
#define CYBSP_A0 WICED_P08
#define CYBSP_THERM_TEMP_SENSE CYBSP_A0
#define CYBSP_D13_ENABLED 1U
#define SPI_CLK_ENABLED CYBSP_D13_ENABLED
#define CYBSP_D13 WICED_P09
#define SPI_CLK CYBSP_D13
#define pwm_1_ENABLED 1U
#define spi_1_ENABLED 1U
#define uart_1_ENABLED 1U

extern const wiced_platform_gpio_t platform_gpio_pins[];
extern const size_t platform_gpio_pin_count;
extern const wiced_platform_led_config_t platform_led[];
extern const size_t led_count;
extern const wiced_platform_button_config_t platform_button[];
extern const size_t button_count;
extern const wiced_platform_gpio_config_t platform_gpio[];
extern const size_t gpio_count;


#if defined(__cplusplus)
}
#endif


#endif /* CYCFG_PINS_H */
