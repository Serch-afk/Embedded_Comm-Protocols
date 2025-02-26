/*
 * Copyright 2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v15.0
processor: RW612
package_id: RW612ETA2I
mcu_data: ksdk2_0
processor_version: 0.16.5
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_io_mux.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: cm33, enableClock: 'true'}
- pin_list:
  - {pin_num: E5, peripheral: FLEXCOMM3, signal: USART_RXD, pin_signal: GPIO_24}
  - {pin_num: E7, peripheral: ENET, signal: ENET_REF_CLK, pin_signal: GPIO_25, slew_rate: weak}
  - {pin_num: B6, peripheral: ENET, signal: 'ENET_RX_DATA, 0', pin_signal: GPIO_22}
  - {pin_num: B5, peripheral: ENET, signal: 'ENET_RX_DATA, 1', pin_signal: GPIO_23}
  - {pin_num: G11, peripheral: ENET, signal: 'ENET_TX_DATA, 0', pin_signal: GPIO_58, slew_rate: weak}
  - {pin_num: D12, peripheral: ENET, signal: 'ENET_TX_DATA, 1', pin_signal: GPIO_59, slew_rate: weak}
  - {pin_num: B13, peripheral: ENET, signal: ENET_MDIO, pin_signal: GPIO_57}
  - {pin_num: H4, peripheral: GPIO, signal: 'PIO0, 21', pin_signal: GPIO_21, direction: OUTPUT, gpio_init_state: 'true'}
  - {pin_num: K6, peripheral: GPIO, signal: 'PIO1, 23', pin_signal: GPIO_55, direction: INPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : 
 *
 * END ****************************************************************************************************************/
/* Function assigned for the Cortex-M33 */
void BOARD_InitPins(void)
{
    /* Enables the clock for the GPIO0 module */
    GPIO_PortInit(GPIO, 0);
    /* Enables the clock for the GPIO1 module */
    GPIO_PortInit(GPIO, 1);

    gpio_pin_config_t gpio0_pinH4_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 1U
    };
    /* Initialize GPIO functionality on pin PIO0_21 (pin H4)  */
    GPIO_PinInit(GPIO, 0U, 21U, &gpio0_pinH4_config);

    gpio_pin_config_t gpio1_pinK6_config = {
        .pinDirection = kGPIO_DigitalInput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PIO1_23 (pin K6)  */
    GPIO_PinInit(GPIO, 1U, 23U, &gpio1_pinK6_config);
    /* Initialize FC3_USART_DATA functionality on pin GPIO_24 (pin E5) */
    IO_MUX_SetPinMux(IO_MUX_FC3_USART_DATA);
    /* Initialize ENET_CLK functionality on pin GPIO_25 (pin E7) */
    IO_MUX_SetPinMux(IO_MUX_ENET_CLK);
    /* Initialize ENET_RX functionality on pin GPIO_22, GPIO_23 (pin B6_B5) */
    IO_MUX_SetPinMux(IO_MUX_ENET_RX);
    /* Initialize ENET_TX functionality on pin GPIO_58, GPIO_59 (pin G11_D12) */
    IO_MUX_SetPinMux(IO_MUX_ENET_TX);
    /* Initialize ENET_MDIO functionality on pin GPIO_57 (pin B13) */
    IO_MUX_SetPinMux(IO_MUX_ENET_MDIO);
    /* Initialize GPIO21 functionality on pin GPIO_21 (pin H4) */
    IO_MUX_SetPinMux(IO_MUX_GPIO21);
    /* Initialize GPIO55 functionality on pin GPIO_55 (pin K6) */
    IO_MUX_SetPinMux(IO_MUX_GPIO55);
    /* Set GPIO_25 (pin E7) configuration - Enable pull-up; weak slew rate */
    IO_MUX_SetPinConfig(25U, IO_MUX_PinConfigPullUpDriveWeak);
    /* Set GPIO_58 (pin G11_D12) configuration - Enable pull-up; weak slew rate */
    IO_MUX_SetPinConfig(58U, IO_MUX_PinConfigPullUpDriveWeak);
    /* Set GPIO_59 (pin G11_D12) configuration - Enable pull-up; weak slew rate */
    IO_MUX_SetPinConfig(59U, IO_MUX_PinConfigPullUpDriveWeak);
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
