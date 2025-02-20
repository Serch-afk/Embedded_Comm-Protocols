/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_silicon_id.h"
#include "board.h"
#include "app.h"
#include "ETHCOMM.h"
#include "ETHCOMM_CFG.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t testTxNum = 0;
    bool NewMsg = false;
    uint8_t Msg[100] = {0};

    /* Hardware Initialization. */
    BOARD_InitHardware();
    ETHCOMM_vInit();

    while (1)
    {
        if (testTxNum < 10)
        {
        	uint8_t Data[10] = {"HOLA MUNDO"};
        	uint32_t u32DataLength = sizeof(Data);
			testTxNum++;
			ETHCOMM_vMsgSend((uint8_t*)&Data, u32DataLength);
		}

        NewMsg = ETHCOMM_MsgReceive(Msg);
        if(NewMsg == true)
        {
        	PRINTF("%s\r\n", Msg);
        }

    }
}
