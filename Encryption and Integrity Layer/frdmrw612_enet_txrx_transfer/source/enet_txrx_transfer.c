/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2024 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "fsl_silicon_id.h"
#include "fsl_enet.h"
#include "fsl_phy.h"
#include "board.h"
#include "app.h"
#include "ETHCOMM.h"
#include "ETHCOMM_CFG.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ENET_DATA_LENGTH       (1000)
#define ENET_TRANSMIT_DATA_NUM (20)

#ifndef EXAMPLE_PHY_LINK_INTR_SUPPORT
#define EXAMPLE_PHY_LINK_INTR_SUPPORT (0U)
#endif
#ifndef EXAMPLE_USES_LOOPBACK_CABLE
#define EXAMPLE_USES_LOOPBACK_CABLE (1U)
#endif

#ifndef PHY_STABILITY_DELAY_US
#if EXAMPLE_USES_LOOPBACK_CABLE
#define PHY_STABILITY_DELAY_US (0U)
#else
/* If cable is not used there is no "readiness wait" caused by auto negotiation. Lets wait 100ms.*/
#define PHY_STABILITY_DELAY_US (100000U)
#endif
#endif


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief MAC transfer. */
static enet_handle_t g_handle;
static uint8_t g_frame[ENET_DATA_LENGTH + 14];

/*! @brief PHY status. */
static phy_handle_t phyHandle;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
    uint32_t testTxNum     = 0;
    uint32_t length        = 0;
    enet_data_error_stats_t eErrStatic;
    status_t status;
    bool link     = false;
    bool tempLink = false;

    /* Hardware Initialization. */
    BOARD_InitHardware();

    PRINTF("\r\nENET example start.\r\n");
    ETHCOMM_vInit();

    while (1)
    {

        /* PHY link status update. */
#if (defined(EXAMPLE_PHY_LINK_INTR_SUPPORT) && (EXAMPLE_PHY_LINK_INTR_SUPPORT))
        if (linkChange)
        {
            linkChange = false;
            PHY_ClearInterrupt(&phyHandle);
            PHY_GetLinkStatus(&phyHandle, &link);
            GPIO_EnableLinkIntr();
        }
//#else
//        PHY_GetLinkStatus(&phyHandle, &link);
//#endif
//        if (tempLink != link)
//        {
//            PRINTF("PHY link changed, link status = %u\r\n", link);
//            tempLink = link;
//        }
//#endif /*EXAMPLE_USES_LOOPBACK_CABLE*/
//        /* Get the Frame size */
//        status = ENET_GetRxFrameSize(&g_handle, &length, 0);
//        /* Call ENET_ReadFrame when there is a received frame. */
//        if (length != 0)
//        {
//            /* Received valid frame. Deliver the rx buffer with the size equal to length. */
//            uint8_t *data = (uint8_t *)malloc(length);
//            status        = ENET_ReadFrame(EXAMPLE_ENET, &g_handle, data, length, 0, NULL);
//            if (status == kStatus_Success)
//            {
//                PRINTF(" A frame received. the length %d ", length);
//                PRINTF(" Dest Address %02x:%02x:%02x:%02x:%02x:%02x Src Address %02x:%02x:%02x:%02x:%02x:%02x \r\n",
//                       data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8], data[9],
//                       data[10], data[11]);
//            }
//            free(data);
//        }
//        else if (status == kStatus_ENET_RxFrameError)
//        {
//            /* Update the received buffer when error happened. */
//            /* Get the error information of the received g_frame. */
//            ENET_GetRxErrBeforeReadFrame(&g_handle, &eErrStatic, 0);
//            /* update the receive buffer. */
//            ENET_ReadFrame(EXAMPLE_ENET, &g_handle, NULL, 0, 0, NULL);
//        }
#endif
        if (testTxNum < ENET_TRANSMIT_DATA_NUM)
        {
        	uint8_t Data[10] = {"HOLA MUNDO"};
        	uint32_t u32DataLength = sizeof(Data);
			testTxNum++;
			ETHCOMM_vMsgSend((uint8_t*)&Data, u32DataLength);
		}

    }
}
