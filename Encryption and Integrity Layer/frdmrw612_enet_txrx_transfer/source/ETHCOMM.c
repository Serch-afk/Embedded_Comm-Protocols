/*
 * ETHCOMM.c
 *
 *  Created on: 10 feb. 2025
 *      Author: Sergio R.
 */

#include "fsl_enet.h"
#include "fsl_phy.h"
#include "fsl_silicon_id.h"
#include "app.h"

#include "fsl_debug_console.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ENET_RXBD_NUM          (4)
#define ENET_TXBD_NUM          (4)
#define ENET_RXBUFF_SIZE       (ENET_FRAME_MAX_FRAMELEN)
#define ENET_TXBUFF_SIZE       (ENET_FRAME_MAX_FRAMELEN)
#define ENET_DATA_LENGTH       (1000)
#define ENET_TRANSMIT_DATA_NUM (20)

#define APP_ENET_BUFF_ALIGNMENT ENET_BUFF_ALIGNMENT

#define PHY_AUTONEGO_TIMEOUT_COUNT (300000)

#define MAC_ADDRESS                        \
    {                                      \
        0x54, 0x27, 0x8d, 0x00, 0x00, 0x00 \
    }
/*******************************************************************************
 * Variables
 ******************************************************************************/
/*! @brief Buffer descriptors should be in non-cacheable region and should be align to "ENET_BUFF_ALIGNMENT". */
AT_NONCACHEABLE_SECTION_ALIGN(enet_rx_bd_struct_t g_rxBuffDescrip[ENET_RXBD_NUM], ENET_BUFF_ALIGNMENT);
AT_NONCACHEABLE_SECTION_ALIGN(enet_tx_bd_struct_t g_txBuffDescrip[ENET_TXBD_NUM], ENET_BUFF_ALIGNMENT);

/*! @brief The data buffers can be in cacheable region or in non-cacheable region.
 * If use cacheable region, the alignment size should be the maximum size of "CACHE LINE SIZE" and "ENET_BUFF_ALIGNMENT"
 * If use non-cache region, the alignment size is the "ENET_BUFF_ALIGNMENT".
 */
SDK_ALIGN(uint8_t g_rxDataBuff[ENET_RXBD_NUM][SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)],
          APP_ENET_BUFF_ALIGNMENT);
SDK_ALIGN(uint8_t g_txDataBuff[ENET_TXBD_NUM][SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT)],
          APP_ENET_BUFF_ALIGNMENT);

/*! @brief MAC transfer. */
static enet_handle_t g_handle;
static uint8_t g_frame[ENET_DATA_LENGTH + 14];

/*! @brief The MAC address for ENET device. */
uint8_t g_macAddr[6] = MAC_ADDRESS;

/*! @brief PHY status. */
static phy_handle_t phyHandle;
/*******************************************************************************
 * Private functions
 ******************************************************************************/
/*! @brief Initialize PHY. */
static void ETHCOMM_vInitPHY(void)
{
	volatile uint32_t count = 0;
    phy_config_t phyConfig = {0};
    status_t status;

    bool link     = false;
    bool autonego = false;

    phyConfig.phyAddr = PHY_ADDRESS;
    phyConfig.autoNeg = true;
    phyConfig.ops      = PHY_OPS;
    phyConfig.resource = PHY_RESOURCE;

	do
	{
		status = PHY_Init(&phyHandle, &phyConfig);
		if (status == kStatus_Success)
		{
			/* Wait for auto-negotiation success and link up */
			count = PHY_AUTONEGO_TIMEOUT_COUNT;
			do
			{
				PHY_GetLinkStatus(&phyHandle, &link);
				if (link)
				{
					PHY_GetAutoNegotiationStatus(&phyHandle, &autonego);
					if (autonego)
					{
						break;
					}
				}
			} while (--count);
			if (!autonego)
			{
				PRINTF("PHY Auto-negotiation failed. Please check the cable connection and link partner setting.\r\n");
			}
		}
	} while (!(link && autonego));
}

/*! @brief Build Frame for transmit. */
static void ETHCOMM_vBuildBroadCastFrame(void)
{
    uint32_t count  = 0;
    uint32_t length = ENET_DATA_LENGTH - 14;

    for (count = 0; count < 6U; count++)
    {
        g_frame[count] = 0xFFU;
    }
    memcpy(&g_frame[6], &g_macAddr[0], 6U);
    g_frame[12] = (length >> 8) & 0xFFU;
    g_frame[13] = length & 0xFFU;

    for (count = 0; count < length; count++)
    {
        g_frame[count + 14] = count % 0xFFU;
    }
}

/*******************************************************************************
 * Global functions
 ******************************************************************************/
void ETHCOMM_vInit(void)
{
    enet_config_t config;
    phy_speed_t speed;
    phy_duplex_t duplex;

    /* Prepare the buffer configuration. */
    enet_buffer_config_t buffConfig[] = {{
        ENET_RXBD_NUM,
        ENET_TXBD_NUM,
        SDK_SIZEALIGN(ENET_RXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        SDK_SIZEALIGN(ENET_TXBUFF_SIZE, APP_ENET_BUFF_ALIGNMENT),
        &g_rxBuffDescrip[0],
        &g_txBuffDescrip[0],
        &g_rxDataBuff[0][0],
        &g_txDataBuff[0][0],
        true,
        true,
        NULL,
    }};

    /* Get default configuration. */
    /*
     * config.miiMode = kENET_RmiiMode;
     * config.miiSpeed = kENET_MiiSpeed100M;
     * config.miiDuplex = kENET_MiiFullDuplex;
     * config.rxMaxFrameLen = ENET_FRAME_MAX_FRAMELEN;
     */
    ENET_GetDefaultConfig(&config);

    /* The miiMode should be set according to the different PHY interfaces. */
    config.miiMode = kENET_RmiiMode;

	/* Initialize PHY and wait auto-negotiation over. */
    ETHCOMM_vInitPHY();

    /* Get the actual PHY link speed and set in MAC. */
    PHY_GetLinkSpeedDuplex(&phyHandle, &speed, &duplex);
    config.miiSpeed  = (enet_mii_speed_t)speed;
    config.miiDuplex = (enet_mii_duplex_t)duplex;

    /* Set special address for each chip. */
    SILICONID_ConvertToMacAddr(&g_macAddr);

    /* Init the ENET. */
    ENET_Init(EXAMPLE_ENET, &g_handle, &config, &buffConfig[0], &g_macAddr[0], EXAMPLE_CLOCK_FREQ);
    ENET_ActiveRead(EXAMPLE_ENET);

    /* Build broadcast for sending. */
    ETHCOMM_vBuildBroadCastFrame();
}
