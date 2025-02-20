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
#include "fsl_crc.h"
#include "aes.h"
#include "ETHCOMM_CFG.h"

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

#define ETHCOMM_CRC32_DATA_SIZE     (4)
#define ETHCOMM_MAC_DATA_SIZE       (6)
#define ETHCOMM_HEADER_MSG_SIZE (14 + ETHCOMM_CRC32_DATA_SIZE)

#define ETHCOMM_DATA_LENGHT_INDEX   (12)
#define ETHCOMM_DATA_BUFFER_INDEX   (14)

#define APP_ENET_BUFF_ALIGNMENT ENET_BUFF_ALIGNMENT
#define PHY_AUTONEGO_TIMEOUT_COUNT (300000)

#define SWAP16(value) (((value >> 8) & 0x00FF) | \
                       ((value << 8) & 0xFF00))

/*******************************************************************************
 * Data Types
 ******************************************************************************/
typedef struct
{
	uint8_t MACdst[ETHCOMM_MAC_DATA_SIZE];
	uint8_t MACsrc[ETHCOMM_MAC_DATA_SIZE];
	uint16_t DataLenght;
	uint8_t DataBuffer[ENET_DATA_LENGTH];
}tstEthMsg;

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
/*! @brief PHY status. */
static phy_handle_t phyHandle;
static CRC_Type *CRC_base = CRC_ENGINE;

static uint8_t key[16] = ETHCOMM_ENCRYPT_KEY;
static uint8_t iv[16] = ETHCOMM_ENCRYPT_IV;

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

static uint16_t ETHCOM_u16PKCS7Padding(uint8_t* pu8Buffer, uint8_t* pu8PaddedBuff, uint16_t u16DataLength)
{
	uint16_t u16PaddedLength = 0U;
	uint16_t u16PadSize = 0U;
	uint16_t u16i;

	u16PadSize = AES_BLOCKLEN - (u16DataLength % AES_BLOCKLEN);

	if(u16PadSize != 0)
	{
		u16PaddedLength = u16DataLength + u16PadSize;
		memcpy(pu8PaddedBuff, pu8Buffer, u16DataLength);

		for(u16i = u16DataLength; u16i <= u16PaddedLength; u16i++)
		{
			pu8PaddedBuff[u16i] = u16PadSize;
		}
	}
	else
	{
		u16PaddedLength = u16DataLength + AES_BLOCKLEN;
		memcpy(pu8PaddedBuff, pu8Buffer, u16DataLength);

		for(u16i = u16DataLength; u16i <= u16PaddedLength; u16i++)
		{
			pu8PaddedBuff[u16i] = AES_BLOCKLEN;
		}
	}

	return u16PaddedLength;
}

static void ETHCOMM_vInitCrc32(void)
{
    crc_config_t config;

    config.polynomial    = kCRC_Polynomial_CRC_32;
    config.reverseIn     = true;
    config.complementIn  = false;
    config.reverseOut    = true;
    config.complementOut = true;
    config.seed          = 0xFFFFFFFFU;

    CRC_Init(CRC_base, &config);
}

static bool ETHCOMM_CheckCRC(uint8_t* pu8Buffer, uint16_t u16MsgLenght)
{
	bool Status = false;
	uint32_t MsgCRC = 0;
	uint32_t CalCRC = 0;

	/* Se obtiene el CRC del mensaje */
	memcpy((uint8_t*)&MsgCRC, &pu8Buffer[u16MsgLenght], ETHCOMM_CRC32_DATA_SIZE);

	/* Se calcula el CRC*/
	ETHCOMM_vInitCrc32();
	CRC_WriteData(CRC_base, pu8Buffer, u16MsgLenght);
	CalCRC = CRC_Get32bitResult(CRC_base);

	/*Se comparan los CRC*/
	if(MsgCRC == CalCRC)
	{
		Status = true;
	}

	return Status;
}

/*******************************************************************************
 * Global functions
 ******************************************************************************/
void ETHCOMM_vInit(void)
{
    enet_config_t config;
    phy_speed_t speed;
    phy_duplex_t duplex;
    uint8_t g_macAddr[ETHCOMM_MAC_DATA_SIZE] = ETHCOM_MAC_ADDRESS_SRC;

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

    /*Reject broadcast msg*/
    config.macSpecialConfig = kENET_ControlRxBroadCastRejectEnable;

    /* Init the ENET. */
    ENET_Init(EXAMPLE_ENET, &g_handle, &config, &buffConfig[0], &g_macAddr[0], EXAMPLE_CLOCK_FREQ);
    ENET_ActiveRead(EXAMPLE_ENET);
}

void ETHCOMM_vMsgSend(uint8_t* pu8Buffer, uint16_t u16DataLength)
{
	struct AES_ctx ctx;
	uint32_t u32CRC = 0;
	uint16_t u16MsgLenght = 0;
	bool link = false;

	tstEthMsg stMsgInfo = {
			.MACdst = ETHCOM_MAC_ADDRESS_DEST,
			.MACsrc = ETHCOM_MAC_ADDRESS_SRC,
	};

	/* Calculo de padding #PKCS7 para la encriptacion del mensaje */
	u16MsgLenght = ETHCOM_u16PKCS7Padding(pu8Buffer, stMsgInfo.DataBuffer, u16DataLength);

	/* Encriptacion del mensaje */
	AES_init_ctx_iv(&ctx, key, iv);
	AES_CBC_encrypt_buffer(&ctx, stMsgInfo.DataBuffer, u16MsgLenght);

	/* Calculo del CRC */
    ETHCOMM_vInitCrc32();
	CRC_WriteData(CRC_base, stMsgInfo.DataBuffer, u16MsgLenght);
	u32CRC = CRC_Get32bitResult(CRC_base);

	stMsgInfo.DataLenght = SWAP16(u16MsgLenght + ETHCOMM_CRC32_DATA_SIZE);
	memcpy(&stMsgInfo.DataBuffer[u16MsgLenght], (uint8_t*)&u32CRC, ETHCOMM_CRC32_DATA_SIZE);

	PHY_GetLinkStatus(&phyHandle, &link);
	if(link)
	{
		ENET_SendFrame(EXAMPLE_ENET, &g_handle, (uint8_t*)&stMsgInfo, u16MsgLenght + ETHCOMM_HEADER_MSG_SIZE, 0, false, NULL);
	}
}

bool ETHCOMM_MsgReceive(uint8_t* pu8MsgBuffer)
{
	enet_data_error_stats_t eErrStatic;
	uint32_t length = 0;
	status_t status;
	uint16_t MsgLenght;
	bool CRC_check = false;
	struct AES_ctx ctx;

	/* Get the Frame size */
	status = ENET_GetRxFrameSize(&g_handle, &length, 0);
	/* Call ENET_ReadFrame when there is a received frame. */
	if (length != 0)
	{
		/* Received valid frame. Deliver the rx buffer with the size equal to length. */
		uint8_t *data = (uint8_t *)malloc(length);
		status = ENET_ReadFrame(EXAMPLE_ENET, &g_handle, data, length, 0, NULL);
		if (status == kStatus_Success)
		{
			/*Get data lenght*/
			memcpy((uint8_t*)&MsgLenght, &data[ETHCOMM_DATA_LENGHT_INDEX], sizeof(MsgLenght));
			MsgLenght = SWAP16(MsgLenght) - ETHCOMM_CRC32_DATA_SIZE;

			/*CRC Check*/
			CRC_check = ETHCOMM_CheckCRC(&data[ETHCOMM_DATA_BUFFER_INDEX], (uint16_t)MsgLenght);
			if(CRC_check == true)
			{
				memcpy(pu8MsgBuffer, &data[ETHCOMM_DATA_BUFFER_INDEX], MsgLenght);

				/*Decrypt Msg*/
				AES_init_ctx_iv(&ctx, key, iv);
				AES_CBC_decrypt_buffer(&ctx, pu8MsgBuffer, MsgLenght);
			}
		}

		free(data);
	}
	else if (status == kStatus_ENET_RxFrameError)
	{
		/* Update the received buffer when error happened. */
		/* Get the error information of the received g_frame. */
		ENET_GetRxErrBeforeReadFrame(&g_handle, &eErrStatic, 0);
		/* update the receive buffer. */
		ENET_ReadFrame(EXAMPLE_ENET, &g_handle, NULL, 0, 0, NULL);
	}

	return CRC_check;
}











