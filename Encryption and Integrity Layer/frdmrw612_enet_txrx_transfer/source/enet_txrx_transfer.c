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
#define TOTAL_QUOTES (16)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
static uint8_t *quotes[TOTAL_QUOTES] = {
	    (uint8_t *)"No todo lo que es oro reluce...",
	    (uint8_t *)"Aún en la oscuridad...",
	    (uint8_t *)"¿Qué es la vida?",
	    (uint8_t *)"No temas a la oscuridad...",
	    (uint8_t *)"Hasta los más pequeños...",
	    (uint8_t *)"No digas que el sol se ha puesto...",
	    (uint8_t *)"El coraje se encuentra...",
	    (uint8_t *)"No todos los tesoros...",
	    (uint8_t *)"Es peligroso...",
	    (uint8_t *)"Un mago nunca llega tarde...",
	    (uint8_t *)"Aún hay esperanza...",
	    (uint8_t *)"El mundo está cambiando...",
	    (uint8_t *)"Las raíces profundas...",
	    (uint8_t *)"No se puede...",
	    (uint8_t *)"Y sobre todo...",
	    (uint8_t *)"De las cenizas, un fuego..."
};

static bool replyflag = false;
/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
int main(void)
{
	uint8_t Quote_Indx = 0;
	uint8_t Response[100] = {0};
	uint16_t NewMsg = 0U;

    /* Hardware Initialization. */
    BOARD_InitHardware();
    ETHCOMM_vInit();

    while (1)
    {
        while(Quote_Indx < TOTAL_QUOTES)
        {
        	if(replyflag)
        	{
        		memset(&Response, 0, sizeof(Response));
        		NewMsg = ETHCOMM_u16MsgReceive(Response);
        		if(NewMsg != 0)
        		{
        			PRINTF("Mensaje recibido: %s\r\n", Response);
        			replyflag = false;
        			Quote_Indx++;
        		}
        	}
        	else
        	{
        		ETHCOMM_vMsgSend(quotes[Quote_Indx], strlen(quotes[Quote_Indx]));
        		PRINTF("\nMensaje enviado: %s\r\n", quotes[Quote_Indx]);
        		replyflag = true;
        	}

        }
    }
}
