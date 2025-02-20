/*
 * ETHCOMM.h
 *
 *  Created on: 10 feb. 2025
 *      Author: cool_
 */

#ifndef ETHCOMM_H_
#define ETHCOMM_H_


/*******************************************************************************
 * API
 ******************************************************************************/
extern void ETHCOMM_vInit(void);
extern void ETHCOMM_vMsgSend(uint8_t* pu8Buffer, uint32_t u32DataLength);
extern uint16_t ETHCOMM_u16MsgReceive(uint8_t* pu8MsgBuffer);

#endif /* ETHCOMM_H_ */
