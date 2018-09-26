/*
 * CAN module object for NXP K64F microcontroller, using Kinetis SDK V2.
 *
 * @file        CO_driver.c
 * @ingroup     CO_driver
 * @author      Janez Paternoster
 * @author		Matthew Watson
 * @copyright   2004 - 2015 Janez Paternoster
 * @copyright 	2017-2018 Matthew Watson
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * CANopenNode is free and open source software: you can redistribute
 * it and/or modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, either version 2 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Following clarification and special exception to the GNU General Public
 * License is included to the distribution terms of CANopenNode:
 *
 * Linking this library statically or dynamically with other modules is
 * making a combined work based on this library. Thus, the terms and
 * conditions of the GNU General Public License cover the whole combination.
 *
 * As a special exception, the copyright holders of this library give
 * you permission to link this library with independent modules to
 * produce an executable, regardless of the license terms of these
 * independent modules, and to copy and distribute the resulting
 * executable under terms of your choice, provided that you also meet,
 * for each linked independent module, the terms and conditions of the
 * license of that module. An independent module is a module which is
 * not derived from or based on this library. If you modify this
 * library, you may extend this exception to your version of the
 * library, but you are not obliged to do so. If you do not wish
 * to do so, delete this exception statement from your version.
 */


#include "CO_driver.h"
#include "CO_Emergency.h"
#include "fsl_flexcan.h"

/* Use either vanilla mb or FIFO to store messages. FIFO currently not implemented. */
//#define USE_RX_FIFO
#define USE_RX_MB


/******************************************************************************/
void CO_CANsetConfigurationMode(int32_t CANbaseAddress){
    /* Put CAN module in configuration mode */
//	FLEXCAN_EnterFreezeMode();
//	FLEXCAN_DisableMbInterrupts(CAN0, 0xFFFF);
}


/******************************************************************************/
void CO_CANsetNormalMode(CO_CANmodule_t *CANmodule){
    /* Put CAN module in normal mode */
//	FLEXCAN_ExitFreezeMode();

	FLEXCAN_EnableMbInterrupts(CAN0, 0xFFFF);
    CANmodule->CANnormal = true;
}


/******************************************************************************/
CO_ReturnError_t CO_CANmodule_init(
        CO_CANmodule_t         *CANmodule,
        int32_t                 CANbaseAddress,
        CO_CANrx_t              rxArray[],
        uint16_t                rxSize,
        CO_CANtx_t              txArray[],
        uint16_t                txSize,
        uint16_t                CANbitRate)
{
    uint16_t i;

    /* verify arguments */
    if(CANmodule==NULL || rxArray==NULL || txArray==NULL){
        return CO_ERROR_ILLEGAL_ARGUMENT;
    }

    /* Configure object variables */
       CANmodule->CANbaseAddress = CANbaseAddress;
       CANmodule->rxArray = rxArray;
       CANmodule->rxSize = rxSize;
       CANmodule->txArray = txArray;
       CANmodule->txSize = txSize;
       CANmodule->CANnormal = false;
       CANmodule->useCANrxFilters = (rxSize <= 16U) ? true : false;/* microcontroller dependent */
       CANmodule->bufferInhibitFlag = false;
       CANmodule->firstCANtxMessage = true;
       CANmodule->CANtxCount = 0U;
       CANmodule->errOld = 0U;
       CANmodule->em = NULL;

       for(i=0U; i<rxSize; i++){
           rxArray[i].ident = 0U;
           rxArray[i].pFunct = NULL;
       }
       for(i=0U; i<txSize; i++){
           txArray[i].bufferFull = false;
       }


#ifdef USE_RX_FIFO
       /* Configure CAN module registers */
       flexcan_config_t flexcanConfig;
       flexcanConfig.clkSrc            = kFLEXCAN_ClkSrcPeri;
       flexcanConfig.baudRate          = 1000000U;
       flexcanConfig.maxMbNum          = 16;
       flexcanConfig.enableLoopBack    = false;
       flexcanConfig.enableSelfWakeup  = false;
       flexcanConfig.enableIndividMask = false;
       flexcanConfig.timingConfig.phaseSeg1 = 7;
       flexcanConfig.timingConfig.phaseSeg2 = 2;
       flexcanConfig.timingConfig.propSeg = 7;
       flexcanConfig.timingConfig.rJumpwidth = 0;
       flexcanConfig.timingConfig.preDivider = 2;
       FLEXCAN_Init(CAN0, &flexcanConfig, 60000000UL);

       /* Disable self reception */
       CAN0->MCR |= CAN_MCR_SRXDIS(1);


       /* Configure rx FIFO */
       flexcan_rx_fifo_config_t rx_fifo_config;
       rx_fifo_config.
       rx_mb_config.id = FLEXCAN_ID_STD(0); /* Not used, as we have enableIndividMask = false */

       for (int i = 0; i<=13; i++)
       	FLEXCAN_SetRxMbConfig(CAN0, i, &rx_mb_config, true);

       /* Configure tx message buffers */
       for (int i = 14; i<=15; i++)
       	FLEXCAN_SetTxMbConfig(CAN0, i, true);

       /* Configure CAN module hardware filters */
       if(CANmodule->useCANrxFilters){
           /* CAN module filters are used, they will be configured with */
           /* CO_CANrxBufferInit() functions, called by separate CANopen */
           /* init functions. */
           /* Configure all masks so, that received message must match filter */
       }
       else{
           /* CAN module filters are not used, all messages with standard 11-bit */
           /* identifier will be received */
           /* Configure mask 0 so, that all messages with standard identifier are accepted */
       	FLEXCAN_SetRxMbGlobalMask(CAN0, 0x00080000);
       }

       /* configure CAN interrupt registers */
       NVIC_SetPriority(CAN0_ORed_Message_buffer_IRQn, 1);
       NVIC_EnableIRQ(CAN0_ORed_Message_buffer_IRQn);
       FLEXCAN_ClearMbStatusFlags(CAN0, 0xFFFF);
   	FLEXCAN_EnableMbInterrupts(CAN0, 0xFFFF);

#elif defined(USE_RX_MB)

    /* Configure CAN module registers */
    flexcan_config_t flexcanConfig;
    flexcanConfig.clkSrc            = kFLEXCAN_ClkSrcPeri;
    flexcanConfig.baudRate          = 1000000U;
    flexcanConfig.maxMbNum          = 16;
    flexcanConfig.enableLoopBack    = false;
    flexcanConfig.enableSelfWakeup  = false;
    flexcanConfig.enableIndividMask = false;
    flexcanConfig.timingConfig.phaseSeg1 = 7;
    flexcanConfig.timingConfig.phaseSeg2 = 2;
    flexcanConfig.timingConfig.propSeg = 7;
    flexcanConfig.timingConfig.rJumpwidth = 0;
    flexcanConfig.timingConfig.preDivider = 2;
    FLEXCAN_Init(CAN0, &flexcanConfig, 60000000UL);


    /* Set Freeze, Halt bits. */
    CAN0->MCR |= CAN_MCR_HALT_MASK;
	/* Wait until the FlexCAN Module enter freeze mode. */
	while (!(CAN0->MCR & CAN_MCR_FRZACK_MASK));

	/* Disable self reception */
	CAN0->MCR |= CAN_MCR_SRXDIS(1);

	/* Clear Freeze, Halt bits. */
	CAN0->MCR &= ~CAN_MCR_HALT_MASK;
	/* Wait until the FlexCAN Module exit freeze mode. */
	while (CAN0->MCR & CAN_MCR_FRZACK_MASK);


    /* Configure rx message buffers */
    flexcan_rx_mb_config_t rx_mb_config;
    rx_mb_config.format = kFLEXCAN_FrameFormatStandard;
    rx_mb_config.type = kFLEXCAN_FrameTypeData;
    rx_mb_config.id = FLEXCAN_ID_STD(0); /* Not used, as we have enableIndividMask = false */

    for (int i = 0; i<=13; i++)
    	FLEXCAN_SetRxMbConfig(CAN0, i, &rx_mb_config, true);

    /* Configure tx message buffers */
    for (int i = 14; i<=15; i++)
    	FLEXCAN_SetTxMbConfig(CAN0, i, true);

    /* Configure CAN module hardware filters */
    if(CANmodule->useCANrxFilters){
        /* CAN module filters are used, they will be configured with */
        /* CO_CANrxBufferInit() functions, called by separate CANopen */
        /* init functions. */
        /* Configure all masks so, that received message must match filter */
    }
    else{
        /* CAN module filters are not used, all messages with standard 11-bit */
        /* identifier will be received */
        /* Configure mask 0 so, that all messages with standard identifier are accepted */
    	FLEXCAN_SetRxMbGlobalMask(CAN0, 0);
    }

    /* configure CAN interrupt registers */
    NVIC_SetPriority(CAN0_ORed_Message_buffer_IRQn, 1);
    NVIC_EnableIRQ(CAN0_ORed_Message_buffer_IRQn);
    FLEXCAN_ClearMbStatusFlags(CAN0, 0xFFFF);
	FLEXCAN_EnableMbInterrupts(CAN0, 0xFFFF);
#endif

    return CO_ERROR_NO;
}


/******************************************************************************/
void CO_CANmodule_disable(CO_CANmodule_t *CANmodule){
    /* turn off the module */
	NVIC_DisableIRQ(CAN0_ORed_Message_buffer_IRQn);
	FLEXCAN_Deinit(CAN0);
}


/******************************************************************************/
uint16_t CO_CANrxMsg_readIdent(const CO_CANrxMsg_t *rxMsg){
    return (rxMsg->ident & CAN_ID_STD_MASK) >> CAN_ID_STD_SHIFT;
}


/******************************************************************************/
CO_ReturnError_t CO_CANrxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        uint16_t                mask,
        bool_t                  rtr,
        void                   *object,
        void                  (*pFunct)(void *object, const CO_CANrxMsg_t *message))
{
    CO_ReturnError_t ret = CO_ERROR_NO;

    if((CANmodule!=NULL) && (object!=NULL) && (pFunct!=NULL) && (index < CANmodule->rxSize)){
        /* buffer, which will be configured */
        CO_CANrx_t *buffer = &CANmodule->rxArray[index];

        /* Configure object variables */
        buffer->object = object;
        buffer->pFunct = pFunct;

        /* CAN identifier and CAN mask */
        buffer->ident = ident & 0x07FFU;
        buffer->mask = mask & 0x07FFU;

        /* Set CAN hardware module filter and mask. */
        if(CANmodule->useCANrxFilters){

        }
    }
    else{
        ret = CO_ERROR_ILLEGAL_ARGUMENT;
    }

    return ret;
}


/******************************************************************************/
CO_CANtx_t *CO_CANtxBufferInit(
        CO_CANmodule_t         *CANmodule,
        uint16_t                index,
        uint16_t                ident,
        bool_t                  rtr,
        uint8_t                 noOfBytes,
        bool_t                  syncFlag)
{
    CO_CANtx_t *buffer = NULL;

    if((CANmodule != NULL) && (index < CANmodule->txSize)){
        /* get specific buffer */
        buffer = &CANmodule->txArray[index];

        /* CAN identifier, DLC and rtr */
        buffer->ident = (uint16_t) ((ident & 0x07FF));
        buffer->DLC = noOfBytes;

        buffer->bufferFull = false;
        buffer->syncFlag = syncFlag;
    }

    return buffer;
}


/******************************************************************************/
CO_ReturnError_t CO_CANsend(CO_CANmodule_t *CANmodule, CO_CANtx_t *buffer){
    CO_ReturnError_t err = CO_ERROR_NO;

    /* Verify overflow */
    if(buffer->bufferFull){
        if(!CANmodule->firstCANtxMessage){
            /* don't set error, if bootup message is still on buffers */
            CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_CAN_TX_OVERFLOW, CO_EMC_CAN_OVERRUN, buffer->ident);
        }
        err = CO_ERROR_TX_OVERFLOW;
    }

    /* Build FLEXCAN flexcan_frame_t from CANOpenNode CO_CANtx_t. Awkward due to differing endianess between MCU and CANOpen, preventing an inplace copy. */
    flexcan_frame_t txFrame;
    txFrame.id 			= FLEXCAN_ID_STD(buffer->ident);
    txFrame.length 		= buffer->DLC;
    txFrame.format		= 0; // STD format
    txFrame.type		= 0;
    txFrame.dataByte0	= buffer->data[0];
    txFrame.dataByte1	= buffer->data[1];
    txFrame.dataByte2	= buffer->data[2];
    txFrame.dataByte3	= buffer->data[3];
    txFrame.dataByte4	= buffer->data[4];
    txFrame.dataByte5	= buffer->data[5];
    txFrame.dataByte6	= buffer->data[6];
    txFrame.dataByte7	= buffer->data[7];

    CO_LOCK_CAN_SEND();
    status_t status = kStatus_Fail;
    /* are there any queued messages? */
    if(CANmodule->CANtxCount == 0){
    	/* Attempt to send message using each buffer in turn */
    	for (uint32_t mbIdx = 14; mbIdx<16; mbIdx++)
    	{
    		status = FLEXCAN_WriteTxMb(CAN0, mbIdx, &txFrame);
			if (status == kStatus_Success)
			{
				CANmodule->bufferInhibitFlag = buffer->syncFlag;
				break;
			}
    	}
    }

    /* if no buffer is free, message will be sent by interrupt */
    if (status == kStatus_Fail){
        buffer->bufferFull = true;
        CANmodule->CANtxCount++;
    }
    CO_UNLOCK_CAN_SEND();

    return err;
}


/******************************************************************************/
void CO_CANclearPendingSyncPDOs(CO_CANmodule_t *CANmodule){
    uint32_t tpdoDeleted = 0U;

    CO_LOCK_CAN_SEND();
    /* Abort message from CAN module, if there is synchronous TPDO.
     * Take special care with this functionality. */
    if(/*messageIsOnCanBuffer && */CANmodule->bufferInhibitFlag){
        /* clear TXREQ */
        CANmodule->bufferInhibitFlag = false;
        tpdoDeleted = 1U;
    }
    /* delete also pending synchronous TPDOs in TX buffers */
    if(CANmodule->CANtxCount != 0U){
        uint16_t i;
        CO_CANtx_t *buffer = &CANmodule->txArray[0];
        for(i = CANmodule->txSize; i > 0U; i--){
            if(buffer->bufferFull){
                if(buffer->syncFlag){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;
                    tpdoDeleted = 2U;
                }
            }
            buffer++;
        }
    }
    CO_UNLOCK_CAN_SEND();


    if(tpdoDeleted != 0U){
        CO_errorReport((CO_EM_t*)CANmodule->em, CO_EM_TPDO_OUTSIDE_WINDOW, CO_EMC_COMMUNICATION, tpdoDeleted);
    }
}


/******************************************************************************/
void CO_CANverifyErrors(CO_CANmodule_t *CANmodule){
    uint16_t rxErrors, txErrors, overflow;
    CO_EM_t* em = (CO_EM_t*)CANmodule->em;
    uint32_t err;
    uint8_t FLEXCANtxErrors, FLEXCANrxErrors;

    /* get error counters from module. Id possible, function may use different way to
     * determine errors. */
    FLEXCAN_GetBusErrCount(CAN0, &FLEXCANtxErrors, &FLEXCANrxErrors);
    overflow = 0; // @TODO Consider updating this

    txErrors = FLEXCANtxErrors;
    rxErrors = FLEXCANrxErrors;

    err = ((uint32_t)txErrors << 16) | ((uint32_t)rxErrors << 8) | overflow;

    if(CANmodule->errOld != err){
        CANmodule->errOld = err;

        if(txErrors >= 256U){                               /* bus off */
            CO_errorReport(em, CO_EM_CAN_TX_BUS_OFF, CO_EMC_BUS_OFF_RECOVERED, err);
        }
        else{                                               /* not bus off */
            CO_errorReset(em, CO_EM_CAN_TX_BUS_OFF, err);

            if((rxErrors >= 96U) || (txErrors >= 96U)){     /* bus warning */
                CO_errorReport(em, CO_EM_CAN_BUS_WARNING, CO_EMC_NO_ERROR, err);
            }

            if(rxErrors >= 128U){                           /* RX bus passive */
                CO_errorReport(em, CO_EM_CAN_RX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
            }
            else{
                CO_errorReset(em, CO_EM_CAN_RX_BUS_PASSIVE, err);
            }

            if(txErrors >= 128U){                           /* TX bus passive */
                if(!CANmodule->firstCANtxMessage){
                    CO_errorReport(em, CO_EM_CAN_TX_BUS_PASSIVE, CO_EMC_CAN_PASSIVE, err);
                }
            }
            else{
                bool_t isError = CO_isError(em, CO_EM_CAN_TX_BUS_PASSIVE);
                if(isError){
                    CO_errorReset(em, CO_EM_CAN_TX_BUS_PASSIVE, err);
                    CO_errorReset(em, CO_EM_CAN_TX_OVERFLOW, err);
                }
            }

            if((rxErrors < 96U) && (txErrors < 96U)){       /* no error */
                CO_errorReset(em, CO_EM_CAN_BUS_WARNING, err);
            }
        }

        if(overflow != 0U){                                 /* CAN RX bus overflow */
            CO_errorReport(em, CO_EM_CAN_RXB_OVERFLOW, CO_EMC_CAN_OVERRUN, err);
        }
    }
}


/******************************************************************************/
void CO_CANinterrupt(CO_CANmodule_t *CANmodule){

	/* Determine index of message buffer that generated interrupt. Returns 17 if no mb interrupt flags are set, should never happen. */
	uint32_t mbIndex = 0;
	for (mbIndex = 0; mbIndex < 17; mbIndex++) {
		if (FLEXCAN_GetMbStatusFlags(CAN0, 0xFFFF) & (0x01U << mbIndex))
			break;
	}

    /* receive interrupt */
    if(mbIndex <= 13){
        CO_CANrxMsg_t *rcvMsg;      /* pointer to received message in CAN module */
        uint16_t index;             /* index of received message */
        uint32_t rcvMsgIdent;       /* identifier of the received message */
        CO_CANrx_t *buffer = NULL;  /* receive message buffer from CO_CANmodule_t object. */
        bool_t msgMatched = false;

        /* Get received message from message buffer */
        flexcan_frame_t rxFrame;
        if(FLEXCAN_ReadRxMb(CAN0, mbIndex, &rxFrame) != kStatus_Success)
        {
        	while(1) /* Either tried to read empty MB or the MB has overflowed */
        		;
        }

		/* Create instance of CO_CANrxMsg_t for rcvMsg to point to. This was intended to represent the CAN module mb memory space, however this doesn't work for us
		 * due to differing endianess */
		CO_CANrxMsg_t CANOpenMsg;
		rcvMsg = &CANOpenMsg;

		/* Transfer FLEXCAN flexcan_frame_t into CANOpenNode CO_CANrxMsg_t. This is awkward due to differing endianess between MCU and CANOpen protocol. */
        rcvMsg->ident 	= rxFrame.id;
        rcvMsg->DLC		= rxFrame.length;
        rcvMsg->data[0]	= rxFrame.dataByte0;
        rcvMsg->data[1]	= rxFrame.dataByte1;
        rcvMsg->data[2]	= rxFrame.dataByte2;
        rcvMsg->data[3]	= rxFrame.dataByte3;
        rcvMsg->data[4]	= rxFrame.dataByte4;
        rcvMsg->data[5]	= rxFrame.dataByte5;
        rcvMsg->data[6]	= rxFrame.dataByte6;
        rcvMsg->data[7]	= rxFrame.dataByte7;

        rcvMsgIdent = CO_CANrxMsg_readIdent(rcvMsg);
        if(CANmodule->useCANrxFilters){
            /* CAN module filters are used. Message with known 11-bit identifier has */
            /* been received */
            index = 0;  /* get index of the received message here. Or something similar */
            if(index < CANmodule->rxSize){
                buffer = &CANmodule->rxArray[index];
                /* verify also RTR */
                if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                }
            }
        }
        else{
            /* CAN module filters are not used, message with any standard 11-bit identifier */
            /* has been received. Search rxArray form CANmodule for the same CAN-ID. */
            buffer = &CANmodule->rxArray[0];
            for(index = CANmodule->rxSize; index > 0U; index--){
                if(((rcvMsgIdent ^ buffer->ident) & buffer->mask) == 0U){
                    msgMatched = true;
                    break;
                }
                buffer++;
            }
        }

        /* Call specific function, which will process the message */
        if(msgMatched && (buffer != NULL) && (buffer->pFunct != NULL)){
            buffer->pFunct(buffer->object, rcvMsg);
        }

        /* Clear interrupt flag */
        FLEXCAN_ClearMbStatusFlags(CAN0, 1<<mbIndex);
    }
    /* transmit interrupt */
    else if (mbIndex >= 14 && mbIndex <= 15) {
        /* Clear interrupt flag */
        FLEXCAN_ClearMbStatusFlags(CAN0, 1<<mbIndex);

        /* First CAN message (bootup) was sent successfully */
        CANmodule->firstCANtxMessage = false;
        /* clear flag from previous message */
        CANmodule->bufferInhibitFlag = false;
        /* Are there any new messages waiting to be send */
        if(CANmodule->CANtxCount > 0U){
            uint16_t i;             /* index of transmitting message */

            /* first buffer */
            CO_CANtx_t *buffer = &CANmodule->txArray[0];
            /* search through whole array of pointers to transmit message buffers. */
            for(i = CANmodule->txSize; i > 0U; i--){
                /* if message buffer is full, send it. */
                if(buffer->bufferFull){
                    buffer->bufferFull = false;
                    CANmodule->CANtxCount--;

                    /* Copy message to CAN buffer */
                    CANmodule->bufferInhibitFlag = buffer->syncFlag;
                    /* canSend... */

                    /* Build FLEXCAN flexcan_frame_t from CANOpenNode CO_CANtx_t. Awkward due to differing endianess between MCU and CANOpen. */
                    flexcan_frame_t txFrame;
                    txFrame.id 			= FLEXCAN_ID_STD(buffer->ident);
                    txFrame.length 		= buffer->DLC;
                    txFrame.format		= 0; // STD format
                    txFrame.dataByte0	= buffer->data[0];
                    txFrame.dataByte1	= buffer->data[1];
                    txFrame.dataByte2	= buffer->data[2];
                    txFrame.dataByte3	= buffer->data[3];
                    txFrame.dataByte4	= buffer->data[4];
                    txFrame.dataByte5	= buffer->data[5];
                    txFrame.dataByte6	= buffer->data[6];
                    txFrame.dataByte7	= buffer->data[7];

                    /* Copy frame to message buffer */
                    if (FLEXCAN_WriteTxMb(CAN0, mbIndex, &txFrame) != kStatus_Success)
                    {
                    	while(1) // Should never occur, as the interrupt should be indicating a free tx buffer
                    		;
                    }
                    break;                      /* exit for loop */
                }
                buffer++;
            }/* end of for loop */

            /* Clear counter if no more messages */
            if(i == 0U){
                CANmodule->CANtxCount = 0U;
            }
        }
    }
    else{
        /* some other interrupt reason */
    	while(1) // Should never happen
    		;
    }
}
