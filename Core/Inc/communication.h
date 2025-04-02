/*
 * communication.h
 *
 *  Created on: Mar 26, 2025
 *      Author: ruslan
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "main.h"

typedef enum{
    CAN_NO_ERROR                    = HAL_CAN_ERROR_NONE,
    CAN_PROTOCOL_ERROR_WARNING      = HAL_CAN_ERROR_EWG,
    CAN_ERROR_PASSIVE               = HAL_CAN_ERROR_EPV,
    CAN_BUSOFF_ERROR                = HAL_CAN_ERROR_BOF,
    CAN_STUFF_ERROR                 = HAL_CAN_ERROR_STF,
    CAN_FORM_ERROR                  = HAL_CAN_ERROR_FOR,
    CAN_ACK_ERROR                   = HAL_CAN_ERROR_ACK,
    CAN_BIT_RECESSIVE_ERROR         = HAL_CAN_ERROR_BR,
    CAN_BIT_DOMINANT_ERROR          = HAL_CAN_ERROR_BD,
    CAN_CRC_ERROR                   = HAL_CAN_ERROR_CRC,
    CAN_RX_FIFO0_OVERRUN_ERROR      = HAL_CAN_ERROR_RX_FOV0,
    CAN_RX_FIFO1_OVERRUN_ERROR      = HAL_CAN_ERROR_RX_FOV1,
    CAN_TXMAILBOX0_ARBITRATION_LOST = HAL_CAN_ERROR_TX_ALST0,
    CAN_TXMAILBOX0_TRANSMIT_ERROR   = HAL_CAN_ERROR_TX_TERR0,
    CAN_TXMAILBOX1_ARBITRATION_LOST = HAL_CAN_ERROR_TX_ALST1,
    CAN_TXMAILBOX1_TRANSMIT_ERROR   = HAL_CAN_ERROR_TX_TERR1,
    CAN_TXMAILBOX2_ARBITRATION_LOST = HAL_CAN_ERROR_TX_ALST2,
    CAN_TXMAILBOX2_TRANSMIT_ERROR   = HAL_CAN_ERROR_TX_TERR2,
    CAN_TIMEOUT_ERROR               = HAL_CAN_ERROR_TIMEOUT,
    CAN_PERIPHERAL_NOT_INITIALIZED  = HAL_CAN_ERROR_NOT_INITIALIZED,
    CAN_PERIPHERAL_NOT_READY        = HAL_CAN_ERROR_NOT_READY,
    CAN_PERIPHERAL_NOT_STARTED      = HAL_CAN_ERROR_NOT_STARTED,
    CAN_PARAM_ERROR                 = HAL_CAN_ERROR_PARAM
} can_errors_t;

typedef union{
    uint8_t b[4];
    float f;
} can_float_t;


CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8] = {0,};
uint8_t RxData[8] = {0,};
uint32_t TxMailbox = 0;
uint8_t can_errors_count = 0;
can_errors_t can_errors[16] = {CAN_NO_ERROR};
can_float_t can_acc;
can_float_t can_spd;
int8_t acc = 0;

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
    {
        for(uint8_t i = 0; i < 4; i++)
        {
            can_spd.b[i] = RxData[i];
        }
        TrajCtrl_ReachTargetSpeed(can_spd.f);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
    can_errors[can_errors_count] = er;
    can_errors_count += 1;
    can_errors_count %= 16;
}


#endif /* INC_COMMUNICATION_H_ */
