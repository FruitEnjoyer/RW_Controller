/*
 * communication.c
 *
 *  Created on: Apr 2, 2025
 *      Author: ruslan
 */


#include "communication.h"

communication_vars_t can_vars = {
    .can_errors_count = 0,
    .can_errors = {CAN_NO_ERROR},
    .TxData = {0},
    .RxData = {0},
    .TxMailbox = 0,

    .TxHeader.ExtId = 0,
    .TxHeader.RTR = CAN_RTR_DATA, //CAN_RTR_REMOTE
    .TxHeader.IDE = CAN_ID_STD,   // CAN_ID_EXT
    .TxHeader.DLC = 8,
    .TxHeader.TransmitGlobalTime = 0,
    .send_error = 0
};


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(can_vars.RxHeader), can_vars.RxData) == HAL_OK)
    {
        switch(can_vars.RxHeader.StdId)
        {
        case 0x0378:
            break;
        default:
            break;
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t er = HAL_CAN_GetError(hcan);
    can_vars.can_errors[can_vars.can_errors_count % 16] = er;
    can_vars.can_errors_count += 1;
}

void Send_Speed(float speed)
{
    can_vars.TxHeader.StdId = 0x0300;



    if(HAL_CAN_AddTxMessage(&hcan1, &(can_vars.TxHeader), can_vars.TxData, &(can_vars.TxMailbox)) != HAL_OK)
    {
            send_error += 1;
    }
}
