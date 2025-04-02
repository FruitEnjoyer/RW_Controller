/*
 * communication.h
 *
 *  Created on: Mar 26, 2025
 *      Author: ruslan
 */

#ifndef INC_COMMUNICATION_H_
#define INC_COMMUNICATION_H_

#include "main.h"
#include "trajectory_control.h"

#define ERROR_BUF_LEN  16

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

typedef enum{
    TXMAIL_FULL = 0,
    TXMAIL_FREE = 1
} can_is_txmailbox_free_t;

typedef struct{
    CAN_TxHeaderTypeDef TxHeader;
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t TxData[8];
    uint8_t RxData[8];
    uint32_t TxMailbox;

    uint8_t can_errors_count;
    can_errors_t can_errors[16];

    can_float_t acceleration;
    can_float_t speed;
    uint16_t send_error;
} communication_vars_t;

#endif /* INC_COMMUNICATION_H_ */
