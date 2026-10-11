#ifndef TEST_DJMOTOR_FDCAN_H
#define TEST_DJMOTOR_FDCAN_H

#include <stdint.h>

/* Only the HAL fields/constants used by the real DJI driver are modeled. */
typedef struct
{
    uint32_t instance;
} FDCAN_HandleTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t RxFrameType;
} FDCAN_RxHeaderTypeDef;

typedef struct
{
    uint32_t Identifier;
    uint32_t IdType;
    uint32_t TxFrameType;
    uint32_t DataLength;
    uint32_t ErrorStateIndicator;
    uint32_t BitRateSwitch;
    uint32_t FDFormat;
    uint32_t TxEventFifoControl;
    uint32_t MessageMarker;
} FDCAN_TxHeaderTypeDef;

typedef enum
{
    HAL_OK = 0
} HAL_StatusTypeDef;

#define FDCAN_STANDARD_ID 0U
#define FDCAN_DATA_FRAME 0U
#define FDCAN_DLC_BYTES_8 8U
#define FDCAN_ESI_ACTIVE 0U
#define FDCAN_BRS_OFF 0U
#define FDCAN_CLASSIC_CAN 0U
#define FDCAN_NO_TX_EVENTS 0U

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

HAL_StatusTypeDef HAL_FDCAN_AddMessageToTxFifoQ(
    FDCAN_HandleTypeDef *handle, FDCAN_TxHeaderTypeDef *header,
    uint8_t *data);

#endif
