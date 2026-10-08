/**
 * @file    bluetooth_serv.c
 * @brief   蓝牙业务消息层实现(接口见 bluetooth_serv.h)。
 *
 * 结构体即线上包体布局,收发各一次 memcpy。
 */
#include "bluetooth_serv.h"

#include <string.h> /* memcpy */

#include "bluetooth.h" /* Bluetooth_Send / Bluetooth_Receive / BluetoothFrame_t */

bool BT_Serv_Status_Send(const BT_Status_t *status)
{
    uint8_t payload[BT_TX_PAYLOAD_SIZE];

    if (status == NULL)
    {
        return false;
    }

    memcpy(payload, status, sizeof(BT_Status_t));

    return Bluetooth_Send(payload, BT_TX_PAYLOAD_SIZE);
}

bool BT_Serv_Cmd_Receive(BT_Cmd_t *cmd, uint32_t timeout_ms)
{
    BluetoothFrame_t frame;

    if (cmd == NULL)
    {
        return false;
    }

    if (Bluetooth_Receive(&frame, timeout_ms) != true)
    {
        return false;
    }

    memcpy(cmd, frame.payload, sizeof(BT_Cmd_t));

    return true;
}
