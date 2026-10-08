/**
 * @file    UnitreeMotor.c
 * @brief   Unitree GO-M8010-6 RS485 motor driver.
 *
 * Ported from the 4-bus sample to the current single-UART7 RS485 hardware.
 * UART7 是普通 UART + 手动 DE 引脚(见 bsp_board_config.h 的 BOARD_RS485_U7_*),
 * 半双工时序由本驱动自己控制:
 *   发命令帧(TX DMA)→ 发送完成回调里把 DE 切回接收 → 环形 DMA 双缓冲收应答。
 *
 * 接收机制:UART7 的 RX DMA 在 CubeMX 里是循环模式,这里用 DMA 硬件双缓冲
 * (M0/M1 两个 16 字节缓冲,DMA 写完一个自动切到另一个,循环不停止)+ IDLE
 * 空闲中断:每帧应答收完总线空闲 → HAL_UARTEx_RxEventCallback → 取最近一帧。
 * 全部用 HAL 函数(HAL_DMAEx_MultiBufferStart_IT / HAL_UARTEx_RxEventCallback)。
 */
#include "includes.h"
#include "UnitreeMotor.h"
#include "usart.h"
#include "bsp_board_config.h" /* BOARD_RS485_U7_PORT/PIN */
#include "crc_ccitt.h"

#if USE_UNITREE

#define UNITREE_2PI 6.28318f
#define UNITREE_SATURATE(_IN, _MIN, _MAX) \
    {                                     \
        if ((_IN) <= (_MIN))              \
            (_IN) = (_MIN);               \
        else if ((_IN) >= (_MAX))         \
            (_IN) = (_MAX);               \
    }

UnitreeMotor Unitree_motors[UNITREE_MOTOR_NUM];

static bool s_unitree_initialized = false;
static uint8_t s_unitree_tx_index = 0U;
static UnitreeMotorData_t s_unitree_rx_data;
static UART_HandleTypeDef *s_unitree_uart = NULL; /* 由 Init 按 MOTOR_UNITREE_UART 选定 */

/* UART7 应答接收:环形 DMA 双缓冲(两个 16 字节帧槽,硬件自动交替写入)。
   IDLE 时从缓冲取最近一帧;DMA 写入,CPU 读之前需失效 D-Cache。 */
static __RAM_D2_ ALIGN_32B uint8_t s_unitree_rxbuf[2][sizeof(UnitreeMotorFeedbackFrame_t)];
static uint8_t s_unitree_rx_frame[sizeof(UnitreeMotorFeedbackFrame_t)]; /* 拼帧(跨缓冲回绕时) */
static uint16_t s_unitree_rx_read_idx = 0U;                             /* 双缓冲已处理到的位置(0..31) */

static __RAM_D1_ ALIGN_32B UnitreeMotorCmd_t s_unitree_tx_cmd;

static inline UART_HandleTypeDef *UnitreeMotor_GetUart(void)
{
    return &MOTOR_UNITREE_UART;
}

/* RS485 收发方向控制(DE 引脚,高=发送,低=接收)。
   属于半双工 RS485 驱动自己的职责,不属于 UART 抽象层;引脚在 bsp_board_config.h。 */
static inline void Unitree_SetRxMode(void)
{
    HAL_GPIO_WritePin(BOARD_RS485_U7_PORT, BOARD_RS485_U7_PIN, GPIO_PIN_RESET);
}

static inline void Unitree_SetTxMode(void)
{
    HAL_GPIO_WritePin(BOARD_RS485_U7_PORT, BOARD_RS485_U7_PIN, GPIO_PIN_SET);
}

/* 启动 UART7 的环形 DMA 双缓冲接收(与参考工程 USART_RxDMA_DoubleBuffer_Init 同思路):
   M0/M1 两个 16 字节缓冲,DMA 写完一个自动切到另一个,循环不停止;
   IDLE 空闲中断由 HAL_UART_IRQHandler 处理 → HAL_UARTEx_RxEventCallback
   (IRQ/UART_IRQHandler.c 分发到 UnitreeMotor_UART_RxEvent)。
   注意:.ioc 里 UART7 RX DMA 必须保持 Circular;DMA 自身的 TC/HT 回调置空,
   避免与 UART HAL 状态机互相干扰(数据到达统一由 IDLE 上报)。 */
static void Unitree_UartRxDmaDoubleBufferStart(void)
{
    UART_HandleTypeDef *uart = s_unitree_uart;
    DMA_HandleTypeDef *hdma;

    if ((uart == NULL) || (uart->hdmarx == NULL))
    {
        return;
    }
    hdma = uart->hdmarx;

    /* 屏蔽 DMA 自身的 TC/HT/Error 回调:只用 UART 的 IDLE 上报数据到达 */
    hdma->XferCpltCallback = NULL;
    hdma->XferHalfCpltCallback = NULL;
    hdma->XferM1CpltCallback = NULL;
    hdma->XferM1HalfCpltCallback = NULL;
    hdma->XferErrorCallback = NULL;

    hdma->Init.Mode = DMA_CIRCULAR; /* 保持环形(与 .ioc 一致) */
    if (HAL_DMAEx_MultiBufferStart_IT(hdma,
                                      (uint32_t)&uart->Instance->RDR,
                                      (uint32_t)s_unitree_rxbuf[0],
                                      (uint32_t)s_unitree_rxbuf[1],
                                      (uint16_t)sizeof(s_unitree_rxbuf[0])) != HAL_OK)
    {
        return;
    }

    /* 让 UART HAL 的 IDLE 路径生效(等价于 ReceiveToIdle 的语义):
       IDLE 事件 → HAL_UARTEx_RxEventCallback(huart, Size) */
    uart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
    uart->RxState = HAL_UART_STATE_BUSY_RX;
    uart->RxXferSize = (uint16_t)sizeof(s_unitree_rxbuf[0]);
    uart->RxXferCount = 0U;
    __HAL_UART_ENABLE_IT(uart, UART_IT_IDLE);     /* 空闲中断 */
    __HAL_UART_ENABLE_IT(uart, UART_IT_ERR);      /* 错误中断(ORE/FE) */
    SET_BIT(uart->Instance->CR3, USART_CR3_DMAR); /* UART DMA 接收请求 */
}

void UnitreeMotor_Init(void)
{
    UnitreeMotorCmd_t default_cmd;

    memset(&default_cmd, 0, sizeof(default_cmd));
    default_cmd.mode = UNITREE_MOTOR_MODE_IDLE;
    default_cmd.kp = MOTOR_UNITREE_DEFAULT_KP;
    default_cmd.kd = MOTOR_UNITREE_DEFAULT_KW;

    for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
    {
        memset(&Unitree_motors[i], 0, sizeof(Unitree_motors[i]));
        Unitree_motors[i].cmd = default_cmd;
        Unitree_motors[i].cmd.id = (uint16_t)i;
        Unitree_motors[i].data.id = (uint8_t)i;
        Unitree_motors[i].enable = false;
        Unitree_motors[i].begin = true;
    }

    s_unitree_tx_index = 0U;
    s_unitree_uart = UnitreeMotor_GetUart();
    s_unitree_initialized = true;

    /* 上电先把 RS485 方向设为接收,然后启动环形 DMA 双缓冲接收
       (一直运行,每帧应答由 IDLE 触发解析,无需逐帧启停) */
    Unitree_SetRxMode();
    s_unitree_rx_read_idx = 0U;
    Unitree_UartRxDmaDoubleBufferStart();
}

void UnitreeMotor_EncodeFrame(UnitreeMotorCmd_t *cmd)
{
    UnitreeMotorControlFrame_t *frame;
    uint8_t *packet;
    int16_t tor_des = 0;
    int16_t spd_des = 0;
    int32_t pos_des = 0;
    int16_t kp = 0;
    int16_t kd = 0;

    if (cmd == NULL)
    {
        return;
    }

    frame = &cmd->frame;
    packet = (uint8_t *)frame;

    UNITREE_SATURATE(cmd->id, 0, 15);
    UNITREE_SATURATE(cmd->mode, 0, 7);
    UNITREE_SATURATE(cmd->kp, 0.0f, 25.599f);
    UNITREE_SATURATE(cmd->kd, 0.0f, 25.599f);
    UNITREE_SATURATE(cmd->torque, -127.99f, 127.99f);
    UNITREE_SATURATE(cmd->speed, -804.00f, 804.00f);
    UNITREE_SATURATE(cmd->position, -411774.0f, 411774.0f);

    /* 协议实测为转子侧量:上层关节语义需在此做减速比换算 */
    tor_des = (int16_t)(cmd->torque / UNITREE_GEAR_RATIO * 256.0f);
    spd_des = (int16_t)((cmd->speed * UNITREE_GEAR_RATIO) / UNITREE_2PI * 256.0f);
    pos_des = (int32_t)((cmd->position * UNITREE_GEAR_RATIO) / UNITREE_2PI * 32768.0f);
    kp = (int16_t)(cmd->kp / 25.6f * 32768.0f);
    kd = (int16_t)(cmd->kd / 25.6f * 32768.0f);

    packet[0] = 0xFE;
    packet[1] = 0xEE;
    packet[2] = (uint8_t)((cmd->id & 0x0FU) | ((cmd->mode & 0x07U) << 4));
    packet[3] = (uint8_t)(tor_des & 0xFF);
    packet[4] = (uint8_t)((tor_des >> 8) & 0xFF);
    packet[5] = (uint8_t)(spd_des & 0xFF);
    packet[6] = (uint8_t)((spd_des >> 8) & 0xFF);
    packet[7] = (uint8_t)(pos_des & 0xFF);
    packet[8] = (uint8_t)((pos_des >> 8) & 0xFF);
    packet[9] = (uint8_t)((pos_des >> 16) & 0xFF);
    packet[10] = (uint8_t)((pos_des >> 24) & 0xFF);
    packet[11] = (uint8_t)(kp & 0xFF);
    packet[12] = (uint8_t)((kp >> 8) & 0xFF);
    packet[13] = (uint8_t)(kd & 0xFF);
    packet[14] = (uint8_t)((kd >> 8) & 0xFF);

    {
        uint16_t crc = crc_ccitt(0, packet,
                                 sizeof(*frame) - sizeof(frame->crc16));
        packet[15] = (uint8_t)(crc & 0xFF);
        packet[16] = (uint8_t)((crc >> 8) & 0xFF);
    }
}

void UnitreeMotor_DecodeFrame(UnitreeMotorData_t *data)
{
    const UnitreeMotorFeedbackFrame_t *frame;
    const uint8_t *packet;
    uint16_t rx_crc = 0U;
    uint16_t status_force = 0U;
    int16_t speed = 0;
    int16_t torque = 0;
    int32_t pos = 0;

    if (data == NULL)
    {
        return;
    }

    frame = &data->frame;
    packet = (const uint8_t *)frame;

    if (packet[0] != 0xFD || packet[1] != 0xEE)
    {
        data->correct = 0;
        return;
    }

    status_force = (uint16_t)packet[12] | ((uint16_t)packet[13] << 8U);

    data->calc_crc = crc_ccitt(0, packet,
                               sizeof(*frame) - sizeof(frame->crc16));
    rx_crc = (uint16_t)packet[14] | ((uint16_t)packet[15] << 8U);

    if (rx_crc != data->calc_crc)
    {
        memset(&data->frame, 0, sizeof(data->frame));
        data->correct = 0;
        data->bad_msg++;
        return;
    }

    data->id = packet[2] & 0x0FU;
    data->mode = (packet[2] >> 4) & 0x07U;
    data->temperature = (int)(int8_t)packet[11];
    data->error = (int)(status_force & 0x0007U);

    torque = (int16_t)((uint16_t)packet[3] | ((uint16_t)packet[4] << 8U));
    speed = (int16_t)((uint16_t)packet[5] | ((uint16_t)packet[6] << 8U));
    pos = (int32_t)((uint32_t)packet[7] |
                    ((uint32_t)packet[8] << 8U) |
                    ((uint32_t)packet[9] << 16U) |
                    ((uint32_t)packet[10] << 24U));

    /* 协议回包实测为转子侧量:转换到上层统一的关节语义 */
    data->torque = ((float)torque) / 256.0f * UNITREE_GEAR_RATIO;
    data->speed = (((float)speed / 256.0f) * UNITREE_2PI) / UNITREE_GEAR_RATIO;
    data->position = (UNITREE_2PI * ((float)pos) / 32768.0f) / UNITREE_GEAR_RATIO;
    data->foot_force = (int)((status_force >> 3U) & 0x0FFFU);
    data->correct = 1;
}

/* 发送完成中断入口(IRQ/UART_IRQHandler.c 按 huart 分发进来):
   RS485 半双工:发完一帧 → 方向切回接收即可;
   应答由常开的环形 DMA 双缓冲自动捕获(IDLE 触发解析)。 */
void UnitreeMotor_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != s_unitree_uart->Instance)
    {
        return;
    }
    Unitree_SetRxMode();
}

/* 环形 DMA + IDLE 接收入口(IRQ/UART_IRQHandler.c 的 RxEventCallback 分发进来):
   IDLE = 一帧应答收完;从双缓冲里取最近 16 字节解码。 */
void UnitreeMotor_UART_RxEvent(UART_HandleTypeDef *huart)
{
    UnitreeMotorData_t *rx_data = &s_unitree_rx_data;
    DMA_HandleTypeDef *hdma = huart->hdmarx;
    uint8_t *rxbuf = (uint8_t *)s_unitree_rxbuf; /* 双缓冲按字节取(环形 32 字节) */
    uint16_t buf_idx = 0U;
    uint16_t pos = 0U;
    uint16_t avail = 0U;
    uint16_t start = 0U;
    uint16_t i = 0U;

    if (huart->Instance != s_unitree_uart->Instance)
    {
        return;
    }
    if (hdma == NULL)
    {
        return;
    }

    /* DMA 刚写入环形双缓冲,CPU 读之前失效 D-Cache */
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_unitree_rxbuf, (int32_t)sizeof(s_unitree_rxbuf));

    /* 当前写入位置(0..31):CT 位=当前缓冲号,NDTR=当前缓冲内剩余 */
    buf_idx = (READ_BIT(((DMA_Stream_TypeDef *)hdma->Instance)->CR, DMA_SxCR_CT) != 0U) ? 1U : 0U;
    pos = (uint16_t)(buf_idx * (uint16_t)sizeof(s_unitree_rxbuf[0]) + (uint16_t)sizeof(s_unitree_rxbuf[0]) - (uint16_t)__HAL_DMA_GET_COUNTER(hdma));
    if (pos >= (uint16_t)sizeof(s_unitree_rxbuf))
    {
        pos = 0U; /* NDTR=0 的瞬间视为回到起点 */
    }

    /* 自上次处理位置以来收到的字节数(环形距离) */
    avail = (uint16_t)((pos + (uint16_t)sizeof(s_unitree_rxbuf) - s_unitree_rx_read_idx) % (uint16_t)sizeof(s_unitree_rxbuf));
    if (avail >= (uint16_t)sizeof(UnitreeMotorFeedbackFrame_t))
    {
        /* 取最近一帧(可能跨缓冲回绕),拼成连续 16 字节再解码 */
        start = (uint16_t)((pos + (uint16_t)sizeof(s_unitree_rxbuf) - (uint16_t)sizeof(UnitreeMotorFeedbackFrame_t)) % (uint16_t)sizeof(s_unitree_rxbuf));
        for (i = 0U; i < (uint16_t)sizeof(UnitreeMotorFeedbackFrame_t); i++)
        {
            s_unitree_rx_frame[i] = rxbuf[(start + i) % (uint16_t)sizeof(s_unitree_rxbuf)];
        }

        memset(rx_data, 0, sizeof(*rx_data));
        memcpy(&rx_data->frame, s_unitree_rx_frame, sizeof(rx_data->frame));
        UnitreeMotor_DecodeFrame(rx_data);

        if (rx_data->correct == 1 && rx_data->id < UNITREE_MOTOR_NUM)
        {
            uint32_t bad_msg = Unitree_motors[rx_data->id].data.bad_msg;
            Unitree_motors[rx_data->id].data = *rx_data;
            Unitree_motors[rx_data->id].data.bad_msg = bad_msg;
            Unitree_motors[rx_data->id].data.last_rx_ms = HAL_GetTick();
        }
    }
    s_unitree_rx_read_idx = pos;
}

/* 错误中断入口(IRQ/UART_IRQHandler.c 分发进来,ORE/FE 标志已在分发处清掉):
   错误后 HAL 已结束本次接收并中止 DMA(见 HAL_UART_IRQHandler 错误分支),
   重新启动环形双缓冲接收即可。 */
void UnitreeMotor_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != s_unitree_uart->Instance)
    {
        return;
    }
    s_unitree_rx_read_idx = 0U;
    Unitree_UartRxDmaDoubleBufferStart();
}

static void UnitreeMotor_SendCommand(UnitreeMotor *motor)
{
    UART_HandleTypeDef *uart = s_unitree_uart;

    if (motor == NULL || uart == NULL || !motor->begin)
    {
        return;
    }

    /* 上一帧 TX 还没发完就等下一拍(gState 是 TX 状态);
       RX 不需要判忙:环形双缓冲一直运行,发命令期间 DE=发送,收发器不会往
       总线上回数据,应答在 DE 切回接收后由 DMA 自动捕获,丢帧也能自愈 */
    if (uart->gState != HAL_UART_STATE_READY)
    {
        return;
    }

    Unitree_SetTxMode(); /* RS485 方向:发送 */

    s_unitree_tx_cmd = motor->cmd;
    UnitreeMotor_EncodeFrame(&s_unitree_tx_cmd);

    /* CPU 填好帧,DMA 读之前必须先 Clean,把新数据刷回内存
       (H7 启用 D-Cache 后,DMA 不经过 Cache,发送前必须 Clean) */
    SCB_CleanDCache_by_Addr((uint32_t *)&s_unitree_tx_cmd.frame,
                            sizeof(s_unitree_tx_cmd.frame));
    HAL_UART_Transmit_DMA(uart, (uint8_t *)&s_unitree_tx_cmd.frame,
                          sizeof(s_unitree_tx_cmd.frame));
}

void UnitreeMotor_Func(void)
{
    if (!s_unitree_initialized)
    {
        return;
    }

    for (uint32_t i = 0U; i < UNITREE_MOTOR_NUM; i++)
    {
        UnitreeMotor *motor = &Unitree_motors[i];

        if (motor->enable)
        {
            motor->cmd.mode = UNITREE_MOTOR_MODE_FOC;
        }
        else
        {
            motor->cmd.mode = UNITREE_MOTOR_MODE_IDLE;
        }

    }

    UnitreeMotor_SendCommand(&Unitree_motors[s_unitree_tx_index]);

    s_unitree_tx_index++;
    if (s_unitree_tx_index >= UNITREE_MOTOR_NUM)
    {
        s_unitree_tx_index = 0U;
    }
}

#endif /* USE_UNITREE */
