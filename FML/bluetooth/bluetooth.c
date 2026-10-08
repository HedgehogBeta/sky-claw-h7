/**
 * @file    bluetooth.c
 * @brief   蓝牙串口通信模块实现(FreeRTOS 基础 Queue + 任务通知版)。
 *
 * 数据流:
 *   发送: 业务 Bluetooth_Send() → TX Queue → Bluetooth_TxTask → UART DMA
 *   接收: BT_UART DMA/IDLE → HAL_UARTEx_RxEventCallback(IRQ/UART_IRQHandler.c 分发)
 *        → Bluetooth_UartRxEvent() → 环形缓冲
 *        → Bluetooth_RxTask 解析 → RX Queue → 业务 Bluetooth_Receive()
 *
 * 并发原语:
 *   数据用基础 Queue API(xQueueCreate/Send/SendFromISR/Receive);
 *   中断→接收任务的"唤醒"用任务通知(ulTaskNotifyTake/vTaskNotifyGiveFromISR)。
 *
 * UART 归属:本模块拥有 BT_UART(见 FML/fm_uart_cfg.h)的接收缓冲与
 * DMA+IDLE 接收启停,收到数据后的 DCache 失效和接收重启都在本模块内,
 * IRQ/UART_IRQHandler.c 只负责按 huart 分发。
 */
#include "bluetooth.h"

#include "includes.h" /* __RAM_D2_ / ALIGN_32B 等 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "app_cfg.h"     /* APP_PRIO_* 任务优先级 */
#include "fm_uart_cfg.h" /* BT_UART 语义宏(换串口只改这里) */
#include "crc_ccitt.h"   /* 复用现有 CRC-CCITT */
#include "ring_buffer.h" /* 复用现有字节环形缓冲 */

/* ------------------------------------------------------------------ */
/* 缓冲配置:原始字节环形缓冲大小(DMA 接收缓冲大小见 FML/fm_uart_cfg.h)。   */
/* ------------------------------------------------------------------ */
#define BT_RX_RING_SIZE 256U /* 原始字节环形缓冲大小 */

/* ------------------------------------------------------------------ */
/* 计算宏:由 bluetooth_cfg.h 的帧参数宏 + bluetooth_serv.h 的包体大小推导 */
/*                                                                     */
/*   校验范围:包长(如有) + 包体(不含包头)                                */
/*   帧长 = 包头 + 包长(如有) + 包体 + 校验 + 包尾                       */
/*   RX/TX 包体长度各自独立:RX = 上位机命令(定长),TX = 设备状态(定长),   */
/*   取 bluetooth_serv.h 两个 pack(1) 结构体的 sizeof,不共用一个长度。    */
/* ------------------------------------------------------------------ */
#define BT_CHECKSUM_BYTES (BT_CHECKSUM_MODE == 0U ? 2U : 1U)                /* 校验字节数 */
#define BT_HEADER_BYTES ((BT_FRAME_HEADER2 != 0U) ? 2U : 1U)                /* 包头字节数(单/双) */
#define BT_TAIL_BYTES ((BT_FRAME_TAIL2 != 0U) ? 2U : 1U)                    /* 包尾字节数(单/双) */
#define BT_PAYLOAD_OFFSET (BT_HEADER_BYTES + (BT_PAYLOAD_LENGTH ? 1U : 0U)) /* 包体起始位置:0 时无包长字段 */

/* 包体长度按方向区分(bluetooth_serv.h:sizeof(BT_Cmd_t) / sizeof(BT_Status_t)) */
#define BT_RX_PAYLOAD_LEN BT_RX_PAYLOAD_SIZE
#define BT_TX_PAYLOAD_LEN BT_TX_PAYLOAD_SIZE

/* ---- RX 帧布局(接收:上位机命令) ---- */
#define BT_RX_FRAME_LEN (BT_PAYLOAD_OFFSET + BT_RX_PAYLOAD_LEN + BT_CHECKSUM_BYTES + BT_TAIL_BYTES) /* 整帧长度 */
#define BT_RX_CHECKSUM_LEN (BT_PAYLOAD_OFFSET - BT_HEADER_BYTES + BT_RX_PAYLOAD_LEN)                /* 校验覆盖字节数:包长(如有) + 包体 */
#define BT_RX_CHECKSUM_POS (BT_RX_FRAME_LEN - BT_CHECKSUM_BYTES - BT_TAIL_BYTES)                    /* 校验起始位置(小端) */
#define BT_RX_TAIL_POS (BT_RX_FRAME_LEN - BT_TAIL_BYTES)                                            /* 包尾起始位置 */

/* ---- TX 帧布局(发送:设备状态) ---- */
#define BT_TX_FRAME_LEN (BT_PAYLOAD_OFFSET + BT_TX_PAYLOAD_LEN + BT_CHECKSUM_BYTES + BT_TAIL_BYTES) /* 整帧长度 */
#define BT_TX_CHECKSUM_LEN (BT_PAYLOAD_OFFSET - BT_HEADER_BYTES + BT_TX_PAYLOAD_LEN)                /* 校验覆盖字节数:包长(如有) + 包体 */
#define BT_TX_CHECKSUM_POS (BT_TX_FRAME_LEN - BT_CHECKSUM_BYTES - BT_TAIL_BYTES)                    /* 校验起始位置(小端) */
#define BT_TX_TAIL_POS (BT_TX_FRAME_LEN - BT_TAIL_BYTES)                                            /* 包尾起始位置 */

/* ------------------------------------------------------------------ */
/* 静态变量                                                            */
/* ------------------------------------------------------------------ */
static QueueHandle_t s_bt_rx_queue;      /* 业务层接收队列(完整帧) */
static QueueHandle_t s_bt_tx_queue;      /* 发送队列(完整帧) */
static TaskHandle_t s_bt_rx_task_handle; /* 接收任务句柄(模块内部创建) */
static RingBuffer_t s_bt_ring;           /* 原始字节环形缓冲(中断写,任务读) */
static uint8_t s_bt_ring_buf[BT_RX_RING_SIZE];
static uint8_t s_rx_frame[BT_RX_FRAME_LEN]; /* 正在拼接的帧(RX) */
static uint16_t s_rx_pos = 0;               /* 已拼入的字节数 */

/* BT_UART DMA 接收缓冲:放 DMA 可达的 D2 段并 32 字节对齐(H7 D-Cache 要求)。
   数据流:外设→DMA→本缓冲;收到数据后先 Invalidate 再读(见 Bluetooth_UartRxEvent)。 */
static __RAM_D2_ ALIGN_32B uint8_t s_bt_rxbuf[BT_UART_RX_SIZE];

static __RAM_D2_ ALIGN_32B uint8_t s_bt_txbuf[BT_TX_FRAME_LEN]; /* DMA 发送缓冲(D2 段) */
static volatile bool s_tx_busy = false;                         /* DMA 忙标志,发送完成中断里清 */
static bool s_bt_task_created = false; /* 队列/任务已建(Bluetooth_TaskCreate 里置位) */
static bool s_bt_init_done = false;    /* 接收已启动(Bluetooth_Init 里置位) */

static void Bluetooth_Init(void); /* 仅模块内部使用(接收任务调用),不写进头文件 */

/* ------------------------------------------------------------------ */
/* 接收:解析函数(只在接收任务里调用)                                     */
/* ------------------------------------------------------------------ */
/* 校验一帧:先查包尾(单/双),再校验(帧长编译期固定,无需参数)。
   校验范围 = 包长(如有) + 包体,不含包头,与上位机约定一致。 */
static bool Bluetooth_CheckFrame(const uint8_t *frame)
{
    /* 包尾(单/双) */
#if BT_TAIL_BYTES == 2
    if ((frame[BT_RX_TAIL_POS] != BT_FRAME_TAIL1) || (frame[BT_RX_TAIL_POS + 1] != BT_FRAME_TAIL2))
    {
        return false;
    }
#else
    if (frame[BT_RX_TAIL_POS] != BT_FRAME_TAIL1)
    {
        return false;
    }
#endif

#if BT_PAYLOAD_LENGTH > 0
    /* 包长字段校验:接收帧的包长必须等于 RX 包体长度(位置 = 包头字节数)。
       包长本身也在校验范围内,写错一样会被校验拦下;这里显式再查一次,
       让包长字段真正发挥作用(帧长仍是编译期常量,不按它动态收帧)。 */
    if (frame[BT_HEADER_BYTES] != BT_RX_PAYLOAD_LEN)
    {
        return false;
    }
#endif

    /* 校验:范围 = 包长(如有) + 包体,从包头后起算 */
#if BT_CHECKSUM_MODE == 0U
    /* CRC16:小端 2 字节 */
    uint16_t crc_rx = (uint16_t)(frame[BT_RX_CHECKSUM_POS] | ((uint16_t)frame[BT_RX_CHECKSUM_POS + 1] << 8));
    return (crc_ccitt(0xFFFF, &frame[BT_HEADER_BYTES], BT_RX_CHECKSUM_LEN) == crc_rx);
#else
    /* 单字节校验和:覆盖字节求和,取低 8 位 */
    uint8_t sum = 0;
    for (uint16_t i = 0U; i < BT_RX_CHECKSUM_LEN; i++)
    {
        sum += frame[BT_HEADER_BYTES + i];
    }
    return (sum == frame[BT_RX_CHECKSUM_POS]);
#endif
}

/* 把校验通过的一帧转成 BluetoothFrame_t,丢进业务层接收队列 */
static void Bluetooth_PushRxFrame(const uint8_t *frame)
{
    BluetoothFrame_t out;

    out.length = BT_RX_PAYLOAD_LEN;
    memcpy(out.payload, &frame[BT_PAYLOAD_OFFSET], BT_RX_PAYLOAD_LEN);

    /* 队列满了就丢这一帧(业务层来不及处理时丢新帧,不影响后续帧) */
    (void)xQueueSend(s_bt_rx_queue, &out, 0);
}

/* 逐字节拼帧:找包头 → 收满固定帧长 → 校验。
   帧长由宏编译期定死,没有运行时长度判断;
   帧失败后回到找包头状态,靠包头自动重新同步(单/双包头均可)。 */
static void Bluetooth_RxByte(uint8_t b)
{
    if (s_rx_pos == 0U)
    {
        /* 等包头第 1 字节 */
        if (b == BT_FRAME_HEADER1)
        {
            s_rx_frame[s_rx_pos++] = b;
        }
        return;
    }

#if BT_HEADER_BYTES == 2
    if (s_rx_pos == 1U)
    {
        /* 等包头第 2 字节;不是包头就重新找 */
        if (b == BT_FRAME_HEADER2)
        {
            s_rx_frame[s_rx_pos++] = b;
        }
        else if (b == BT_FRAME_HEADER1)
        {
            s_rx_frame[0] = b; /* 当前字节可能是新帧的包头 */
        }
        else
        {
            s_rx_pos = 0;
        }
        return;
    }
#endif

    /* 收满一帧就校验(BT_RX_FRAME_LEN 为编译期常量) */
    s_rx_frame[s_rx_pos++] = b;
    if (s_rx_pos >= BT_RX_FRAME_LEN)
    {
        if (Bluetooth_CheckFrame(s_rx_frame))
        {
            Bluetooth_PushRxFrame(s_rx_frame);
        }
        s_rx_pos = 0;
    }
}

/* ------------------------------------------------------------------ */
/* 接收:任务与中断入口                                                  */
/* ------------------------------------------------------------------ */
/* 接收任务:先启动接收,再阻塞在唤醒信号上,醒来后把环形缓冲里的字节全部喂给解析器 */
static void Bluetooth_RxTask(void *argument)
{
    (void)argument;

    Bluetooth_Init(); /* 进循环前启动接收,必须晚于调度器启动与任务创建 */

    for (;;)
    {
        /* 等"有数据"通知(每收到一段 DMA 数据 Give 一次) */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* 一次取完缓冲里所有字节(中断里可能已塞了不止一段) */
        uint8_t b;
        while (RingBuffer_Pop(&s_bt_ring, &b))
        {
            Bluetooth_RxByte(b);
        }
    }
}

/* BT_UART IDLE 中断入口(IRQ/UART_IRQHandler.c 按 huart 分发进来):
   只搬数据 + 发信号,不做任何解析;最后重启 DMA 接收。
   注意重启必须放在最后:重启后 DMA 会立刻往 s_bt_rxbuf 写新数据,
   必须先把它读进环形缓冲再放开缓冲。 */
void Bluetooth_UartRxEvent(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance != BT_UART.Instance) /* 防御:不是我们的串口就不处理 */
    {
        return;
    }

    /* DMA 已经把数据写进 s_bt_rxbuf,CPU 读之前必须先失效 D-Cache,
       否则可能读到 Cache 里的旧数据(H7 启用 D-Cache 后的固定动作) */
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    SCB_InvalidateDCache_by_Addr((uint32_t *)s_bt_rxbuf, size);

    /* 1. 把 DMA 收到的原始字节搬进环形缓冲(段吞吐,满了就丢,不阻塞中断) */
    RingBuffer_PushSegment(&s_bt_ring, s_bt_rxbuf, size);

    /* 2. 通知接收任务;
          真正的数据在环形缓冲里,任务醒来后自己取 */
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(s_bt_rx_task_handle, &woken);
    if (woken == pdTRUE)
    {
        portYIELD_FROM_ISR(woken);
    }

    /* 3. 重启 DMA+IDLE 接收,等待下一段数据 */
    HAL_UARTEx_ReceiveToIdle_DMA(&BT_UART, s_bt_rxbuf, sizeof(s_bt_rxbuf));
}

/* BT_UART 发送完成中断入口(IRQ/UART_IRQHandler.c 分发进来):清忙标志 */
void Bluetooth_UartTxCplt(UART_HandleTypeDef *huart)
{
    if (huart->Instance != BT_UART.Instance)
    {
        return;
    }
    s_tx_busy = false;
}

/* BT_UART 错误中断入口(IRQ/UART_IRQHandler.c 分发进来,ORE/FE 标志已在分发处清掉):
   直接重启 DMA 接收即可,错误的这一小段数据丢掉 */
void Bluetooth_UartRxError(UART_HandleTypeDef *huart)
{
    if (huart->Instance != BT_UART.Instance)
    {
        return;
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&BT_UART, s_bt_rxbuf, sizeof(s_bt_rxbuf));
}

/* ------------------------------------------------------------------ */
/* 发送:组帧、发送任务、DMA 状态                                        */
/* ------------------------------------------------------------------ */
/* 把业务帧拼成线上帧。
   包头:BT_FRAME_HEADER2 = 0 时单包头,否则双包头。
   BT_PAYLOAD_LENGTH = 0 时不写包长字段;= N 时写 1 字节包长(TX 包体长度)。
   包体固定为 TX 包体长度(BT_TX_PAYLOAD_SIZE = sizeof(BT_Status_t))。
   校验范围 = 包长(如有) + 包体,不含包头。
   包尾:BT_FRAME_TAIL2 = 0 时单包尾,否则双包尾。
   帧长由宏编译期定死,没有运行时判断。 */
static void Bluetooth_BuildTxFrame(const BluetoothFrame_t *frame, uint8_t *buf)
{
    buf[0] = BT_FRAME_HEADER1;
#if BT_HEADER_BYTES == 2
    buf[1] = BT_FRAME_HEADER2;
#endif
#if BT_PAYLOAD_LENGTH > 0
    buf[BT_HEADER_BYTES] = BT_TX_PAYLOAD_LEN;
#endif
    memcpy(&buf[BT_PAYLOAD_OFFSET], frame->payload, BT_TX_PAYLOAD_LEN);

    /* 校验:范围 = 包长(如有) + 包体,从包头后起算 */
#if BT_CHECKSUM_MODE == 0U
    uint16_t crc = crc_ccitt(0xFFFF, &buf[BT_HEADER_BYTES], BT_TX_CHECKSUM_LEN);
    buf[BT_TX_CHECKSUM_POS] = (uint8_t)(crc & 0xFF);
    buf[BT_TX_CHECKSUM_POS + 1] = (uint8_t)(crc >> 8);
#else
    uint8_t sum = 0;
    for (uint16_t i = 0U; i < BT_TX_CHECKSUM_LEN; i++)
    {
        sum += buf[BT_HEADER_BYTES + i];
    }
    buf[BT_TX_CHECKSUM_POS] = sum;
#endif

    /* 包尾(单/双) */
    buf[BT_TX_TAIL_POS] = BT_FRAME_TAIL1;
#if BT_TAIL_BYTES == 2
    buf[BT_TX_TAIL_POS + 1] = BT_FRAME_TAIL2;
#endif
}

/* 发送任务:从 TX Queue 一帧一帧取,启动 DMA 发送。
   多个任务同时 Bluetooth_Send() 时队列自动排队,一次只发一帧。 */
static void Bluetooth_TxTask(void *argument)
{
    (void)argument;
    BluetoothFrame_t frame;

    for (;;)
    {
        if (xQueueReceive(s_bt_tx_queue, &frame, portMAX_DELAY) != pdPASS)
        {
            continue;
        }

        /* 组帧 → 刷 D-Cache → 启动 DMA(直接调 HAL,与数据手册的调用方式一致) */
        Bluetooth_BuildTxFrame(&frame, s_bt_txbuf);

        /* CPU 填好 s_bt_txbuf,DMA 读之前必须先 Clean,把新数据刷回内存 */
        SCB_CleanDCache_by_Addr((uint32_t *)s_bt_txbuf, BT_TX_FRAME_LEN);

        s_tx_busy = true;
        if (HAL_UART_Transmit_DMA(&BT_UART, s_bt_txbuf, BT_TX_FRAME_LEN) != HAL_OK)
        {
            s_tx_busy = false; /* 启动失败,直接处理下一帧 */
            continue;
        }

        /* 等 DMA 发完(发送完成中断里清 s_tx_busy),再发下一帧 */
        while (s_tx_busy)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}

/* ------------------------------------------------------------------ */
/* 对外 API                                                            */
/* ------------------------------------------------------------------ */
/* 业务层发送:只组一个帧丢进 TX Queue,不碰硬件。
   length 是本帧有效数据长度(<= 包体长度,多余字节补 0 一起发出)。 */
bool Bluetooth_Send(const uint8_t *payload, uint16_t length)
{
    if ((payload == NULL) || (length == 0U) || (length > BT_TX_PAYLOAD_LEN))
    {
        return false;
    }

    BluetoothFrame_t frame;
    frame.length = length;
    memset(frame.payload, 0, BT_PAYLOAD_MAX);
    memcpy(frame.payload, payload, length);

    /* 队列满时最多等 BT_SEND_TIMEOUT_MS,还不行就返回 false */
    return (xQueueSend(s_bt_tx_queue, &frame, pdMS_TO_TICKS(BT_SEND_TIMEOUT_MS)) == pdPASS);
}

/* 业务层接收:从 RX Queue 取一帧,超时返回 false(timeout_ms=0 时非阻塞) */
bool Bluetooth_Receive(BluetoothFrame_t *frame, uint32_t timeout_ms)
{
    if (frame == NULL)
    {
        return false;
    }
    return (xQueueReceive(s_bt_rx_queue, frame, pdMS_TO_TICKS(timeout_ms)) == pdPASS);
}

/* ------------------------------------------------------------------ */
/* 初始化(两段式:先建对象,再开接收)                                     */
/* ------------------------------------------------------------------ */
/* 建队列 + 建收发任务(启动调度器之前调用;只创建 FreeRTOS 对象,不开中断) */
void Bluetooth_TaskCreate(void)
{
    if (s_bt_task_created)
    {
        return;
    }

    /* 两个业务队列:队列里存的是完整 BluetoothFrame_t(按值拷贝) */
    s_bt_rx_queue = xQueueCreate(BT_RX_QUEUE_SIZE, sizeof(BluetoothFrame_t));
    s_bt_tx_queue = xQueueCreate(BT_TX_QUEUE_SIZE, sizeof(BluetoothFrame_t));

    /* 接收侧:任务通知 + 原始字节环形缓冲 */
    RingBuffer_Init(&s_bt_ring, s_bt_ring_buf, sizeof(s_bt_ring_buf));

    /* 收发任务在模块内部创建,方便整个模块整体搬走 */
    xTaskCreate(Bluetooth_RxTask, "BtRxTask", BT_TASK_STACK_WORDS, NULL,
                BT_RX_TASK_PRIORITY, &s_bt_rx_task_handle);
    xTaskCreate(Bluetooth_TxTask, "BtTxTask", BT_TASK_STACK_WORDS, NULL,
                BT_TX_TASK_PRIORITY, NULL);

    s_bt_task_created = true;
}

/* 启动 BT_UART 的 DMA+IDLE 接收(IDLE/收满时回调到 IRQ/UART_IRQHandler.c 分发) */
static void Bluetooth_Init(void)
{
    if (s_bt_init_done)
    {
        return;
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&BT_UART, s_bt_rxbuf, sizeof(s_bt_rxbuf));

    s_bt_init_done = true;
}