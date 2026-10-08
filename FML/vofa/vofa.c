/**
 * @file    vofa.c
 * @brief   VOFA+ JustFloat 打波模块(只发送,USART9,115200)。
 *
 * 注册即观测:业务在变量定义处写一行 VOFA_ADD_FLOAT(ch, var),
 * __attribute__((constructor)) 在 main() 之前把 &var 登记进第 ch 路;
 * VOFA_SendTask 每帧现场解引用,观测值永远是最新的,业务代码不写任何发送代码。
 *
 * 通道号由注册那行写死:代码本身就是通道表,没有名字表,也不依赖构造顺序。
 *
 * 注册项必须写在文件作用域(写进函数体会编译报错)—— 这条编译期约束挡住了
 * "注册局部变量 → 出栈后指针悬空";VOFA_RegisterAt 再补检查:
 * 通道号越界、该路已被占用、地址未 4 字节对齐。
 *
 * 帧布局:第 0 路 … 第 (帧通道数-1) 路 → 4 字节 JustFloat 尾,空槽补 0。
 *
 * 发送:VOFA_SendTask 按 VOFA_TaskCreate(period_ms) 传入的周期组包 → HAL_UART_Transmit_DMA(&VOFA_UART),
 * 发送前 SCB_CleanDCache_by_Addr;链路是否空闲由 HAL 自己的 gState 判断,忙时丢本帧,
 * 绝不阻塞业务(丢帧数见 VOFA_GetDropCount())。本模块不接任何中断回调。
 * 只发不收,不启动 USART9 接收。
 */
#include "vofa.h"

#include "includes.h"    /* __RAM_D2_ / ALIGN_32B 等 */
#include "fm_uart_cfg.h" /* VOFA_UART 语义宏(换串口只改这里) */

#include "FreeRTOS.h"
#include "task.h"

/* JustFloat 包尾固定 4 字节:00 00 80 7F */
#define VOFA_SUFFIX1 0x00
#define VOFA_SUFFIX2 0x00
#define VOFA_SUFFIX3 0x80
#define VOFA_SUFFIX4 0x7F

#if APP_VOFA_ENABLE

/* 一帧最多:VOFA_MAX_CH 路 + 1 个 4 字节尾 */
#define VOFA_MAX_PACKET_SIZE (4U * (VOFA_MAX_CH + 1U))
/* 缓冲按 32B 取整:SCB_CleanDCache_by_Addr 要求地址 32B 对齐,长度取整后不会洗到数组外 */
#define VOFA_BUFFER_SIZE (((VOFA_MAX_PACKET_SIZE) + 31U) & ~31U)

static const volatile float *s_addr[VOFA_MAX_CH];
static uint8_t s_ch_num = 0U; /* 帧通道数 = 已注册的最大通道号 + 1 */

static volatile uint32_t s_drop_cnt = 0U; /* 链路忙丢弃的帧数(诊断用) */

static __RAM_D2_ ALIGN_32B uint8_t vofa_buffer[VOFA_BUFFER_SIZE] = {0};

static void VOFA_SendTask(void *argument);

uint8_t VOFA_GetChannelCount(void)
{
    return s_ch_num;
}

uint32_t VOFA_GetDropCount(void)
{
    return s_drop_cnt;
}

void VOFA_TaskCreate(uint32_t period_ms)
{
    static bool s_init_done = false;
    /* 任务参数:静态存储,生命周期覆盖任务(s_init_done 后不再改动) */
    static uint32_t s_period_ms;

    if (s_init_done || (period_ms == 0U))
    {
        return;
    }

    s_period_ms = period_ms;

    /* 发送任务在模块内部创建,方便整个模块整体搬走(与 Bluetooth/IMU 一致) */
    xTaskCreate(VOFA_SendTask, "VOFA_SendTask", VOFA_TASK_STACK_WORDS,
                &s_period_ms, VOFA_TASK_PRIORITY, NULL);

    s_init_done = true;
}

/**
 * @param buf 输出缓冲,容量须 >= VOFA_BUFFER_SIZE
 */
static uint32_t VOFA_BuildFrame(uint8_t *buf)
{
    float *frame = (float *)buf;
    uint32_t n = 0U;

    for (uint8_t ch = 0U; ch < s_ch_num; ch++)
    {
        /* 现场解引用:观测值永远是最新的;空槽补 0,通道位置不会串 */
        frame[n++] = (s_addr[ch] != NULL) ? *(s_addr[ch]) : 0.0f;
    }

    uint8_t *tail = (uint8_t *)&frame[n];
    tail[0] = VOFA_SUFFIX1;
    tail[1] = VOFA_SUFFIX2;
    tail[2] = VOFA_SUFFIX3;
    tail[3] = VOFA_SUFFIX4;
    n++;

    return n * 4U;
}

/**
 * @param argument 指向周期(ms)的 uint32_t,由 VOFA_TaskCreate 传入
 */
static void VOFA_SendTask(void *argument)
{
    const TickType_t period_ticks = pdMS_TO_TICKS(*(const uint32_t *)argument);

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        if (VOFA_UART.gState != HAL_UART_STATE_READY)
        {
            s_drop_cnt++; /* 上一帧还没发完:丢本帧,不阻塞业务 */
        }
        else
        {
            uint32_t len = VOFA_BuildFrame(vofa_buffer);

            if (len > 4U) /* 只有尾字节说明一路都没注册,不用发 */
            {
                SCB_CleanDCache_by_Addr((uint32_t *)vofa_buffer, (int32_t)len);

                if (HAL_UART_Transmit_DMA(&VOFA_UART, vofa_buffer, (uint16_t)len) != HAL_OK)
                {
                    s_drop_cnt++; /* 没收下(比如 DMA 启动失败):也算丢帧 */
                }
            }
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

bool VOFA_RegisterAt(uint8_t channel, const volatile float *addr)
{
    if (addr == NULL || channel >= VOFA_MAX_CH)
    {
        return false;
    }
    if (s_addr[channel] != NULL) /* 该通道号已经被别的变量占了 */
    {
        return false;
    }
    if (((uintptr_t)addr & 3U) != 0U)
    {
        return false;
    }

    s_addr[channel] = addr;

    if ((uint8_t)(channel + 1U) > s_ch_num)
    {
        s_ch_num = (uint8_t)(channel + 1U);
    }

    return true;
}

#endif /* APP_VOFA_ENABLE */
