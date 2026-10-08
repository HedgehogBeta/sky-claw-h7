#ifndef VOFA_PLUS_H
#define VOFA_PLUS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "includes.h"
#include "app_cfg.h"
#include "usart.h"

/* 通道槽位数:合法通道号 0 ~ VOFA_MAX_CH-1 */
#define VOFA_MAX_CH 8

/* 发送任务配置(模块内部创建,不依赖 CubeMX 任务) */
#define VOFA_TASK_STACK_WORDS 128U
#define VOFA_TASK_PRIORITY APP_PRIO_NORMAL

#if APP_VOFA_ENABLE
    /**
     * @brief 发送任务创建(模块内部创建,不依赖 CubeMX 任务)
     * @param period_ms 发送周期(ms),须 > 0
     */
    void VOFA_TaskCreate(uint32_t period_ms);

    /**
     * @brief 注册观测变量到指定通道号,注册后 VOFA_SendTask 会周期发送该变量的值。
     * @param channel 通道号(0 ~ VOFA_MAX_CH-1),越界或该路已被占用时返回 false
     * @param addr    待观测变量地址,须非 NULL 且 4 字节对齐
     */
    bool VOFA_RegisterAt(uint8_t channel, const volatile float *addr);

    uint8_t  VOFA_GetChannelCount(void);
    uint32_t VOFA_GetDropCount(void);

    /**
     * @brief 推荐使用的注册宏,在文件作用域写一行即可注册观测变量。
     * @brief 注意:注册宏展开的函数在编译期就会被调用,所以注册行必须写在文件作用域,不能写在函数里。
     * @param ch  通道号(0 ~ VOFA_MAX_CH-1)
     * @param var 待观测变量,须为全局/static,注册行写在文件作用域
     */
#define VOFA_ADD_FLOAT(ch, var)                                         \
    typedef char vofa_ch_ok_##ch[((ch) < VOFA_MAX_CH) ? 1 : -1];        \
    static void vofa_reg_##ch(void) __attribute__((used, constructor)); \
    static void vofa_reg_##ch(void)                                     \
    {                                                                   \
        (void)VOFA_RegisterAt((ch), &(var));                            \
    }                                                                   \
    typedef char vofa_reg_typedef_##ch[1]

#else
/* 模块关闭:注册宏展开为空,业务代码里的 VOFA_ADD_FLOAT 不用删 */
#define VOFA_ADD_FLOAT(ch, var)
#endif /* APP_VOFA_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* VOFA_PLUS_H */
