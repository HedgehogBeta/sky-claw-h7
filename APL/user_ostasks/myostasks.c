/**
 * @file    tasks.c
 * @brief   任务函数实现(强定义覆盖 freertos.c 里的 __weak 占位)。
 *
 * 新建任务的标准流程:
 *   1. CubeMX 里创建任务(生成 freertos.c 的任务创建代码和 __weak 任务函数);
 *   2. 在 CubeMX 任务函数的 USER CODE 里写业务,或者在本文件写强定义覆盖。
 *
 * 注:VOFA+ 发送任务已收回到 Communication/VOFA 模块内部(VOFA_TaskCreate() 里创建),
 *    不依赖 CubeMX 任务配置。
 */
#include "includes.h"
#include "myostasks.h"
#include "FreeRTOS.h"
#include "task.h"
// #include "EventRecorder.h"
#include "app_cfg.h"
#include "bsp_board_config.h"

#include "led.h"


/* CubeMX 默认任务 AlarmTask:四灯流水 */
void Alarm_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        for (uint8_t i = 0; i < BOARD_LED_NUM; i++)
        {
            Led_On(i);
            osDelay(APP_LED_TASK_PERIOD_MS);
            Led_Off(i);
            osDelay(APP_LED_TASK_PERIOD_MS);
        }
        // EventStartA(0);
		// printf("LED round ,%lu stack left \n", uxTaskGetStackHighWaterMark(NULL));
        // EventStopA(0);
        // /* 任务周期延时 */
        osDelay(APP_LED_TASK_PERIOD_MS);
    }
}
