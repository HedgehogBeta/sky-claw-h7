/** @file myostasks.c
 *  @brief 指示灯任务。
 */
#include "myostasks.h"
#include "app_cfg.h"
#include "bsp_board_config.h"
#include "led.h"

void Alarm_Task(void *argument)
{
    (void)argument;
    for (;;)
    {
        for (uint8_t index = 0; index < BOARD_LED_NUM; ++index)
        {
            Led_On(index);
            osDelay(APP_LED_TASK_PERIOD_MS);
            Led_Off(index);
            osDelay(APP_LED_TASK_PERIOD_MS);
        }
    }
}
