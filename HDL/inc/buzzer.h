/**
 * @file    buzzer.h
 * @brief   蜂鸣器设备接口(HDL 层,引脚映射见 HDL/inc/bsp_board_config.h)。
 */
#ifndef BUZZER_H
#define BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);
void Buzzer_Toggle(void);

/** 阻塞式开机音:on-off-on-off,各 50 ms。 */
void Buzzer_StartupBeep(void);

/** 阻塞式报警:times 次 on_ms/off_ms 循环。 */
void Buzzer_Alarm(uint8_t times, uint32_t on_ms, uint32_t off_ms);

#ifdef __cplusplus
}
#endif

#endif /* BUZZER_H */
