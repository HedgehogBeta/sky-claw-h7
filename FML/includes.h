/**
 * @file    includes.h
 * @brief   模板聚合头:标准库 + 常用宏。
 *
 * 用户代码文件习惯上先 include 这个头,再 include 自己需要的模块头。
 * 电机开关 USE_DJ / USE_VESC / USE_ZMDR / USE_UNITREE 在 FML/motor/motor_cfg.h。
 *
 * RAM 段宏与链接脚本 MDK-ARM/stm32h723zxx_flash.sct 中的执行区域一一对应:
 *   __RAM_D1_ -> .RAM_D1:AXI SRAM     0x24000000(窗口 320KB,可被 D-Cache 缓存)
 *   __RAM_D2_ -> .RAM_D2:D2 SRAM1+2   0x30000000(共 32KB,DMA1/2 可达)
 *   __RAM_D3_ -> .RAM_D3:SRAM4        0x38000000(16KB,仅 BDMA 可达)
 *
 * 注意:
 *  - 普通全局变量 / 堆 / 栈不需要这些宏,sct 已把全部普通 RW/ZI 放进 DTCM;
 *  - DTCM / ITCM 任何 DMA 都访问不到,DMA 缓冲请用 __RAM_D2_(或 __RAM_D1_);
 *  - .RAM_D2 所在的 D2 SRAM 已由 MPU Region2 配成 non-cacheable,DMA 与 CPU
 *    共享无需手动维护 Cache;模块内现存的 SCB_Clean/InvalidateDCache_by_Addr()
 *    调用保留也无害(no-op);
 *  - .RAM_D1(AXI SRAM)可被 D-Cache 缓存:若放 DMA 缓冲,需 32B 对齐(ALIGN_32B)
 *    并在收发前后手动维护一致性;
 *  - 不要用宏把数据放 ITCM:ITCM 0x0~0x3FF 被 main() 用作运行时向量表。
 */
#ifndef __INCLUDES_H__
#define __INCLUDES_H__

#ifdef __cplusplus
extern "C"
{
#endif

#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "mathFunc.h"

/* RAM 段放置宏,配合 MDK-ARM/stm32h723zxx_flash.sct 使用 */
#define __RAM_D1_ __attribute__((section(".RAM_D1")))
#define __RAM_D2_ __attribute__((section(".RAM_D2")))
#define __RAM_D3_ __attribute__((section(".RAM_D3")))
#define __ITCM_ __attribute__((section(".ITCM")))

/* 32B 对齐 = D-Cache line size(配合 DMA + SCB_Clean/InvalidateDCache_by_Addr) */
#define ALIGN_32B __attribute__((aligned(32)))

#ifdef __cplusplus
}
#endif

#endif /* __INCLUDES_H__ */
