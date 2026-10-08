# VOFA+ 打波调试说明

[VOFA+](https://www.vofa.plus/) 是一款串口上位机调试软件,支持波形/表格/示波器显示。
本工程在 `FML/vofa/` 提供基于 **JustFloat** 协议的发送模块:把变量注册成通道,
模块自带任务周期发送。

- 帧格式:第 0 路 … 第 N-1 路,每路 4 字节 float32(小端),末尾固定跟 4 字节帧尾 `00 00 80 7F`;
- 帧长 = `4 × (通道数 + 1)` 字节;
- 通道号 0 ~ `VOFA_MAX_CH - 1`(默认 8);
- 没注册的通道号,帧里发 0(通道位置不会错位);
- 发送周期 `APP_VOFA_TASK_PERIOD_MS`(默认 10 ms),经 UART9 DMA 发出。

## 1. 硬件接线

| 项 | 值 |
| --- | --- |
| 出口串口 | UART9(`huart9`) |
| 引脚 | PD14 = RX,PD15 = TX(USB-TTL 的 RX/TX 交叉接) |
| 波特率 | 115200(在 CubeMX 的 UART9 里改) |
| 连接 | USB-TTL → PC,必须**共地** |

> 出口串口为 `VOFA_UART`(见 `FML/fm_uart_cfg.h`),换串口改那里一行即可。

## 2. 软件配置

```c
/* APL/app_cfg.h */
#define APP_VOFA_ENABLE          1     /* 1 = 编译并使用 VOFA+ 模块 */
#define APP_VOFA_TASK_PERIOD_MS  10U   /* 发送周期(ms),main.c 里作为实参传给 VOFA_TaskCreate() */

/* FML/vofa/vofa.h */
#define VOFA_MAX_CH              8     /* 通道槽位数:合法通道号 0 ~ VOFA_MAX_CH-1 */
#define VOFA_TASK_STACK_WORDS    128U  /* 发送任务栈大小(字) */
#define VOFA_TASK_PRIORITY       APP_PRIO_NORMAL   /* 发送任务优先级 */
```

- `APP_VOFA_ENABLE = 0`:不编译模块本体与发送任务;业务里的 `VOFA_ADD_FLOAT` 自动展开为空,
  注册行不用删,`main.c` 里的 `VOFA_TaskCreate(APP_VOFA_TASK_PERIOD_MS)` 调用也被宏包住。

## 3. 接口速查(FML/vofa/vofa.h)

| 接口 | 类型 | 说明 |
| --- | --- | --- |
| `VOFA_ADD_FLOAT(ch, var)` | 宏 | 把变量 `var` 注册成第 `ch` 路,写在文件作用域 |
| `VOFA_RegisterAt(channel, addr)` | 函数 | 按通道号登记变量地址(循环/条件注册时用) |
| `VOFA_TaskCreate(period_ms)` | 函数 | 创建发送任务,`period_ms` = 发送周期(ms,须 > 0);`main()` 里调度器启动前调用一次 |
| `VOFA_GetChannelCount()` | 函数 | 当前帧通道数(已注册的最大通道号 + 1) |
| `VOFA_GetDropCount()` | 函数 | 因串口忙而丢掉的帧数 |

当前机构已占用通道 0～7，映射见 [控制方案](control-plan.md)。增加观测量时调整已有映射，或扩大通道数并核对带宽。

## 4. 用法

### 注册通道:

在**变量的定义处**写一行 `VOFA_ADD_FLOAT(ch, var)`:

```c
static float leg_length;
static float swing_angle;

VOFA_ADD_FLOAT(0, leg_length);
VOFA_ADD_FLOAT(1, swing_angle);
```

规则:

- 宏必须写在**文件作用域**(变量定义旁边);写进函数体会编译报错;
- `ch` 必须小于 `VOFA_MAX_CH`,超出编译报错;通道编号应唯一;
- `var` 可以是普通变量、结构体成员、数组元素:`VOFA_ADD_FLOAT(2, motor[0].angle)`;
- 变量必须是全局或 `static`(静态存储期),整个运行期都有效;
- 注册项的书写先后顺序与通道号无关,帧里第 `ch` 路就是这一行注册的变量;
- 变量的值由发送任务每帧直接读取,业务代码里**不需要**做任何赋值或发送动作;
- 注册项在 `main()` 之前自动填好,运行期不需要、也不应再注册。

### 程序化注册(可选)

需要按数量循环注册时用函数:

```c
for (uint8_t i = 0U; i < motor_num; i++)
{
    (void)VOFA_RegisterAt(i, &motor[i].speed);
}
```

- 返回 `false` 表示该路没登记成功:通道号越界、该通道号已被占用、地址未 4 字节对齐,或入参为 `NULL`;
- 在 `main()` 里 `VOFA_TaskCreate(period_ms)` 之前调用(运行期再补通道会让帧长中途变化)。

## 5. 启动接入

`Core/Src/main.c` 已接好,不需要改:

```c
#include "vofa.h"

#if APP_VOFA_ENABLE
  VOFA_TaskCreate(APP_VOFA_TASK_PERIOD_MS);   /* 建发送任务:周期由调用处给,不依赖 CubeMX 任务配置 */
#endif
```

## 6. 注意事项

- **帧率与带宽**:帧长 = `4 × (通道数 + 1)` 字节。115200(约 11.5 kB/s)按 10 ms 周期算上限约 27 路,
  当前 `VOFA_MAX_CH = 8` 余量充足;要提频或加路数,先提高 UART9 波特率;
- **丢帧**:串口忙时模块丢本帧,不阻塞业务;丢帧数用 `VOFA_GetDropCount()` 观察;
- **发送缓冲**:在 `__RAM_D2_`(D2 SRAM)且 32 字节对齐;D2 段已由 MPU Region2 配成 non-cacheable;
- **UART9 只发不收**:模块不启动接收,也不接任何中断回调;
- **多任务写同一变量**:发送任务是在别的上下文读这些变量的,写侧自行保证读写不撕裂
  (32 位对齐的 float 读写本身是原子的)。
