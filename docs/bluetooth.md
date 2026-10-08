# 蓝牙串口通信模块使用说明(FML/bluetooth)

> 面向“刚学会 STM32 + FreeRTOS 任务/队列”的队友。本文只讲怎么用、怎么改,
> 原理上每个文件里都有注释。

## 1. 这个模块解决什么问题

本工程保留模板的蓝牙串口通信模块；当前机构的手动入口为调试器命令，蓝牙尚未连接机构控制接口。

机器人上要通过蓝牙串口(鼎新图蓝牙模块,接到 USART6:PC6=TX, PC7=RX)
和上位机互发数据。串口数据是“流”,没有边界;我们约定一种带包头包尾和校验的
**帧格式**,把流切成一条条完整的消息。

蓝牙只传**一种数据**:业务层拿到 payload 后按自己的约定解析(比如前 2 字节是
指令,后面是参数),模块本身不关心内容,也没有消息类型字段。

业务层只需要:

```c
Bluetooth_Send(数据, 长度);          // 发一条消息
Bluetooth_Receive(&frame, 超时ms);   // 收一条消息
```

业务层**不需要知道**:UART、DMA、IDLE、CRC、缓冲区、发送忙不忙。

## 2. 数据流向

```
发送:
  业务数据 --Bluetooth_Send()--> TX Queue --> Bluetooth_TxTask
              --组帧(包头/包长开关/包体/CRC/包尾)--> UART DMA --> 蓝牙模块 --> 上位机

接收:
  上位机 --> 蓝牙模块 --> UART DMA 收到 --> IDLE 中断
              --Bluetooth_UartRxEvent()--> 环形缓冲(只搬数据,不解析)
              --任务通知(唤醒)--> Bluetooth_RxTask(找包头->读包长开关->收满一帧->查包尾->算CRC)
              --> RX Queue --> 业务 Bluetooth_Receive()
```

中断里只做“搬数据 + 通知”,**所有解析(包头/包尾/CRC)都在任务里**,不会卡死中断。

## 3. 线上帧格式(收发双方必须一致)

帧格式由几个配置宏编译期定死(收发双方约定一致,无运行时判断):

| 组成 | 配置宏                                  | 说明                                        |
| ---- | --------------------------------------- | ------------------------------------------- |
| 包头 | `BT_FRAME_HEADER1` / `BT_FRAME_HEADER2` | 双包头;**包头2 设为 0 = 单包头**(只用包头1) |
| 包长 | `BT_PAYLOAD_LENGTH`                     | 0 = 无包长字段;N(1..32) = 带 1 字节包长字段 |
| 包体 | —                                       | **按方向区分**:RX = `BT_RX_PAYLOAD_SIZE`,TX = `BT_TX_PAYLOAD_SIZE`(帧层 `bluetooth.h` 里取业务结构体的 `sizeof`) |
| 校验 | `BT_CHECKSUM_MODE`                      | 0 = CRC16,1 = 单字节校验和                  |
| 包尾 | `BT_FRAME_TAIL1` / `BT_FRAME_TAIL2`     | 双包尾;**包尾2 设为 0 = 单包尾**(只用包尾1) |

示意(双包头 + 有包长 + 双包尾为例):

| 字节     | 内容       | 说明                             |
| -------- | ---------- | -------------------------------- |
| 0 .. H-1 | 包头       | H = 包头字节数(单=1 / 双=2)      |
| H        | N          | 包长(仅当 BT_PAYLOAD_LENGTH > 0) |
| ..       | payload[N] | 包体,固定 N 字节                 |
| ..       | 校验       | CRC16(2 字节小端)或单字节校验和  |
| 最后 1/2 | 包尾       | T = 包尾字节数(单=1 / 双=2)      |

**帧长 = 包头(H) + 包长(1,如有) + 包体(N) + 校验(1/2) + 包尾(T)**

> **包体长度按方向各自独立**:收(RX,上位机命令)包体 = `BT_RX_PAYLOAD_SIZE`,
> 发(TX,设备状态)包体 = `BT_TX_PAYLOAD_SIZE`,两者在帧层 `bluetooth.h` 里直接取
> `bluetooth_serv.h` 两个 `#pragma pack(1)` 业务结构体的 `sizeof`(见第 6 节)。
> 因此 RX 帧和 TX 帧的整帧长度**可不同**,帧层各自用 `BT_RX_FRAME_LEN` /
> `BT_TX_FRAME_LEN` 收/发,不再共用一个包体长度。当前结构体配置:RX 9 字节、TX 7 字节。

**校验方式(`BT_CHECKSUM_MODE`,bluetooth_cfg.h,与上位机约定二选一)**:

- `0` = **CRC16**:CRC-CCITT(init=0xFFFF),2 字节小端,用 FML/algorithm/crc_ccitt.c
  (查表多项式 0x8408,初值 0xFFFF);
- `1` = **单字节校验和**:覆盖字节求和,取低 8 位,1 字节。

**校验范围(不含包头)**:

- 有包长字段:`包长 + 包体`;
- 无包长字段:`包体` 本身。

示例(包体 N = 9,CRC16 / SUM8):

| 包头  | 包长 | 包尾  | 帧长(CRC16)    | 帧长(SUM8) |
| ----- | ---- | ----- | -------------- | ---------- |
| 双(2) | 无   | 单(1) | 2+9+2+1 = 14   | 13         |
| 双(2) | 有   | 单(1) | 2+1+9+2+1 = 15 | 14         |
| 单(1) | 无   | 双(2) | 1+9+2+2 = 14   | 13         |
| 单(1) | 有   | 双(2) | 1+1+9+2+2 = 15 | 14         |

补充说明:

- **包头/包尾单双**:把 `BT_FRAME_HEADER2` / `BT_FRAME_TAIL2` 设为 0 即取消第二个字节;
  单双由收发双方约定,编译期定死,运行时不变;
- **包长字段的作用**:帧长按宏编译期定死(单帧类型场景,不做运行时变长);
  接收时除了校验,还会显式核对收到的包长字节 == 约定值,写错直接丢帧;
- **想省带宽**(蓝牙串口慢):改用 `BT_CHECKSUM_MODE = 1`(少 1 字节校验),
  或在 `bluetooth_serv.h` 的业务结构体里删字段来缩短包体;
  `BT_PAYLOAD_LENGTH` 只决定是否带包长字段(带上会多 1 字节);
- 校验范围里包含包长,包长被干扰时校验会失败,不会收错帧;
- 想改包长/校验方式:改 bluetooth_cfg.h的宏,收发两边一起改。

## 4. 业务层用法

```c
#include "bluetooth.h"

// 初始化:main.c 在 osKernelStart() 之前调 Bluetooth_TaskCreate()(建队列/建任务);
// 启动接收的 Bluetooth_Init() 由接收任务内部调用 —— 业务层两个都不用管

// ---- 发送 ----
uint8_t cmd[2] = {0x01, 0x02};
if (Bluetooth_Send(cmd, 2)) {
    // 发送成功(只是入队成功,不代表已经发出去;真正发由 TX 任务负责)
}

// ---- 接收 ----
BluetoothFrame_t frame;
if (Bluetooth_Receive(&frame, 100)) {        // 等 100ms,没消息返回 false
    // frame.payload[0 .. frame.length-1] 就是本帧数据
    // frame.length 是实际包体长度(<= BT_PAYLOAD_MAX,该宏为共享帧缓冲容量的较大值)
    // 业务层在这里按自己的约定解析 payload
}
```

队列深度、超时、任务栈等参数都在 bluetooth.h 顶部,改数字就行。

## 5. 已经帮你接好的线(新人了解即可)

| 位置                         | 做了什么                                                                                     |
| ---------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| FML/bluetooth/bluetooth.h/.c | 模块拥有 `BT_UART`(见 FML/fm_uart_cfg.h)的 DMA 接收缓冲与收发逻辑;`Bluetooth_TaskCreate()` 建队列/建任务,`Bluetooth_Init()` 用 `HAL_UARTEx_ReceiveToIdle_DMA(&BT_UART, ...)` 启动接收 |
| Core/Src/main.c              | `osKernelStart()` 之前调 `Bluetooth_TaskCreate()`;接收由 `Bluetooth_RxTask` 进循环前调 `Bluetooth_Init()` 启动(中断里要调 FreeRTOS API,必须等调度器在跑、句柄有效、环形缓冲已初始化) |
| IRQ/UART_IRQHandler.c | `HAL_UARTEx_RxEventCallback` 按 huart 分发到 `Bluetooth_UartRxEvent`,TxCplt/Error 同理                                   |
| MDK-ARM/*.uvprojx            | FML/Bluetooth 组挂 bluetooth.c + bluetooth_serv.c;业务结构体在 bluetooth_serv.h(include path 已含 FML/bluetooth)      |

换串口:CubeMX 里改引脚/DMA,代码里改 `FML/fm_uart_cfg.h` 的 `BT_UART` 一行即可(收发与接收
重启逻辑都在模块内部;IRQ/UART_IRQHandler.c 的分支用宏比较,自动跟随)。

## 6. Payload 协议层(结构体 = 线上格式,pack(1) 直拷)

**结构体怎么写、线上就怎么排**:`FML/bluetooth/bluetooth_serv.h` 里的两个
`#pragma pack(1)` 结构体,既是业务层用的 C 结构体,也是包体的字节布局本身。

- `BT_Cmd_t`(RX:上位机 → 板子,9 字节);
- `BT_Status_t`(TX:板子 → 上位机,7 字节:`online`/`ready` + `rpm` + `current`)。

字段类型按协议约定:`bool`(flag 位域)/ `int8_t` / `int16_t` / `int32_t` / `float`,小端。

布局:

| 位置    | 内容                                                                                     |
| ------- | ---------------------------------------------------------------------------------------- |
| flag 区 | 开头的 `bool : 1` 位域:8 个占 1 字节,不足 8 个也占满整字节(字节里没用到的位双方不读)     |
| 之后    | 数据字段:`int8_t` / `int16_t` / `int32_t` / `float` 完整成员,小端                         |

`pack(1)` 保证没有填充字节,所以 `sizeof(结构体)` 就是包体字节数:帧层 `bluetooth.h` 里
`BT_RX_PAYLOAD_SIZE` / `BT_TX_PAYLOAD_SIZE` 直接取 `sizeof`(不用手工维护数量宏),
共享帧缓冲容量 `BT_PAYLOAD_MAX` = 两者较大。

```c
/* FML/bluetooth/bluetooth_serv.h */
#pragma pack(1)
typedef struct
{
    bool enable    : 1;
    bool auto_mode : 1;
    /* ... */
    bool reset     : 1;
    int16_t speed;
    int16_t angle;
    float   vx;
} BT_Cmd_t;
#pragma pack()
```

- 收发各一次 `memcpy`(`bluetooth_serv.c` 里没有逐字段 Pack/Parse,也没有索引计数):

```c
uint8_t payload[BT_TX_PAYLOAD_SIZE];
memcpy(payload, status, sizeof(BT_Status_t));
Bluetooth_Send(payload, BT_TX_PAYLOAD_SIZE);
```

- 业务层用法(flag 和数据字段都当普通成员读写):

```c
BT_Status_t st;
memset(&st, 0, sizeof(st));
st.online = true;
st.rpm    = 3000;
BT_Serv_Status_Send(&st);

BT_Cmd_t cmd;
if (BT_Serv_Cmd_Receive(&cmd, 10)) {   /* 等 10ms */
    if (cmd.enable) { /* ... */ }
}
```

- 编译期自检(断言失败 = 数组长度为负 = 编译报错):启用包长字段(`BT_PAYLOAD_LENGTH > 0`)时
  包体须 ≤ 255(帧层 `bluetooth.h`);大端目标直接 `#error`(`bluetooth_serv.h` 结构体处)。
- 层次:业务层 `bluetooth_serv.h` 只声明结构体和收发接口,包体长度/帧长这类宏都在帧层
  `bluetooth.h`(结构体是唯一来源,数量/长度都不用手工维护)。
- 改协议:只改结构体(增删字段 / 改名 / 调顺序 / 加 flag),`bluetooth_serv.c` 不用动。

用 pack 直拷要接受的代价:

- 结构体的成员类型和顺序**就是**协议:字段只能用 `bool : 1` / `int8_t` / `int16_t` /
  `int32_t` / `float`;数据字段必须是完整成员,写成位域会挤进 flag 所在的字节;
- 位域位序依赖编译器 ABI:本工程是 armclang(AC6)+ 小端 ARM,顺序 = 声明顺序、从 bit0 起;
  换编译器或换大端平台要重新核对;
- 结构体是未对齐的(pack(1)):整块 `memcpy` 没问题,但**别把成员地址交给 DMA/外设**;
  上位机也要按“小端 + 无填充”解析。

> 省带宽:帧层默认 `BT_PAYLOAD_LENGTH = 0`(无包长字段,少 1 字节)。
> 包体长度 = 结构体大小,不用手动设帧长;

## 7. 常见问题

- **收不到**:先看蓝牙模块波特率(默认 115200)和上位机是否一致;再用串口助手
  直接看 UARTX_RxBuffer 有没有原始数据。
- **收到但解析失败**:检查上下位机协议是否对齐(包头/包尾/包长/校验方式/帧长/字节序),收发双方的帧层宏必须一致。
- **发送队列满**(Bluetooth_Send 返回 false):把 BT_TX_QUEUE_SIZE 调大,
  或检查 TX 任务是不是卡在 DMA 上(发送完成中断没清 s_tx_busy)。
- **DMA 相关**:确认缓冲在 D2 段(`__RAM_D2_`);该段已由 MPU Region2 配成
  non-cacheable,模块/IRQHandler 里现有的发送前 Clean、接收后 Invalidate
  (`SCB_Clean/InvalidateDCache_by_Addr`)调用保留为 no-op,不用删;
  若日后把缓冲挪到 AXI(`.RAM_D1`,可缓存)则必须保留并按 32B 对齐维护。