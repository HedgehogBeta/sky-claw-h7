/**
 * @file    ring_buffer.h
 * @brief   Byte ring buffer, useful for UART/referee protocols.
 */
#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint8_t *buffer;
    uint16_t size;
    volatile uint16_t head;   /* write index */
    volatile uint16_t tail;   /* read index  */
    volatile uint16_t count;
} RingBuffer_t;

/* 单字节吞吐:成功返回 true,满/空返回 false */
void     RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size);
bool     RingBuffer_Push(RingBuffer_t *rb, uint8_t byte);
bool     RingBuffer_Pop(RingBuffer_t *rb, uint8_t *byte);

/* 段吞吐:一次搬入/搬出最多 len 字节,自动处理头尾 wrap(memcpy,比逐字节快)。
   Push 只写实际放得下的字节(满则少写/不写,不覆盖旧数据),返回实际写入字节数;
   Pop 只读当前实际可用的字节(可能小于 len),返回实际读出字节数。
   两者都要求 data 非空、len > 0,否则返回 0。 */
uint16_t RingBuffer_PushSegment(RingBuffer_t *rb, const uint8_t *data, uint16_t len);
uint16_t RingBuffer_PopSegment(RingBuffer_t *rb, uint8_t *data, uint16_t len);

uint16_t RingBuffer_Available(const RingBuffer_t *rb);
bool     RingBuffer_IsEmpty(const RingBuffer_t *rb);
bool     RingBuffer_IsFull(const RingBuffer_t *rb);
void     RingBuffer_Flush(RingBuffer_t *rb);

#ifdef __cplusplus
}
#endif

#endif /* RING_BUFFER_H */
