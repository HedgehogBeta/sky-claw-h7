/**
 * @file    ring_buffer.c
 * @brief   Byte ring buffer implementation.
 */
#include "ring_buffer.h"

#include <string.h> /* memcpy */

void RingBuffer_Init(RingBuffer_t *rb, uint8_t *buffer, uint16_t size)
{
    rb->buffer = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}

bool RingBuffer_Push(RingBuffer_t *rb, uint8_t byte)
{
    if (RingBuffer_IsFull(rb)) {
        return false;
    }
    rb->buffer[rb->head] = byte;
    rb->head = (uint16_t)((rb->head + 1U) % rb->size);
    rb->count = (uint16_t)(rb->count + 1U);
    return true;
}

bool RingBuffer_Pop(RingBuffer_t *rb, uint8_t *byte)
{
    if (RingBuffer_IsEmpty(rb)) {
        return false;
    }
    *byte = rb->buffer[rb->tail];
    rb->tail = (uint16_t)((rb->tail + 1U) % rb->size);
    rb->count = (uint16_t)(rb->count - 1U);
    return true;
}

/* 段吞吐:一次写入最多 len 字节(只写实际放得下的),返回实际写入字节数。
   head 在 buffer 中间时,若 len 超出到末尾的空闲,自动拆两段 memcpy(wrap)。 */
uint16_t RingBuffer_PushSegment(RingBuffer_t *rb, const uint8_t *data, uint16_t len)
{
    if ((rb == NULL) || (data == NULL) || (len == 0U)) {
        return 0U;
    }

    const uint16_t space = (uint16_t)(rb->size - rb->count);
    const uint16_t n = (space < len) ? space : len;
    if (n == 0U) {
        return 0U;
    }

    const uint16_t first = (uint16_t)(rb->size - rb->head); /* 到 buffer 末尾的连续空闲 */
    if (n <= first) {
        memcpy(&rb->buffer[rb->head], data, n);
        rb->head = (uint16_t)(rb->head + n);
    } else {
        memcpy(&rb->buffer[rb->head], data, first);
        memcpy(&rb->buffer[0], &data[first], (uint16_t)(n - first));
        rb->head = (uint16_t)(n - first);
    }

    rb->count = (uint16_t)(rb->count + n);
    return n;
}

/* 段吞吐:一次读出最多 len 字节(只读当前实际可用的),返回实际读出字节数。
   tail 在 buffer 中间时,若 len 超出到末尾的数据,自动拆两段 memcpy(wrap)。 */
uint16_t RingBuffer_PopSegment(RingBuffer_t *rb, uint8_t *data, uint16_t len)
{
    if ((rb == NULL) || (data == NULL) || (len == 0U)) {
        return 0U;
    }

    const uint16_t n = (rb->count < len) ? rb->count : len;
    if (n == 0U) {
        return 0U;
    }

    const uint16_t first = (uint16_t)(rb->size - rb->tail); /* 到 buffer 末尾的连续数据 */
    if (n <= first) {
        memcpy(data, &rb->buffer[rb->tail], n);
        rb->tail = (uint16_t)(rb->tail + n);
    } else {
        memcpy(data, &rb->buffer[rb->tail], first);
        memcpy(&data[first], &rb->buffer[0], (uint16_t)(n - first));
        rb->tail = (uint16_t)(n - first);
    }

    rb->count = (uint16_t)(rb->count - n);
    return n;
}

uint16_t RingBuffer_Available(const RingBuffer_t *rb)
{
    return rb->count;
}

bool RingBuffer_IsEmpty(const RingBuffer_t *rb)
{
    return (rb->count == 0U);
}

bool RingBuffer_IsFull(const RingBuffer_t *rb)
{
    return (rb->count >= rb->size);
}

void RingBuffer_Flush(RingBuffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->count = 0;
}
