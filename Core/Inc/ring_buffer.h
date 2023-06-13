/*
* ring_buffer.h
*
*  Created on: 2023-05-25
*   by Joshua Butler
*
*/

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_BUFFER_SIZE 32

typedef struct {
    void* data;  // Pointer to the buffer data
    size_t size;  // Size of each element in the buffer
    uint16_t buffer_size; // Maximum number of elements in the buffer
    uint16_t count; // Number of elements in the buffer
    uint16_t head;  // Index of the head (write position)
    uint16_t tail;  // Index of the tail (read position)
    bool isFull;    // Flag indicating if the buffer is full
} RingBuffer;

enum {
    RING_BUFFER_OK,
    RING_BUFFER_ERROR,
    RING_BUFFER_MALLOC_FAILED
};

uint8_t ring_buffer_init(RingBuffer* buffer, uint16_t buffer_size, size_t data_size);
uint8_t ring_buffer_destroy(RingBuffer* buffer);
bool is_ring_buffer_empty(const RingBuffer* buffer);
bool is_ring_buffer_full(const RingBuffer* buffer);
bool ring_buffer_enqueue(RingBuffer* buffer, const void* value);
bool ring_buffer_dequeue(RingBuffer* buffer, void* value);
void ring_buffer_flush(RingBuffer* buffer);

#endif /* RING_BUFFER_H_ */