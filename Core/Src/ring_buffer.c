/*
 * ring_buffer.c
 *
 *  Created on: May 24, 2023
 *      Author: josh
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "misc.h"
#include "ring_buffer.h"


uint8_t ring_buffer_init(RingBuffer* buffer, uint16_t buffer_size, size_t data_size) {
    buffer->data = rtos_malloc((size_t)buffer_size * data_size);
    if (buffer->data == NULL) {
        return RING_BUFFER_MALLOC_FAILED;
    }
    buffer->size = data_size;
    buffer->buffer_size = buffer_size;
    buffer->count = 0;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->isFull = false;

    return RING_BUFFER_OK;
}

uint8_t ring_buffer_destroy(RingBuffer* buffer) {
    free(buffer->data);
    if (buffer->data != NULL) {
        return RING_BUFFER_ERROR;
    }
    buffer->data = NULL;
    buffer->buffer_size = 0;
    buffer->count = 0;
    buffer->size = 0;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->isFull = false;

    return RING_BUFFER_OK;
}

bool is_ring_buffer_empty(const RingBuffer* buffer) {
    return (!buffer->isFull && buffer->head == buffer->tail);
}

bool is_ring_buffer_full(const RingBuffer* buffer) {
    return buffer->isFull;
}

bool ring_buffer_enqueue(RingBuffer* buffer, const void* value) {
    if (buffer->isFull) {
        return false;  // Buffer is full, cannot enqueue
    }

    uint8_t* buffer_data = (uint8_t*)buffer->data;
    const uint8_t* value_data = (const uint8_t*)value;

    for (int i = 0; i < buffer->size; i++) {
        buffer_data[(buffer->head * buffer->size + i) % (buffer->size * buffer->buffer_size)] = value_data[i];
    }

    buffer->head = (buffer->head + 1) % buffer->buffer_size;
    buffer->count++;

    if (buffer->count == buffer->buffer_size) {
        buffer->isFull = true;  // Buffer is full after enqueueing
    }

    return true;
}

bool ring_buffer_dequeue(RingBuffer* buffer, void* value) {
    if (is_ring_buffer_empty(buffer)) {
        return false;  // Buffer is empty, cannot dequeue
    }

    uint8_t* buffer_data = (uint8_t*)buffer->data;
    uint8_t* value_data = (uint8_t*)value;

    for (size_t i = 0; i < buffer->size; i++) {
        value_data[i] = buffer_data[(buffer->tail * buffer->size + i) % (buffer->size * buffer->buffer_size)];
    }

    buffer->tail = (buffer->tail + 1) % buffer->buffer_size;
    buffer->count--;
    buffer->isFull = false;  // Buffer is no longer full

    return true;
}

void ring_buffer_flush(RingBuffer* buffer) {
    memset(buffer->data, 0, buffer->size * buffer->buffer_size);
    buffer->count = 0;
    buffer->head = 0;
    buffer->tail = 0;
    buffer->isFull = false;
}
