#ifndef CIRCULAR_BUFFER_H
#define CIRCULAR_BUFFER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * CircularBuffer:
 *   - buffer: pointer to an external block of memory for storing bytes.
 *   - capacity: capacity of the buffer (must be a power of two).
 *   - mask: (capacity - 1), used to wrap indexes via bitwise &.
 *   - head: index of the next byte to write.
 *   - tail: index of the next byte to read.
 *   - full: indicates whether the buffer is currently full.
 */
typedef struct {
    uint8_t *buffer;
    size_t   capacity;
    size_t   mask;
    size_t   head;
    size_t   tail;
} CircularBuffer;

/*
 * cbuf_init:
 *   Initializes a CircularBuffer with an existing memory block of size 'cap'.
 *   'cap' must be a power of two. This function returns:
 *      - true if 'cap' is a power of two and everything is set up,
 *      - false otherwise (the circular buffer is not valid in that case).
 */
bool cbuf_init(CircularBuffer *cbuf, uint8_t *mem_block, size_t cap);

/*
 * cbuf_reset:
 *   Resets the buffer to an empty state (head = tail = 0, full = false).
 */
void cbuf_reset(CircularBuffer *cbuf);

/*
 * cbuf_put:
 *   Attempts to write a single byte into the buffer.
 *   Returns true on success, false if the buffer is full (no overwrite).
 */
bool cbuf_put(CircularBuffer *cbuf, uint8_t data);

/*
 * cbuf_get:
 *   Attempts to read a single byte from the buffer.
 *   Returns true on success, false if the buffer is empty.
 */
bool cbuf_get(CircularBuffer *cbuf, uint8_t *data);

/*
 * cbuf_full:
 *   Returns 'true' if the buffer is full, 'false' otherwise.
 */
bool cbuf_full(const CircularBuffer *cbuf);

/*
 * cbuf_empty:
 *   Returns 'true' if the buffer is empty, 'false' otherwise.
 */
bool cbuf_empty(const CircularBuffer *cbuf);

/*
 * cbuf_size:
 *   Returns the current number of bytes stored in the buffer.
 */
size_t cbuf_size(const CircularBuffer *cbuf);

/*
 * cbuf_capacity:
 *   Returns the total capacity (in bytes) of the buffer.
 */
size_t cbuf_capacity(const CircularBuffer *cbuf);

#endif /* CIRCULAR_BUFFER_H */
