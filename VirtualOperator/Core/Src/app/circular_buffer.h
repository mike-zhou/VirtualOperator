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
    volatile uint8_t *buffer;
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
bool cbuf_init(volatile CircularBuffer * const cbuf, volatile uint8_t * const mem_block, size_t const cap);

/*
 * cbuf_reset:
 *   Resets the buffer to an empty state (head = tail = 0, full = false).
 */
void cbuf_reset(volatile CircularBuffer * const cbuf);

/*
 * cbuf_put:
 *   Attempts to write a single byte into the buffer.
 *   Returns true on success, false if the buffer is full (no overwrite).
 */
bool cbuf_put(volatile CircularBuffer * const cbuf, uint8_t const data);

/*
 * cbuf_get:
 *   Attempts to read a single byte from the buffer.
 *   Returns true on success, false if the buffer is empty.
 */
bool cbuf_get(volatile CircularBuffer * const cbuf, uint8_t * const data);

/*
 * cbuf_full:
 *   Returns 'true' if the buffer is full, 'false' otherwise.
 */
bool cbuf_full(const volatile CircularBuffer * const cbuf);

/*
 * cbuf_empty:
 *   Returns 'true' if the buffer is empty, 'false' otherwise.
 */
bool cbuf_empty(const volatile CircularBuffer * const cbuf);

/*
 * cbuf_size:
 *   Returns the current number of bytes stored in the buffer.
 */
size_t cbuf_size(const volatile CircularBuffer * const cbuf);

/*
 * cbuf_capacity:
 *   Returns the total capacity (in bytes) of the buffer.
 */
size_t cbuf_capacity(const volatile CircularBuffer * const cbuf);

#endif /* CIRCULAR_BUFFER_H */
