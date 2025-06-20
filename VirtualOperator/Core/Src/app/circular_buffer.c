#include "circular_buffer.h"

/*
 * A simple helper to test if 'value' is a power of two (and nonzero).
 * Returns true if yes, false if no.
 */
static bool _is_power_of_two(size_t const value)
{
    // A number is a power of two if it has exactly 1 bit set (and is not 0).
    return (value != 0) && ((value & (value - 1)) == 0);
}

/*
 * Helpers to advance head/tail pointers.
 * We rely on 'capacity' being a power-of-two, so index wrapping is done by & mask.
 */
static inline void _advance_head(volatile CircularBuffer * const cbuf)
{
    cbuf->head = (cbuf->head + 1) & cbuf->mask;
}

static inline void _advance_tail(volatile CircularBuffer * const cbuf)
{
    cbuf->tail = (cbuf->tail + 1) & cbuf->mask;
}

bool cbuf_full(const volatile CircularBuffer * const cbuf)
{
    return ((cbuf->head + 1) & cbuf->mask) == cbuf->tail;
}

bool cbuf_empty(const volatile CircularBuffer * const cbuf)
{
    return cbuf->head == cbuf->tail;
}

/*
 * cbuf_init:
 *   Returns true if 'cap' is a power of two, otherwise false.
 */
bool cbuf_init(volatile CircularBuffer * const cbuf, volatile uint8_t * const mem_block, size_t const cap)
{
    if (!_is_power_of_two(cap)) {
        // Not a power of two, fail
        return false;
    }

    cbuf->buffer   = mem_block;
    cbuf->capacity = cap;
    cbuf->mask     = cap - 1; // works only if cap is a power of two
    cbuf->head     = 0;
    cbuf->tail     = 0;

    return true;
}

void cbuf_reset(volatile CircularBuffer * const cbuf)
{
    cbuf->head = 0;
    cbuf->tail = 0;
}

bool cbuf_put(volatile CircularBuffer * const cbuf, uint8_t const data)
{
    if (cbuf_full(cbuf)) {
        // Buffer is full; do not overwrite
        return false;
    }

    cbuf->buffer[cbuf->head] = data;
    _advance_head(cbuf);

    return true;
}

bool cbuf_get(volatile CircularBuffer * const cbuf, uint8_t * const data)
{
    if (cbuf_empty(cbuf)) {
        // Buffer is empty, nothing to read
        return false;
    }

    *data = cbuf->buffer[cbuf->tail];
    _advance_tail(cbuf);
    return true;
}

size_t cbuf_capacity(const volatile CircularBuffer * const cbuf)
{
    return cbuf->capacity - 1;
}

size_t cbuf_size(const volatile CircularBuffer * const cbuf)
{
    if (cbuf->head >= cbuf->tail) {
        return (cbuf->head - cbuf->tail);
    } else {
        return (cbuf->capacity + cbuf->head - cbuf->tail);
    }
}
