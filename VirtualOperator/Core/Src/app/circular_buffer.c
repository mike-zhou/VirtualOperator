#include "circular_buffer.h"

/*
 * A simple helper to test if 'value' is a power of two (and nonzero).
 * Returns true if yes, false if no.
 */
static bool is_power_of_two(size_t value)
{
    // A number is a power of two if it has exactly 1 bit set (and is not 0).
    return (value != 0) && ((value & (value - 1)) == 0);
}

/*
 * Helpers to advance head/tail pointers.
 * We rely on 'capacity' being a power-of-two, so index wrapping is done by & mask.
 */
static void advance_head(CircularBuffer *cbuf)
{
    cbuf->head = (cbuf->head + 1) & cbuf->mask;
    cbuf->full = (cbuf->head == cbuf->tail);
}

static void advance_tail(CircularBuffer *cbuf)
{
    cbuf->full = false;
    cbuf->tail = (cbuf->tail + 1) & cbuf->mask;
}

/*
 * cbuf_init:
 *   Returns true if 'cap' is a power of two, otherwise false.
 */
bool cbuf_init(CircularBuffer *cbuf, uint8_t *mem_block, size_t cap)
{
    if (!is_power_of_two(cap)) {
        // Not a power of two, fail
        return false;
    }

    cbuf->buffer   = mem_block;
    cbuf->capacity = cap;
    cbuf->mask     = cap - 1; // works only if cap is a power of two
    cbuf->head     = 0;
    cbuf->tail     = 0;
    cbuf->full     = false;

    return true;
}

void cbuf_reset(CircularBuffer *cbuf)
{
    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;
}

bool cbuf_put(CircularBuffer *cbuf, uint8_t data)
{
    if (cbuf_full(cbuf)) {
        // Buffer is full; do not overwrite
        return false;
    }

    cbuf->buffer[cbuf->head] = data;
    advance_head(cbuf);

    return true;
}

bool cbuf_get(CircularBuffer *cbuf, uint8_t *data)
{
    if (cbuf_empty(cbuf)) {
        // Buffer is empty, nothing to read
        return false;
    }

    *data = cbuf->buffer[cbuf->tail];
    advance_tail(cbuf);
    return true;
}

bool cbuf_full(const CircularBuffer *cbuf)
{
    return cbuf->full;
}

bool cbuf_empty(const CircularBuffer *cbuf)
{
    // If not full and head == tail, it's empty
    return (!cbuf->full && (cbuf->head == cbuf->tail));
}

size_t cbuf_capacity(const CircularBuffer *cbuf)
{
    return cbuf->capacity;
}

size_t cbuf_size(const CircularBuffer *cbuf)
{
    if (cbuf->full) {
        return cbuf->capacity;
    } else if (cbuf->head >= cbuf->tail) {
        return (cbuf->head - cbuf->tail);
    } else {
        return (cbuf->capacity + cbuf->head - cbuf->tail);
    }
}
