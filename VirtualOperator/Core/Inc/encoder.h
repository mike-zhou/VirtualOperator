#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum 
{
    ENCODER_ID_0 = 0,
    ENCODER_ID_1,
    ENCODER_ID_2,
    ENCODER_ID_3,
    ENCODER_ID_4,
    ENCODER_ID_5,
    ENCODER_ID_6,
    ENCODER_ID_7,
    ENCODER_ID_COUNT,
    ENCODER_ID_INVALID = 0xFF
} EncoderId;

// initialize internal data for encoders.
void encoders_init();

// this function is called to calculate how far the encoder has moved.
bool encoder_poll(const EncoderId encoderId);

// reset the zero point of the encoder
bool encoder_reset(const EncoderId encoderId);

uint16_t encoder_get_count(const EncoderId encoderId);
int32_t encoder_get_offset(const EncoderId encoderId);

#endif
