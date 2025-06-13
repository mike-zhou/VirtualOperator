#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>

typedef enum 
{
    ENCODER_0 = 0,
    ENCODER_1,
    ENCODER_2,
    ENCODER_3,
    ENCODER_4,
    ENCODER_5,
    ENCODER_6,
    ENCODER_7,
    ENCODER_COUNT,
    ENCODER_INVALID_ID = 0xFF
} EncoderId;

uint16_t encoder_get_count(const EncoderId encoderId);

#endif
