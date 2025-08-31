#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_

#include <stdint.h>

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

uint16_t encoder_get_count(const EncoderId encoderId);

#endif
