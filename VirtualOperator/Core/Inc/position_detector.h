/*
 * position_detector.h
 *
 *  Created on: Jul 13, 2025
 *      Author: mike
 */
#ifndef INC_POSITION_DETECTOR_H_
#define INC_POSITION_DETECTOR_H_

#include <stdint.h>
#include <stdbool.h>

void position_detector_init_data();
bool position_detector_set_masks(const uint16_t * const pMasks, const uint16_t * const pCriticalMasks, const uint8_t length);
void poll_position_detector();

#endif /* INC_POSITION_DETECTOR_H_ */
