/*
 * usb_fs.h
 *
 *  Created on: Feb 23, 2025
 *      Author: Mike
 */

#ifndef INC_USB_FS_H_
#define INC_USB_FS_H_

#include <stdint.h>
#include <stdbool.h>

// interface to the USB FS driver
// some data is received in the OUT endpoint
void on_usb_fs_receiving_complete(const uint8_t * buffer, uint32_t length);
// some data is sent out in the IN endpoint
void on_usb_fs_sending_complete(const uint8_t * buffer, uint32_t length);


void init_usb_fs();
void poll_usb_fs();

bool get_usb_fs_byte(uint8_t * p_buffer);
bool send_usb_fs_bytes(const uint8_t * p_buffer, uint32_t length);


#endif /* INC_USB_FS_H_ */
