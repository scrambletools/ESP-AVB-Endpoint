/*
 * Copyright 2024-2026 Scramble Tools
 * License: MIT
 *
 * AVB Endpoint status OLED, public interface.
 */

#ifndef AVB_ENDPOINT_OLED_H_
#define AVB_ENDPOINT_OLED_H_

/* Start the status display task. Safe to call right after
 * avb_start(), the task waits for the codec I2C bus to appear and
 * exits quietly if no panel answers on the bus. */
void avb_endpoint_oled_start(void);

#endif /* AVB_ENDPOINT_OLED_H_ */
