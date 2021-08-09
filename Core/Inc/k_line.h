/*
 * k_line.h
 *
 *  Created on: Aug 2, 2021
 *      Author: bkereziev
 *
 * K-line implemetation
 *
 * The K-line driver uses USART1 in interrupt mode.
 * All initializations & ISRs are handled in the k_line.c file.
 *
 * Note that the k-line is a single-wire bus and all data is echoed back on the Rx line.
 * That implies some additional processing in the ISR to ensure that the data sent out
 * is read back immediately, and does not enter the input queue.
 */

#ifndef INC_K_LINE_H_
#define INC_K_LINE_H_

#define K_LINE_RX_BUFFER_SIZE	256
#define K_LINE_TX_BUFFER_SIZE	32

#define K_LINE_TX_TIMEOUT		10
#define K_LINE_RX_TIMEOUT		10

#include "circular_buffer.h"

extern cbuf_handle_t k_line_rx_buffer_handle;					/* The k-line receive buffer is a circular array; the ISR fills it with data. It is used in the OBD2 parser functions, so needs to be public */

size_t stringlen(const char *str);								/* Replacement of strlen */
void k_line_driver_init();										/* Initialize the driver and receive buffer. Needs to be called before using the queue! */
int k_line_slow_init();												/* Perform 5-baud KWP2000 slow init and initialize the UART */
void k_line_send_data_it(const uint8_t *str, const int size);					/* Send a byte array via ISR */
void k_line_send_byte_it(const uint8_t ch);						/* Send a byte via ISR */
int k_line_send_byte(const uint8_t ch);							/*  Send a byte polling */
int k_line_send_data(const uint8_t *data, const int size);						/* Send a byte array polling */

#endif /* INC_K_LINE_H_ */
