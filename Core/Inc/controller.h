/*
 * controller.h
 *
 *  Created on: Jul 17, 2021
 *      Author: bkereziev
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "circular_buffer.h"

/*
 * FRAME STRUCTURE
 *
 * CMD_START_BYTE | CMD_LEN_BYTE | CMD | PAYLOAD | CHECKSUM
 */

#define CMD_START_BYTE		0xFE
#define CMD_LEN_BYTE		0x01		/* The command length byte position. The length is excluding the first and second byte */

#define RESPONSE_OK			0xF1
#define RESPONSE_ERROR		0xFF
#define RESPONSE_LEN_BYTE	0x01
#define ERROR_RESPONSE_LEN	0x09
#define GET_VERSION_RESPONSE_LEN	0x11

/*
 *	Pointer to function handler:
	const uint8_t *command_buffer - a pointer to the command buffer is passed to the caller
	const uint32_t command_buffer_length - the size of the command buffer is passed to the caller. The length includes the checksum
	uint8_t *response - the callee should populate the pointer with the response WITHOUT the checksum byte
	uint8_t *response_length - the callee should assign the response length WITHOUT the checksum byte
 */
typedef void (*command_handler)(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length);


/*
 * Initializes controller task.
 * Needs to be called before using the controller
 * data_queue: A queue which contains input data, which is parsed for valid commands
 * tx_queue: The response from each function is placed into the queue
 * Requires: buffers are valid and created by circular_buf_init
 */
void controller_init(const cbuf_handle_t data_queue, const cbuf_handle_t tx_queue);

/*
 *	Consumes bytes from the the incoming data queue and constructs a command.
 If a command is found, a function with corresponding index
 is dispatched.
 */
void controller_task();

/*
 *	Add a command handler to the handlers array. It is inserted at position cmd,
 and called with the corresponding arguments when a command starting with 'cmd' byte
 arrives.
 */
void controller_add_command_handler(const uint8_t cmd, const command_handler handler);

#endif /* INC_CONTROLLER_H_ */
