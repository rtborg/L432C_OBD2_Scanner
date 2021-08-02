/*
 * obd2_parser.h
 *
 *  Created on: Aug 1, 2021
 *      Author: bkereziev
 *
 * OBD2 Parser
 *
 * The file follows the structure of controller.h, which shows an elementary general-purpose
 * protocol parsing code.
 *
 * The parser consumes bytes from a queue and looks for a valid OBD2 response frame. If one is found,
 * a handler function is called.
 */

#ifndef INC_OBD2_PARSER_H_
#define INC_OBD2_PARSER_H_

#include "circular_buffer.h"

/*
 *	Pointer to function handler:
	const uint8_t *command_buffer - a pointer to the command buffer is passed to the caller
	const uint32_t command_buffer_length - the size of the command buffer is passed to the caller. The length includes the checksum
 */
typedef void (*obd2_command_handler)(const uint8_t *command_buffer, const uint32_t command_buffer_length);


/**
 * Initialize the OBD2 parser
 * const cbuf_handle_t data_queue - handler of the OBD2 receive queue
 * Requires: buffer is valid
 */
void obd2_parser_init(const cbuf_handle_t data_queue);

/*
 *	Consumes bytes from the the incoming data queue and constructs an OBD2 response frame.
If a valid response is detected, a function based on the PID is called.
 */
void obd2_parser_task();

/*
 *	Add a command handler to the handlers array. It is inserted at position pid,
 and called with the corresponding arguments when a OBD2 frame with the correct PID arrives
 */
void obd2_parser_add_command_handler(const uint8_t pid, const obd2_command_handler handler);

/*
 *	Add a default command handler.
 */
void obd2_parser_add_default_handler(const obd2_command_handler handler);

#endif /* INC_OBD2_PARSER_H_ */
