/*
 * obd2_parser.c
 *
 *  Created on: Aug 1, 2021
 *      Author: bkereziev
 */

#include "obd2_parser.h"
#include "stm32l4xx_hal.h"

/*
 * OBD2 response frame structure:
 * | FMT - DST - SRC | MODE+0x40 - PID - BYTE 0 - ... LAST BYTE | CHECKSUM
 * The first three bytes are the header.
 * The payload is from bytes 3 to len-1.
 * Last byte is the checksum.
 */

/* Defines */
#define FUNCTION_HANDLERS_BUFFER_SIZE 256
#define CMD_BUFFER_SIZE 256
#define CMD_TIMEOUT_MS	20

/* File private variables */
static obd2_command_handler commands[FUNCTION_HANDLERS_BUFFER_SIZE];			/* Array of pointers to command handlers */
static cbuf_handle_t incoming_queue = 0;							/* The queue which is checked for data on each run */
static uint8_t obd2_response_buffer[CMD_BUFFER_SIZE];				/* OBD2 response buffer (linear). Valid OBD2 responses are placed here. */
static uint8_t buffer_index;
static obd2_command_handler default_handler;								/* Default OBD2 command handler function - just returns */

/* Private functions, responsible for processing the command */
void (*process_command)(uint8_t);				/* Pointer to a function, which can take any of the functions below. In effect, this is the state - a *process_command can be assigned to any state function */
static void state_await_header(uint8_t c);		/* waits for CMD_START_BYTE, and goes to state_await_len */
static void state_in_message(uint8_t c);		/* Places all  bytes in the buffer, up until length - 2 */

static uint8_t inter_byte_time_exceeded();		/* Needs to be called upon receiving each byte */
static uint8_t checksum(const uint8_t *data, uint16_t size);		/* Calculate CheckSum8 Modulo 256:  Sum of Bytes % 256 */
void empty_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length);		/* Default function handler */

/************************************************************************/
/* Initialize OBD2 parser task                                           */
/************************************************************************/
void obd2_parser_init(const cbuf_handle_t data_queue)
{
	incoming_queue = data_queue;				/* Get the pointer to the receive queue */
	buffer_index = 0;
	default_handler = NULL;						/* Initialize the default OBD2 function handler */

	process_command = state_await_header;		/* Initialize state pointer to the first state */

	for (int i = 0; i < CMD_BUFFER_SIZE; i++)	/* Zero command buffer */
	{
		obd2_response_buffer[i] = 0x0;
	}

	for (int i = 0; i < FUNCTION_HANDLERS_BUFFER_SIZE; i++)		/* Initialize function pointers buffer */
	{
		commands[i] = empty_handler;
	}

}

/************************************************************************/
/* OBD2 parser task - runs on each 										*/
/************************************************************************/
void obd2_parser_task()
{
	static uint8_t ch;

	while (!circular_buf_empty(incoming_queue))
	{
		if (inter_byte_time_exceeded())
		{
			process_command = state_await_header;		/* If false, reset command state*/
			buffer_index = 0;							/* Reset buffer */
		}


		circular_buf_get(incoming_queue, &ch);			/* Get incoming char */
		process_command(ch);							/* Processes the command and calls handler */
	}
}

/************************************************************************/
/* Add a Service 01 handler												*/
/************************************************************************/
void obd2_parser_add_command_handler(const uint8_t pid, const obd2_command_handler handler)
{
	commands[pid] = handler;
}

/************************************************************************/
/* Add a default handler. All OBD2 responses which do not have
 * a handler are passed to the default handler.						*/
/************************************************************************/
void obd2_parser_add_default_handler( const obd2_command_handler handler)
{
	default_handler = handler;
}

/************************************************************************/
/* Calculate message checksum                                           */
/************************************************************************/
uint8_t checksum(const uint8_t *data, uint16_t size)
{
	uint8_t sum = 0;
    while (size--) {
        sum += *data++;
    }

    return sum;
}

/************************************************************************/
/* Default function handler just returns                               */
/************************************************************************/
void empty_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length)
{
	return;
}

/************************************************************************
 * Check time between each received byte
 ************************************************************************/
uint8_t inter_byte_time_exceeded()
{
	static uint32_t _lastByteReceivedTime = 0;
	uint8_t temp = 0;

	uint32_t time_now = HAL_GetTick();

	if (time_now - _lastByteReceivedTime >= CMD_TIMEOUT_MS)
	{
		temp = 1;
	}

	_lastByteReceivedTime = time_now;
	return temp;
}

/************************************************************************
 * Command states
 ************************************************************************/
void state_await_header(uint8_t c)
{
	/*
	 * Handle receiving the three header bytes.
	 * When they are received, change state
	 * */

	obd2_response_buffer[buffer_index++] = c;

	if (3 == buffer_index)
	{
		process_command = state_in_message;
	}

}

void state_in_message(uint8_t c)
{
	uint8_t chk;

	obd2_response_buffer[buffer_index++] = c;
	if (buffer_index == CMD_BUFFER_SIZE) buffer_index = 0;		/* A sloppy way of handling buffer overflow */

	if ( ( (obd2_response_buffer[0] & 0x3F) +4 ) == buffer_index)				/* Check message length; it is encoded in the format byte */
	{
		chk = checksum(obd2_response_buffer, buffer_index-1);					/*Each command ends with a checksum byte */

		if (chk == obd2_response_buffer[buffer_index-1] )						/* If both length and checksum are a match, a valid command is received */
		{
			// OBD2 response to a service PID request contains the service and the request as the first two bytes in the payload.

			if (0x41 == obd2_response_buffer[3])											/* This is a response to service 0x01 request */
			{
				commands[obd2_response_buffer[4]](obd2_response_buffer, buffer_index);		/* Call user handler */
			}
			else		/* All other responses are handled here. That includes all services different from 0x01 */
			{
				if (default_handler != NULL)		/* Check if a default handler has been installed */
				{
					default_handler(obd2_response_buffer, buffer_index);
				}
			}
		}

		process_command = state_await_header;					/* Go back to initial state */
		buffer_index = 0;
	}
}
