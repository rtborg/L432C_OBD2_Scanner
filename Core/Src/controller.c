/*
 * controller.c
 *
 * The controller task consumes bytes from a queue (most likely produced in UART ISR),
 * and parses commands. If a command is found, a corresponding function pointer is
 * called.
 *
 *  Created on: Jul 17, 2021
 *      Author: bkereziev
 */

/* Protocol is implemented in a FSM. A good description
at  https://hackaday.com/2015/09/04/embed-with-elliot-practical-state-machines/ */

#include "controller.h"
#include "stm32l4xx_hal.h"

/* Defines */
#define CMD_FUNCTIONS_BUFFER_SIZE 128
#define CMD_BUFFER_SIZE 128
#define CMD_TIMEOUT_MS	20

/* File private variables */
static command_handler commands[CMD_FUNCTIONS_BUFFER_SIZE];			/* Array of pointers to command handlers */
static cbuf_handle_t incoming_queue = 0;		/* The queue which is checked for data on each run */
static cbuf_handle_t outgoing_queue = 0;		/* The response of a function is placed in this queue */
static uint8_t cmd_buffer[CMD_BUFFER_SIZE];		/* Command buffer (linear) */
static uint8_t buffer_index;

/* Private functions, responsible for processing the command */
void (*process_command)(uint8_t);				/* Pointer to a function, which can take any of the functions below. In effect, this is the state - a *process_command can be assigned to any state function */
static void state_await_header(uint8_t c);		/* waits for CMD_START_BYTE, and goes to state_await_len */
static void state_await_len(uint8_t c);			/* Waits for the second byte, which is the command length, and goes to state_in_message */
static void state_in_message(uint8_t c);		/* Places all  bytes in the buffer, up until length - 2 */

static uint8_t inter_byte_time_exceeded();		/* Needs to be called upon receiving each byte */
static uint8_t checksum(const uint8_t *data, uint16_t size);		/* Calculate CheckSum8 Modulo 256:  Sum of Bytes % 256 */


/************************************************************************/
/* Initialize controller task                                           */
/************************************************************************/
void controller_init(const cbuf_handle_t data_queue, const cbuf_handle_t tx_queue)
{
	incoming_queue = data_queue;
	outgoing_queue = tx_queue;
	buffer_index = 0;

	process_command = state_await_header;		/* Initialize state pointer to the first state */

	for (int i = 0; i < CMD_BUFFER_SIZE; i++)	/* Zero command buffer */
	{
		cmd_buffer[i] = 0x0;
	}

	for (int i = 0; i < CMD_FUNCTIONS_BUFFER_SIZE; i++)		/* Zero command function pointers buffer */
	{
		commands[i] = 0x0;
	}
}

/************************************************************************/
/* Controller task - should run on each main loop iteration             */
/************************************************************************/
void controller_task()
{
	static uint8_t ch;

	while (!circular_buf_empty(incoming_queue))
	{
		if (inter_byte_time_exceeded())
			process_command = state_await_header;		/* If false, reset command state*/

		circular_buf_get(incoming_queue, &ch);			/* Get incoming char */
		process_command(ch);							/* Processes the command and calls handler */
	}
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
	buffer_index = 0;
	if (c == CMD_START_BYTE)
	{
		cmd_buffer[buffer_index++] = c;
		process_command = state_await_len;
	}
	else
	{
		buffer_index = 0;
	}
}

void state_await_len(uint8_t c)
{
	if (buffer_index == 1 && c != 0x0)
	{
		cmd_buffer[buffer_index++] = c;
		process_command = state_in_message;
	}
	else
	{
		buffer_index = 0;
		process_command = state_await_header;
	}
}

void state_in_message(uint8_t c)
{
	uint8_t chk;
	static uint8_t response[0xff];
	uint8_t response_length = 0;

	cmd_buffer[buffer_index++] = c;
	if (buffer_index == CMD_BUFFER_SIZE) buffer_index = 0;

	if ((buffer_index) == (cmd_buffer[CMD_LEN_BYTE] + 2))		/* A sequence of length cmd_buffer[1] received */
	{
		chk = checksum(cmd_buffer, buffer_index-1);				/*Each command ends with a checksum byte */

		if (chk == cmd_buffer[buffer_index-1])					/* If both length and checksum are a match, a valid command is received */
		{
			if (commands[cmd_buffer[2]] != 0x00)
				commands[cmd_buffer[2]](cmd_buffer, buffer_index, response, &response_length);	/* Launch the command handler */
		}

		if (response_length)									/* If the command handler is populated the response, send it out */
		{
			chk = checksum(response, response_length);			/* Calculate & append checksum */
			response[response_length] = chk;
			for (int i = 0; i < response_length + 1; i++)		/* Copy response buffer into transmit queue */
			{
				circular_buf_put(outgoing_queue, response[i]);
			}
		}

		process_command = state_await_header;					/* Go back to initial state */
		buffer_index = 0;
	}
}

/************************************************************************
 * End of command states
 ************************************************************************/

void controller_add_command_handler(const uint8_t cmd, const command_handler handler)
{
	commands[cmd] = handler;
}
