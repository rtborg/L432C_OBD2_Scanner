/*
 * k_line.c
 *
 *  Created on: Aug 2, 2021
 *      Author: bkereziev
 */

#include "k_line.h"
#include "stm32l4xx_hal.h"

#define INIT_BYTE_POS	0x00
#define KW0_POS			0x01
#define KW1_POS			0x02
#define INVERTED_INIT_BYTE_POS	0x03

/****************************************************************************************
 * File variables
 ****************************************************************************************/

uint8_t k_line_rx_buffer[K_LINE_RX_BUFFER_SIZE];			/* The underlying buffer for the k-line receive queue */
uint8_t k_line_tx_buffer[K_LINE_TX_BUFFER_SIZE];

cbuf_handle_t k_line_rx_buffer_handle;						/* EXTERNAL - The k-line receive queue. It is filled by the USART1 ISR and consumed in user task */
cbuf_handle_t k_line_tx_buffer_handle;

/* Private functions */
static void k_line_uart_init(const uint32_t baud);
static void k_line_uart_deinit();

/****************************************************************************************
 * PUBLIC FUNCTIONS
 ****************************************************************************************/

/****************************************************************************************
 * Initialize the K-line driver
 * The function takes care for buffer allocation and needs to be called before making
 * references to the k_line_rx_buffer_handle!
 *
 ****************************************************************************************/
void k_line_driver_init()
{
	k_line_rx_buffer_handle = circular_buf_init(k_line_rx_buffer, K_LINE_RX_BUFFER_SIZE);
	k_line_tx_buffer_handle = circular_buf_init(k_line_tx_buffer, K_LINE_TX_BUFFER_SIZE);
}

/****************************************************************************************
 * Calculate string length
 ****************************************************************************************/
size_t stringlen(const char *str)
{
	const char *s;
	for (s = str; *s; ++s);
	return (s - str);
}

/****************************************************************************************
 * Send an array of bytes via ISR
 ****************************************************************************************/
void k_line_send_data_it(const uint8_t *str, const int size)
{
	// Check if there's enough space in buffer
	size_t buffer_capacity = circular_buf_capacity(k_line_tx_buffer_handle);
	int i;

	if (buffer_capacity < size)
	{
		return;
	}

	for (i = 0; i < size; i++)
	{
		circular_buf_put(k_line_tx_buffer_handle, *str++);
	}

	USART1->CR1 &= ~USART_CR1_RE;						/* Disable reception during transmission */
	USART1->CR1 &= ~USART_CR1_RXNEIE;					/* Disable receive interrupt */
	uint16_t temp __attribute__((unused)) = USART1->RDR;						/* Read any data in the receive buffer */
	USART1->CR1 |= USART_CR1_TXEIE;						/* Enable TX interrupt */
}

/****************************************************************************************
 * Send a byte via ISR
 ****************************************************************************************/
void k_line_send_byte_it(const uint8_t ch)
{
	if (circular_buf_capacity(k_line_tx_buffer_handle) == 0x00)
	{
		return;
	}

	USART1->CR1 &= ~USART_CR1_RE;						/* Disable reception during transmission */
	USART1->CR1 &= ~USART_CR1_RXNEIE;					/* Disable receive interrupt */
	uint16_t temp __attribute__((unused)) = USART1->RDR;						/* Read any data in the receive buffer */
	circular_buf_put(k_line_tx_buffer_handle, ch);		/* Put char in queue */
	USART1->CR1 |= USART_CR1_TXEIE;						/* Enable TX interrupt */
}

/****************************************************************************************
 * Read a byte from the receive queue
 ****************************************************************************************/
uint8_t k_line_read_byte(uint8_t *data)
{
	uint8_t status = !(circular_buf_empty(k_line_rx_buffer_handle));

	if (status == 1)
	{
		circular_buf_get(k_line_rx_buffer_handle, data);
	}

	return status;
}

/****************************************************************************************
 * Send a byte polling. Not used
 ****************************************************************************************/
int k_line_send_byte(const uint8_t ch)
{
	uint32_t start_time;
	uint8_t temp;

	start_time = HAL_GetTick();

	while (!(USART1->ISR & USART_ISR_TXE))		/* Wait for transmit buffer empty signal */
	{
		if ((HAL_GetTick() - start_time) > K_LINE_TX_TIMEOUT)
			return 0;
	}

	USART1->TDR = ch & 0xff;					/* Place data in buffer */

	while (!(USART1->ISR & USART_ISR_TC))		/* Wait for transmission complete */
		continue;

	start_time = HAL_GetTick();

	while (!(USART1->ISR & USART_ISR_RXNE))		/* Wait for character to be received */
	{
		if ((HAL_GetTick() - start_time) > K_LINE_RX_TIMEOUT)
			return 0;
	}

	temp = USART1->RDR;

	return (temp == ch);
}

/****************************************************************************************
 * Send an array polling. Not used
 ****************************************************************************************/
int k_line_send_data(const uint8_t *data, const int size)
{
	int i;
	int ret = 0;

	for (i = 0; i < size; i++)
	{
		ret &= k_line_send_byte(data[i]);
	}

	return ret;
}


/****************************************************************************************
 * Perform 5-baud slow init
 ****************************************************************************************/
int k_line_slow_init()
{
	/* See https://github.com/iwanders/OBD9141/blob/master/src/OBD9141.cpp */
	/**
	 * Deinitialize UART
	 * Configure pin PA9 as output with pull-up
	 * Delay 3 seconds
	 * PA9 low for 200 ms	- start
	 * PA9 high for 400 ms	- first two bits
	 * PA9 low for 400 ms   - second two bits
	 * PA9 high for 400 ms - third pair
	 * PA9 low for 400 ms   - last pair
	 * PA9 high for 200 ms - stop bit
	 *
	 * Deinitialize PA9
	 *
	 * Initialize UART1 for 10400 baud rate
	 * Read one byte, blocking mode, timeout 500ms
	 * Confirm received byte is 0x55
	 *
	 * Read two bytes v1 and v2, blocking mode, timeout 50 ms
	 * Confirm bytes are identical for ISO9141
	 * For KWP2000, they are not identical
	 *
	 * Wait 30 ms and send inverted v2
	 * Read one byte. blocking mode, timeout 50 ms
	 *
	 * If byte == 0xcc, return true
	 */

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint8_t obd2_response[4] = {0,0,0,0};
	int start_time = 0x0;

	k_line_uart_deinit();

	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);

	HAL_Delay(2500);										/* Initial delay */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);	/* Start bit */
	HAL_Delay(200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);		/* First two bits */
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);	/* Second pair of bits */
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);		/* Third pair of bits */
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);	/* Last pair of bits */
	HAL_Delay(400);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);		/* Stop bit */
	HAL_Delay(200);

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9);						/* Deinitialize pin 9 */
	k_line_uart_init(10400);								/* Initialize UART */

	start_time = HAL_GetTick();								/* Record start time - waiting for byte 0x55 and two keywords */

	while ((3 > circular_buf_size(k_line_rx_buffer_handle)))		/* Wait until three bytes received or timeout */
	{
		if ( (HAL_GetTick() - start_time) > (300 + 20 + 20))
		{
			return 0;		/* Timeout */
		}
	}

	circular_buf_get(k_line_rx_buffer_handle, &obd2_response[INIT_BYTE_POS]);		/* Consume three bytes from the queue */
	circular_buf_get(k_line_rx_buffer_handle, &obd2_response[KW0_POS]);
	circular_buf_get(k_line_rx_buffer_handle, &obd2_response[KW1_POS]);

	if ( 	0x55 != obd2_response[INIT_BYTE_POS] 	||								/* Confirm received bytes are part of initializing sequence of KWP2000 slow init procedure */
			0xE9 != obd2_response[KW0_POS]	 		||
			0x8F != obd2_response[KW1_POS])
	{
		return 0;
	}

	HAL_Delay(32);
	k_line_send_byte_it(~obd2_response[KW1_POS]);			/* Send inverted byte */
	start_time = HAL_GetTick();								/* Record time */

	while (circular_buf_empty(k_line_rx_buffer_handle))
	{
		if ( (HAL_GetTick() - start_time) > 50)
		{
			return 0;		/* Timeout */
		}
	}

	circular_buf_get(k_line_rx_buffer_handle, &obd2_response[INVERTED_INIT_BYTE_POS]);		/* Read back ECU response */

	HAL_Delay(35);		/* A small delay before continuing is needed */

	return (0xCC == obd2_response[INVERTED_INIT_BYTE_POS]);
}

/****************************************************************************************
 * PRIVATE FUNCTIONS
 ****************************************************************************************/

/****************************************************************************************
 * Handle interrupts and fill buffers
 ****************************************************************************************/
void USART1_IRQHandler(void)
{
	uint32_t isrflags   = USART1->ISR;
	static uint8_t ch;

	if (isrflags & USART_ISR_ORE)		/* Overrun error */
	{
		USART1->ICR = USART_ICR_ORECF;
	}
	if (isrflags & USART_ISR_NE)		/* Noise error */
	{
		USART1->ICR = USART_ICR_NCF;
	}
	if (isrflags & USART_ISR_FE)		/* Framing error */
	{
		USART1->ICR = USART_ICR_FECF;
	}

	if ((isrflags & USART_ISR_TXE) && (USART1->CR1 & USART_CR1_TXEIE))								/* Transmit data register empty */
	{
		if (!circular_buf_empty(k_line_tx_buffer_handle))		/* Check if k-line tx buffer has data */
		{
			circular_buf_get(k_line_tx_buffer_handle, &ch);		/* Get data from buffer and place it in USART TDR */
			USART1->TDR = ch;									/* Write to TDR also clears interrupt flag */
		}
		else if (USART1->ISR & USART_ISR_TC)
		{
			USART1->CR1 &= ~USART_CR1_TXEIE;					/* If not data in buffer, disable TX interrupt, but only after last transmission is complete */
			USART1->CR1 |= (USART_CR1_RE | USART_CR1_RXNEIE);	/* Enable reception */
			//ch = USART1->RDR;									/* Read byte from receive register to clear RXNE flag */
		}
	}

	if (USART1->ISR & USART_ISR_RXNE && USART1->CR1 & USART_CR1_RE && USART1->CR1 & USART_CR1_RXNEIE)								/* Received character flag set. Here we test the register, not the snapshot isrflags */
	{
		ch = (uint8_t) USART1->RDR & 0xff;						/* Get char and place it in k-line receive buffer */
		circular_buf_put(k_line_rx_buffer_handle, ch);
	}
}

/****************************************************************************************
 * Initialize the UART
 ****************************************************************************************/
void k_line_uart_init(const uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
	 */
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USART1 interrupt Init */
	HAL_NVIC_SetPriority(USART1_IRQn, 4, 5);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	/* Baud rate calculation:  */
	uint32_t processor_clock = HAL_RCC_GetSysClockFreq();
	uint32_t brr = processor_clock / baud;

	/* Initialize USART1 registers */
	USART1->CR1 = (USART_CR1_TXEIE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE);	/* 8N1,  oversampling by 16, */
	USART1->BRR = brr;	/* program baud rate */

	USART1->CR1 |= USART_CR1_UE;	/* Enable peripheral */
}

/****************************************************************************************
 * De-initialize the UART
 ****************************************************************************************/
void k_line_uart_deinit()
{
	/* Peripheral clock disable */
	__HAL_RCC_USART1_CLK_DISABLE();

	/**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
	 */
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

	/* USART1 interrupt DeInit */
	HAL_NVIC_DisableIRQ(USART1_IRQn);
}
