/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *  Notes on the implementation
 *
 *  The k-line is a one-wire bus, so all bytes are echoed back to the transmitter.
 *  This is handled in the k_line.c file; in short, reception is disabled during
 *  transmission.
 *
 * ISO9141 Information
 *
 * OBD2 Frame Format (Vehicle->Device)
 * PRIORITY | RECEIVER | TRASMITTER | PAYLOAD 7 BYTES MAX | CHECKSUM
 *
 * OBD2 Request Frame Fromat (Device->Vehicle)
 * 0x68 | 0x6A | 0xF1 |  MODE | PID | CHECKSUM
 * Mode - normally 01
 * Example - Engine Speed
 * 0x68 0x6a 0xf1 0x01 0x0c 0xd0
 * Response:
 * 0x48 0x6b 0x11 0x41 0x0c 0x00 0x00 0x11
 *
 * END OF ISO9141 Information
 *
 * ISO 14230-4 KWP SLOW (5 baud init, 10.4 kbaud). Used in Corolla Verso 2007.
 * https://m0agx.eu/2018/01/02/reading-obd2-data-without-elm327-part-2-k-line/
 * The request frame is: 0xC2 <ECU_ADDRESS> 0xF1 <MODE> <PID> <CHECKSUM>.
 * The format byte - C2, carries bit-encoded information: 0x1100 0010 - with address information, functional addressing, len = 2
 *
 * Example request: C2 33 F1 01 0C F3
 * Response: 84 F1 11 41 0C 1F 40 32
 *
 * Request for coolant temperature: C2 33 F1 01 05 EC
 *
 * Notice how the response header is different, and contains response length in the lower nibble of the header byte.
 * The length is the payload length only. The total message length is payload + 4 bytes
 * The first byte of the payload is the mode + 0x40
 * Second byte of the payload is the PID
 *
 * Notes on initialization: if it fails, code should wait at least 2.5 seconds
 * In addition, k-line communication channel is closed after 5 seconds of inactivity.
 *
 * Check
 * http://obdcon.sourceforge.net/2010/06/obd-ii-pids/
 *
 *  RECEIVING COMMANDS VIA UART
 *  USART2 (connected to ST Link VCP) fills a circular buffer in the interrupt
 *  handler callback.
 *
 *  USART1 - K-line, pins PA9 - Tx and PA10 - Rx
 *
 *  KWP2000 Timing constants
 *  P1 Inter byte time for ECU response - 20 ms
 *  P2 Time between tester request and ECU response or two ECU responses - 50 ms
 *  P3 Time between end of ECU responses and start of new tester reques - 5000 ms
 *  P4 Inter byte time for tester request - 20 ms
 *
 *  HARDWARE NOTES
 *  The Nucleo is powered via the 5V rail using a voltage regulator.
 *  The NRST pin needs to be pulled high in this case, because when the ST-LINK is not powered,
 *  it's undefined.
 *
 *
 *
 * ****************************************************************************
 * SOFTWARE MAIN POINTS
 *
 * The software is based on cooperative scheduling of tasks. See tasks_config.c/h
 * and controller.h files for details. The scheduler runs in the main loop and
 * makes a decision which task to run.
 *
 * There are two controller tasks in the application. The controller task consumes
 * bytes from a queue, and tries to parse them in order to construct a command via
 * a FSM. See controller.c/h for implementation details.
 *
 * Controller - files controller.c/h. It looks for commands and in the queue,
 * filled in with data from the VCP, and calls custom handlers, if a command is found.
 * The frame structure is explained in the source files.
 * A command handler for a corresponding byte is called with the command buffer
 * as an argument. The handler can also send back data to the VCP, using the
 * arguments provided.
 *
 * OBD2 Controller - files obd2_controller.c/h. Consumes bytes from the queue
 * filled in by UART1 (see k_line.c/h, where communication with the K-line is
 * handled). It works on the same principle as the previous controller.
 * Custom handlers are added to an array of functions, and then called when a
 * valid OBD2 frame for service 0x01 is received. A default handler is also provided.
 *
 * I2C 3 on pins A6 (clock) and D12 (SDA), Nucleo bord pin designators
 * https://controllerstech.com/oled-display-using-i2c-stm32/
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <assert.h>
#include <stdio.h>		/* for sprintf */
#include <stdlib.h>		/* for malloc & free */
#include <obd2_controller.h>
#include "circular_buffer.h"
#include "controller.h"
#include "scheduler.h"
#include "tasks_config.h"
#include "k_line.h"
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USART2_RX_BUFFER_SIZE	0xff
#define USART2_TX_BUFFER_SIZE	0x80
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

/* USER CODE BEGIN PV */

/* VCP variables */
static uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];				/* USART2 receive buffer */
static uint8_t usart2_tx_buffer[USART2_TX_BUFFER_SIZE];				/* USART2 transmit buffer */
static cbuf_handle_t usart2_rx_buffer_handle;						/* Queue handle */
static cbuf_handle_t usart2_tx_buffer_handle;						/* Queue handle */

/* Scheduler variables. See scheduler files */
static uint32_t tick = 0;
static uint8_t task_index = 0;
static uint8_t number_of_tasks;
static TaskType *task_ptr;

static int k_line_initialized;										/* K-line initialization status */
static char display_buffer[32];										/* SSD1306 buffer */
static int k_line_last_msg_timestamp;								/* Keeps track of last recevied k-line message timestamp. Used to keep track of k-line timeout */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */
void UART2_init(uint32_t baud);								/* Initialize UART2, virtual com port */
void TIM6_Init();											/* Timer 6 interrupts every 1000 ms and toggles an LED - that's a keepalive */
int k_line_init(uint32_t attempts);							/* Tries to initialize k-line and prints number of attempts on display */
void set_k_line_msg_timestamp(const uint32_t timestamp);			/* Update k_line_last_msg_timestamp */
uint32_t get_k_line_msg_timestamp();								/* Get value of k_line_last_msg_timestamp */

/* Controller handlers. The controller takes its data from the VCP */
void change_tim6_period_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length);		/* Controller handler. Example function only; changes TIM6 period */
void transmit_obd2_request_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length);	/* Send an OBD2 request handler. Sends the data from the VCP via the k-line */
void initialize_k_line_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length);		/* Perform a k-line slow init procedure, command is fe03a203a6 for 3 attempts */

/* OBD2 handlers. The OBD2 parser takes its data from UART1 (K-line) */
void obd2_default_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length);													/* OBD2 handler - default OBD2 response handler. Sends out the data via USART2 */
void obd2_coolant_temp_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length);											/* Convert the data to temperature and display it */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	usart2_rx_buffer_handle = circular_buf_init(usart2_rx_buffer, USART2_RX_BUFFER_SIZE);		/* Initialize UART2 queues (virtual COM port) */
	usart2_tx_buffer_handle = circular_buf_init(usart2_tx_buffer, USART2_TX_BUFFER_SIZE);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C3_Init();
	/* USER CODE BEGIN 2 */

	UART2_init(115200);			/* Virtual com port */
	TIM6_Init();

	SSD1306_Init();				/* Display */
	sprintf(display_buffer, "Initializing...");
	SSD1306_GotoXY (1,1);
	SSD1306_Puts (display_buffer, &Font_7x10, 1); // print message
	SSD1306_UpdateScreen(); // update screen

	controller_init(usart2_rx_buffer_handle, usart2_tx_buffer_handle);					/* Initialize command controller with already initialized circular buffers */
	controller_add_command_handler(0xAB, change_tim6_period_handler);					/* Add command handlers to controller */	/* Try sending fe04ab3e806b or fe04abfa00a7 */
	controller_add_command_handler(0xA1, transmit_obd2_request_handler);				/* Send out an OBD2 request */
	controller_add_command_handler(0xA2, initialize_k_line_handler);

	k_line_driver_init();																/* Initialize k-line rx & tx buffers */
	k_line_initialized = k_line_init(3);												/* Try initializing the K-line */														/* Perform a slow init and initialize the k-line UART */

	obd2_controller_init(k_line_rx_buffer_handle);										/* Initialize OBD2 controller with the buffer */
	obd2_controller_add_default_handler(obd2_default_handler);							/* Install default OBD2 response handler. All responses for a service different from 0x01 will be sent to that handler */
	obd2_controller_add_command_handler(0x05, obd2_coolant_temp_handler);				/* Add a few Service 01 PID handlers */

	number_of_tasks = get_number_of_tasks();											/* Initialize scheduler */
	task_ptr = tasks_get_config();

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* Scheduler - runs all tasks in order, depending on their interval */
		tick = HAL_GetTick();
		for (task_index = 0; task_index < number_of_tasks; task_index++)
		{
			if (task_ptr[task_index].Interval == 0)
			{
				(*task_ptr[task_index].Func)();
			}
			else if ((tick - task_ptr[task_index].LastTick >= task_ptr[task_index].Interval))
			{
				(*task_ptr[task_index].Func)();
				task_ptr[task_index].LastTick = tick;
			}
		}	/* End of scheduler */

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
	{
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void)
{

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.Timing = 0x00300F38;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
	{
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : VCP_TX_Pin */
	GPIO_InitStruct.Pin = VCP_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : VCP_RX_Pin */
	GPIO_InitStruct.Pin = VCP_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
	HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*********************************************************************************************
 * Initialize UART2, VCP
 *********************************************************************************************/
void UART2_init(uint32_t baud)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* Peripheral clock enable */
	__HAL_RCC_USART2_CLK_ENABLE();

	__HAL_RCC_GPIOA_CLK_ENABLE();
	/**USART2 GPIO Configuration
	    PA2     ------> USART2_TX
	    PA15 (JTDI)     ------> USART2_RX
	 */
	GPIO_InitStruct.Pin = VCP_TX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = VCP_RX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
	HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

	/* USART2 interrupt Init */
	HAL_NVIC_SetPriority(USART2_IRQn, 4, 5);
	HAL_NVIC_EnableIRQ(USART2_IRQn);

	/* Baud rate calculation:  */
	uint32_t processor_clock = HAL_RCC_GetSysClockFreq();
	uint32_t brr = processor_clock / baud;

	/* Initialize USART2 registers */
	USART2->CR1 = (USART_CR1_TXEIE | USART_CR1_RXNEIE | USART_CR1_TE | USART_CR1_RE);	/* 8N1,  oversampling by 16, */
	USART2->BRR = brr;	/* program baud rate */

	USART2->CR1 |= USART_CR1_UE;	/* Enable peripheral */
}

/****************************************************************************************
 * USART2 Handle interrupts and fill buffers
 ****************************************************************************************/
void USART2_IRQHandler(void)
{
	uint32_t isrflags   = USART2->ISR;
	static uint8_t ch;

	if (isrflags & USART_ISR_ORE)		/* Overrun error */
	{
		USART2->ICR = USART_ICR_ORECF;
	}
	if (isrflags & USART_ISR_NE)		/* Noise error */
	{
		USART2->ICR = USART_ICR_NCF;
	}
	if (isrflags & USART_ISR_FE)		/* Framing error */
	{
		USART2->ICR = USART_ICR_FECF;
	}

	if ((isrflags & USART_ISR_TXE) && (USART2->CR1 & USART_CR1_TXEIE))								/* Transmit data register empty */
	{
		if (!circular_buf_empty(usart2_tx_buffer_handle))		/* Check if k-line tx buffer has data */
		{
			circular_buf_get(usart2_tx_buffer_handle, &ch);		/* Get data from buffer and place it in USART TDR */
			USART2->TDR = ch;									/* Write to TDR also clears interrupt flag */
		}
		else if (USART2->ISR & USART_ISR_TC)
		{
			USART2->CR1 &= ~USART_CR1_TXEIE;					/* If not data in buffer, disable TX interrupt, but only after last transmission is complete */
			USART2->CR1 |= (USART_CR1_RE | USART_CR1_RXNEIE);	/* Enable reception */
		}
	}

	if (USART2->ISR & USART_ISR_RXNE && USART2->CR1 & USART_CR1_RE && USART2->CR1 & USART_CR1_RXNEIE)								/* Received character flag set. Here we test the register, not the snapshot isrflags */
	{
		ch = (uint8_t) USART2->RDR & 0xff;						/* Get char and place it in k-line receive buffer */
		circular_buf_put(usart2_rx_buffer_handle, ch);
	}
}

/*********************************************************************************************
 * Initialize Timer 6 for interrupt every second
 *********************************************************************************************/
void TIM6_Init()
{
	// Enable TIM6 clock
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM6EN;

	// Reset TIM6 configuration
	TIM6->CR1 = 0x0000;
	TIM6->CR2 = 0x0000;

	// Set TIM6 prescaler
	// Fck = 32MHz -> /32000 = 1kHz counting frequency
	TIM6->PSC = (uint16_t) 32000 -1;

	// Set TIM6 auto-reload register for 1000 ms
	TIM6->ARR = (uint16_t) 100 -1;

	// Enable auto-reload preload
	TIM6->CR1 |= TIM_CR1_ARPE;

	// Enable Interrupt upon Update Event
	TIM6->DIER |= TIM_DIER_UIE;

	// Set priority level for TIM6 interrupt
	NVIC_SetPriority(TIM6_DAC_IRQn, 6);

	// Enable TIM6 interrupts
	NVIC_EnableIRQ(TIM6_DAC_IRQn);

	// Start TIM6 counter
	TIM6->CR1 |= TIM_CR1_CEN;
}

/*********************************************************************************************
 * Timer 6 interrupt handler. Toggles the on board LED as a keepalive light
 *********************************************************************************************/
void TIM6_DAC_IRQHandler()
{
	// Test for TIM6 update pending interrupt
	if ((TIM6->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		// Clear pending interrupt flag
		TIM6->SR &= ~TIM_SR_UIF;
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
}

/*********************************************************************************************
 * K-line initialization helper
 *********************************************************************************************/
int k_line_init(uint32_t attempts)
{
	SSD1306_Clear();
	int status = 0;

	for (int i = 0; i < attempts; i++)
	{
		sprintf(display_buffer, "K-Line Attempt %d", i+1);
		SSD1306_GotoXY (1,1); // goto 10, 10
		SSD1306_Puts (display_buffer, &Font_7x10, 1); // print message
		SSD1306_UpdateScreen(); // update screen

		status = k_line_slow_init();
		if (status)
			break;
	}

	if (status)
	{
		sprintf(display_buffer, "Success!");
		set_k_line_msg_timestamp(HAL_GetTick());
	}
	else
	{
		sprintf(display_buffer, "Failed!");
		set_k_line_msg_timestamp(0);
	}

	SSD1306_GotoXY (1,11); // goto 10, 10
	SSD1306_Puts (display_buffer, &Font_7x10, 1); // print message
	SSD1306_UpdateScreen(); // update screen

	return status;
}

/*********************************************************************************************
 * Update last k-line message timestamp
 *********************************************************************************************/
void set_k_line_msg_timestamp(const uint32_t timestamp)
{
	k_line_last_msg_timestamp = timestamp;
}

/*********************************************************************************************
 * Get last k-line message timestamp
 *********************************************************************************************/
uint32_t get_k_line_msg_timestamp()
{
	return k_line_last_msg_timestamp;
}

/*********************************************************************************************
 * Example command handler. Send a command via VCP to see it working
 *********************************************************************************************/
void change_tim6_period_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length)
{
	uint16_t period = command_buffer[3];
	period <<= 8;
	period |= (command_buffer[4]);
	TIM6->PSC = (uint16_t) period -1;

	response[0] = 0xde;
	response[1] = 0xad;
	response[2] = 0xbe;
	response[3] = 0xa7;

	*response_length = 4;
}

/*********************************************************************************************
 * Sends the bytes (OBD2 request) from the VCP buffer via the K-line
 * The OBD2 request is between bytes 3 and len-1, complete with checksum
 *********************************************************************************************/
void transmit_obd2_request_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length)
{
	/*
	 * Try sending FE0DA186f1104100983b80132e0810 to call this handler
	 */

	if (!k_line_initialized)
	{
		return;
	}

	uint8_t *request;		/* Need to allocate storage for copying the OBD2 request in a new array */
	size_t request_len = (size_t) ( (command_buffer_length - 4) * sizeof(uint8_t) );
	int i;

	request = (uint8_t *) malloc(request_len);		/* Allocate memory */

	for (i = 0; i < request_len; i++)				/* Copy request bytes */
	{
		request[i] = command_buffer[3+i];
	}

	obd2_controller_send_request(request, request_len);

	response[0] = 0x41;
	response[1] = 0x31;
	response[2] = 0x4f;
	response[3] = 0x4b;
	*response_length = 4;

	free(request);
}

/*********************************************************************************************
 * Handler for initializing the k-line - send a command via the VCP to call this
 *********************************************************************************************/
void initialize_k_line_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length)
{
	uint8_t attempts = command_buffer[3];
	char msg[32];

	int init_status = 0;

	while (attempts--)
	{
		init_status = k_line_slow_init();
		if (init_status) break;
	}

	k_line_initialized = init_status;

	if (k_line_initialized)
	{
		sprintf(msg, "Init OK");
	}
	else
	{
		sprintf(msg, "Init Failed!");
	}

	SSD1306_GotoXY (10,10); // goto 10, 10
	SSD1306_Puts (msg, &Font_7x10, 1); // print message
	SSD1306_UpdateScreen(); // update screen

	/* A response message sent out via VCP */
	response[0] = 0x41;
	response[1] = 0x31;
	response[2] = 0x4f;
	response[3] = 0x4b;
	response[4] = init_status;

	*response_length = 5;
}

/*********************************************************************************************
 * Default OBD2 response handler. Sends out the OBD2 data via the virtual com port
 *********************************************************************************************/
void obd2_default_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length)
{
	set_k_line_msg_timestamp(HAL_GetTick());
	for (int i = 0; i < command_buffer_length; i++)
	{
		circular_buf_put(usart2_tx_buffer_handle, command_buffer[i]);
	}
}

/*********************************************************************************************
 * OBD2 Coolant Temperature handler
 *********************************************************************************************/
void obd2_coolant_temp_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length)
{
	set_k_line_msg_timestamp(HAL_GetTick());

	char msg[32];

	if (command_buffer_length != 7)
	{
		return;
	}

	if (command_buffer[4] != 0x05)
	{
		return;
	}

	sprintf(msg, "Temp: %d", command_buffer[5] - 40);
	SSD1306_GotoXY (1,30); //
	SSD1306_Puts (msg, &Font_7x10, 1); // print message
	SSD1306_UpdateScreen(); // update screen

	/* Send the data out the VCP - for debug only */
	circular_buf_put(usart2_tx_buffer_handle, 'T');
	circular_buf_put(usart2_tx_buffer_handle, ':');
	circular_buf_put(usart2_tx_buffer_handle, command_buffer[5]);
	circular_buf_put(usart2_tx_buffer_handle, 'C');

}

/*********************************************************************************************
 * UART2 Response task - consumes bytes from the usart2 rx circular buffer
 * and sends them out via interrupt. The command handlers like change_tim6_period()
 * fill the transmit queue, which is then sent via this periodic function
 *********************************************************************************************/
void uart2_response_task()
{
	if (!circular_buf_empty(usart2_tx_buffer_handle))
	{
		USART2->CR1 |= USART_CR1_TXEIE;		/* Kick-start transmission */
	}
}

/*********************************************************************************************
 * Periodic task to send a OBD2 query for coolant temperature
 *********************************************************************************************/
void k_line_coolant_query_task()
{
	static uint8_t coolant_request[] = {0xC2, 0x33, 0xF1, 0x01, 0x05, 0xEC};
	if (k_line_initialized)
	{
		obd2_controller_send_request(coolant_request, 6);
	}
}

/*********************************************************************************************
 * Periodic task to check k-line status by checking if the last obd2 messager timestamp is
 * from 5 seconds (or more). If that is the case, the k-line is re-initialized
 *********************************************************************************************/
void k_line_health_task()
{
	if (HAL_GetTick() - get_k_line_msg_timestamp() > 5000)
	{
		k_line_initialized = k_line_init(1);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
