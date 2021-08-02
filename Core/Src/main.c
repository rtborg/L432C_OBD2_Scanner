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
  *  The k-line is a one-wire bus, so all bytes are echoed back to the transmitter
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
  * ISO 14230-4 KWP SLOW (5 baud init, 10.4 kbaud)
  * https://m0agx.eu/2018/01/02/reading-obd2-data-without-elm327-part-2-k-line/
  * The request frame is: 0xC2 <ECU_ADDRESS> 0xF1 <MODE> <PID> <CHECKSUM>.
  * The format byte - C2, carries bit-encoded information: 0x1100 0010 - with address information, functional addressing, len = 2
  *
  * Example request: C2 33 F1 01 0C F3
  * Response: 84 F1 11 41 0C 1F 40 32
  *
  * Notice how the response header is different, and contains response length in the lower nibble of the header byte.
  * The length is the payload length only. The total message length is payload + 4 bytes
  * The first byte of the payload is the mode + 0x40
  * Second byte of the payload is the PID
  *
  * Notes on initialization: if it fails, code should wait at least 2.5 seconds
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
  *
  *  KWP2000 Timing constants
  *  P1 Inter byte time for ECU response - 20 ms
  *  P2 Time between tester request and ECU response or two ECU responses - 50 ms
  *  P3 Time between end of ECU responses and start of new tester reques - 5000 ms
  *  P4 Inter byte time for tester request - 20 ms
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <assert.h>
#include "circular_buffer.h"
#include "controller.h"
#include "scheduler.h"
#include "tasks_config.h"
#include "k_line.h"
#include "obd2_parser.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t ch;														/* Temporary storage variable */

/* VCP variables */
uint8_t usart2_rx_buffer[USART2_RX_BUFFER_SIZE];				/* USART2 receive buffer */
uint8_t usart2_tx_buffer[USART2_TX_BUFFER_SIZE];				/* USART2 transmit buffer */
cbuf_handle_t usart2_rx_buffer_handle;							/* Queue handle */
cbuf_handle_t usart2_tx_buffer_handle;							/* Queue handle */

/* Scheduler variables */
uint32_t tick = 0;
uint8_t task_index = 0;
uint8_t number_of_tasks;
TaskType *task_ptr;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void TIM6_Init();												/* Timer 6 interrupts every 1000 ms and toggles an LED */
void HAL_UART2_RxCpltCallback(UART_HandleTypeDef *huart);		/* USART2 receive callback */
void change_tim6_period(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length);		/* Example function only; changes TIM6 period */
void obd2_default_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length);													/* Default OBD2 response handler. Sends out the data via USART2 */
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
  TIM6_Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  usart2_rx_buffer_handle = circular_buf_init(usart2_rx_buffer, USART2_RX_BUFFER_SIZE);		/* Initialize UART2 queues (virtual COM port) */
  usart2_tx_buffer_handle = circular_buf_init(usart2_tx_buffer, USART2_TX_BUFFER_SIZE);

  controller_init(usart2_rx_buffer_handle, usart2_tx_buffer_handle);				/* Initialize command controller with already initialized circular buffers */
  controller_add_command_handler(0xAB, change_tim6_period);						 	/* Add command handlers to controller */	/* Try sending fe04ab3e806b or fe04abfa00a7 */

  k_line_driver_init();																/* Initialize k-line rx & tx buffers */
  k_line_slow_init();																/* Perform a slow init and initialize the k-line UART */

  obd2_parser_init(k_line_rx_buffer_handle);										/* Initialize OBD2 parser with the buffer */
  obd2_parser_add_default_handler(obd2_default_handler);							/* Install default OBD2 response handler. All responses for a service different from 0x01 will be sent to that handler */

  number_of_tasks = get_number_of_tasks();											/* Initialize scheduler */
  task_ptr = tasks_get_config();

  HAL_UART_Receive_IT(&huart2, &ch, 0x01);											/* Kick-start USART2 reception */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  if (HAL_UART_RegisterCallback(&huart2, HAL_UART_RX_COMPLETE_CB_ID, HAL_UART2_RxCpltCallback) != HAL_OK)
  {
	  Error_Handler();
  }
  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
 * USART2 (VCP) receive complete callback. Places the received byte in the queue
 *********************************************************************************************/
void HAL_UART2_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t rx = (uint8_t) huart->Instance->RDR & 0xff;		/* Get incoming byte */
	circular_buf_put(usart2_rx_buffer_handle, rx);			/* Enqueue byte */
	HAL_UART_Receive_IT(huart, &ch, 0x01);					/* Re-enable RX ISR */
}

/*********************************************************************************************
 * Example command handler. Send a command via VCP
 *********************************************************************************************/
void change_tim6_period(const uint8_t *command_buffer, const uint32_t command_buffer_length, uint8_t *response, uint8_t *response_length)
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
 * Default OBD2 response handler. Sends out the OBD2 data via the virtual com port
 *********************************************************************************************/
void obd2_default_handler(const uint8_t *command_buffer, const uint32_t command_buffer_length)
{
	HAL_UART_Transmit_IT(&huart2, command_buffer, command_buffer_length);
}

/*********************************************************************************************
 * UART2 Response task - consumes bytes from the usart2 rx circular buffer
 * and sends them out via interrupt. The command handlers like change_tim6_period()
 * fill the transmit queue, which is then sent via this periodic function
 *********************************************************************************************/
void uart2_response_task()
{
	static uint8_t usart_rx_data[USART2_TX_BUFFER_SIZE];
	static uint8_t ch;

	if (circular_buf_empty(usart2_tx_buffer_handle))
		return;

	uint8_t i = 0;

	while (!circular_buf_empty(usart2_tx_buffer_handle))
	{
		circular_buf_get(usart2_tx_buffer_handle, &ch);
		usart_rx_data[i++] = ch;
	}

	HAL_UART_Transmit_IT(&huart2, usart_rx_data, i);
}

/*********************************************************************************************
 * Sample periodic task
 *********************************************************************************************/
void task_100ms()
{
//	uint8_t temp_byte;
//
//	static const uint8_t data[] = {0xde, 0xad, 0xbe, 0xa7, 0xab, 0xcd, 0xff, 0x01};
//	K_Line_SendDataIT(data);
//
//	/* Consume a byte from k line receive buffer, if available */
//	K_Line_ReadByte(&temp_byte);
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
