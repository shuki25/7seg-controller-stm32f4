/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "ssd1306.h"
#include "rtc_sync.h"
#include "ring_buffer.h"
#include "i2c_wrapper.h"
#include "pcf8563.h"
#include "seven_segment.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE BEGIN PD */
#define NUM_DIGITS 3
#define NUM_LED_DIGIT 29
#define USE_BRIGHTNESS 1
#define NUM_SACRIFICIAL_LED 1
#define PI 3.14159265358979323846
#define CODE_BUFFER_SIZE 8
#define UART_RX_BUFFER_SIZE 64

// Remote Code Defines
#define REMOTE_UP		0xFF18E7
#define REMOTE_DOWN		0xFF4AB5
#define REMOTE_LEFT		0xFF10EF
#define REMOTE_RIGHT	0xFF5AA5
#define REMOTE_OK 		0xFF38C7
#define REMOTE_NUM_1	0xFFA25D
#define REMOTE_NUM_2 	0xFF629D
#define REMOTE_NUM_3 	0xFFE21D
#define REMOTE_NUM_4 	0xFF22DD
#define REMOTE_NUM_5 	0xFF02FD
#define REMOTE_NUM_6 	0xFFC23D
#define REMOTE_NUM_7 	0xFFE01F
#define REMOTE_NUM_8 	0xFFA857
#define REMOTE_NUM_9 	0xFF906F
#define REMOTE_NUM_0 	0xFF9867
#define REMOTE_STAR 	0xFF6897
#define REMOTE_POUND 	0xFFB04F

// Date Format
#define DATE_FORMAT "%02d/%02d/%04d"
#define TIME_FORMAT "%02d:%02d:%02d"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch1_trig;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
		.name = "defaultTask",
		.stack_size = 384 * 4,
		.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for heartbeatTask */
osThreadId_t heartbeatTaskHandle;
const osThreadAttr_t heartbeatTask_attributes = {
		.name = "heartbeatTask",
		.stack_size = 128 * 4,
		.priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

// Data Buffer Variables
char lcd_buffer[64];

// LED Strip Variables
seven_segment_t led;
// uint8_t LED_Data[(NUM_LED_DIGIT + NUM_SACRIFICIAL_LED) * NUM_DIGITS][4];
// uint8_t LED_Mod[(NUM_LED_DIGIT + NUM_SACRIFICIAL_LED) * NUM_DIGITS][4];
// uint16_t pwmData[(24 * (NUM_LED_DIGIT + NUM_SACRIFICIAL_LED) * NUM_DIGITS)+50];

// IR Remote Variables

uint32_t tempCode;
uint8_t bitIndex;
uint8_t cmd;
uint8_t cmdli;
uint32_t code;
uint32_t prevCode;
uint8_t codeReady;
uint32_t codeCmd;
uint8_t startIR;
uint8_t repeatCode;
uint8_t potentialRepeatCode;
uint8_t repeatCodeCount;
RingBuffer codeBuffer;

// UART Variables
RingBuffer uartRxBuffer;
uint8_t uartRxData[2] = {0};

// Misc Variables
uint8_t use_external_rtc = 0;
uint8_t has_external_rtc = 0;
uint8_t mute_status_led = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartHeartbeatTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t pwmData[29];

void send(int green, int red, int blue)
{
	uint32_t data = (green << 16) | (red << 8) | blue;
	for (int i = 29; i >=0; i--) {
		if (data & (1 << i)) {
			pwmData[i] = 60;
		} else {
			pwmData[i] = 30;
		}
	}
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_1, (uint32_t *)pwmData, 29);
}

int Get_Digit(int num, int digit) 
{
	int the_digit = (num / (int)pow(10, digit - 1)) % 10;
	return the_digit;
}

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

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_RTC_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */

	HAL_RTC_Init(&hrtc);
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	HAL_RTC_WaitForSynchro(&hrtc);
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

	if (ring_buffer_init(&uartRxBuffer, UART_RX_BUFFER_SIZE, sizeof(uint8_t)) == RING_BUFFER_MALLOC_FAILED) {
		Error_Handler();
	} else {
		HAL_UART_Receive_IT(&huart2, uartRxData, 1);
	}
	RTC_sync_init(&hrtc, &huart2, &htim1, &uartRxBuffer);

	if (ring_buffer_init(&codeBuffer, UART_RX_BUFFER_SIZE, sizeof(uint32_t)) == RING_BUFFER_MALLOC_FAILED) {
		Error_Handler();
	}

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim2);

	__HAL_TIM_SET_COUNTER(&htim2, 0);

	HAL_GPIO_WritePin(OE_GPIO_Port, OE_Pin, GPIO_PIN_RESET);

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of heartbeatTask */
	heartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &heartbeatTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */

	/* Start scheduler */
	osKernelStart();
	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 168;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 7;
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
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

	/* USER CODE BEGIN RTC_Init 0 */

	/* USER CODE END RTC_Init 0 */

	/* USER CODE BEGIN RTC_Init 1 */

	/* USER CODE END RTC_Init 1 */

	/** Initialize RTC Only
	 */
	hrtc.Instance = RTC;
	hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
	hrtc.Init.AsynchPrediv = 127;
	hrtc.Init.SynchPrediv = 255;
	hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
	hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
	hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
	if (HAL_RTC_Init(&hrtc) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN RTC_Init 2 */

	/* USER CODE END RTC_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 84-1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 84-1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 105-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

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
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LED_STATUS_Pin|LED_HB_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, OE_Pin|EEPROM_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : IR_RECV_Pin */
	GPIO_InitStruct.Pin = IR_RECV_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IR_RECV_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED_STATUS_Pin LED_HB_Pin */
	GPIO_InitStruct.Pin = LED_STATUS_Pin|LED_HB_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : OE_Pin EEPROM_Pin */
	GPIO_InitStruct.Pin = OE_Pin|EEPROM_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : DIP3_Pin DIP2_Pin DIP1_Pin */
	GPIO_InitStruct.Pin = DIP3_Pin|DIP2_Pin|DIP1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == IR_RECV_Pin) {

		int counter = (int)__HAL_TIM_GET_COUNTER(&htim2);
		// HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);

		if (counter > 13000) {
			tempCode = 0;
			bitIndex = 0;
			startIR = 1;
			repeatCode = 0;
			potentialRepeatCode = 1;
			repeatCodeCount = 0;
			prevCode = codeCmd;
		}
		else if (counter > 11000 && potentialRepeatCode && !repeatCode && startIR && prevCode) {
			repeatCodeCount++;

			codeCmd = prevCode;
			if (repeatCodeCount > 1) {  // ignore first repeat code
				repeatCode = 1;
				ring_buffer_enqueue(&codeBuffer, (void *)&codeCmd);
			}
		}
		else if (counter > 1700) {
			tempCode |= (1UL << (31-bitIndex));  // write 1
			bitIndex++;
			potentialRepeatCode = 0;
		}
		else if (counter > 1000) {
			tempCode &= ~(1UL << (31-bitIndex));  // write 0
			bitIndex++;
			potentialRepeatCode = 0;
		}
		if (bitIndex == 32) {
			cmdli = ~tempCode; // Logical inversion last 8 bits
			cmd = tempCode >> 8; // Second last 8 bits
			if (cmdli == cmd) {
				code = tempCode;
				codeCmd = code;
				ring_buffer_enqueue(&codeBuffer, (void *)&codeCmd);
				// HAL_GPIO_WritePin(LED_UI_GPIO_Port, LED_UI_Pin, GPIO_PIN_RESET);
			}
			bitIndex = 0;
			startIR = 0;
		}
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		// HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (!ring_buffer_enqueue(&uartRxBuffer, (void *)uartRxData)) {
		Error_Handler();
	}
	HAL_UART_Receive_IT(&huart2, uartRxData, 1);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_1);
	// HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
	led.data_sent_flag = 1;
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* init code for USB_DEVICE */
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN 5 */

	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	uint32_t inf_loop = 0;
	uint32_t remote_cmd = 0;
	struct tm remote_time;
	// char remote_time_str[24];
	pcf8563_t pcf;
	pcf8563_err_t pcf_err;
	int prev_second = 0;
	int counter = 0;

	seven_segment_error_t led_error;

	led_error = seven_segment_init(&led, &htim3, TIM_CHANNEL_1, htim3.Init.Period, 1);
	if (led_error != SEVEN_SEGMENT_OK) {
		Error_Handler();
	}

	seven_segment_set_blank(&led, 0);
	seven_segment_set_digit(&led, 0, 1, 0);
	seven_segment_set_digit(&led, 1, 2, 1);
	seven_segment_set_digit(&led, 2, 3, 2);
	seven_segment_set_brightness(&led, 25);
	seven_segment_WS2812_send(&led);

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(16, 3);
	ssd1306_WriteString("BUTLER", Font_16x26, White);
	ssd1306_UpdateScreen();
	osDelay(2000);

	ssd1306_Fill(Black);
	ssd1306_SetCursor(3, 7);
	ssd1306_WriteString("ELECTRONICS", Font_11x18, White);
	ssd1306_UpdateScreen();
	osDelay(2000);

	ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 7);
	ssd1306_WriteString("(C) 2023", Font_11x18, White);
	ssd1306_UpdateScreen();
	osDelay(2000);

	// Initialize External RTC
	pcf.read = &i2c_read;
	pcf.write = &i2c_write;
	pcf.handle = &hi2c1;
	pcf_err = pcf8563_init(&pcf);
	if (pcf_err != PCF8563_OK) {
		has_external_rtc = 0;
		sprintf(lcd_buffer, "RTC init failed");
		ssd1306_Fill(Black);
		ssd1306_SetCursor(5, 7);
		ssd1306_WriteString(lcd_buffer, Font_7x10, White);
		ssd1306_UpdateScreen();
		osDelay(2000);
	} else {
		has_external_rtc = 1;
	}

	use_external_rtc = HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin) == GPIO_PIN_SET ? 1 : 0;

	if (use_external_rtc == 1 && has_external_rtc == 1) {
		pcf8563_read(&pcf, &remote_time);
		if (remote_time.tm_year < 0) {
			sprintf(lcd_buffer, "Clock not set");
			ssd1306_Fill(Black);
			ssd1306_SetCursor(5, 7);
			ssd1306_WriteString(lcd_buffer, Font_7x10, White);
			ssd1306_UpdateScreen();
			osDelay(2000);
		}
		else {
			RTC_TimeTypeDef sTime;
			RTC_DateTypeDef sDate;

			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

			sTime.Seconds = remote_time.tm_sec;
			sTime.Minutes = remote_time.tm_min;
			sTime.Hours = remote_time.tm_hour;
			sDate.Date = remote_time.tm_mday;
			sDate.Month = remote_time.tm_mon + 1;
			sDate.Year = remote_time.tm_year - 100;

			HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		}
	}
	ssd1306_Fill(Black);

	/* Infinite loop */
	for(;;)
	{
		use_external_rtc = HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin) == GPIO_PIN_SET ? 1 : 0;
		led.sacrificial_led_flag = HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin) == GPIO_PIN_SET ? 1 : 0;
		mute_status_led = HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin) == GPIO_PIN_SET ? 1 : 0;

		if (!(inf_loop & 0b111)) {
			counter++;
			if (counter > 1000) counter = 0;

			if (counter < 10) {
				seven_segment_set_digit(&led, 0, counter, 0);
				seven_segment_set_blank(&led, 1);
				seven_segment_set_blank(&led, 2);
			}
			else if (counter >= 10 && counter < 100) {
				seven_segment_set_digit(&led, 0, counter / 10, 0);
				seven_segment_set_digit(&led, 1, counter % 10, 0);
				seven_segment_set_blank(&led, 2);				
			}
			else if (counter >= 100) {
				seven_segment_set_digit(&led, 0, counter / 100, 0);
				seven_segment_set_digit(&led, 1, (counter / 10) % 10, 0);
				seven_segment_set_digit(&led, 2, counter % 10, 0);
			}
			seven_segment_set_brightness(&led, 15);
			seven_segment_WS2812_send(&led);
		}
		if (mute_status_led) {
			HAL_GPIO_WritePin(LED_HB_GPIO_Port, LED_HB_Pin, GPIO_PIN_RESET);
		}

		if(!(inf_loop & 0b1111) && !mute_status_led) {
			HAL_GPIO_TogglePin(LED_HB_GPIO_Port, LED_HB_Pin);
		}

		if (!is_ring_buffer_empty(&codeBuffer)) {
			ring_buffer_dequeue(&codeBuffer, &remote_cmd);
			sprintf(lcd_buffer, "Code: %0X     ", (unsigned int)remote_cmd);
			ssd1306_SetCursor(5, 20);
			ssd1306_WriteString(lcd_buffer, Font_7x10, White);
			ssd1306_UpdateScreen();
		}

		if (!(inf_loop & 0b11)) {
			if (has_external_rtc == 1 && use_external_rtc == 1) {
				pcf8563_read(&pcf, &remote_time);
				if (remote_time.tm_sec != prev_second) {
					prev_second = remote_time.tm_sec;
					sprintf(lcd_buffer, "%02d:%02d:%02d", remote_time.tm_hour, remote_time.tm_min, remote_time.tm_sec);
					ssd1306_SetCursor(20, 0);
					ssd1306_WriteString(lcd_buffer, Font_11x18, White);
					ssd1306_UpdateScreen();
				}
			} else {
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
				if (sTime.Seconds != prev_second) {
					prev_second = sTime.Seconds;
					sprintf(lcd_buffer, "%02d:%02d:%02d", sTime.Hours, sTime.Minutes, sTime.Seconds);
					ssd1306_SetCursor(20, 0);
					ssd1306_WriteString(lcd_buffer, Font_11x18, White);
					ssd1306_UpdateScreen();
				}
			}
		}

		osDelay(50);
		inf_loop++;
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
 * @brief Function implementing the heartbeatTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
	/* USER CODE BEGIN StartHeartbeatTask */
	/* Infinite loop */
	for(;;)
	{
		osDelay(1);
	}
	/* USER CODE END StartHeartbeatTask */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM4 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM4) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

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
