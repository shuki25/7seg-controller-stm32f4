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
#include "eeprom.h"
#include "bcd_util.h"
#include "nmea.h"
#include "sync_gps_rtc.h"
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
#define UART_RX_BUFFER_SIZE 512
#define IR_REMOTE_BUFFER_SIZE 16

// Remote Code Defines
#define REMOTE_UP		0x807F8C73
#define REMOTE_DOWN		0x807FA55A
#define REMOTE_LEFT		0x807F8877
#define REMOTE_RIGHT	0x807FAD52
#define REMOTE_OK 		0x807F9C63
#define REMOTE_NUM_1	0x807FD12E
#define REMOTE_NUM_2 	0x807FB14E
#define REMOTE_NUM_3 	0x807FF10E
#define REMOTE_NUM_4 	0x807F916E
#define REMOTE_NUM_5 	0x807F817E
#define REMOTE_NUM_6 	0x807FE11E
#define REMOTE_NUM_7 	0x807FF00F
#define REMOTE_NUM_8 	0x807FD42B
#define REMOTE_NUM_9 	0x807FC837
#define REMOTE_NUM_0 	0x807FCC33
#define REMOTE_STAR 	0x807FB44B
#define REMOTE_POUND 	0x807FD827

// Date Format
#define DATE_FORMAT "%04d-%02d-%02d"
#define TIME_FORMAT "%02d:%02d:%02d"

// EEPROM Defines
#define EEPROM_DATE_PAGE 0
#define EEPROM_DATE_OFFSET 0

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
TIM_HandleTypeDef htim5;
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
static void MX_TIM5_Init(void);
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
	MX_TIM5_Init();
	/* USER CODE BEGIN 2 */

	HAL_RTC_Init(&hrtc);
	__HAL_RTC_WRITEPROTECTION_DISABLE(&hrtc);
	HAL_RTC_WaitForSynchro(&hrtc);
	__HAL_RTC_WRITEPROTECTION_ENABLE(&hrtc);

	if (ring_buffer_init(&uartRxBuffer, UART_RX_BUFFER_SIZE, sizeof(uint8_t)) == RING_BUFFER_MALLOC_FAILED) {
		Error_Handler();
	}
	if (sync_gps_rtc_init(&hrtc, &huart2, &htim5, &uartRxBuffer) != SYNC_GPS_RTC_STATE_OK) {
		Error_Handler();
	}

	RTC_sync_init(&hrtc, &huart2, &htim1, &uartRxBuffer);

	if (ring_buffer_init(&codeBuffer, IR_REMOTE_BUFFER_SIZE, sizeof(uint32_t)) == RING_BUFFER_MALLOC_FAILED) {
		Error_Handler();
	}

	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start(&htim5);

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
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 8400-1;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 4294967295;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
	{
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */

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
	huart2.Init.BaudRate = 9600;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_RX;
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
	ring_buffer_enqueue(&uartRxBuffer, (void *)uartRxData);
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
	uint8_t brightness = 20;
	uint8_t color_index = 0;
	uint8_t is_paused = 0;
	bcd_time_t start_datetime;
	bcd_time_t current_datetime;
	bcd_time_t sync_gps_datetime;
	eeprom_t eeprom;
	uint8_t display_state = 0;
	uint32_t days_since_start = 0;
	uint32_t days_since_start_prev = 0;
	uint8_t hours_since_start = 0;
	uint8_t hours_since_start_prev = 0;
	uint8_t update_display = 0;
	uint8_t sacrifical_led_prev = 0;
	sync_gps_rtc_state_t sync_gps_rtc_state = 0;

	seven_segment_error_t led_error;

	led_error = seven_segment_init(&led, &htim3, TIM_CHANNEL_1, htim3.Init.Period, 1);
	if (led_error != SEVEN_SEGMENT_OK) {
		Error_Handler();
	}
	led.data_sent_flag = 1;

	// Splash Screen
	seven_segment_set_blank(&led, 0);
	seven_segment_set_digit(&led, 0, 10, 0);
	seven_segment_set_digit(&led, 1, 11, 1);
	seven_segment_set_digit(&led, 2, 12, 2);
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

	// Initialize EEPROM if not initialized
	eeprom_status_t eeprom_status = eeprom_init(&eeprom, &hi2c1, EEPROM_GPIO_Port, EEPROM_Pin);
	if (eeprom_status == EEPROM_OK) {
		eeprom_status =	eeprom_read(&eeprom, EEPROM_DATE_PAGE, EEPROM_DATE_OFFSET, (uint8_t *)&start_datetime, sizeof(bcd_time_t));
		if (eeprom_status != EEPROM_OK) {
			Error_Handler();
		}
		if (start_datetime.year < 2023 || start_datetime.year > 2050) {
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			bcd_rtc_to_bcd_time(&sTime, &sDate, &start_datetime);
			if(eeprom.write_protected == 1) {
				eeprom_write_protect(&eeprom, 0);
			}
			eeprom_status = eeprom_write(&eeprom, EEPROM_DATE_PAGE, EEPROM_DATE_OFFSET, (uint8_t *)&start_datetime, sizeof(bcd_time_t));
			if (eeprom_status != EEPROM_OK) {
				Error_Handler();
			}
			eeprom_write_protect(&eeprom, 1);
		}
	} else {
		Error_Handler();
	}

	// start_datetime.hours = 23;
	// start_datetime.minutes = 19;
	// start_datetime.day = 12;
	hours_since_start = 0;

	/* Infinite loop */
	for(;;)
	{
		use_external_rtc = HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin) == GPIO_PIN_SET ? 1 : 0;
		led.sacrificial_led_flag = HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin) == GPIO_PIN_SET ? 1 : 0;
		mute_status_led = HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin) == GPIO_PIN_SET ? 1 : 0;

		if (sacrifical_led_prev != led.sacrificial_led_flag) {
			update_display = 1;
			sacrifical_led_prev = led.sacrificial_led_flag;
		}

		if (has_external_rtc && use_external_rtc) {
			pcf8563_read(&pcf, &remote_time);
			current_datetime.year = remote_time.tm_year + 1900;
			current_datetime.month = remote_time.tm_mon + 1;
			current_datetime.day = remote_time.tm_mday;
			current_datetime.hours = remote_time.tm_hour;
			current_datetime.minutes = remote_time.tm_min;
			current_datetime.seconds = remote_time.tm_sec;
			current_datetime.day_of_week = remote_time.tm_wday;
		}
		else {
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			bcd_rtc_to_bcd_time(&sTime, &sDate, &current_datetime);
		}

		if (current_datetime.year > start_datetime.year+5 || current_datetime.year < start_datetime.year) {
			sync_gps_rtc_set(&start_datetime);
			HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
			bcd_rtc_to_bcd_time(&sTime, &sDate, &current_datetime);
		}

		days_since_start = bcd_days_between_dates(&start_datetime, &current_datetime);

		// Rotating Messages
		if (!(inf_loop & 0b11111)) {
			if (display_state == 0) {
				sprintf(lcd_buffer, " Start Date ");
				ssd1306_SetCursor(22, 20);
			}
			else if (display_state == 1) {
				sprintf(lcd_buffer, DATE_FORMAT, start_datetime.year, start_datetime.month, start_datetime.day);
				ssd1306_SetCursor(28, 20);
			}
			else if (display_state == 3) {
				sprintf(lcd_buffer, " Current Date ");
				ssd1306_SetCursor(15, 20);
			}
			else if (display_state == 4) {
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
				sprintf(lcd_buffer, "  " DATE_FORMAT "  ", sDate.Year + 2000, sDate.Month, sDate.Date);
				ssd1306_SetCursor(15, 20);
			}
			else if (display_state == 6) {
				sprintf(lcd_buffer, "Nbr Days");
				ssd1306_SetCursor(36, 20);
			}
			else if (display_state == 7) {
				if (days_since_start == 1) {
					sprintf(lcd_buffer, "  1 day   ");
				}
				else {
					sprintf(lcd_buffer, "  %d days  ", (int)days_since_start);

				}
				int center = (128 - (strlen(lcd_buffer) * 7)) / 2;
				ssd1306_SetCursor(center, 20);
			}
			else {
				sprintf(lcd_buffer, "                  ");
				ssd1306_SetCursor(0, 20);
			}

			ssd1306_WriteString(lcd_buffer, Font_7x10, White);
			ssd1306_UpdateScreen();
			display_state++;
			if (display_state > 8) display_state = 0;
		}

		if (days_since_start < 1) {
			hours_since_start = bcd_hours_between_times(&start_datetime, &current_datetime);
			if (hours_since_start < 24) {
				if (hours_since_start != hours_since_start_prev || update_display) {
					uint8_t hour_color_index = color_index + 1;
					if (hour_color_index > 6) hour_color_index = 0;

					if (hours_since_start < 10) {
						seven_segment_set_digit(&led, 0, hours_since_start, color_index);
						seven_segment_set_digit(&led, 1, 16, hour_color_index);
						seven_segment_set_blank(&led, 2);
					}
					else if (hours_since_start >= 10 && hours_since_start < 100) {
						seven_segment_set_digit(&led, 0, hours_since_start / 10, color_index);
						seven_segment_set_digit(&led, 1, hours_since_start % 10, color_index);
						seven_segment_set_digit(&led, 2, 16, hour_color_index);				
					}
					seven_segment_set_brightness(&led, brightness);
					seven_segment_WS2812_send(&led);
					hours_since_start_prev = hours_since_start;
					update_display = 0;
				}
			}
		}

		if (days_since_start >= 1) {
			if (days_since_start != days_since_start_prev || update_display) {
				if (days_since_start < 10) {
					seven_segment_set_digit(&led, 0, days_since_start, color_index);
					seven_segment_set_blank(&led, 1);
					seven_segment_set_blank(&led, 2);
				}
				else if (days_since_start >= 10 && days_since_start < 100) {
					seven_segment_set_digit(&led, 0, days_since_start / 10, color_index);
					seven_segment_set_digit(&led, 1, days_since_start % 10, color_index);
					seven_segment_set_blank(&led, 2);				
				}
				else if (days_since_start >= 100) {
					seven_segment_set_digit(&led, 0, days_since_start / 100, color_index);
					seven_segment_set_digit(&led, 1, (days_since_start / 10) % 10, color_index);
					seven_segment_set_digit(&led, 2, days_since_start % 10, color_index);
				}
				seven_segment_set_brightness(&led, brightness);
				seven_segment_WS2812_send(&led);
				days_since_start_prev = days_since_start;
				update_display = 0;
			}
		}

		if (!is_ring_buffer_empty(&codeBuffer)) {
			ring_buffer_dequeue(&codeBuffer, &remote_cmd);
			sprintf(lcd_buffer, "Code: %0X     ", (unsigned int)remote_cmd);
			ssd1306_SetCursor(5, 20);
			ssd1306_WriteString(lcd_buffer, Font_7x10, White);
			ssd1306_UpdateScreen();

			switch (remote_cmd)
			{
			case REMOTE_UP:
				brightness += 5;
				if (brightness > 45) brightness = 45;
				update_display = 1;
				break;
			case REMOTE_DOWN:
				brightness -= 5;
				if (brightness > 45) brightness = 0;
				update_display = 1;
				break;
			case REMOTE_LEFT:
				color_index--;
				if (color_index > 6) color_index = 6;
				update_display = 1;
				break;
			case REMOTE_RIGHT:
				color_index++;
				if (color_index > 6) color_index = 0;
				update_display = 1;
				break;
			case REMOTE_OK:
				is_paused = !is_paused;
				break;
			case REMOTE_NUM_1:			
				break;
			case REMOTE_NUM_2:				
				break;
			case REMOTE_NUM_3:				
				break;
			case REMOTE_NUM_4:				
				break;
			case REMOTE_NUM_5:				
				break;
			case REMOTE_NUM_6:				
				break;
			case REMOTE_NUM_7:				
				break;
			case REMOTE_NUM_8:				
				break;
			case REMOTE_NUM_9:				
				break;
			case REMOTE_NUM_0:				
				break;
			case REMOTE_STAR:
				HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
				HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
				bcd_rtc_to_bcd_time(&sTime, &sDate, &start_datetime);
				if(eeprom.write_protected == 1) {
					eeprom_write_protect(&eeprom, 0);
				}
				eeprom_status = eeprom_write(&eeprom, EEPROM_DATE_PAGE, EEPROM_DATE_OFFSET, (uint8_t *)&start_datetime, sizeof(bcd_time_t));
				if (eeprom_status != EEPROM_OK) {
					Error_Handler();
				}
				eeprom_write_protect(&eeprom, 1);
				break;
			case REMOTE_POUND:
				ssd1306_Fill(Black);
				sprintf(lcd_buffer, "Syncing...");
				ssd1306_SetCursor(3, 0);
				ssd1306_WriteString(lcd_buffer, Font_11x18, White);
				ssd1306_UpdateScreen();
				sprintf(lcd_buffer, "Waiting for GPS");
				ssd1306_SetCursor(8, 20);
				ssd1306_WriteString(lcd_buffer, Font_7x10, White);
				ssd1306_UpdateScreen();

				// Start listening for NMEA data
				ring_buffer_flush(&uartRxBuffer);
				__HAL_TIM_SET_COUNTER(&htim5, 0);
				HAL_UART_Receive_IT(&huart2, uartRxData, 1);
				memset(&sync_gps_datetime, 0, sizeof(bcd_time_t));
				sync_gps_rtc_state = sync_gps_rtc_sync(&sync_gps_datetime);
				HAL_UART_AbortReceive_IT(&huart2);
				if(sync_gps_rtc_state == SYNC_GPS_RTC_STATE_SYNCED) {
					sync_gps_rtc_set(&sync_gps_datetime);
					sprintf(lcd_buffer, " Syncing Clock ");
					ssd1306_SetCursor(8, 20);
					ssd1306_WriteString(lcd_buffer, Font_7x10, White);
					ssd1306_UpdateScreen();
					osDelay(1000);
					ssd1306_Fill(Black);
				} else if (sync_gps_rtc_state == SYNC_GPS_RTC_STATE_TIMEOUT) {
					sprintf(lcd_buffer, "   Timed Out   ");
					ssd1306_SetCursor(8, 20);
					ssd1306_WriteString(lcd_buffer, Font_7x10, White);
					ssd1306_UpdateScreen();
					osDelay(1000);
					ssd1306_Fill(Black);
				} else if (sync_gps_rtc_state == SYNC_GPS_RTC_STATE_ERROR) {
					sprintf(lcd_buffer, "   Error   ");
					ssd1306_SetCursor(8, 20);
					ssd1306_WriteString(lcd_buffer, Font_7x10, White);
					ssd1306_UpdateScreen();
					osDelay(1000);
					ssd1306_Fill(Black);
				}
				break;
			default:
				break;
			}

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
		HAL_GPIO_TogglePin(LED_HB_GPIO_Port, LED_HB_Pin);
		osDelay(1000);
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
		int i;
		HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		for(i=0;i<2000000;i++) { __NOP(); }
		HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		for(i=0;i<2000000;i++) { __NOP(); }
		HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		for(i=0;i<2000000;i++) { __NOP(); }
		HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
		for(i=0;i<8000000;i++) { __NOP(); }
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
