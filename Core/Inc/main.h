/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal_conf.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VERSION_MAJOR 1
#define VERSION_MINOR 1

#define IR_RECV_Pin GPIO_PIN_4
#define IR_RECV_GPIO_Port GPIOA
#define IR_RECV_EXTI_IRQn EXTI4_IRQn
#define LED_STATUS_Pin GPIO_PIN_5
#define LED_STATUS_GPIO_Port GPIOA
#define RGB_LED_Pin GPIO_PIN_6
#define RGB_LED_GPIO_Port GPIOA
#define LED_HB_Pin GPIO_PIN_7
#define LED_HB_GPIO_Port GPIOA
#define OE_Pin GPIO_PIN_2
#define OE_GPIO_Port GPIOB
#define EEPROM_Pin GPIO_PIN_12
#define EEPROM_GPIO_Port GPIOB
#define DIP3_Pin GPIO_PIN_10
#define DIP3_GPIO_Port GPIOC
#define DIP2_Pin GPIO_PIN_11
#define DIP2_GPIO_Port GPIOC
#define DIP1_Pin GPIO_PIN_12
#define DIP1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

// Remote Code Defines
#define REMOTE_UP		  0x00FF18E7
#define REMOTE_DOWN		0x00FF4AB5
#define REMOTE_LEFT		0x00FF10EF
#define REMOTE_RIGHT	0x00FF5AA5
#define REMOTE_OK 		0x00FF38C7
#define REMOTE_NUM_1	0x00FFA25D
#define REMOTE_NUM_2 	0x00FF629D
#define REMOTE_NUM_3 	0x00FFE21D
#define REMOTE_NUM_4 	0x00FF22DD
#define REMOTE_NUM_5 	0x00FF02FD
#define REMOTE_NUM_6 	0x00FFC23D
#define REMOTE_NUM_7 	0x00FFE01F
#define REMOTE_NUM_8 	0x00FFA857
#define REMOTE_NUM_9 	0x00FF906F
#define REMOTE_NUM_0 	0x00FF9867
#define REMOTE_STAR 	0x00FF6897
#define REMOTE_POUND 	0x00FFB04F

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
