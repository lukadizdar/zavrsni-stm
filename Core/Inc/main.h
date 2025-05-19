/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void strip_crlf(char *str);
void LED_Checking();
void UART_String_Handling();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define RX_BUFFER_CH1_SIZE 256
#define RX_BUFFER_CH2_SIZE 256
#define HAL_NODISPLAY 0
#define HAL_YESDISPLAY 1
#define HAL_PHASE_SETUP 0
#define HAL_PHASE_SENSOR 1
#define HAL_RX_INITIALIZED 0
#define HAL_RX_SETUP_START 1
#define HAL_RX_SENSOR_START 2


extern char tx_buffer_ch1[RX_BUFFER_CH1_SIZE];
extern char tx_buffer_ch2[RX_BUFFER_CH2_SIZE];
extern char rx_buffer_ch1[RX_BUFFER_CH1_SIZE];
extern char string_buffer_1[RX_BUFFER_CH1_SIZE];
extern char rx_buffer_ch2[RX_BUFFER_CH2_SIZE];
extern char string_buffer_1[RX_BUFFER_CH1_SIZE];
extern char string_buffer_2[RX_BUFFER_CH2_SIZE];



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
