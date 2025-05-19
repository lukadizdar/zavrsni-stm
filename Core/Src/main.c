/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "uart_strings.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_CH1_SIZE 256
#define RX_BUFFER_CH2_SIZE 256
#define HAL_NODISPLAY 0
#define HAL_YESDISPLAY 1
#define HAL_PHASE_SETUP 0
#define HAL_PHASE_SENSOR 1
#define HAL_RX_INITIALIZED 0
#define HAL_RX_SETUP_START 1
#define HAL_RX_SENSOR_START 2
#define SENSOR_ADDR 0x70
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char tx_buffer_ch1[RX_BUFFER_CH1_SIZE];
char tx_buffer_ch2[RX_BUFFER_CH2_SIZE];
char rx_buffer_ch1[RX_BUFFER_CH1_SIZE];
char rx_buffer_ch2[RX_BUFFER_CH2_SIZE];
char string_buffer_1[RX_BUFFER_CH1_SIZE];
char string_buffer_2[RX_BUFFER_CH2_SIZE];


volatile uint8_t uart_tx_done = 0;
volatile uint8_t rx_event_usart1 = 0;
volatile uint8_t rx_event_uart5 = 0;
uint8_t setup_flags = 0;
uint8_t display_state = HAL_NODISPLAY;
uint8_t working_phase = HAL_PHASE_SETUP;
uint8_t keyboard_last_tick = 0;
uint8_t error_flag = 0;
uint8_t ok_flag = 1;


float temp, hum;
volatile uint8_t tim_flag = 0;
volatile uint8_t i2c_tx_flag = 0;
volatile uint8_t i2c_rx_flag = 0;
uint8_t tx_buf[2] = { 0x7C, 0xA2 };
uint8_t rx_buf[6];
uint8_t timer_init = 1;

uint16_t rx_event_counter = 0;
uint16_t iteration_counter = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  //Enabling both setup RX channels
  HAL_Send_AT_Message("AT+CIPCLOSE\r\n");
  HAL_Delay(500);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)rx_buffer_ch1, RX_BUFFER_CH1_SIZE);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_TC);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, (uint8_t*)rx_buffer_ch2, RX_BUFFER_CH2_SIZE);
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
  __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_TC);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  //SETUP PHASE
	  if (working_phase == HAL_PHASE_SETUP) {
			  //USART SETUP RX ENABLE
		  	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET); //Setup phase: Blue LED turned on constantly

		  	  //USART RX EVENTS
		  	  if (rx_event_usart1 == 1) {
				  rx_event_usart1 = 0;
				  UART_String_Handling_CH1();
				  LED_Checking();
				  rx_event_counter++;
			  }
			  if (rx_event_uart5 == 1) {
				  rx_event_uart5 = 0;
				  UART_String_Handling_CH2();
			  }
				  //Checking for display connected
			  if (display_state == HAL_NODISPLAY) {
				  if (strstr(string_buffer_2, "DISPLAY") != NULL) {
					HAL_UART_Transmit_DMA(&huart5, "AT+DISPLAYACK", strlen("AT+DISPLAYACK"));
					display_state = HAL_YESDISPLAY;
				  }
			  }
			  //SETUP PHASE IF NO KEYBOARD
			  if (display_state == HAL_NODISPLAY) {
				  if (setup_flags < 4) {
					  iteration_counter++;
					  uint8_t diff = iteration_counter - rx_event_counter;
					  if (diff > 1) {
						  iteration_counter -= diff; //lower iteration count, but don't send anything new
					  }
					  else { //if the difference is 1, send new and increment next send
						  if (error_flag) {
							  error_flag = 0;
							  if (setup_flags > 0) setup_flags--; // avoid underflow
							  ESP32_Initialization_Steps(setup_flags);
							  //setup_flags++;
						  }
						  else if (ok_flag){
							  ok_flag = 0;
							  ESP32_Initialization_Steps(setup_flags++);
						  }
						  else {
							 iteration_counter--;
						  }
					  }
				  }
				  else {
					  working_phase = HAL_PHASE_SENSOR;
				  }
			  }
		  }
	  //SENSOR SENDING DATA PHASE
	  if (working_phase == HAL_PHASE_SENSOR) {
		  if (timer_init) {
			  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			  HAL_TIM_Base_Start_IT(&htim2);
			  timer_init = 0;
		  }
		  if (tim_flag) {
			  tim_flag = 0;
			  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
			  uint8_t cmd[2] = { 0x7C, 0xA2 };  // "Wake + Measure T,F" command for SHTC3
			  HAL_I2C_Master_Transmit_DMA(&hi2c1, SENSOR_ADDR << 1, cmd, 2);
		  }
		  if (i2c_tx_flag) {
			  i2c_tx_flag = 0;
			  HAL_I2C_Master_Receive_DMA(&hi2c1, SENSOR_ADDR << 1 | 0x01, rx_buf, 6);
		  }
		  if (i2c_rx_flag) {
			  i2c_rx_flag = 0;
			  uint16_t raw_temp = (rx_buf[0] << 8) | rx_buf[1];
			  uint16_t raw_hum  = (rx_buf[3] << 8) | rx_buf[4];
			  temp = -45.0 + 175.0 * ((float)raw_temp / 65535.0);
			  hum  = 100.0 * ((float)raw_hum / 65535.0);
			  char sensor_data[64];
			  snprintf(sensor_data, sizeof(sensor_data),"Temperature: %.2f °C, Humidity: %.2f", temp, hum);
			  int msg_len = strlen(sensor_data);
			  char at_command[32];
			  snprintf(at_command, sizeof(at_command), "AT+CIPSEND=%d\r\n", msg_len);
			  HAL_Send_AT_Message(at_command);
			  HAL_Delay(5);
			  HAL_Send_AT_Message(sensor_data);
		  }
		  if (!tim_flag && !rx_event_usart1 && !rx_event_uart5 && !i2c_tx_flag && !i2c_rx_flag) {
		      __WFI();  // Puts the CPU in Sleep mode!
		  }



	  }


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		uart_tx_done = 1;
		//HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 1);
	}
	if (huart->Instance == UART5) {

	}
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == USART1) {
		rx_event_usart1 = 1;
		cnt = Size;
	}
	if (huart->Instance == UART5) {
		rx_event_uart5 = 1;
		cnt_2 = Size;
	}
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    i2c_tx_flag = 1;
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	i2c_rx_flag = 1;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		tim_flag = 1;
	}
}



void LED_Checking() {
	  if (strstr(string_buffer_1, "OK") != NULL) {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
		  ok_flag = 1;
	  }
	  else if (strstr(string_buffer_1, "ERROR") != NULL) {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		  error_flag = 1;
	  }
	  else {
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
	  }
}


void strip_crlf(char *str) {
    size_t len = strlen(str);
    while (len > 0 && (str[len - 1] == '\r' || str[len - 1] == '\n')) {
        str[--len] = '\0';
    }
}

void HAL_Send_AT_Message(char* buffer) {
	  sprintf(tx_buffer_ch1, buffer);
	  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)tx_buffer_ch1, strlen(tx_buffer_ch1));
}

void ESP32_Initialization_Steps(uint8_t setup_value) {
	  switch (setup_value) {
		case 0:
			HAL_Send_AT_Message("AT\r\n");
			break;
		case 1:
			HAL_Send_AT_Message("AT+CWMODE=1\r\n");
			break;
		case 2:
			HAL_Send_AT_Message("AT+CWJAP=\"A1-Dizdar-24ghz\",\"xbaja888\"\r\n");
			break;
		case 3:
			HAL_Send_AT_Message("AT+CIPSTART=\"TCP\",\"192.168.100.21\",5500\r\n");
			break;
		default:
			break;
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
