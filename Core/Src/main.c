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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "cli.h"
#include "i2c_bus.h"
#include "bme280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//todo define dla pozostalych rejestrow
#define I2C_TIMEOUT 100
#define BME280_ADDR 0x76
#define OLED_ADDR 0x3C

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint8_t uart_rx_byte = 0;

void u32_to_bin_str(uint32_t value, uint8_t bits, char *out, size_t out_size)
{
	for (size_t i = 0; i < out_size; i++)
	        out[i] = '\0';

    uint8_t pos = 0;

    for (int i = bits - 1; i >= 0; i--)
    {
        out[pos++] = (value & (1UL << i)) ? '1' : '0';

        if (i % 4 == 0 && i != 0)
            out[pos++] = ' ';
    }

    out[pos++] = '\r';
    out[pos++] = '\n';
    out[pos] = '\0';
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
  MX_USART2_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  cli_init(&huart2);

  struct i2c_bus bus = {&hi2c3, I2C_TIMEOUT};

  struct bme280 sensor = {BME280_ADDR, &bus};


  uint8_t id = 0;

  bme280_soft_reset(&sensor);
  HAL_Delay(100);
  bme280_read_id(&sensor, &id);

  if (id == 0x60)
  {
	  cli_sendln("ID correct");
	  char buf[5];
	  snprintf(buf, sizeof(buf), "0x%02X", id);
	  cli_sendln(buf);
  }



  uint8_t ctrl_hum = 0;

  bme280_read_reg(&sensor, 0xF2, &ctrl_hum, 1);

  char buf_1[15];
  snprintf(buf_1, sizeof(buf_1), "ctrl_hum: 0x%02X", ctrl_hum);

  cli_sendln(buf_1);

  uint8_t mask = 0b00000111;
  uint8_t new_ctrl_hum = 0b00000001; //oversampling x1
  ctrl_hum = (ctrl_hum & ~mask) | (new_ctrl_hum); //zanotowac dzialanie maski

  char buf_2[19];
  snprintf(buf_2, sizeof(buf_2), "new ctrl_hum: 0x%02X", ctrl_hum);

  cli_sendln(buf_2);
  bme280_write(&sensor, 0xF2, &ctrl_hum);


  uint8_t ctrl_meas = 0b00100101; //temperature and pressure oversampling x1, forced mode

  bme280_write(&sensor, 0xF4, &ctrl_meas);

  HAL_Delay(20);

  uint8_t raw[8] = {0};

  bme280_read_raw(&sensor, raw);

  uint32_t raw_press = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);	//0xF7, 0xF8, 0xF9[7:4]
  uint32_t raw_temp = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);	//0xFA, 0xFB, 0xFC[7:4]
  uint16_t raw_hum = (raw[6] << 8) | raw[7]; 							//0xFD, 0xFE

  cli_sendln("Raw data acquired: ");

  char buf_3[64];
  u32_to_bin_str(raw_press, 20, buf_3, sizeof(buf_3));
  cli_sendln(buf_3);

  u32_to_bin_str(raw_temp, 20, buf_3, sizeof(buf_3));
  cli_sendln(buf_3);

  u32_to_bin_str(raw_hum, 16, buf_3, sizeof(buf_3));
  cli_sendln(buf_3);




  uint8_t raw_compensation[33] = {0};
  bme280_read_reg(&sensor, 0x88, raw_compensation, 26);
  bme280_read_reg(&sensor, 0xE1, &raw_compensation[26], 7);

  struct bme280_compensation_params params;

  bme280_parse_compensation(raw_compensation, &params);

  int32_t t_fine = 0;

  int32_t temperature = bme280_calculate_temp(raw_temp, &params, &t_fine);
  uint32_t pressure = bme280_calculate_press(raw_press, &params, t_fine);
  uint32_t humidity = bme280_calculate_hum(raw_hum, &params, t_fine);

  pressure = pressure >> 8; //result in q24.8, turncating fractional part for simplicity
 //result in q22.10, turncating fractional part for simplicity
  float hum_temp = (float)humidity / 1024.0f;

  char buf_4[16];
  snprintf(buf_4, sizeof(buf_4), "%ld", (long)temperature); //todo rozpracowac co ta funkcja robi
  cli_sendln("Calculated temperature: ");
  cli_sendln(buf_4);

  char buf_5[16];
  snprintf(buf_5, sizeof(buf_5), "%lu", (unsigned long)pressure);
  cli_sendln("Calculated pressure: ");
  cli_sendln(buf_5);

  char buf_6[32];
  snprintf(buf_6, sizeof(buf_6), "%.3f", hum_temp);
  cli_sendln("Calculated humidity: ");
  cli_sendln(buf_6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (event_cli_line_ready())
	  {
		  cli_process_line();
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c3.Init.Timing = 0x10909CEC;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//check if r  x comes from usart2
    if (huart->Instance == USART2)
    {
    	cli_on_rx_char(uart_rx_byte);
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
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
