/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * This program is setup to receive the joystick X and Y coordinates from the motor
  * control tx project. The coordinates will determine which direction(forward or reverse)
  * to set the motors in.
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
#include "../../../../include/NRF24L01.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define JOYSTICK_IN1_PORT GPIOA
#define JOYSTICK_IN1_PIN  GPIO_PIN_9
#define JOYSTICK_IN2_PORT GPIOA
#define JOYSTICK_IN2_PIN  GPIO_PIN_8
#define JOYSTICK_IN3_PORT GPIOB
#define JOYSTICK_IN3_PIN  GPIO_PIN_5
#define JOYSTICK_IN4_PORT GPIOB
#define JOYSTICK_IN4_PIN  GPIO_PIN_3


uint8_t rx_addr[] = {0xEE, 0xDD, 0xCC, 0xBB, 0xAA};
uint8_t rx_data[32];
char coordinate_arr[100];
uint8_t x_coordinate_val = 0;
uint8_t y_coordinate_val = 0;
uint8_t xy_coordinate_val[2];


// This function receives the x and y coordinates from the NRF24L01 rx module
// and assigns it to a  variable. This variable is later used to set the direction
// in the direction function
void joystick_coordiantes_rx()
{
          if (data_available(1)==1)
          {
              rx(rx_data);

              //rx buffer has the x data on the 0th element and y data on the 4th element due to a
              //conversion from uint32_t to uint8_t in the motor_control tx project
              xy_coordinate_val[0] = rx_data[0];
              xy_coordinate_val[1] = rx_data[4];

              //Used to debug and verify the correct coordinates are being transmitted to the NRF24L01 Receiver
              sprintf(coordinate_arr,  "\rX-Coordinate: %d Y-Coordinate: %d \n", xy_coordinate_val[0],xy_coordinate_val[1]);
              HAL_UART_Transmit(&huart2, (uint8_t*)coordinate_arr, 50, 1000);
          }
}

//This function sets both motors in the forward direction
void forward(uint8_t y_coordinate_val)
{
    //check if were still moving in the same direction
    while(y_coordinate_val < 100)
    {
        HAL_GPIO_WritePin (JOYSTICK_IN1_PORT, JOYSTICK_IN1_PIN, 1);
        HAL_GPIO_WritePin (JOYSTICK_IN2_PORT, JOYSTICK_IN2_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN3_PORT, JOYSTICK_IN3_PIN, 1);
        HAL_GPIO_WritePin (JOYSTICK_IN4_PORT, JOYSTICK_IN4_PIN, 0);

        //assigns xy_coordinate_val array to a new set of x and y coordinates from the RF module
        joystick_coordiantes_rx();

        //update y variable to use the most updated joystick value
        y_coordinate_val = xy_coordinate_val[1];
    }
}

//This function sets both motors in the reverse direction
void reverse(uint8_t y_coordinate_val)
{
    //check if were still moving in the same direction
    while(y_coordinate_val > 160)
    {
        HAL_GPIO_WritePin (JOYSTICK_IN1_PORT, JOYSTICK_IN1_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN2_PORT, JOYSTICK_IN2_PIN, 1);
        HAL_GPIO_WritePin (JOYSTICK_IN3_PORT, JOYSTICK_IN3_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN4_PORT, JOYSTICK_IN4_PIN, 1);

        //assigns xy_coordinate_val array to a new set of x and y coordinates from the RF module
        joystick_coordiantes_rx();

        //update y variable to use the most updated joystick value
        y_coordinate_val = xy_coordinate_val[1];
    }
}

//This function sets the left motor in the forward direction and turns the right motor off
void clockwise(uint8_t x_coordinate_val)
{
    //check if were still moving in the same direction
    while(x_coordinate_val > 160)
    {
        HAL_GPIO_WritePin (JOYSTICK_IN1_PORT, JOYSTICK_IN1_PIN, 1);
        HAL_GPIO_WritePin (JOYSTICK_IN2_PORT, JOYSTICK_IN2_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN3_PORT, JOYSTICK_IN3_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN4_PORT, JOYSTICK_IN4_PIN, 0);

        //assigns xy_coordinate_val array to a new set of x and y coordinates from the RF module
        joystick_coordiantes_rx();

        //update y variable to use the most updated joystick value
        x_coordinate_val = xy_coordinate_val[0];
    }
}

//This function sets the right motor in the forward direction and turns the left motor off
void counter_clockwise(uint8_t x_coordinate_val)
{
    //check if were still moving in the same direction
    while(x_coordinate_val < 100)
    {
        HAL_GPIO_WritePin (JOYSTICK_IN1_PORT, JOYSTICK_IN1_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN2_PORT, JOYSTICK_IN2_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN3_PORT, JOYSTICK_IN3_PIN, 1);
        HAL_GPIO_WritePin (JOYSTICK_IN4_PORT, JOYSTICK_IN4_PIN, 0);

        //assigns xy_coordinate_val array to a new set of x and y coordinates from the RF module
        joystick_coordiantes_rx();

        //update y variable to use the most updated joystick value
        x_coordinate_val = xy_coordinate_val[0];
    }
}

//turn both motors off
void no_move()
{
        HAL_GPIO_WritePin (JOYSTICK_IN1_PORT, JOYSTICK_IN1_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN2_PORT, JOYSTICK_IN2_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN3_PORT, JOYSTICK_IN3_PIN, 0);
        HAL_GPIO_WritePin (JOYSTICK_IN4_PORT, JOYSTICK_IN4_PIN, 0);
}

//This function will execute a specific direction function for the motors
//based on the updated x and y coordinate values
void direction()
{

    while(1){

        joystick_coordiantes_rx();

        x_coordinate_val = xy_coordinate_val[0];
        y_coordinate_val = xy_coordinate_val[1];

        if (y_coordinate_val < 100)
        {
            forward(y_coordinate_val);
        }
        else if (y_coordinate_val > 160)
        {
            reverse(y_coordinate_val);
        }
        else if (x_coordinate_val >160)
        {
            clockwise(x_coordinate_val);
        }

        else if (x_coordinate_val <100)
        {
            counter_clockwise(x_coordinate_val);
        }

        else
        {
            no_move();
        }
    }

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
//  uint8_t xy_coordinate_val[2];

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  nrf24_init();
  setup_rx_mode(rx_addr, 100);
  HAL_Delay (1000);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      direction();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, IN2_Pin|IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN4_Pin|IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : CE_Pin */
  GPIO_InitStruct.Pin = CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IN2_Pin IN1_Pin */
  GPIO_InitStruct.Pin = IN2_Pin|IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin IN3_Pin CS_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|IN3_Pin|CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
