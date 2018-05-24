
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
/* Uncomment this if session will be debugged via UART */
#define DEBUG_UART

#define MAX_BUFFER 256

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static void * LSM6DSL_X_0_handle = NULL;
/* State of the System */
uint8_t StatusFlag = 0;
/* Number of potholes detected */
static uint8_t mems_event_detected = 0;
/* Mode of the system (Timed or Delta)
 * 0 : Timed - Send counted potholes every 30 seconds
 * 1 : Burst - Send pothole as soon as detected
 */
static enum typesOfmodeTx {Timed,Burst} modeTx;
/* I2C Address of Seeeduino */
uint16_t I2C_ADDRESS = 4;
/* String message for printing stuff */
char msg [MAX_BUFFER] = "";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
static void initializeAllSensors( void );
static void enableAllSensors( void );
static void blinkLED(uint8_t counts);
static void i2c_scan( void );
static void print_UART( void );
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	memset(msg, 0, MAX_BUFFER);
	ACCELERO_Event_Status_t status;
	HAL_StatusTypeDef errorcode = HAL_ERROR;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  BSP_LED_Init( 0 );
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  sprintf(msg,"Initializating board...\n");
  print_UART();
  initializeAllSensors();
  enableAllSensors();
  i2c_scan();

  HAL_TIM_Base_Start_IT(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  switch(StatusFlag) {
		  case 0 : {
			  break;
		  }
		  case 1 : {
			if(mems_event_detected){
				// Send counted events via I2C
				errorcode = HAL_I2C_Master_Transmit(&hi2c3,I2C_ADDRESS,&mems_event_detected,sizeof(mems_event_detected),HAL_MAX_DELAY);
				sprintf(msg,"Send via I2C [%d] events is %s\n",mems_event_detected,((errorcode == HAL_OK) ? "OK" : "ERROR"));
				print_UART();
			}
			mems_event_detected = 0;
			StatusFlag = 0;
			break;
		  }
		  case 2 : {
			  StatusFlag = 0;
			  // FreeFall detection mode
			  if ( BSP_ACCELERO_Get_Event_Status_Ext( LSM6DSL_X_0_handle, &status ) == COMPONENT_OK ){
				if ( status.FreeFallStatus != 0 ){
					// Increment events detected
					mems_event_detected++;

					sprintf(msg,"Detected event\n");
					print_UART();
					// If mode delta then send via I2C immediately (status = 1)
					if( modeTx == Burst) StatusFlag = 1;
				  }
			  }
			  break;
		  }

		  case 3 : {
			  // Delay which allow press more times the debug button (and go to 3|6|9 StatusFlag mode) via ISR
			  HAL_Delay(500);
			  // Debug button mode status
			  switch(StatusFlag) {
				  case 3 : {
					  // Print general values (Events, timer and Registers)
					  sprintf(msg,"Events: %d Timer is %s: [%lu,%lu,%lu]\n",mems_event_detected,(htim1.Instance->DMAR > 0) ? "Running" : "Stopped",htim1.Instance->CNT,htim1.Instance->PSC,htim1.Instance->ARR);
					  print_UART();
					  break;
				  }
				  case 6 : {
					  // Change mode of the system (Timed or Delta)
					  (htim1.Instance->DMAR > 0) ? HAL_TIM_Base_Stop_IT(&htim1) : HAL_TIM_Base_Start_IT(&htim1);
					  modeTx = !modeTx;
					  // And print result
					  sprintf(msg,"STM32 is now in mode %s\n",((modeTx > 0) ? "Burst" : "Timed"));
					  print_UART();
					  // Blink 6 times if Burst mode or 3 times if Timed mode
					  blinkLED(((modeTx > 0) ? 6 : 3));
					  break;
				  }
				  case 9 : {
					  i2c_scan();
					  break;
				  }
			  }
			  // Reset Flag
			  StatusFlag = 0;
			  break;
		  }
		  default : {
			  // Unknown mode status
			  sprintf(msg,"Error: FlagStatus Unknown\n");
			  print_UART();
			  StatusFlag = 0;
		  }
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 65535;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 32768;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PB3   ------> I2C2_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void initializeAllSensors( void )
{

  if(BSP_ACCELERO_Init( LSM6DSL_X_0, &LSM6DSL_X_0_handle ) == COMPONENT_ERROR)
  {
    /* LSM6DSL not detected, switch on LED2 and go to infinity loop */
	 sprintf(msg,"Error: LSM6DSL Initialization Error\n");
     print_UART();
	_Error_Handler(__FILE__, __LINE__);
  }
}

/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void enableAllSensors( void )
{
	BSP_ACCELERO_Sensor_Enable( LSM6DSL_X_0_handle );
	if ( BSP_ACCELERO_Enable_Free_Fall_Detection_Ext( LSM6DSL_X_0_handle, INT1_PIN ) != COMPONENT_OK )
	{
		sprintf(msg,"Error: LSM6DSL Enabling Free Fall Detection\n");
	     print_UART();
		_Error_Handler(__FILE__, __LINE__);
	}
}
/**
 * @brief  Blink LED X times
 * @param  None
 * @retval None
 */
static void blinkLED(uint8_t counts){
	uint8_t counter;
	for(counter = 0;counter < counts ; counter ++){
		BSP_LED_On( 0 );
		HAL_Delay(100);
		BSP_LED_Off( 0 );
		HAL_Delay(100);
	}
}
/**
 * @brief  EXTI line detection callbacks
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin )
{

  /* User button. */
  if(GPIO_Pin == B1_Pin)
  {
    if ( BSP_PB_GetState( 0 ) == GPIO_PIN_RESET )
    {
      // If debug button mode is UNPRESED then go to debug mode (3,6,9)
    	StatusFlag += 3;
    }
  }

  /* Free fall detection INterrupt PIN (available only for LSM6DSL sensor). */
  else if ( GPIO_Pin == LSM6DSL_INT1_O_PIN )
  {
	  StatusFlag = 2;
  }
}
static void i2c_scan( void )
{
	sprintf(msg,"Scanning I2C devices\n");
    print_UART();
	for(uint16_t i=1;i<128;i++)
	{
		if(HAL_I2C_IsDeviceReady(&hi2c3,i<<1,1,2)==HAL_OK)
		{
			sprintf(msg,"Device %d ready\n",i);
		    print_UART();
			I2C_ADDRESS = i<<1;
		}
		HAL_Delay(1);
	}
	sprintf(msg,"Scan finished\n");
    print_UART();
}
static void print_UART( void ){
#ifdef DEBUG_UART
	HAL_UART_Transmit(&huart2,(uint8_t*)msg,strlen(msg),HAL_MAX_DELAY);
#endif
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  sprintf(msg,"Error file %s at line %d\n",file,line);
  print_UART();
  while(1)
  {
		BSP_LED_On( 0 );
		HAL_Delay(100);
		BSP_LED_Off( 0 );
		HAL_Delay(100);
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
