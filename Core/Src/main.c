/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

#include "VL53L1X_API.h"
#include "VL53l1X_calibration.h"
#include "FIRFilter.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
char buff[15];
uint16_t dev = 0x52;
int status;
uint8_t byteData, sensorState=0;
uint16_t wordData;
uint16_t Distance;
uint8_t dataReady;

uint16_t ledLevel;
uint16_t fbits = 0;
int thisled = 0;

uint16_t led[10]={0x0010,0x0020,0x0040,0x0080,0x0001,0x0200,0x0002,0x8000,0x0100,0x0400};

uint8_t a,b,newTrigState;		// These variables stores Manual/auto Status value
uint8_t float_sensor;
int Threshold_min = 2000,Threshold_max = 20;
uint8_t prevTrigBtnState=1;
uint8_t MotorState;

FIRFilter sensor_Read;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void AutonomousLowPowerRangingTest(void); /* see Autonomous ranging example implementation in USER CODE BEGIN 4 section */
void ModeSelect(void);
void led_bar(void);
uint32_t map(uint32_t x, uint32_t in_min, uint32_t in_max, uint32_t out_min, uint32_t out_max);
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
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM17_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /*
  status = VL53L1_RdByte(dev, 0x010F, &byteData);
  status = VL53L1_RdByte(dev, 0x0110, &byteData);
  status = VL53L1_RdWord(dev, 0x010F, &wordData);


	 while(sensorState==0){

		 status = VL53L1X_BootState(dev, &sensorState);
		 HAL_Delay(2);
	  }

	  status = VL53L1X_SensorInit(dev);
	  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

      HAL_TIM_Base_Start_IT(&htim17);					// Starting Timer Interrupt for Sensor Readings
*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,500);  //PWM frequency set to 2Hz. 1000 value means 50% of ARR. This Timer will blink the LED Motor ON twice per sec

	 ModeSelect();

	 led_bar();

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10707DBC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  htim3.Init.Prescaler = 4000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 8000-1;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 10000-1;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, led_5_Pin|led_7_Pin|led_1_Pin|led_2_Pin
                          |led_3_Pin|led_4_Pin|led_9_Pin|led_6_Pin
                          |led_10_Pin|led_8_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ALARM_Pin|MTR_RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : led_5_Pin led_7_Pin led_1_Pin led_2_Pin
                           led_3_Pin led_4_Pin led_9_Pin led_6_Pin
                           led_10_Pin led_8_Pin */
  GPIO_InitStruct.Pin = led_5_Pin|led_7_Pin|led_1_Pin|led_2_Pin
                          |led_3_Pin|led_4_Pin|led_9_Pin|led_6_Pin
                          |led_10_Pin|led_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIGGER_INPUT_Pin B_INPUT_Pin A_INPUT_Pin FLT_SENSE_Pin */
  GPIO_InitStruct.Pin = TRIGGER_INPUT_Pin|B_INPUT_Pin|A_INPUT_Pin|FLT_SENSE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ALARM_Pin MTR_RELAY_Pin */
  GPIO_InitStruct.Pin = ALARM_Pin|MTR_RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void AutonomousLowPowerRangingTest(void){

	  while (dataReady == 0){

		  status = VL53L1X_CheckForDataReady(dev, &dataReady);

	  }
	  dataReady = 0;

	  status = VL53L1X_GetDistance(dev, &Distance);
	  status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/

	   FIRFilter_Update(&sensor_Read, Distance);
	  sprintf(buff, "%.1f \n\r", sensor_Read.out);
	 HAL_UART_Transmit(&huart2, (uint8_t*)buff, sizeof(buff), 200);

}


void led_bar()
{


	ledLevel = map(Distance, 55, 335, 0, 10);


	  // loop over the LED array:
	  for (thisled = 0; thisled < 10; thisled++) {
	    // if the array element's index is less than ledLevel,
	    // turn the pin for this element on:
	    if (thisled < ledLevel) {

	    	HAL_GPIO_WritePin(GPIOA,(uint16_t)led[thisled], GPIO_PIN_RESET);

	    }
	    // turn off all pins higher than the ledLevel:
	    else {

	    	HAL_GPIO_WritePin(GPIOA,(uint16_t)led[thisled], GPIO_PIN_SET);

	    }

	  }

}


void ModeSelect()
{
		a = HAL_GPIO_ReadPin(GPIOB, A_INPUT_Pin);		// Reads Input for Auto/Manual
		b = HAL_GPIO_ReadPin(GPIOB, B_INPUT_Pin);		// Reads Input for Auto/Manual

		newTrigState = HAL_GPIO_ReadPin(GPIOB, TRIGGER_INPUT_Pin);		// Reads Input from the Trigger Pin

		float_sensor = HAL_GPIO_ReadPin(GPIOB, FLT_SENSE_Pin);		//Reads Input from the Float Sensor



		//sprintf(buff, "%d,%d,%d, %d \n\r",a,b,newTrigState,float_sensor);

		//sprintf(buff, "%d \n\r",float_sensor);
		//HAL_UART_Transmit(&huart2, (uint8_t*)buff, sizeof(buff), 200);
       // HAL_Delay(200);


		 if((a==1) && (b==0)){						// The switch is in Auto Mode. ie the switch position is at =

					  // newTrigState = HAL_GPIO_ReadPin(GPIOA, TRIGGER_INPUT_Pin);

							if(newTrigState == 0){

							//	if (MotorState == 0 && ((Distance >= Threshold_min) || (Distance <= Threshold_min)  || float_sensor!=0)){					//Water is less than Threshold level, then turn ON the Motor
								if (MotorState == 0 && float_sensor!=0){					//Water is less than Threshold level, then turn ON the Motor
									HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
									HAL_GPIO_WritePin(GPIOB,MTR_RELAY_Pin,GPIO_PIN_SET);
									MotorState = 1;
								}

							//} else if(MotorState == 1 && ((Distance <= Threshold_max) || float_sensor==0)){		//Water has reached the Threshold Level, turn OFF the Motor
							  } else if(MotorState == 1 && float_sensor==0){		//Water has reached the Threshold Level, turn OFF the Motor

								HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
							    HAL_GPIO_WritePin(GPIOB,MTR_RELAY_Pin,GPIO_PIN_RESET);
							    HAL_GPIO_WritePin(GPIOB,ALARM_Pin,GPIO_PIN_SET);
							    MotorState = 0;
							  }	else if((MotorState == 0 && newTrigState == 1) || float_sensor==0){				//Water has reached the Threshold Level, turn OFF the Motor

								HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
							    HAL_GPIO_WritePin(GPIOB,MTR_RELAY_Pin,GPIO_PIN_RESET);
							   // HAL_GPIO_WritePin(GPIOB,ALARM_Pin,GPIO_PIN_RESET);
							  }

				   } else if((a==0) && (b==1)){						// The switch is in Manual Mode. When switch symbol is - then we are in manual mode, when switch symbol is = then we are in auto-mode

					   if(newTrigState==0 && prevTrigBtnState==1){    //If the state has changed, increment the counter

						   if(MotorState == 0){        //If the current state of Motor is LOW, then the button went from off to on

							HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);			//  Motor_On Blinky Status
							HAL_GPIO_WritePin(GPIOB,MTR_RELAY_Pin,GPIO_PIN_SET);
							MotorState=1;

							}else{

							HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
							HAL_GPIO_WritePin(GPIOB,MTR_RELAY_Pin,GPIO_PIN_RESET);
							MotorState=0;
							}
						   HAL_Delay(5);

				    }
					   	   prevTrigBtnState = newTrigState;

				   }else {// The switch is in Off/Center Mode
					   MotorState =0;
					   HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
					   HAL_GPIO_WritePin(GPIOB,MTR_RELAY_Pin,GPIO_PIN_RESET);
					   HAL_GPIO_WritePin(GPIOB,ALARM_Pin,GPIO_PIN_RESET);
		           }

}

void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)

{

  if(htim == &htim17){

    // AutonomousLowPowerRangingTest();				//This function or API triggers after every 30s. Update Event handler.

  }

}


uint32_t  map(uint32_t  x, uint32_t  in_min, uint32_t  in_max, uint32_t  out_min, uint32_t  out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
