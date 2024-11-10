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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUF_LEN 16

//SWITCH Reading
#define ReadSW1 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)
#define ReadSW2 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11)
#define ReadBlueSW HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)

//ENCODER Reading
#define ReadENA1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)
#define ReadENB1 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)
#define ReadENA2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)
#define ReadENB2 HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
int norm_ss[8];
float position =0;
unsigned char text1[100];
int button_count =0;
int isStartButton = 0;
int isPress = 0;
uint32_t ss_min[8] = {4095,4095,4095,4095,4095,4095,4095,4095};
uint32_t ss_max[8] = {0,0,0,0,0,0,0,0};
uint32_t sensorConst[8];
float value, avg, sum ,online, lastVal = 0;
float error =0, lastError = 0, IError = 0;
float baseSpeed = 600, initSpeed = 400, nowSpeed;
float  P = 51.455, D = 120.867, I =  5.091* 1e-5;
float P2 = 53.42, D2 = 69.66, I2 = 7.66*1e-5;
float L,R;
int count=0;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t sensorArray[ADC_BUF_LEN];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void setLED(int a){
	if(a == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
	}else if(a == 2){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
	}else if(a == 3){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
	}
	else if(a == 4){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, SET);
	}
	else if(a == 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);
	}
}
void getVal(){
	for(int i=0;i<8;i++){
		//norm_ss[i]=1-(((float)adc_current[i]-(float)sensors_min[i])/((float)sensors_max[i]-(float)sensors_min[i]));
//		norm_ss[i]=1500 - map(sensorArray[i], ss_min[i], ss_max[i], -100, 1500);
//		norm_ss[i] = sensorArray[i];
		}
		sprintf(&text1,"%d, %d, %d, %d, %d, %d, %d, %d \r\n",norm_ss[0],norm_ss[1],norm_ss[2],norm_ss[3],norm_ss[4],norm_ss[5],norm_ss[6],norm_ss[7]);
		HAL_UART_Transmit(&huart2,text1,strlen(text1),1000);
}

void calibrate(){
	setLED(4);
	for(int i=0;i<8;i++){
		if(ss_max[i]<sensorArray[i])ss_max[i]=sensorArray[i];
		if(ss_min[i]>sensorArray[i])ss_min[i]=sensorArray[i];
		sensorConst[i] = 1600/(ss_max[i]-ss_min[i]);
	}
	HAL_Delay(75);
	setLED(0);
	HAL_Delay(75);
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void PID(){
	avg=0;
	sum =0;
	online=0;
	nowSpeed = baseSpeed;
	for(int i=0;i<8;i++){
		norm_ss[i]=12000 - map(sensorArray[i], ss_min[i], ss_max[i], -1000, 12000);

	  	if(norm_ss[i] > 10000) norm_ss[i] = 10000;
	  	else if(norm_ss[i] < 0) norm_ss[i] = 0;

	  	if(norm_ss[i] > 2000)online =1;
	  	if(norm_ss[i]>500){
	  		avg += norm_ss[i]*10000*i;
	  		sum += norm_ss[i];
	  	}

	  	if(!online){
	  		if(lastVal < 35000)lastVal = 0;
	  		else lastVal = 70000;
	  	} else{
	  		lastVal = avg/sum;
	  	}
	  	error = (lastVal - 35000)/1000;
	  	if(error > 300)nowSpeed=initSpeed;
	  	IError += error;
//	  	if(IError > 2000)IError = 2000;
	  	float powerM = (P*error)+ (D*(error - lastError))+ (I*IError);
	  	lastError = error;
	  	L = nowSpeed - powerM;
	  	R = nowSpeed + powerM;
	  	if(L > 1000)L = 1000;
	  	else if(L<-1000)L = -1000;
	  	if(R>1000)R = 1000;
	  	else if(R<-1000) R = -1000;
	}

}

void PID2(){
	avg=0;
	sum =0;
	online=0;
	count =0;
	for(int i=0;i<8;i++){
		norm_ss[i]=12000 - map(sensorArray[i], ss_min[i], ss_max[i], -1000, 12000);

	  	if(norm_ss[i] >= 10000){
	  		norm_ss[i] = 10000;
	  		count++;
	  	}
	  	else if(norm_ss[i] < 0) norm_ss[i] = 0;

	  	if(norm_ss[i] > 2000)online =1;
	  	if(norm_ss[i]>500){
	  		avg += norm_ss[i]*10000*i;
	  		sum += norm_ss[i];
	  	}

	  	if(!online){
	  		  		lastVal = lastVal;
	  	}else{
	  		lastVal = avg/sum;
	  	}
	  	error = (lastVal - 35000)/1000;
	  	if(error > 300)nowSpeed=initSpeed;
	  	IError += error;
//	  	if(IError > 2000)IError = 2000;
	  	float powerM = (P2*error)+ (D2*(error - lastError))+ (I/2*IError);
	  	lastError = error;
	  	L = initSpeed - powerM;
	  	R = initSpeed + powerM;
	  	if(L > 1000)L = 1000;
	  	else if(L<-1000)L = -1000;
	  	if(R>1000)R = 1000;
	  	else if(R<-1000) R = -1000;
	}

}

void motor(){
	L = L*624/1000;
	R = R*624/1000;
	if (L < 0){
		 TIM3->CCR1 = -L;
		 TIM3->CCR2 = 0;
	}else{
		 TIM3->CCR1 = 0;
		 TIM3->CCR2 = L;
	}

	if (R<0){
		 TIM3->CCR3 = 0;
		 TIM3->CCR4 = -R;
	}else{
		 TIM3->CCR3 = R;
		 TIM3->CCR4 = 0;
	}
	if(L >0 && R <-500){
		 TIM3->CCR1 = 0;
		 TIM3->CCR2 = 1.095*L;
		 TIM3->CCR3 = 0;
		 TIM3->CCR4 = -1.095*R;
		 HAL_Delay(20);
	}
	if(L <-500 && R >0){
		 TIM3->CCR1 = -1.095*L;
		 TIM3->CCR2 = 0;
		 TIM3->CCR3 = 1.095*R;
		 TIM3->CCR4 = 0;
		 HAL_Delay(20);
	}
}

void motor2(){
	L = L*624/1000;
	R = R*624/1000;
	if (L < 0){
		 TIM3->CCR1 = -L;
		 TIM3->CCR2 = 0;
	}else{
		 TIM3->CCR1 = 0;
		 TIM3->CCR2 = L;
	}

	if (R<0){
		 TIM3->CCR3 = 0;
		 TIM3->CCR4 = -R;
	}else{
		 TIM3->CCR3 = R;
		 TIM3->CCR4 = 0;
	}
	if(L >0 && R <-500){
		 TIM3->CCR1 = 0;
		 TIM3->CCR2 = 1.5*L;
		 TIM3->CCR3 = 0;
		 TIM3->CCR4 = -1.5*R;
//		 HAL_Delay(2);

	}
	if(L <-500 && R >0){
		 TIM3->CCR1 = -1.5*L;
		 TIM3->CCR2 = 0;
		 TIM3->CCR3 = 1.4*R;
		 TIM3->CCR4 = 0;
//		 HAL_Delay(2);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //A in 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); //B in 1
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); //A in 2
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4); //B in 2
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*)sensorArray,ADC_BUF_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
   while (1)
   {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	   isPress = 0;
		 while(!ReadBlueSW){
			 if(isPress == 0){
				 if(isStartButton == 0)isStartButton = 1;
				 else isStartButton =0;
				 isPress = 1;
			 }
			 HAL_Delay(30);

		 }
	   while(!ReadSW1){
		   if(isPress == 0){
			   button_count += 1 ;
			   isPress = 1;
			   isStartButton = 0;
		   }
		   HAL_Delay(30);
	   	}

	   	if(button_count > 3)button_count = 0;
	   	switch(button_count){
	   		case 0:
	   			setLED(0);
	   			TIM3->CCR1 = 0;
	   			TIM3->CCR2 = 0;
	   			TIM3->CCR3 = 0;
	   			TIM3->CCR4 = 0;
	   			break;
	   	 	case 1:
	   	 		setLED(1);
	   	 		if(isStartButton){
	   	 			setLED(4);
	   	 			calibrate();
	   	 		}
	   	 		getVal();
	   			break;
	   	 	 case 2:
	   		 	setLED(2);
	   		 	if(isStartButton){
	   		 		setLED(4);
	   		 		PID();
	   		 		motor();
	   		 	}else{
		   			TIM3->CCR1 = 0;
		   			TIM3->CCR2 = 0;
		   			TIM3->CCR3 = 0;
		   			TIM3->CCR4 = 0;
	   		 	}
	   	 		break;
	   	 	 case 3:
	   	 		setLED(3);
	   	 		if (isStartButton) {
	   	 			setLED(4);
	   	 			PID2();
	   	 			motor2();
	   	 		} else {
	   	 			TIM3->CCR1 = 0;
	   	 			TIM3->CCR2 = 0;
	   	 			TIM3->CCR3 = 0;
	   	 			TIM3->CCR4 = 0;
	   	 		}
	   	 		break;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 624;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA10 PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
