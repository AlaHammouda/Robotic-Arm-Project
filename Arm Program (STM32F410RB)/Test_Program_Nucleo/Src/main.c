
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
	
	#include "math.h"
	#include "string.h"
	#include <stdlib.h>
	#include <stdio.h>	
	

	
	// step1 360 °    ->  5907-steps  done !!!
  // step2 194 ° ->  3506-steps          .....    55 
  // step3 227.15 ° ->  4177-steps  done !!!
  // step4 360 °    ->  6490-steps presque done ? !!!

	
	
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
osThreadId Stepper1_TaskHandle;
osThreadId Stepper2_TaskHandle;
osThreadId Stepper3_TaskHandle;
osThreadId Stepper4_TaskHandle;
osThreadId Main_Arm_TaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
   
	
  #define step1_resolution    0.060944641    // 1 step => 0.060944641 ° deg
	#define step2_resolution    0.055333713
	#define step3_resolution    0.054381134
	#define step4_resolution    0.055469953
	#define x_init         0.00
	#define y_init         0.00
	#define z_init         0.00
	#define x_def_tank     5000.00
	#define y_def_tank   	 6500.00
	#define z_def_tank   	 300.00
	#define O0_init     	 0.00
	#define O1_init  			 0.00
	#define O2_init 			 0.00
	#define O3_init  			 0.00
	
	#define l1  		  162.00
	#define l2   		  162.00
	#define l3		    80.00
	
	
	const char sleeping='a';    // 1 step => 0.060944641 ° deg
	const char tracking ='b';
	const char extracting='c';
	
	double O0,O1,O2,O3=0;              // current angles 
	double O0_t,O1_t,O2_t,O3_t=0;     // target angles
	float x,y,z=0;                  // effector coordinates 
	char state = sleeping;
	int Step1_done,Step2_done,Step3_done,Step4_done = 0;
	
	int sens_1=1; 
  int sens_2=1;	
	int sens_3=1; 
  int sens_4=1;
	
	
	int adc=0;
  int x_Defected=0;                         
	int y_Defected=0;
	int i=0; int nb=0;
	char	PC_Data[6];        
	char	x_tab[3]={0};      
	char	y_tab[3]={0};
	               
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void const * argument);
void Stepper1_Task_function(void const * argument);
void Stepper2_Task_function(void const * argument);
void Stepper3_Task_function(void const * argument);
void Stepper4_Task_function(void const * argument);
void Main_Arm_Task_function(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

		
#ifdef __GNUC__
   #define  PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else  
   #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f) 
#endif 
	 
	 

void step_1(int dist,int sens);
void step_2(int dist,int sens);	 
void step_3(int dist,int sens);
void step_4(int dist,int sens);
int check_Defected(void);	 
void Set_joint_angles(float xt,float yt,float zt);

	 	 
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

	 PUTCHAR_PROTOTYPE{
		 
	HAL_UART_Transmit(&huart2,(uint8_t *)&ch,1,0xFFFF);
   return ch;
	 }
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM6_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
 	__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
 
  //step_3(2*5907,0);

	
/*
step_2(994,1);//76
HAL_Delay(2000);
step_2(2512,1);//76
HAL_Delay(2000);
step_2(2512,0);//76 
HAL_Delay(2000);
step_2(994,0);//76 
HAL_Delay(2000);
*/

/*
step_3(2467,1);//76
HAL_Delay(2000);
step_3(1710,1);
HAL_Delay(2000);
step_3(1710,0);
HAL_Delay(2000);
step_3(2467,0);
HAL_Delay(2000);
*/
/*
 step_1(1477,0);
HAL_Delay(1000);
 step_1(1477,0); // step1 360 °
HAL_Delay(1000);
 step_1(1477,0);
HAL_Delay(1000);
 step_1(1476,0); // step1 360 °
HAL_Delay(3000);
 step_1(5907,1); // step1 360 °
*/
/*
 step_2(1200,1);205.38
 step_1(1800,0);
 step_2(600,0);
 step_3(1300,1);
 step_4(800,1);
 */
 
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Stepper1_Task */
  osThreadDef(Stepper1_Task, Stepper1_Task_function, osPriorityNormal, 0, 128);
  Stepper1_TaskHandle = osThreadCreate(osThread(Stepper1_Task), NULL);

  /* definition and creation of Stepper2_Task */
  osThreadDef(Stepper2_Task, Stepper2_Task_function, osPriorityNormal, 0, 128);
  Stepper2_TaskHandle = osThreadCreate(osThread(Stepper2_Task), NULL);

  /* definition and creation of Stepper3_Task */
  osThreadDef(Stepper3_Task, Stepper3_Task_function, osPriorityIdle, 0, 128);
  Stepper3_TaskHandle = osThreadCreate(osThread(Stepper3_Task), NULL);

  /* definition and creation of Stepper4_Task */
  osThreadDef(Stepper4_Task, Stepper4_Task_function, osPriorityIdle, 0, 128);
  Stepper4_TaskHandle = osThreadCreate(osThread(Stepper4_Task), NULL);

  /* definition and creation of Main_Arm_Task */
  osThreadDef(Main_Arm_Task, Main_Arm_Task_function, osPriorityNormal, 0, 128);
  //Main_Arm_TaskHandle = osThreadCreate(osThread(Main_Arm_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 
O0_t=180;
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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM6 init function */
static void MX_TIM6_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 2;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA6 PA8 PA9 
                           PA10 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_6|GPIO_PIN_8|GPIO_PIN_9 
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

		
	void step_1(int dist,int sens){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,sens);
		for(int i=0;i<dist;i++){
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);
     osDelay(1);
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);
     osDelay(1);
		}
	}
	
		
		void step_2(int dist,int sens){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,sens);
		for(int i=0;i<dist;i++){
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
    osDelay(1);
     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
     osDelay(1);
		}
	}
		
		void step_3(int dist,int sens){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,sens);
		for(int i=0;i<dist;i++){
		 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1);
     osDelay(1);
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0);
    osDelay(1);
		}
	}

void step_4(int dist,int sens){
		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,sens);
		for(int i=0;i<dist;i++){
		 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
     osDelay(1);
     HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
     osDelay(1);
		}
	}	

	
int check_Defected(){	
			HAL_ADC_Start(&hadc1);
		  adc=HAL_ADC_GetValue(&hadc1);	
      return (adc<100);	
		}

		
void  Set_joint_angles(float xt,float yt , float zt){	
	
	// project Module      // !!!!!!!!!!!!!! under test ( rad to deg !!! singularitires !!!! .. !!!!!!!!!!)
	
	float a=sqrt(xt*xt+yt*yt);
	float b=zt+l3;
	float c=(pow(l2,2)-pow(l1,2)-pow(a,2)-pow(b,2))/(-2*l1);
	
	if(xt==0) O0_t=90; else O0_t=atan(yt/xt);
	O1_t=2*atan((b-sqrt(pow(a,2)+pow(b,2)-pow(c,2)))/(a+c));
	O2_t=acos(x/l2*cos(O0_t)-l1/l2*cos(O1_t));
	O3_t=-90.0;	
	
	Step1_done=0; Step2_done=0; Step3_done=0; Step4_done=0; 
	
}	
		
void Get_Defected(){
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,1);     // pump ON
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1);    
		for(int i=0;i<100;i++){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);osDelay(1);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);osDelay(1);
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0);
		for(int i=0;i<100;i++){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);osDelay(1);
      HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);osDelay(1);
		}
}



/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1); // 500 us
  }
  /* USER CODE END 5 */ 
}

/* Stepper1_Task_function function */
void Stepper1_Task_function(void const * argument)
{
  /* USER CODE BEGIN Stepper1_Task_function */
  /* Infinite loop */
  for(;;)
  {	 
		if(abs(O0_t-O0)>0.1){		
			if((O0<O0_t)&&(sens_1==-1)){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,1); osDelay(1500); sens_1= 1; }
			if((O0>O0_t)&&(sens_1==1)){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,0); osDelay(1500);  sens_1= -1;}		
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1);
			osDelay(1);   // equal to 500 micro-second !!!
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);
			osDelay(1);
			nb++;
	    O0+=(step1_resolution*sens_1);     		 
     }
		else{
			Step1_done=1;
		 }
			
	}
  /* USER CODE END Stepper1_Task_function */
}

/* Stepper2_Task_function function */
void Stepper2_Task_function(void const * argument)
{
  /* USER CODE BEGIN Stepper2_Task_function */
  /* Infinite loop */
  for(;;)
  {
    	if(abs(O1_t-O1)>0.1){		
			if((O1<O1_t)&&(sens_1==-1)){	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,1); osDelay(1500); sens_2= 1; }
			if((O1>O1_t)&&(sens_1==1)){		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,0); osDelay(1500);  sens_2= -1;}		
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1);
			osDelay(1);   // equal to 500 micro-second !!!
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
			osDelay(1);
	    O1+=(step2_resolution*sens_2);     		 
     }
		else{
			Step2_done=1;
		 }
  }
  /* USER CODE END Stepper2_Task_function */
}

/* Stepper3_Task_function function */
void Stepper3_Task_function(void const * argument)
{
  /* USER CODE BEGIN Stepper3_Task_function */
  /* Infinite loop */
  for(;;)
  {
    	if(abs(O2_t-O2)>0.1){		
			if((O2<O2_t)&&(sens_3==-1)){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,1); osDelay(1500); sens_3= 1; }
			if((O2>O2_t)&&(sens_3==1)){HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0); osDelay(1500);  sens_3= -1;}		
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1);
			osDelay(1);   // equal to 500 micro-second !!!
			 HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0);
			osDelay(1);
	    O2+=(step3_resolution*sens_3);     		 
     }
		else{
			Step3_done=1;
		 }
  }
  /* USER CODE END Stepper3_Task_function */
}

/* Stepper4_Task_function function */
void Stepper4_Task_function(void const * argument)
{
  /* USER CODE BEGIN Stepper4_Task_function */
  /* Infinite loop */
  for(;;)
  {
    	if(abs(O3_t-O3)>0.1){		
			if((O3<O3_t)&&(sens_4==-1)){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,1);osDelay(1500); sens_4= 1; }
			if((O3>O3_t)&&(sens_4==1)){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,0); osDelay(1500);  sens_4= -1;}		
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1);
			osDelay(1);   // equal to 500 micro-second !!!
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0);
			osDelay(1);
	    O3+=(step4_resolution*sens_4);     		 
     }
		else{
			Step4_done=1;
		 }
  }
  /* USER CODE END Stepper4_Task_function */
}

/* Main_Arm_Task_function function */
void Main_Arm_Task_function(void const * argument)
{
  /* USER CODE BEGIN Main_Arm_Task_function */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
               //Project modal   
    x=(l1*cos(O1)+l2*cos(O2))*cos(O0);     // we not need them ??
		y=(l1*cos(O1)+l2*cos(O2))*sin(O0);
		z=l1*sin(O1)+l2*sin(O2)-l3;
		
		if(Step1_done && Step2_done && Step3_done && Step4_done && state==tracking )
		{
			if(check_Defected()){
				state=extracting;
				Get_Defected();
				Set_joint_angles(x_def_tank,y_def_tank,z_def_tank);			
			}
			else{
				Set_joint_angles(x_init,y_init,z_init); state=sleeping;			
			}
		}
		
		if(Step1_done && Step2_done && Step3_done && Step4_done && state==extracting )
		{		
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0);   // pump OFF  
			Set_joint_angles(x_init,y_init,z_init); state=sleeping;	
		}
		
  }
  /* USER CODE END Main_Arm_Task_function */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
  while(1)
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
