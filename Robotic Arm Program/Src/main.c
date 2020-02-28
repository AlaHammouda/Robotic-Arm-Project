
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

#define PI 								3.141592653
#define JOINT1_RESOLUTION 0.060944641 /*  Joint1 resolution in degree  */
#define JOINT2_RESOLUTION 0.055333713	/*  Joint2 resolution in degree  */
#define JOINT3_RESOLUTION 0.055301333 /*  Joint3 resolution in degree  */   
#define X_INIT       			132.861984	/*  x value of the initial position */
#define Y_INIT        		0.00        /*  y value of the initial position */
#define Z_INIT        		-25.3336658 /*  z value of the initial position */
#define SENS_SWITCH_DELAY	100         /*  Short delay, When switching the sens of the stepper  */
#define MAX_ARM_RANGE     324         /*  The maximum range that the effector can reach   */
#define l2  		  				162.00      /*  Length of the arm 2  */
#define l3   		 					162.00			/*  Length of the arm 3  */
#define l4		  					122.5       /*  Length of the arm 4  */
	
	
typedef struct{                                    
	short x;                           
	short y;
	short z;
	}Color_Area;                          
	
	
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
 	
const char SLEEPING='z';   
const char TRACKING ='t';
const char SORTING='s';
			
float O1=0;        /* Current O1 joint in degree */ 
float O2=-35.2;    /* Current O2 joint in degree */ 
float O3=60.1;     /* Current O3 joint in degree */
float O4=-90;      /* Current O4 joint in degree */
float O1_t=0;      /* O1 of the current target  */
float O2_t=90;		 /* O2 of the current target  */
float O3_t=-35;		 /* O3 of the current target  */
float O4_t=-90;    /* O1 of the current target  */

char state = SLEEPING;     /* Status of the Arm  */
unsigned char Step1_done,Step2_done,Step3_done = 0;	/* Status of each Stepper Motor  */
short sens_1=1; 		/* Current sens of the stepper 1 */
short sens_2=1;			/* Current sens of the stepper 2 */
short sens_3=-1; 		/* Current sens of the stepper 3 */
short  x_target=0;  /* x value of the current target */
short  y_target=0;	/* y value of the current target */

char	PC_Data[7]={0};												        		    
char	x_tab[3]={0};      
char	y_tab[3]={0};

char color ='g';             						/* Color of the current object */                      
Color_Area  Green_Area={150,195,-130};	/* Coordinates of the green area */
Color_Area  Red_Area={250,195,-130};		/* Coordinates of the red area */
Color_Area  Blue_Area={150,-195,-130};	/* Coordinates of the blue area */
Color_Area  Yellow_Area={250,-195,-130};/* Coordinates of the yellow area */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
void Stepper1_Task_function(void const * argument);
void Stepper2_Task_function(void const * argument);
void Stepper3_Task_function(void const * argument);
void Main_Arm_Task_function(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

int Check_Object(void);             /*  Verify if there is an object or no  */
unsigned char Steppers_Ready(void);	/*  Get the state of all Stepper Motors  */ 
/*  Set(O1_t,O2_t,O3_t,O4_t) of the new target */
void Set_Joint_Angles(float xt,float yt,float zt);     

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

  /* definition and creation of Stepper1_Task */
  osThreadDef(Stepper1_Task, Stepper1_Task_function, osPriorityNormal, 0, 128);
  Stepper1_TaskHandle = osThreadCreate(osThread(Stepper1_Task), NULL);

  /* definition and creation of Stepper2_Task */
  osThreadDef(Stepper2_Task, Stepper2_Task_function,osPriorityNormal, 0, 128);
  Stepper2_TaskHandle = osThreadCreate(osThread(Stepper2_Task), NULL);

  /* definition and creation of Stepper3_Task */
  osThreadDef(Stepper3_Task, Stepper3_Task_function, osPriorityNormal, 0, 128);
  Stepper3_TaskHandle = osThreadCreate(osThread(Stepper3_Task), NULL);


  /* definition and creation of Main_Arm_Task */
  osThreadDef(Main_Arm_Task, Main_Arm_Task_function, osPriorityNormal, 0, 128);
  Main_Arm_TaskHandle = osThreadCreate(osThread(Main_Arm_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

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

  /*Configure GPIO pins : LD2_Pin PA8 PA9 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC9 PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

int sign(float x){
	return (x > 0) - (x < 0);
}
	
   /* Convert radians to degrees */
float rad_to_deg(double rad){                       
				return(rad*360/(2*PI));
}

   /* 1 <=> Object detected    0 <=> No Object detected  */
int Check_Object(){	
	int  adc=0;
  for(int i=0;i<100;i++){		
    	HAL_ADC_Start(&hadc1);
		  adc+=HAL_ADC_GetValue(&hadc1);	
		  }
		 adc/=100;
		  return (adc<100);
}

		/*  Set(O1_t,O2_t,O3_t,O4_t) of the new target */
void  Set_Joint_Angles(float xt,float yt , float zt){		
	 	
	float a=sqrt(pow(xt,2)+pow(yt,2));
	float sphere_radius=sqrt(a*a+pow((zt+l4),2));
	float alpha=(a*a+pow((zt+l4),2)+pow(l3,2)-pow(l2,2))/(2*l3);                                                           
	float beta=atan((zt+l4)/a);

	if(sphere_radius<=MAX_ARM_RANGE){ /* Verifying that the new target is in the range of the arm  */
	
		if(xt==0) O1_t=PI/2*sign(yt); else  O1_t=atan(yt/xt);   
		O2_t=acos(alpha/sphere_radius)+beta;
		O3_t=acos((a-l3*cos(O2_t))/l2)*sign(zt);
			
		O1_t=rad_to_deg(O1_t);  /* Converting angles from radians to degrees  */ 
		O2_t=rad_to_deg(O2_t);
		O3_t=rad_to_deg(O3_t);
		O4_t=-90.00;
		
		Step1_done=0; Step2_done=0; Step3_done=0; /* Initializing steppers state to 0  */
	}	
}

		 /*  Get the state of all Stepper Motors  */ 
unsigned char Steppers_Ready(){
	return(Step1_done && Step2_done && Step3_done); 
}

/* USER CODE END 4 */


/* Stepper1_Task_function function */
void Stepper1_Task_function(void const * argument)
{
  /* USER CODE BEGIN Stepper1_Task_function */
  /* Infinite loop */
  for(;;)
  {	
		if(fabs(O1_t-O1)>(JOINT1_RESOLUTION+0.01)){		                      
				if((O1<O1_t)&&(sens_1==-1)){osDelay(SENS_SWITCH_DELAY); sens_1= 1; }  /*  Short delay while reversing    */
				if((O1>O1_t)&&(sens_1==1)){osDelay(SENS_SWITCH_DELAY);  sens_1= -1;}	/*	the sens of the stepper motor */
				
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,sens_1-1); /*  Set the direction of the stepper */  
			
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,1); /* Sending one pulse command to the stepper motor <=> 1 step */
				osDelay(1);                                  
				HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,0);
				osDelay(1);
	  
				O1+=(JOINT1_RESOLUTION*sens_1); /* Increment or Decrement the joint angle with each step */
     }
		else{
			Step1_done=1; /* Set the stepper state to 1 */
		  osDelay(1);
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
    	if(fabs(O2_t-O2)>(JOINT2_RESOLUTION+0.01)){		
					if((O2>O2_t)&&(sens_2==1)){	 osDelay(SENS_SWITCH_DELAY); sens_2= -1;} /*  Short delay while reversing  */
					if((O2<O2_t)&&(sens_2==-1)){ osDelay(SENS_SWITCH_DELAY);  sens_2= 1;}	/*	the sens of the stepper motor */

					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,sens_2-1);/*  Set the direction of the stepper */  
					
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,1); /* Sending one pulse command to the stepper motor <=> 1 step */
					osDelay(1);    
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,0);
					osDelay(1);
					
					O2+=(JOINT2_RESOLUTION*sens_2); /* Increment or Decrement the joint angle with each step */
     }
		else{
			Step2_done=1;	/* Set the stepper state to 1 */
			osDelay(1);
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
    	if(fabs(O3_t-O3)>(JOINT3_RESOLUTION+0.01)){		
					if((O3<O3_t)&&(sens_3==-1)){osDelay(SENS_SWITCH_DELAY); sens_3= 1;} /*  Short delay while reversing  */
					if((O3>O3_t)&&(sens_3==1)){osDelay(SENS_SWITCH_DELAY);  sens_3=-1;}	/*	the sens of the stepper motor */
					
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,sens_3+1); /*  Set the direction of the stepper */  

					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,1); /* Sending one pulse command to the stepper motor <=> 1 step */
					osDelay(1);   
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_6,0);
					osDelay(1);  
					
					O3+=(JOINT3_RESOLUTION*sens_3); /* Increment or Decrement the joint angle with each step */	 
     }
		else{
			Step3_done=1;	/* Set the stepper state to 1 */
	    osDelay(1);
		}
  }
  /* USER CODE END Stepper3_Task_function */
}

/* Main_Arm_Task_function function */
void Main_Arm_Task_function(void const * argument)
{
  /* Infinite loop */
  for(;;)
  {
	   if(Steppers_Ready() && state==TRACKING ){		
			 if(Check_Object()){  /* Object detected */
				 
					state=SORTING;
			  	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,1);  /* pump ON */  				 
					Set_Joint_Angles(x_target,y_target,-178);  /* Get object */ 
					while(!Steppers_Ready());		 
					Set_Joint_Angles(x_target,y_target,-116);	 /* Pick object up  */
					while(!Steppers_Ready()); 	     			 
					switch (color){   /* Place each object in the appropriate color area  */
						case 'g':Set_Joint_Angles(Green_Area.x,Green_Area.y,Green_Area.z);	  Green_Area.z+=25;	  break;    
						case 'r':Set_Joint_Angles(Red_Area.x,Red_Area.y,Red_Area.z);					Red_Area.z+=25;	 		break;
						case 'b':Set_Joint_Angles(Blue_Area.x,Blue_Area.y,Blue_Area.z);				Blue_Area.z+=25; 		break;
						case 'y':Set_Joint_Angles(Yellow_Area.x,Yellow_Area.y,Yellow_Area.z); Yellow_Area.z+=25;  break;
						default : break;
				 }     			 
				}
				else{  /* No object detected */
					state=SLEEPING;  
					Set_Joint_Angles(X_INIT,Y_INIT,Z_INIT); /* Set the Arm to the initial position */	
				}
			}
			if(Steppers_Ready() && state==SORTING ){
					HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,0); /* pump OFF */  
					while(Check_Object());                      
					Set_Joint_Angles(X_INIT,Y_INIT,Z_INIT); /* Set the Arm to the initial position */	
					state=SLEEPING;
			}
		osDelay(1);
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
