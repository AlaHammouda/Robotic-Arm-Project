/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "cmsis_os.h"

/* USER CODE BEGIN 0 */

	extern	int x_target;                           // les variables d'odométries
	extern	int y_target;
	extern  int i;

	extern	char	PC_Data[7];  
  extern  char  cmd_Data[12];	
	extern	char	x_tab[3];        
	extern	char	y_tab[3];
  extern	char	x_cmd[4];	
	extern	char	y_cmd[4];	
	extern	char	z_cmd[4];	
	extern  char  state;
  extern  const char  sorting;	
  extern  const char  sleeping;   // 1 step => 0.060944641 ° deg
	extern  const char  tracking;
  extern  void Set_joint_angles(float xt,float yt,float zt);
	extern char color;
	float x_test;
	float y_test;
	float z_test;
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  osSystickHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles TIM1 update interrupt.
*/
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
	      /* HAL_UART_Receive_IT(&huart2,(uint8_t *)&cmd_Data,12); i++;        
	        if(i==12){
           x_cmd[0]=cmd_Data[0];x_cmd[1]=cmd_Data[1];x_cmd[2]=cmd_Data[2];x_cmd[3]=cmd_Data[3];			     
	         y_cmd[0]=cmd_Data[4];y_cmd[1]=cmd_Data[5];y_cmd[2]=cmd_Data[6];y_cmd[3]=cmd_Data[7];			         
	         z_cmd[0]=cmd_Data[8];z_cmd[1]=cmd_Data[9];z_cmd[2]=cmd_Data[10];z_cmd[3]=cmd_Data[11];
	         z_test=atoi(z_cmd); z_cmd[0]=' ';z_cmd[1]=' ';z_cmd[2]=' ';z_cmd[3]=' ';
					 y_test=atoi(y_cmd); y_cmd[0]=' ';y_cmd[1]=' ';y_cmd[2]=' ';y_cmd[3]=' ';
					 x_test=atoi(x_cmd);							
					 i=0;
						x_test+=7;
						if(y_test>89){y_test+=4;}
					 Set_joint_angles(x_test,y_test,z_test);				
					 state=tracking;
					  }*/
					
				HAL_UART_Receive_IT(&huart2,(uint8_t *)&PC_Data,7);  i++;
					
   					x_tab[0]=PC_Data[0];     
						x_tab[1]=PC_Data[1];  
						x_tab[2]=PC_Data[2];    
						y_tab[0]=PC_Data[3];     
						y_tab[1]=PC_Data[4];   
						y_tab[2]=PC_Data[5];					
						if(i==7){		
						if(state==sleeping){
						color=PC_Data[6];		
						y_target=atoi(y_tab)/1000000;y_tab[0]=' ';y_tab[1]=' ';y_tab[2]=' ';	
						x_target=atoi(x_tab);
						y_target=0.4972*y_target-155.623;
						x_target=-0.506*x_target+327.53;
						x_target+=7;
						if(y_target>89){y_target+=4;}					
						Set_joint_angles(x_target,y_target,-146);  
						state=tracking;				
			      }
					i=0;
				}
  /* USER CODE END USART2_IRQn 1 */
  }

/**
* @brief This function handles TIM6 global interrupt and DAC channel underrun error interrupt.
*/
void TIM6_DAC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_IRQn 0 */

  /* USER CODE END TIM6_DAC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  /* USER CODE BEGIN TIM6_DAC_IRQn 1 */

  /* USER CODE END TIM6_DAC_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
