/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ssd1306.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define ms_converter 	17  //at timer pr =439 : 1.8ms ==32 step => 1ms ==17 step
#define time_out 	ms_converter*10  //10ms at timer pr =439
#define EEPROM_I2C &hi2c1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM14_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */
bool flg = false;
int  bit_cntr=0;
uint8_t signal_state=0;
uint8_t byte_cntr=0,temporary_data=0;
uint8_t data_repository[255];
uint8_t data_repository_to_binary[255];
uint8_t data_display[40];
bool tx_flag=0;
uint8_t oled_convert_data[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
uint8_t oled_convert_data_decimal[10]={'0','1','2','3','4','5','6','7','8','9'};
uint8_t bzgtrn=0;//most biggest data
uint8_t riztrn=0;//most smalling :) data
uint8_t data_bit_lenght=0;//data len per bits
uint8_t average=0;//average betwen riz and big
uint8_t tedad=0;
//////////////////////////////////////////////////////////////////menu
#define start_up_layer 0
#define home_layer 1
#define save_layer 2
#define memory_layer 3
#define search_layer 4
#define save_notif_layer 5

#define AT24ADDRESS 0xA0
struct menu
{
	uint8_t layer;
};
struct menu my_menu;
/////////////////////////////////////////////////////////////////save menu
uint8_t name_monitor[30]={'A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z'};

struct at24c256
{
	uint8_t	read_data[32];
	uint8_t saver_buff[32];
	uint8_t last_add[2];//0...0x7fff
}	;
struct at24c256 at24;
	
struct name_menu
{
	uint8_t name_buff_index;
	uint8_t name_x_loc;
	uint8_t name_y_loc;
	uint8_t name_buf[20];
};
struct name_menu my_name;
	
//struct current_key_info
//{
//	uint8_t tedad[2];
//	uint8_t data[10];
//	uint8_t name_buf[20];
//};
//struct current_key_info informaition;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void lcd_manager (uint8_t lcd_status);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{//HAL_UART_Transmit(&huart1,(uint8_t *)&signal_state,1,100);
	//HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
	if(GPIO_Pin==GPIO_PIN_5 && tx_flag==0 && my_menu.layer==home_layer)
	{
		switch (signal_state)
		{
/*-------------------------------------------------------------------------*/
			case 0 :			//waiting for preamble
				
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0)//falling edge
				{
					
					signal_state=1;
					__HAL_TIM_SET_COUNTER(&htim14,0);
					HAL_TIM_Base_Start(&htim14);
				}
				break;

/*-------------------------------------------------------------------------*/
			case 1 :		//first preamble cheking
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)//rising edge
				{
					if(__HAL_TIM_GET_COUNTER(&htim14)>3*ms_converter)//3 ms(at timer_pr=439 : 1.8ms == 32 step)
					{
						
						signal_state=2;
						HAL_TIM_Base_Stop(&htim14);
						__HAL_TIM_SET_COUNTER(&htim14,0);
					}
					else
					{	signal_state--; }
						
				}
				else
				{	signal_state--; }
				break;
/*-------------------------------------------------------------------------*/				
			case 2 :
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0)//falling edge
				{
					
					signal_state=3;
				}
				else
				{
					signal_state=0;
				}
				break;
/*-------------------------------------------------------------------------*/
			case 3 :////data process
				
				if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==1)//rising edge
				{
					__HAL_TIM_SET_COUNTER(&htim14,0);
					HAL_TIM_Base_Start(&htim14);
				}
				else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)==0)//falling edge
				{
				//	if(__HAL_TIM_GET_COUNTER(&htim14)>=10)//wide
					//{
						data_repository[bit_cntr]=__HAL_TIM_GET_COUNTER(&htim14);
						HAL_TIM_Base_Stop(&htim14);
					//}
					//else
					//{
						//data_repository[bit_cntr]=0;
						
				//}
					bit_cntr++;
					data_bit_lenght=bit_cntr;
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
				}
				
				if(__HAL_TIM_GET_COUNTER(&htim14)>=5*ms_converter || bit_cntr>=255)//__HAL_TIM_GET_COUNTER(&htim14)>=5*ms_converter || 
				{
					HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
					bit_cntr=0;
					byte_cntr=0;
					signal_state=0;
					tx_flag=1;
					HAL_TIM_Base_Stop(&htim14);
					HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
				}
				break;
				
/*-------------------------------------------------------------------------*/
			default:
				break;
		}	
	}
	//************************************************************************* key pad
	else if(GPIO_Pin==GPIO_PIN_0)//1 6 7 0 //1:right
	{

	//	HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
		if(my_menu.layer==home_layer && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1) //save key pressed
		{
			my_menu.layer=save_layer;
			lcd_manager(save_layer);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
		}
		else if(my_menu.layer==save_layer)
		{
		if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==0 && my_menu.layer==save_layer)
		{
			__HAL_TIM_SET_COUNTER(&htim16,0);
			HAL_TIM_Base_Start(&htim16);
		}
		else if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1 && my_menu.layer==save_layer)
		{
			HAL_TIM_Base_Stop(&htim16);
			if(__HAL_TIM_GET_COUNTER(&htim16)<=400)
			{

				 if(my_menu.layer==save_layer && my_name.name_x_loc<10)
				{
					my_name.name_buf[my_name.name_x_loc]=name_monitor[my_name.name_buff_index];
					my_name.name_x_loc++;
					lcd_manager(save_layer);
				}
			}
			else//long holding
			{
				///saving to at24
			//	HAL_I2C_Mem_Write(&hi2c1,0xA0,(3<<6),2,write_data,20,1000);
				//memset(at24.saver_buff,0,32);
			//	HAL_I2C_mem
				memcpy(at24.saver_buff+2,data_display,4);
				memcpy(at24.saver_buff+12,my_name.name_buf,my_name.name_x_loc);//my_name.name_buff_index
				
				//HAL_UART_Transmit(&huart1,(uint8_t *)&at24.saver_buff,32,100);
				//HAL_I2C_Mem_Write(EEPROM_I2C,AT24ADDRESS,0*32,2,at24.saver_buff,32,1000);
				HAL_I2C_Mem_Write(EEPROM_I2C,AT24ADDRESS,0*32,2,at24.saver_buff,32,1000);
				lcd_manager(save_notif_layer);
			
			}
	
		}
	}
		else if(my_menu.layer==start_up_layer && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0)==1)
		{
			my_menu.layer=home_layer;
			lcd_manager(home_layer);
		}
		__HAL_TIM_SET_COUNTER(&htim16,0);
	}
	//********************************************************************
	else if(GPIO_Pin==GPIO_PIN_6)//1 6 7 0///2 up
	{
		
		if(my_menu.layer==save_layer) //up
		{
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
			if(my_name.name_buff_index<25)
			my_name.name_buff_index++;
			else 
				my_name.name_buff_index=0;
			lcd_manager(save_layer);
		}
			else if(my_menu.layer==start_up_layer)
		{
			my_menu.layer=home_layer;
			lcd_manager(home_layer);
		}
	//	
	}
	//***************************************************************
	else	if(GPIO_Pin==GPIO_PIN_1)//1 6 7 0///4 left
	{
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
		if(my_menu.layer==save_layer && my_name.name_x_loc>0)
				{
					//my_name.name_buf[my_name.name_x_loc]=name_monitor[my_name.name_buff_index];
					my_name.name_x_loc--;
					lcd_manager(save_layer);
				}
			else if(my_menu.layer==start_up_layer)
		{
			my_menu.layer=home_layer;
			lcd_manager(home_layer);
		}
	}
	//*********************************************************
	
	else	if(GPIO_Pin==GPIO_PIN_7)//1 6 7 0///3 down
	{
		if(my_menu.layer==save_layer) //down
		{
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
			if(my_name.name_buff_index>0)
			my_name.name_buff_index--;
			else
				my_name.name_buff_index=25;
			lcd_manager(save_layer);
		}
		//
				else if(my_menu.layer==start_up_layer)
		{
			my_menu.layer=home_layer;
			lcd_manager(home_layer);
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
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
		ssd1306_Init();
		//HAL_UART_Transmit(&huart1,(uint8_t *)&"hilooooo",32,100);
	
		//HAL_Delay(1000);
		HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_4);
		my_menu.layer=start_up_layer;
		lcd_manager(my_menu.layer);
		//HAL_I2C_Mem_Write(&hi2c1,AT24ADDRESS,1,2,data_display,20,1000);
						HAL_I2C_Mem_Read(EEPROM_I2C, AT24ADDRESS, 0*32, 2, at24.read_data, 32, 1000);
		HAL_UART_Transmit(&huart1,(uint8_t *)&at24.read_data,32,100);
		
				HAL_I2C_Mem_Read(EEPROM_I2C, AT24ADDRESS, 1*32, 2, at24.read_data, 32, 1000);
		HAL_UART_Transmit(&huart1,(uint8_t *)&at24.read_data,32,100);
		
				HAL_I2C_Mem_Read(EEPROM_I2C, AT24ADDRESS, 2*32, 2, at24.read_data, 32, 1000);
		HAL_UART_Transmit(&huart1,(uint8_t *)&at24.read_data,32,100);
		
						HAL_I2C_Mem_Read(EEPROM_I2C, AT24ADDRESS, 3*32, 2, at24.read_data, 32, 1000);
		HAL_UART_Transmit(&huart1,(uint8_t *)&at24.read_data,32,100);
  while (1)
  {
		if(__HAL_TIM_GET_COUNTER(&htim14)>=15*ms_converter && signal_state==3){
			HAL_NVIC_DisableIRQ(EXTI4_15_IRQn);
			bit_cntr=0;
			byte_cntr=0;
			signal_state=0;
			tx_flag=1;
			HAL_TIM_Base_Stop(&htim14);
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
		}
		if(tx_flag==1)//--------------------------------------------------show data
		{//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
			//HAL_UART_Transmit(&huart1,(uint8_t *)&data_bit_lenght,1,100);
			//HAL_UART_Transmit(&huart1,(uint8_t *)&data_repository,32,100);
			for(uint8_t i=0;i<data_bit_lenght-1;i++)//find longest width
			{
				if(bzgtrn<data_repository[i])
				bzgtrn=data_repository[i];
			}
			//***************************
			for(uint8_t i=0;i<data_bit_lenght-1;i++)//find  shortest width
			{
				if(riztrn>data_repository[i])
				riztrn=data_repository[i];
			}
			//***********************************avg
			average=(riztrn+bzgtrn)/2;
			//********************************assign bits from width
			for(uint8_t i=0;i<data_bit_lenght;i++)
			{
				if(data_repository[i]>=average)
				{
					data_repository_to_binary[i]=1;
				}
				else
				{
					data_repository_to_binary[i]=0;
				}
			//	HAL_UART_Transmit(&huart1,(uint8_t *)&data_repository[i],1,100);
			}
			//************************
			memset(data_display,0,sizeof(data_display));
			for (uint8_t i=0;i<data_bit_lenght;i++)
			{

				data_display[i/8]|=(data_repository_to_binary[i]&0x01)<<(7-(i%8));
				
			}
			
			//****************************************tedad
			if(((data_bit_lenght-1)%8)==0)
			{
			tedad=((data_bit_lenght-1)/8);
			}
			else
			{
				tedad=((data_bit_lenght-1)/8)+1;
			}
			//HAL_UART_Transmit(&huart1,(uint8_t *)&tedad,1,100);
			for(int i=0;i<=tedad+1 ; i++){
				HAL_UART_Transmit(&huart1,(uint8_t *)&data_display[i],1,100);
			}
			//HAL_UART_Transmit(&huart1,(uint8_t *)&data_display,tedad,100);
			lcd_manager(home_layer);
			//----------------------------------------------cleare old data
			for (uint8_t i=0;i<4;i++)
			{
				//data_display[i]=0;
			}
			//HAL_Delay(500);
			tx_flag=0;
			bzgtrn=0;
			riztrn=0;
			
			HAL_Delay(200);
			//HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
			HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);
			//HAL_UART_Transmit(&huart1,(uint8_t *)&signal_state,1,100);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 439;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 50000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 39999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 65535;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void lcd_manager (uint8_t lcd_status)
{
	//----------------------------------------------------------------oled
	if(lcd_status == home_layer)
	{
			ssd1306_Fill(Black);
			for(uint8_t i=0;i<tedad;i++)
			{
				if(i<=4){
					ssd1306_SetCursor(i*22,0);
					ssd1306_WriteChar(oled_convert_data[data_display[i]/16],Font_11x18 ,White);
					ssd1306_SetCursor((i*22)+11,0);
					ssd1306_WriteChar(oled_convert_data[data_display[i]%16],Font_11x18 ,White); 
				}
				else if(i<10 && i>=5){
					ssd1306_SetCursor((i-5)*22,20);
					ssd1306_WriteChar(oled_convert_data[data_display[i]/16],Font_11x18 ,White);
					ssd1306_SetCursor(((i-5)*22)+11,20);
					ssd1306_WriteChar(oled_convert_data[data_display[i]%16],Font_11x18 ,White); 
				}
				
				else if(i>=10 && i<13){
					ssd1306_SetCursor((i-10)*22,40);
					ssd1306_WriteChar(oled_convert_data[data_display[i]/16],Font_11x18 ,White);
					ssd1306_SetCursor(((i-10)*22)+11,40);
					ssd1306_WriteChar(oled_convert_data[data_display[i]%16],Font_11x18 ,White); 
				}
		}
			//show lenght
			ssd1306_SetCursor(75,40);
			ssd1306_WriteString("len:",Font_7x10 ,White); 
			ssd1306_SetCursor(105,40);
			ssd1306_WriteChar(oled_convert_data[tedad/100],Font_7x10 ,White); 
					ssd1306_SetCursor(112,40);
			ssd1306_WriteChar(oled_convert_data[tedad/10],Font_7x10 ,White); 
					ssd1306_SetCursor(119,40);
			ssd1306_WriteChar(oled_convert_data[tedad%10],Font_7x10 ,White); 
					ssd1306_SetCursor(90,53);
			ssd1306_WriteString("SAVE",Font_7x10 ,White); 
			ssd1306_UpdateScreen();
	}
	//*****************************************************************
	else if(lcd_status== save_layer)
	{
		ssd1306_Fill(Black);
			for(uint8_t i=0;i<tedad;i++)
			{
				if(i<=4){
					ssd1306_SetCursor(i*22,25);
					ssd1306_WriteChar(oled_convert_data[data_display[i]/16],Font_11x18 ,White);
					ssd1306_SetCursor((i*22)+11,25);
					ssd1306_WriteChar(oled_convert_data[data_display[i]%16],Font_11x18 ,White); 
				}
			}
						for(uint8_t i=0;i<my_name.name_x_loc;i++)
			{
				///saved
			ssd1306_SetCursor(i*11,my_name.name_y_loc);
			ssd1306_WriteChar(my_name.name_buf[i],Font_11x18 ,White);
			}
			//last
			ssd1306_SetCursor(my_name.name_x_loc*11,my_name.name_y_loc);
			ssd1306_WriteChar(name_monitor[my_name.name_buff_index],Font_11x18 ,White);

						ssd1306_UpdateScreen();
	}
	//*****************************************************************
	else if(lcd_status== save_notif_layer)
	{
		ssd1306_Fill(Black);
		ssd1306_SetCursor(30,10);
		ssd1306_WriteString("DATA",Font_11x18 ,White); 
				ssd1306_SetCursor(25,40);
		ssd1306_WriteString("SAVED!",Font_11x18 ,White); 
		ssd1306_UpdateScreen();
	}
	//*************************************
	else if(lcd_status == start_up_layer)
	{
		ssd1306_Fill(Black);
		ssd1306_SetCursor(0,0);
		ssd1306_WriteString("REMOTE",Font_7x10 ,White); 

		ssd1306_SetCursor(0,15);
		ssd1306_WriteString("TESTER",Font_7x10 ,White);
		ssd1306_SetCursor(0,30);
		ssd1306_WriteString("professional",Font_7x10 ,White);
		
		ssd1306_SetCursor(0,45);
		ssd1306_WriteString("DESIGNED BY OS REZA",Font_7x10 ,White); 
		ssd1306_UpdateScreen();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
