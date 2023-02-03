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
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
#include "LED.h"
#include "string.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//rtcʱ����ض���
RTC_TimeTypeDef Time;
RTC_TimeTypeDef Setting_Time;
RTC_DateTypeDef Data;
//LCD��ʾ��ض���
unsigned char str1[30],str2[30];
uint8_t LCD_State =0;//������ʾ����
uint8_t Setting_State = 0;//0�޸��� 1�޸ķ��ӣ�2�޸�Сʱ
//Adc��ض���
float k = 0.5;
float Value=0;
//LED��˸״̬����
uint8_t LED_Option = 1;
uint8_t LED_State = 0; // ����LED�Ƶ�ƽ��ת�ﵽ��˸
uint8_t LED_ON_State = 0; // LED״̬��־λ��1ON,0 OFF
//UART���ڱ���
uint8_t Rx_Buf[1000],Rx_Buf_Temp[1000];
uint16_t Rx_Size=1000;
unsigned char str4[30],str3[30],temp[30];
uint8_t Appear_Flag =0;//0�����ϱ���ɣ�1�����ϱ�δ��ɡ���ֹ�ϱ����
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Get_Time(void)
{
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &Data, RTC_FORMAT_BIN);
}

void Adc(void)
{
	HAL_ADC_Start(&hadc2);
	Value = (HAL_ADC_GetValue(&hadc2)*3.3)/4096;
}

void Led1_Toggle(void)
{
	if(LED_State==0)
	{
		LEDx_ON(1);
		LED_State=1;
	}
	else
	{
		LEDx_OFF(1);
		LED_State=0;
	}
}

//int fputc(int ch, FILE *f) {
//	HAL_UART_Transmit_IT(&huart1, (uint8_t *)&ch, 1);
//	return ch;
//}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart == &huart1)
	{
		if(Size == 6)
		{
			memcpy(Rx_Buf,Rx_Buf_Temp,Size);
			if(Rx_Buf[0]=='k' && Rx_Buf[1]=='0' && Rx_Buf[2]=='.'){
				k = 1.*(Rx_Buf[3]-48)*0.1;
				sprintf(str4,"ok \r\n");
				HAL_UART_Transmit_IT(&huart1, str4, sizeof(str4));
			}
		}
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_Buf_Temp, Rx_Size);
	}
}

void LCD_Show(void)
{
	if(LCD_State == 0){ //��ʾ����1
		sprintf(str2,"    V1: %.2f", Value);
		LCD_DisplayStringLine(Line1, str2);
		sprintf(str2,"    K:%.1f",k);
		LCD_DisplayStringLine(Line2, str2);
		if(LED_ON_State == 1)  sprintf(str2,"    LED:ON       ");
		else                   sprintf(str2,"    LED:OFF      ");
		LCD_DisplayStringLine(Line3, str2);
		sprintf(str1,"    T:%02d-%02d-%02d",Time.Hours,Time.Minutes,Time.Seconds);
		LCD_DisplayStringLine(Line4, str1);
		sprintf(str2,"                                  ");
		LCD_DisplayStringLine(Line6, str2);
	}
	else if(LCD_State == 1)
	{
		sprintf(str2,"                         ");
		LCD_DisplayStringLine(Line1, str2);
		LCD_DisplayStringLine(Line2, str2);
		LCD_DisplayStringLine(Line4, str2);
		sprintf(str2,"       Setting           ");
		LCD_DisplayStringLine(Line3, str2);
		sprintf(str1,"    T:%02d-%02d-%02d",Setting_Time.Hours,Setting_Time.Minutes,Setting_Time.Seconds);
		LCD_DisplayStringLine(Line6, str1);
	}
	
}
void Key_Scan(void)
{
	if(HAL_GPIO_ReadPin(KEY_1_GPIO_Port,KEY_1_Pin) == 0)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(KEY_1_GPIO_Port,KEY_1_Pin) == 0)
		{
			if(LED_ON_State == 0) LED_ON_State = 1;
			else                  LED_ON_State = 0;
		}
	}
	
	if(HAL_GPIO_ReadPin(KEY_2_GPIO_Port,KEY_2_Pin) == 0)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(KEY_2_GPIO_Port,KEY_2_Pin) == 0)
		{
			if(LCD_State ==0)   LCD_State =1;
			else                LCD_State =0; 
		}
	}
	
	if(HAL_GPIO_ReadPin(KEY_3_GPIO_Port,KEY_3_Pin) == 0)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(KEY_3_GPIO_Port,KEY_3_Pin) == 0)
		{
			if(Setting_State<3)
				Setting_State++;
			if(Setting_State == 3)
				Setting_State =0;
		}
	}
	
	if(HAL_GPIO_ReadPin(KEY_4_GPIO_Port,KEY_4_Pin) == 0)
	{
		HAL_Delay(10);
		if(HAL_GPIO_ReadPin(KEY_4_GPIO_Port,KEY_4_Pin) == 0)
		{
			if(Setting_State == 0)//������
			{
				Setting_Time.Seconds++;
				if(Setting_Time.Seconds == 60)
					Setting_Time.Seconds = 0;
			}
			else if(Setting_State == 1)
			{
				Setting_Time.Minutes++;
				if(Setting_Time.Minutes == 60)
					Setting_Time.Minutes = 0;
			}
			else if(Setting_State == 2)
			{
				Setting_Time.Hours++;
				if(Setting_Time.Hours == 24)
					Setting_Time.Hours =0;
			}
		}
	}
}
void Reporting(void)
{
	  //��ʱ�ϱ�  LCD_State == 0 ��ʾ���ؽ���1����ʱ�����Զ�����ʱ�䣬�ſ��Խ����ϱ���
	  if(Time.Hours == Setting_Time.Hours && Time.Minutes == Setting_Time.Minutes && Time.Seconds == Setting_Time.Seconds && LCD_State == 0 && Appear_Flag == 0)
	  {
		  sprintf(str3,"%.2f+%.1f+%02d%02d%02d\r\n",Value,k,Time.Hours,Time.Minutes,Time.Seconds);
		  HAL_UART_Transmit_IT(&huart1, str3, sizeof(str3));
		  Appear_Flag = 1;//�ϱ�����,��ֹ����ϱ�
	  }
	  if(Time.Seconds != Setting_Time.Seconds)//δ�ﵽ�ϱ�ʱ�䣬�ȴ��ϱ�
		  Appear_Flag = 0;
}


//i2c��ز���
void iic_24c02_write(uint8_t *pucBuf , uint8_t ucAddr , uint8_t ucNum)//д����
{
	//pucbuf����������ݵĵ�ַ�������飻ucAddr�����ֽڿ�ʼλ�ã�ucNum���������ֽ���
	
	I2CStart();
	I2CSendByte(0xa0);//0xa0Ϊд����ʱ����豸��ַ
	I2CWaitAck(); //�ȴ�ͨѶ���
	
	I2CSendByte(ucAddr);//����Ҫ�������ݵĵ�ַ��0�����һ���ֽڴ����0��λ��1�����1���ֽڴ���1��λ
	I2CWaitAck();//�ȴ����
	
	while(ucNum--)//�൱��һ���ֽ�һ���ֽ�ȥ�洢
	{
		I2CSendByte(*pucBuf++);
		I2CWaitAck();
	}
	I2CStop();
	HAL_Delay(50);//��ֹ�ڶ�ʱ���ڶ�δ�������
}

void iic_24c02_read(uint8_t *pucBuf , uint8_t ucAddr , uint8_t ucNum)//������ָ��
{
	//ucNumΪ��Ҫ��ȡ���Լ�����pucbufΪ��ȡ���������ݴ����Ŀ������
	I2CStart();
	I2CSendByte(0xa0);//0xa0Ϊд����ʱ����豸��ַ
	I2CWaitAck(); //�ȴ�ͨѶ���
	
	I2CSendByte(ucAddr);//����Ҫ�������ݵĵ�ַ��0�����һ���ֽڴ����0��λ��1�����1���ֽڴ���1��λ
	I2CWaitAck();//�ȴ����
	
	I2CStart();
	I2CSendByte(0xa1);//0xa1Ϊ������ʱ����豸��ַ
	I2CWaitAck(); //�ȴ�ͨѶ���
	
	while(ucNum--)
	{
		*pucBuf++ = I2CReceiveByte();
		if(ucNum)
			I2CSendAck();
		else
			I2CSendNotAck();
	}
	I2CStop();
}
//���干������и������洢
typedef union EEPROM_Float
{
	float a;
	uint8_t b[4];
}float_data;

float_data float_read;
float_data float_write;
void EEPROM_Store(void)
{
	float_write.a = k;
	iic_24c02_write(float_write.b , 0 , 4);
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
  MX_RTC_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  //LCD��ʼ��
  LCD_Init();
  LCD_Clear(White);
  //ADC��ʼ��
  HAL_ADC_Init(&hadc2);
  //uwTick�����趨
  uint32_t Time_uwTick = 0;
  uint32_t LED_uwTick = 0;
  uint32_t LCD_uwTick = 0;
  uint32_t Key_uwTick = 0;
  //USART��������
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Rx_Buf_Temp, Rx_Size);
  //LED��ʼ��
  LED_All_Close();
  //EEPROM��ʼ��
  I2CInit();
  //��ȡ�洢��EEPROM�е����ݲ���
//  iic_24c02_read(float_read.b , 0 , 4);
//  k = float_read.a;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(uwTick - Time_uwTick>100)
	  {
		  Time_uwTick = uwTick;
		  Get_Time();
		  Adc();		
		  EEPROM_Store();
		  if(Value>3.3*k) LED_Option=1;
		  else{           
			  LED_Option=0;
			  LEDx_OFF(1);
		  }
		  Reporting();
	  }
	  if(LED_Option==1 && LED_ON_State == 1)
	  {
		  if(uwTick-LED_uwTick>200)
		  {
			  LED_uwTick=uwTick;
			  Led1_Toggle();
		  }
	  }
	  if(uwTick - LCD_uwTick >100)
	  {
		  LCD_uwTick = uwTick;
		  LCD_Show();
	  }
	  
	  if(uwTick - Key_uwTick>200)
	  {
		  Key_uwTick = uwTick;
		  Key_Scan();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 75;
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
