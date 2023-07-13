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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "control.h"
#include "connect.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Blink HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13)
uint16_t adc_buf[3];
int adc_data;

motor M;
pid pid_v;
pid pid_a;	//摆角度环
pid pid_p;	//电机位置环

pole pole_ins;
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
//获取adc2，即PA1引脚电压值
int get_adc2(void){
	float data;
	HAL_ADC_Start(&hadc2);
	data = HAL_ADC_GetValue(&hadc2);//*3.3f/4096;
	return data;
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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adc_buf,3);
	//开启编码器
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim3,0x7fff);
	//开启PWM
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	motor_Init(&M);
	//开启TIM中断
	HAL_TIM_Base_Start_IT(&htim4);	//100Hz
	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);
	//开启U2串口中断
	HAL_UART_Receive_IT(&huart2, &(UartBuff_Ins.Rxbuf), 1);
	
	pole_ins.angle = 0;
	pole_ins.angle_last = 0;
	pole_ins.alpha = 0;
	UartBuff_Ins.Flag = 0x0e;
//	float kpa = 10*0.5,kia = 0,kda = 15*0.8;	//角度环PID
//	float kpp = 0.2,kip = 0.0,kdp = 10;	//位置环
	float kpa = 10*0.6,kia = 0,kda = 25*0.6;	//角度环PID
	float kpp = 0.46,kip = 0.0,kdp = 20;	//位置环
//	float kpa = 10*0.5,kia = 0,kda = 30*0.7;	//角度环PID
//	float kpp = 0.4,kip = 0.0,kdp = 25;	//位置环
//	
	float kpv = 0.175,kiv = 0.025,kdv = 0;
	
//	float kpp = -0.65,kip = -0.001,kdp = 0;	//位置环
//	float kpa = 7,kia = 0,kda = 20;	//角度环PID
//	float kpa = 4.2,kia = 0,kda = 16.2;	//角度环PID
	pid_init(&pid_a,kpa,kia,kda);
	pid_init(&pid_p,kpp,kip,kdp);
	pid_init(&pid_v,kpv,kiv,kdv);
	
//	motor_pwm_set(40);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//
		
		
	//	HAL_ADC_Start(&hadc2);
	//	 HAL_ADC_PollForConversion(&hadc2, 50);
		//adc_data = adc_buf[0]*3.3f/4096;	
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static int counter;
	int pwm;
	char str[30];
	//100Hz
	if(htim->Instance==TIM4){
		counter++;
		if(counter==10){
			counter=0;
			sprintf(str,"main.t0.txt=\"角度：%.2f\"",pole_ins.angle);
			HMISends(str);
			sprintf(str,"main.t1.txt=\"位置：%.2f\"",M.ang);
			HMISends(str);
		}
		adc2angle();
		speed_cal();
//		if(fabs(180-pole_ins.angle)<45)
//			erect_loop();
//		else motor_pwm_set(0);
		if(mission_select==1){
			mission1();
		}
		else if(mission_select==2){
			mission2();
		}
		else if(mission_select==3){
			if(fabs(179-pole_ins.angle)<45)
				erect_loop();
			else motor_pwm_set(0);
		}
		else if(mission_select==4){
			mission4();
		}
		else if(mission_select==0){
			motor_pwm_set(0);
		}
//		if(fabs(179-pole_ins.angle)<45)
//			erect_loop();
//		else 
//			motor_pwm_set(0);
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
