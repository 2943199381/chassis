/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
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
#include "fdcan.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "INA226.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "INA226.h"
#include "arm_math.h"
#include "stdio.h"
#include "stdarg.h"
#include "bsp_can.h"
#include "string.h"
#define TESTPOINT 1000
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float q;    // ��������Э���ϵͳ�仯�ٶȣ�ֵԽ����ӦԽ�죩
    float r;    // ��������Э���������������ֵԽ���˲�Խǿ��
    float p;    // �������Э����ڲ�״̬�������ʼ����
    float k;    // ���������棨�ڲ�״̬�������ʼ����?
    float x;    // ���Ź���ֵ���˲���������
} KalmanFilter;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

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


void USART_printf(char *fmt, ...)
{
	va_list ap;
	char str[128];
	va_start(ap,fmt);
	vsprintf(str,fmt,ap);
	va_end(ap);
	HAL_UART_Transmit(&huart2, (uint8_t*)str, strlen(str),0xFFFF);
}

/* USART���ͺ��� */
void USART_SendCurrent(float current)
{
    // ����һ����СΪ 32 ���ַ�����
    char tx_buf[32];
    // ʹ�� sprintf ��ʽ���ַ���
    int len = sprintf(tx_buf, "Current: %fmA\r\n", current);
    // ʹ�� HAL �ⷢ������
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, len, 100);
}

void USART_SendDis(float MM)
{
    // ����һ����СΪ 32 ���ַ�����
    char tx_buf[32];
    // ʹ�� sprintf ��ʽ���ַ���
    int len = sprintf(tx_buf, "Dis: %fmm\r\n", MM);
    // ʹ�� HAL �ⷢ������
    HAL_UART_Transmit(&huart2, (uint8_t*)tx_buf, len, 100);
}

void kalman_init(KalmanFilter* kf, float q, float r) {
    kf->q = q;
    kf->r = r;
    kf->p = 1.0f;  // ��ʼ���Э���ͨ�����?1��
    kf->k = 0.0f;  // ��ʼ����������
    kf->x = 0.0f;  // ��ʼ״ֵ̬������Ϊ�״β���ֵ��
}

float kalman_update(KalmanFilter* kf, float measurement) {
    // 1. Ԥ��׶�?
    kf->p += kf->q;  // �������Э���P = P + Q��

    // 2. ���㿨��������
    kf->k = kf->p / (kf->p + kf->r);

    // 3. ����״̬����
    kf->x += kf->k * (measurement - kf->x);

    // 4. �������Э����?
    kf->p *= (1 - kf->k);

    return kf->x;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
   uint32_t time[6] = {0};
	  float MM, D_MM;
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
  MX_FDCAN2_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  MX_FDCAN3_Init();
  /* USER CODE BEGIN 2 */
/*
	* ����ת��ʱ��1.1ms,��ƽ��ֵ����16������ģʽΪ��������������ģʽ
	* ������ת��ʱ�� = 1.1*16 = 17.6ms 
	0100 010��16��ƽ����100�����ߵ�ѹת��ʱ��1.1ms���ò�����100��������ѹת��ʱ��1.1m��101��������ѹ����ģʽ��
	0100 0101 0010 0101 = 0x4525
	*/
    INA226_SetConfig(0x4525);
	
	/*
	* ������������ѹ = 32768 * 0.0000025V = 0.08192V
	* ���÷�����ѹת����ת������:����3.9R���ֱ���2.5uV/3.9R=0.641uA
	* ��ʽ1
	* Current_LSB = Ԥ�������� / 2^15
	* Current_LSB = 25mA	/ 32768 = 0.00076294mA ,ѡ0.001ma
	* ��ʽ2
	* CAL = 0.00512/(Current_LSB*R)
	* CAL = 0.00512/(0.000001*3.9)=1312.8205 ,ȡ1313 = 0x0521
	*/
  INA226_SetCalibrationReg(0x0521);
		
		
  float fCurrent,fv;
	float result;
	static float rawdata[TESTPOINT];
	const int min=200;                      //����DT50���ø�������������?
	const int max=7000;
//	const float vmin=0.0879;                  //���ݵ�·������vmin��vmax
//	const float vmax=2.4004;  
//dt50-2 dis = 0.2734*fv-1501.1 
	const float k=0.2734;        //(max-min)/(vmax-vmin);
	float bias=-1501.1;          //min-k*vmin;                  //�������?
 
	
	KalmanFilter current_kf;
  kalman_init(&current_kf, 0.1, 0.1);  // q=0.01, r=0.1
	 time[0] = HAL_GetTick();
  time[1] = time[0];
  time[2] = time[0];
  time[3] = time[0];
  time[4] = time[0];
  time[5] = time[0];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
//        
		time[0] = HAL_GetTick();
//        // ˫ͨ�����?
        //FDCAN_SendCurrent(curren
 				//fCurrent=INA226_GetCurrent();           //ma
//			arm_mean_f32(rawdata,TESTPOINT,&result);				
        //USART_SendCurrent(fCurrent);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		    fv = INA226_GetShuntV();
				result=fv*2.5/1000/3.9;
//		    fCurrent = INA226_GetCurrent();
				//USART_SendCurrent(result);
				//float filtered_current = kalman_update(&current_kf, result);
				//USART_SendCurrent(filtered_current);
//		    //FDCAN_SendCurrent(result);
//				USART_SendCurrent(fCurrent);
//				USART_SendCurrent(result);
//				USART_SendCurrent(fv);
				MM=k*fv+bias; 
				
				
			
		   if (time[0] - time[1] > 1000)
    {
      USART_SendDis(MM);
      FDCAN_SendCurrent(MM);
      time[1] = time[0];
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV5;
  RCC_OscInitStruct.PLL.PLLN = 68;
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
