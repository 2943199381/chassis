/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityRealtime7,
  .stack_size = 128 * 4
};
/* Definitions for Taskcommand */
osThreadId_t TaskcommandHandle;
const osThreadAttr_t Taskcommand_attributes = {
  .name = "Taskcommand",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for cha_control */
osThreadId_t cha_controlHandle;
const osThreadAttr_t cha_control_attributes = {
  .name = "cha_control",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Dji_ctrl_Task */
osThreadId_t Dji_ctrl_TaskHandle;
const osThreadAttr_t Dji_ctrl_Task_attributes = {
  .name = "Dji_ctrl_Task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskcommand(void *argument);
void StartTask_cha_control(void *argument);
void Dji_ctrl_Task_Fun(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Taskcommand */
  TaskcommandHandle = osThreadNew(StartTaskcommand, NULL, &Taskcommand_attributes);

  /* creation of cha_control */
  cha_controlHandle = osThreadNew(StartTask_cha_control, NULL, &cha_control_attributes);

  /* creation of Dji_ctrl_Task */
  Dji_ctrl_TaskHandle = osThreadNew(Dji_ctrl_Task_Fun, NULL, &Dji_ctrl_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
__weak void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  osThreadTerminate(defaultTaskHandle);
  /* Infinite loop */
  for(;;)
  {


    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskcommand */
/**
* @brief Function implementing the Taskcommand thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskcommand */
__weak void StartTaskcommand(void *argument)
{
  /* USER CODE BEGIN StartTaskcommand */
  /* Infinite loop */
  for(;;)
  {HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_10);
    osDelay(1000);
  }
  /* USER CODE END StartTaskcommand */
}

/* USER CODE BEGIN Header_StartTask_cha_control */
/**
* @brief Function implementing the cha_control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_cha_control */
__weak void StartTask_cha_control(void *argument)
{
  /* USER CODE BEGIN StartTask_cha_control */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask_cha_control */
}

/* USER CODE BEGIN Header_Dji_ctrl_Task_Fun */
/**
* @brief Function implementing the Dji_ctrl_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Dji_ctrl_Task_Fun */
__weak void Dji_ctrl_Task_Fun(void *argument)
{
  /* USER CODE BEGIN Dji_ctrl_Task_Fun */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Dji_ctrl_Task_Fun */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

