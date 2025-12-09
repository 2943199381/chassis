/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
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
#include "motor_logic.h" // 只需要包含这一个
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define M_PI 3.1415926535f
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_TASK_FREQUENCY_MS 10 
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
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  主逻辑任务：负责发送高层指令
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
    
    // 初始化并校准零点 (使用 ID 1)
    Motor_InitZeroing(1);

    float target1 = 0.0f; 
    float target2 = 0.0f;          
    int flag = 0;

  /* Infinite loop */
  for(;;)
  {
      if (!Motor_IsMoving())
      {
		  flag++;
		  if(flag>=2)
		  {
		  float next_target = target1 + 6.33f * M_PI*2;
           target1 = next_target;
          // 发送指令：目标位置, 最大速度 24, 加速度 24
          Motor_MoveTo(next_target, 400.0f, 430.0f);
		  }
          
          
      }

      osDelay(8000); 
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief  控制循环任务：负责高频执行 Update
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  TickType_t xLastWakeTime;
  const uint32_t xFrequency_ticks = pdMS_TO_TICKS(CONTROL_TASK_FREQUENCY_MS);
  xLastWakeTime = osKernelGetTickCount(); 

  /* Infinite loop */
  for(;;)
  {
    xLastWakeTime += xFrequency_ticks;
    osDelayUntil(xLastWakeTime);

    // 只需要调用这一行
    Motor_UpdateLoop(CONTROL_TASK_FREQUENCY_MS);
  }
  /* USER CODE END StartTask02 */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */ 