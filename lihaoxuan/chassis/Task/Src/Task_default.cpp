#include "cmsis_os.h"
#include "Task_default.h"
#include <stdio.h>
#include "RS01.h"
extern osThreadId_t defaultTaskHandle;
extern "C" {
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
   taskENTER_CRITICAL();



  RobStride_Motor RobStride_01(0x7F, false);

  taskEXIT_CRITICAL();
  osThreadTerminate(defaultTaskHandle);
  /* Infinite loop */
  for(;;)
  {


    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}
}