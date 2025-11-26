#include "cmsis_os.h"
#include "Task_default.h"
#include <stdio.h>
#include "RS01.h"
extern osThreadId_t defaultTaskHandle;
osMessageQueueId_t myQueue = NULL;
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
   taskENTER_CRITICAL();
  myQueue = osMessageQueueNew(16, sizeof(cha_Speed), NULL);
  
  if (myQueue == NULL) {
        // printf("Failed to create message queue.\n");
    }

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