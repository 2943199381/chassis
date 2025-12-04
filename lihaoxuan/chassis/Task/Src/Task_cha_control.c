
#include <stdio.h>
#include "Task_default.h"
#include "cmsis_os.h"
#include "chassis_driver.h"
#include "Task_cha_control.h"
void StartTask_cha_control(void *argument)
{
  /* USER CODE BEGIN StartTask_cha_control */
  cha_Speed received;
  /* Infinite loop */
  for(;;)
  {
     
    osMessageQueueGet(cha_speedqueueHandle, &received, NULL, 0);
    cha_remote(received.vx, received.vy, received.w);
                // 成功接收，处理 rx
        
        
    
    osDelay(10);
  }
  /* USER CODE END StartTask_cha_control */
}