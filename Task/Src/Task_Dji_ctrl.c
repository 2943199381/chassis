#include <stdio.h>
#include "cmsis_os.h"
#include "stm32g474xx.h"
#include "dji_3508_2006_motor.h"

void Dji_ctrl_Task_Fun(void *argument)
{
  /* USER CODE BEGIN StartTask_cha_control */
  TickType_t xLastWakeTime;
  /* Infinite loop */
  for(;;)
  {

    Dji_3508_all_motor_control();


    
    xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil( &xLastWakeTime, 1 );
    
  }
  /* USER CODE END StartTask_cha_control */
}