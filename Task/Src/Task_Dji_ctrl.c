#include <stdio.h>
#include "cmsis_os.h"
#include "stm32g474xx.h"
#include "dji_3508_2006_motor.h"

void Dji_ctrl_Task_Fun(void *argument)
{
  /* USER CODE BEGIN StartTask_cha_control */
  /* Infinite loop */
  for(;;)
  {

    Dji_3508_all_motor_control();
    osDelay(1);
  }
  /* USER CODE END StartTask_cha_control */
}