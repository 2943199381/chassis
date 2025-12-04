#ifndef __CAN_CONTROL_H
#define __CAN_CONTROL_H


#include "stdint.h"
#include "stdio.h"


void motor_control(uint32_t motorId, float torque, float speed, float position, float Kpos, float Kspd);

#endif
