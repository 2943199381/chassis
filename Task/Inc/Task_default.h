// Task_default.h
#ifndef TASK_DEFAULT_H_
#define TASK_DEFAULT_H_
#include "cmsis_os.h"
extern osMessageQueueId_t myQueue;
extern osThreadId_t defaultTaskHandle;

typedef struct {
    uint8_t id;
    float vx;
    float vy;
    float w;
} cha_Speed;
#endif /* TASK_DEFAULT_H_ */