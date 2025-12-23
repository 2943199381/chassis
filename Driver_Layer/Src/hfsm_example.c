//
// Created by GitHub Copilot on 2025/12/23.
//
// HFSM使用示例
// 此文件展示如何使用层次状态机库

#include "hfsm.h"

/**
 * @brief 使用示例：交通信号灯状态机
 * 
 * 状态层次结构：
 *   - Working (工作状态)
 *     - Green (绿灯)
 *     - Yellow (黄灯)
 *     - Red (红灯)
 *   - Fault (故障状态)
 */

// 定义事件
#define EVENT_TIMER_EXPIRED     1
#define EVENT_FAULT_DETECTED    2
#define EVENT_FAULT_CLEARED     3

// 前置声明状态处理函数
static HFSM_Status_t WorkingStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event);
static HFSM_Status_t GreenStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event);
static HFSM_Status_t YellowStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event);
static HFSM_Status_t RedStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event);
static HFSM_Status_t FaultStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event);

// 定义状态实例
static HFSM_State_t s_working_state;
static HFSM_State_t s_green_state;
static HFSM_State_t s_yellow_state;
static HFSM_State_t s_red_state;
static HFSM_State_t s_fault_state;

// 状态机实例
static HFSM_StateMachine_t traffic_light_sm;

/**
 * @brief 初始化交通灯状态机示例
 */
void TrafficLight_Init(void) {
    // 初始化各个状态
    HFSM_StateInit(&s_working_state, WorkingStateHandler, NULL, "Working");
    HFSM_StateInit(&s_green_state, GreenStateHandler, &s_working_state, "Green");
    HFSM_StateInit(&s_yellow_state, YellowStateHandler, &s_working_state, "Yellow");
    HFSM_StateInit(&s_red_state, RedStateHandler, &s_working_state, "Red");
    HFSM_StateInit(&s_fault_state, FaultStateHandler, NULL, "Fault");
    
    // 初始化状态机，从绿灯状态开始
    HFSM_Init(&traffic_light_sm, &s_green_state, NULL);
}

/**
 * @brief Working状态处理函数（父状态）
 */
static HFSM_Status_t WorkingStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    switch (event) {
        case HFSM_EVENT_ENTRY:
            // 进入Working状态
            return HFSM_HANDLED;
            
        case HFSM_EVENT_EXIT:
            // 退出Working状态
            return HFSM_HANDLED;
            
        case EVENT_FAULT_DETECTED:
            // 所有Working子状态都处理故障事件
            HFSM_Transition(me, &s_fault_state);
            return HFSM_TRANSITION;
            
        default:
            return HFSM_UNHANDLED;
    }
}

/**
 * @brief Green状态处理函数
 */
static HFSM_Status_t GreenStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    switch (event) {
        case HFSM_EVENT_ENTRY:
            // 进入绿灯状态，启动定时器
            // TODO: 启动30秒定时器
            return HFSM_HANDLED;
            
        case HFSM_EVENT_EXIT:
            // 退出绿灯状态
            return HFSM_HANDLED;
            
        case EVENT_TIMER_EXPIRED:
            // 定时器到期，转换到黄灯
            HFSM_Transition(me, &s_yellow_state);
            return HFSM_TRANSITION;
            
        default:
            // 未处理的事件传递给父状态
            return HFSM_UNHANDLED;
    }
}

/**
 * @brief Yellow状态处理函数
 */
static HFSM_Status_t YellowStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    switch (event) {
        case HFSM_EVENT_ENTRY:
            // 进入黄灯状态，启动定时器
            // TODO: 启动3秒定时器
            return HFSM_HANDLED;
            
        case HFSM_EVENT_EXIT:
            // 退出黄灯状态
            return HFSM_HANDLED;
            
        case EVENT_TIMER_EXPIRED:
            // 定时器到期，转换到红灯
            HFSM_Transition(me, &s_red_state);
            return HFSM_TRANSITION;
            
        default:
            return HFSM_UNHANDLED;
    }
}

/**
 * @brief Red状态处理函数
 */
static HFSM_Status_t RedStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    switch (event) {
        case HFSM_EVENT_ENTRY:
            // 进入红灯状态，启动定时器
            // TODO: 启动30秒定时器
            return HFSM_HANDLED;
            
        case HFSM_EVENT_EXIT:
            // 退出红灯状态
            return HFSM_HANDLED;
            
        case EVENT_TIMER_EXPIRED:
            // 定时器到期，转换到绿灯
            HFSM_Transition(me, &s_green_state);
            return HFSM_TRANSITION;
            
        default:
            return HFSM_UNHANDLED;
    }
}

/**
 * @brief Fault状态处理函数
 */
static HFSM_Status_t FaultStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    switch (event) {
        case HFSM_EVENT_ENTRY:
            // 进入故障状态，所有灯闪烁
            // TODO: 启动闪烁控制
            return HFSM_HANDLED;
            
        case HFSM_EVENT_EXIT:
            // 退出故障状态
            return HFSM_HANDLED;
            
        case EVENT_FAULT_CLEARED:
            // 故障清除，回到绿灯状态
            HFSM_Transition(me, &s_green_state);
            return HFSM_TRANSITION;
            
        default:
            return HFSM_UNHANDLED;
    }
}

/**
 * @brief 状态机事件处理（在主循环或定时器中调用）
 */
void TrafficLight_ProcessEvent(HFSM_Event_t event) {
    HFSM_Dispatch(&traffic_light_sm, event);
}
