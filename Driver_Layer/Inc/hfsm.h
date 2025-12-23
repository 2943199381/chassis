//
// Created by GitHub Copilot on 2025/12/23.
//

#ifndef R1_CHASSIS_HFSM_H
#define R1_CHASSIS_HFSM_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief 最大状态嵌套深度
 */
#define HFSM_MAX_DEPTH 16

/**
 * @brief 状态机事件类型定义
 */
typedef uint16_t HFSM_Event_t;

/**
 * @brief 状态处理函数返回值
 */
typedef enum {
    HFSM_HANDLED,           ///< 事件已处理
    HFSM_UNHANDLED,         ///< 事件未处理，传递给父状态
    HFSM_TRANSITION         ///< 发生状态转换
} HFSM_Status_t;

/**
 * @brief 状态机前置声明
 */
typedef struct HFSM_StateMachine HFSM_StateMachine_t;
typedef struct HFSM_State HFSM_State_t;

/**
 * @brief 状态处理函数类型定义
 * @param me 状态机实例指针
 * @param event 事件类型
 * @return 处理状态
 */
typedef HFSM_Status_t (*HFSM_StateHandler_t)(HFSM_StateMachine_t* me, HFSM_Event_t event);

/**
 * @brief 状态结构体定义
 */
struct HFSM_State {
    HFSM_StateHandler_t handler;    ///< 状态处理函数
    HFSM_State_t* parent;           ///< 父状态指针（层次化支持）
    const char* name;               ///< 状态名称（调试用）
};

/**
 * @brief 状态机结构体定义
 */
struct HFSM_StateMachine {
    HFSM_State_t* current_state;    ///< 当前状态
    HFSM_State_t* next_state;       ///< 下一个状态（用于转换）
    void* user_data;                ///< 用户数据指针
};

/**
 * @brief 特殊事件定义
 */
#define HFSM_EVENT_ENTRY    0xFFFE  ///< 进入状态事件
#define HFSM_EVENT_EXIT     0xFFFF  ///< 退出状态事件

/**************公共接口begin**************/

/**
 * @brief 初始化状态机
 * @param me 状态机实例指针
 * @param initial_state 初始状态指针
 * @param user_data 用户数据指针
 */
void HFSM_Init(HFSM_StateMachine_t* me, HFSM_State_t* initial_state, void* user_data);

/**
 * @brief 处理事件
 * @param me 状态机实例指针
 * @param event 事件类型
 * @return 处理状态
 */
HFSM_Status_t HFSM_Dispatch(HFSM_StateMachine_t* me, HFSM_Event_t event);

/**
 * @brief 状态转换
 * @param me 状态机实例指针
 * @param target_state 目标状态指针
 */
void HFSM_Transition(HFSM_StateMachine_t* me, HFSM_State_t* target_state);

/**
 * @brief 初始化状态
 * @param state 状态指针
 * @param handler 状态处理函数
 * @param parent 父状态指针（NULL表示顶层状态）
 * @param name 状态名称
 */
void HFSM_StateInit(HFSM_State_t* state, HFSM_StateHandler_t handler, 
                    HFSM_State_t* parent, const char* name);

/**
 * @brief 获取当前状态
 * @param me 状态机实例指针
 * @return 当前状态指针
 */
HFSM_State_t* HFSM_GetCurrentState(HFSM_StateMachine_t* me);

/**
 * @brief 检查是否处于某个状态
 * @param me 状态机实例指针
 * @param state 要检查的状态指针
 * @return 1表示在该状态或其子状态中，0表示不在
 */
uint8_t HFSM_IsInState(HFSM_StateMachine_t* me, HFSM_State_t* state);

/**************公共接口end**************/

#endif //R1_CHASSIS_HFSM_H
