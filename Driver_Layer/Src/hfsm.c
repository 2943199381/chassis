//
// Created by GitHub Copilot on 2025/12/23.
//

#include "hfsm.h"
#include <string.h>

/**************内部函数声明begin**************/
static HFSM_State_t* FindLCA(HFSM_State_t* source, HFSM_State_t* target);
static void ExitToState(HFSM_StateMachine_t* me, HFSM_State_t* target);
static void EnterFromState(HFSM_StateMachine_t* me, HFSM_State_t* source);
static uint8_t IsAncestor(HFSM_State_t* ancestor, HFSM_State_t* state);
/**************内部函数声明end**************/

/*
1.函数功能：初始化状态机
2.入参：状态机指针，初始状态指针，用户数据指针
3.返回值：none
4.用法及调用要求：在使用状态机前必须调用此函数初始化
5.其它：
*/
void HFSM_Init(HFSM_StateMachine_t* me, HFSM_State_t* initial_state, void* user_data) {
    if (me == NULL || initial_state == NULL) {
        return;
    }
    
    me->current_state = NULL;
    me->next_state = NULL;
    me->user_data = user_data;
    
    // 转换到初始状态
    HFSM_Transition(me, initial_state);
}

/*
1.函数功能：初始化状态
2.入参：状态指针，处理函数，父状态指针，状态名称
3.返回值：none
4.用法及调用要求：在定义状态时调用，用于设置状态属性
5.其它：
*/
void HFSM_StateInit(HFSM_State_t* state, HFSM_StateHandler_t handler, 
                    HFSM_State_t* parent, const char* name) {
    if (state == NULL || handler == NULL) {
        return;
    }
    
    state->handler = handler;
    state->parent = parent;
    state->name = name;
}

/*
1.函数功能：事件分发处理
2.入参：状态机指针，事件
3.返回值：处理状态
4.用法及调用要求：向状态机发送事件，由当前状态及其父状态链处理
5.其它：
*/
HFSM_Status_t HFSM_Dispatch(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    if (me == NULL || me->current_state == NULL) {
        return HFSM_UNHANDLED;
    }
    
    HFSM_Status_t status = HFSM_UNHANDLED;
    HFSM_State_t* state = me->current_state;
    
    // 从当前状态向上遍历父状态链，直到事件被处理
    while (state != NULL && status == HFSM_UNHANDLED) {
        if (state->handler != NULL) {
            status = state->handler(me, event);
        }
        state = state->parent;
    }
    
    // 如果发生了状态转换，执行转换
    if (me->next_state != NULL) {
        HFSM_State_t* target = me->next_state;
        me->next_state = NULL;
        
        // 找到最近公共祖先
        HFSM_State_t* lca = FindLCA(me->current_state, target);
        
        // 退出到LCA
        ExitToState(me, lca);
        
        // 进入目标状态
        me->current_state = lca;
        EnterFromState(me, target);
        
        return HFSM_TRANSITION;
    }
    
    return status;
}

/*
1.函数功能：状态转换
2.入参：状态机指针，目标状态指针
3.返回值：none
4.用法及调用要求：在状态处理函数中调用，设置下一个状态
5.其它：实际转换会在Dispatch函数中完成
*/
void HFSM_Transition(HFSM_StateMachine_t* me, HFSM_State_t* target_state) {
    if (me == NULL || target_state == NULL) {
        return;
    }
    
    // 如果是首次初始化（当前状态为空）
    if (me->current_state == NULL) {
        me->current_state = target_state;
        // 触发ENTRY事件
        if (target_state->handler != NULL) {
            target_state->handler(me, HFSM_EVENT_ENTRY);
        }
        return;
    }
    
    me->next_state = target_state;
}

/*
1.函数功能：获取当前状态
2.入参：状态机指针
3.返回值：当前状态指针
4.用法及调用要求：查询状态机当前所处状态
5.其它：
*/
HFSM_State_t* HFSM_GetCurrentState(HFSM_StateMachine_t* me) {
    if (me == NULL) {
        return NULL;
    }
    return me->current_state;
}

/*
1.函数功能：检查是否在某个状态中
2.入参：状态机指针，要检查的状态指针
3.返回值：1表示在该状态或其子状态中，0表示不在
4.用法及调用要求：用于判断当前状态是否为某个状态或其子状态
5.其它：
*/
uint8_t HFSM_IsInState(HFSM_StateMachine_t* me, HFSM_State_t* state) {
    if (me == NULL || state == NULL || me->current_state == NULL) {
        return 0;
    }
    
    return IsAncestor(state, me->current_state) || (me->current_state == state);
}

/**************内部函数实现begin**************/

/*
1.函数功能：查找两个状态的最近公共祖先（LCA）
2.入参：源状态指针，目标状态指针
3.返回值：最近公共祖先状态指针
4.用法及调用要求：内部函数，用于状态转换时确定退出和进入路径
5.其它：
*/
static HFSM_State_t* FindLCA(HFSM_State_t* source, HFSM_State_t* target) {
    if (source == NULL || target == NULL) {
        return NULL;
    }
    
    // 如果target是source的祖先
    if (IsAncestor(target, source)) {
        return target;
    }
    
    // 如果source是target的祖先
    if (IsAncestor(source, target)) {
        return source;
    }
    
    // 向上遍历source的父状态链，找到第一个也是target祖先的状态
    HFSM_State_t* s = source->parent;
    while (s != NULL) {
        if (IsAncestor(s, target)) {
            return s;
        }
        s = s->parent;
    }
    
    return NULL;
}

/*
1.函数功能：检查一个状态是否是另一个状态的祖先
2.入参：祖先状态指针，状态指针
3.返回值：1表示是祖先，0表示不是
4.用法及调用要求：内部函数，用于层次关系判断
5.其它：
*/
static uint8_t IsAncestor(HFSM_State_t* ancestor, HFSM_State_t* state) {
    if (ancestor == NULL || state == NULL) {
        return 0;
    }
    
    HFSM_State_t* s = state->parent;
    while (s != NULL) {
        if (s == ancestor) {
            return 1;
        }
        s = s->parent;
    }
    
    return 0;
}

/*
1.函数功能：退出状态直到目标状态
2.入参：状态机指针，目标状态指针
3.返回值：none
4.用法及调用要求：内部函数，在状态转换时调用
5.其它：从当前状态向上退出，直到目标状态（不包括目标状态）
*/
static void ExitToState(HFSM_StateMachine_t* me, HFSM_State_t* target) {
    if (me == NULL || me->current_state == NULL) {
        return;
    }
    
    HFSM_State_t* state = me->current_state;
    
    // 退出当前状态直到目标状态
    while (state != NULL && state != target) {
        if (state->handler != NULL) {
            state->handler(me, HFSM_EVENT_EXIT);
        }
        state = state->parent;
    }
}

/*
1.函数功能：从某状态进入到目标状态
2.入参：状态机指针，目标状态指针
3.返回值：none
4.用法及调用要求：内部函数，在状态转换时调用
5.其它：从当前状态开始，沿着父状态链找到目标状态的路径，然后逐层进入
*/
static void EnterFromState(HFSM_StateMachine_t* me, HFSM_State_t* target) {
    if (me == NULL || target == NULL) {
        return;
    }
    
    // 构建从LCA到target的路径
    HFSM_State_t* path[HFSM_MAX_DEPTH];
    int depth = 0;
    
    HFSM_State_t* state = target;
    while (state != NULL && state != me->current_state && depth < HFSM_MAX_DEPTH) {
        path[depth++] = state;
        state = state->parent;
    }
    
    // 检查是否超出最大深度
    if (depth >= HFSM_MAX_DEPTH && state != NULL && state != me->current_state) {
        // 超出最大嵌套深度，状态转换可能不完整
        // 在实际应用中可以添加错误日志或断言
        return;
    }
    
    // 从父状态到子状态依次进入
    for (int i = depth - 1; i >= 0; i--) {
        me->current_state = path[i];
        if (path[i]->handler != NULL) {
            path[i]->handler(me, HFSM_EVENT_ENTRY);
        }
    }
}

/**************内部函数实现end**************/
