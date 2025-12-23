# HFSM库使用说明

## 简介

HFSM（Hierarchical Finite State Machine，层次有限状态机）是一个轻量级的C语言状态机库，专为STM32微控制器设计。该库支持层次化状态结构，允许状态继承和复用，简化复杂系统的状态管理。

## 特性

- **层次化状态支持**：支持父子状态关系，子状态可继承父状态的行为
- **事件驱动**：通过事件触发状态转换
- **轻量级**：代码简洁，内存占用小，适合嵌入式系统
- **易于使用**：简单直观的API接口
- **状态进入/退出回调**：支持状态进入和退出时的自动回调

## 文件说明

- `Driver_Layer/Inc/hfsm.h` - HFSM库头文件
- `Driver_Layer/Src/hfsm.c` - HFSM库实现文件
- `Driver_Layer/Src/hfsm_example.c` - 使用示例（交通信号灯）

## 核心概念

### 状态（State）

状态是系统可能处于的一种模式或配置。每个状态包含：
- 状态处理函数（handler）
- 父状态指针（支持层次结构）
- 状态名称（用于调试）

### 事件（Event）

事件是触发状态转换或状态内部行为的信号。特殊事件：
- `HFSM_EVENT_ENTRY (0xFFFE)` - 进入状态时自动触发
- `HFSM_EVENT_EXIT (0xFFFF)` - 退出状态时自动触发

### 状态机（State Machine）

状态机管理状态之间的转换，包含：
- 当前状态
- 用户数据指针（可选）

## API参考

### 状态机初始化

```c
void HFSM_Init(HFSM_StateMachine_t* me, 
               HFSM_State_t* initial_state, 
               void* user_data);
```

初始化状态机并设置初始状态。

**参数：**
- `me` - 状态机实例指针
- `initial_state` - 初始状态指针
- `user_data` - 用户数据指针（可为NULL）

### 状态初始化

```c
void HFSM_StateInit(HFSM_State_t* state, 
                    HFSM_StateHandler_t handler, 
                    HFSM_State_t* parent, 
                    const char* name);
```

初始化一个状态。

**参数：**
- `state` - 状态实例指针
- `handler` - 状态处理函数
- `parent` - 父状态指针（NULL表示顶层状态）
- `name` - 状态名称（用于调试）

### 事件分发

```c
HFSM_Status_t HFSM_Dispatch(HFSM_StateMachine_t* me, 
                            HFSM_Event_t event);
```

向状态机发送事件。

**参数：**
- `me` - 状态机实例指针
- `event` - 事件类型

**返回值：**
- `HFSM_HANDLED` - 事件已处理
- `HFSM_UNHANDLED` - 事件未处理
- `HFSM_TRANSITION` - 发生了状态转换

### 状态转换

```c
void HFSM_Transition(HFSM_StateMachine_t* me, 
                     HFSM_State_t* target_state);
```

请求转换到目标状态。实际转换在`HFSM_Dispatch()`函数返回时执行。

**参数：**
- `me` - 状态机实例指针
- `target_state` - 目标状态指针

### 获取当前状态

```c
HFSM_State_t* HFSM_GetCurrentState(HFSM_StateMachine_t* me);
```

获取状态机当前所处的状态。

### 检查状态

```c
uint8_t HFSM_IsInState(HFSM_StateMachine_t* me, 
                       HFSM_State_t* state);
```

检查状态机是否处于指定状态或其子状态中。

## 使用示例

### 1. 定义状态处理函数

```c
static HFSM_Status_t IdleStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    switch (event) {
        case HFSM_EVENT_ENTRY:
            // 进入空闲状态
            LED_Off();
            return HFSM_HANDLED;
            
        case HFSM_EVENT_EXIT:
            // 退出空闲状态
            return HFSM_HANDLED;
            
        case EVENT_START:
            // 收到启动事件，转换到运行状态
            HFSM_Transition(me, &s_running_state);
            return HFSM_TRANSITION;
            
        default:
            return HFSM_UNHANDLED;
    }
}
```

### 2. 初始化状态和状态机

```c
// 定义状态实例
static HFSM_State_t s_idle_state;
static HFSM_State_t s_running_state;
static HFSM_StateMachine_t my_sm;

void MyApp_Init(void) {
    // 初始化状态
    HFSM_StateInit(&s_idle_state, IdleStateHandler, NULL, "Idle");
    HFSM_StateInit(&s_running_state, RunningStateHandler, NULL, "Running");
    
    // 初始化状态机，从空闲状态开始
    HFSM_Init(&my_sm, &s_idle_state, NULL);
}
```

### 3. 处理事件

```c
void MyApp_ProcessEvent(HFSM_Event_t event) {
    HFSM_Dispatch(&my_sm, event);
}
```

## 层次状态示例

层次状态允许子状态继承父状态的行为：

```c
// 父状态：工作状态
static HFSM_Status_t WorkingStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    switch (event) {
        case EVENT_EMERGENCY_STOP:
            // 所有子状态都能处理紧急停止
            HFSM_Transition(me, &s_stop_state);
            return HFSM_TRANSITION;
        default:
            return HFSM_UNHANDLED;
    }
}

// 子状态：加速状态
static HFSM_Status_t AccelStateHandler(HFSM_StateMachine_t* me, HFSM_Event_t event) {
    switch (event) {
        case HFSM_EVENT_ENTRY:
            Motor_Accelerate();
            return HFSM_HANDLED;
        case EVENT_SPEED_REACHED:
            HFSM_Transition(me, &s_cruise_state);
            return HFSM_TRANSITION;
        default:
            // 未处理的事件传递给父状态
            return HFSM_UNHANDLED;
    }
}

// 初始化时建立层次关系
void Init(void) {
    HFSM_StateInit(&s_working_state, WorkingStateHandler, NULL, "Working");
    HFSM_StateInit(&s_accel_state, AccelStateHandler, &s_working_state, "Accel");
    HFSM_StateInit(&s_cruise_state, CruiseStateHandler, &s_working_state, "Cruise");
}
```

在上面的例子中，当`AccelStateHandler`接收到`EVENT_EMERGENCY_STOP`事件时，由于它返回`HFSM_UNHANDLED`，该事件会自动传递给父状态`WorkingStateHandler`处理。

## 状态转换流程

当发生状态转换时，HFSM会：

1. 从当前状态开始，向上退出到最近公共祖先（LCA）
   - 对每个退出的状态调用其处理函数并传入`HFSM_EVENT_EXIT`
2. 从LCA开始，向下进入到目标状态
   - 对每个进入的状态调用其处理函数并传入`HFSM_EVENT_ENTRY`

这确保了状态转换的正确性和一致性。

## 最佳实践

1. **状态处理函数返回值**：
   - 处理了事件：返回`HFSM_HANDLED`
   - 未处理事件（让父状态处理）：返回`HFSM_UNHANDLED`
   - 请求状态转换后：返回`HFSM_TRANSITION`

2. **事件定义**：建议使用枚举或宏定义事件ID，避免使用保留值（0xFFFE, 0xFFFF）

3. **状态名称**：为每个状态提供有意义的名称，便于调试

4. **用户数据**：可以通过`user_data`指针传递上下文信息给状态处理函数

5. **线程安全**：当前实现不是线程安全的，如果需要在多线程环境使用，需要添加互斥保护

## 内存占用

- `HFSM_State_t`：约12字节（取决于指针大小）
- `HFSM_StateMachine_t`：约12字节
- 总体RAM占用：根据状态数量线性增长

## 注意事项

1. 不要在状态处理函数中递归调用`HFSM_Dispatch()`
2. 最多支持16层状态嵌套（可通过修改源码调整）
3. 状态转换在`HFSM_Dispatch()`返回时完成，不是立即执行
4. 确保状态对象在状态机生命周期内保持有效

## 完整示例

参见`Driver_Layer/Src/hfsm_example.c`文件，其中包含一个完整的交通信号灯状态机实现示例。

## 移植说明

该库设计为独立的，只依赖标准C库中的`stdint.h`、`stddef.h`和`string.h`，可以轻松移植到其他平台。

## 许可证

本库作为R1底盘项目的一部分发布。
