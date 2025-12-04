//
// Created by 马皓然 on 2025/11/5.
//
// 文件名: chassis_driver.h
// 功能描述: 底盘驱动模块的头文件，包含底盘类型定义、参数配置、数据结构和函数声明。
//          该文件为底盘控制提供了统一的接口和配置入口。
//
// Changed by 闫一航 on 2025/11/22
// 在马皓然代码的基础上添加了对于三角全向轮和三轮舵轮的代码的姐算
// 通过AI添加了代码的相应的注释
//

#ifndef R1_CHASSIS_CHASSIS_DRIVER_H
#define R1_CHASSIS_CHASSIS_DRIVER_H

// --- 1. 底盘类型宏定义 ---
// 说明: 通过宏定义选择当前使用的底盘类型。每次仅能启用一种。
//       启用不同的宏会编译对应的运动学解算和控制逻辑。
//#define CHASSIS_TYPE_DUOLUN                // 四轮舵轮底盘 (Swerve Drive)
// #define CHASSIS_TYPE_QUANXIANGLUN        // 四轮全向轮底盘 (Mecanum Drive)
// #define CHASSIS_TYPE_TRIANGLE_QUANXIANG  // 三角全向轮底盘 (Omni Drive)
 #define CHASSIS_TYPE_TRIANGLE_DUOLUN     // 三轮舵轮底盘 (3-Wheel Swerve)

// --- 2. 底盘参数配置 ---
// 说明: 以下参数需根据实际硬件规格进行精确测量和配置。
//       错误的参数会导致底盘运动精度下降或失控。

#if defined(CHASSIS_TYPE_TRIANGLE_QUANXIANG) || defined(CHASSIS_TYPE_TRIANGLE_DUOLUN)
#define WHEEL_NUM       3                // 三角布局底盘的轮子数量
#else
#define WHEEL_NUM       4                // 四轮布局底盘的轮子数量
#endif

#define SPEED_LIMIT_XY  12000.0f         // 底盘在XY平面内的最大合成线速度限制 (单位：毫米/秒, mm/s)
#define MOTOR_VEL_LIMIT 10000.0f         // 单个电机的最大转速限制 (单位：转/分, RPM 或自定义单位)
#define CHASSIS_RADIUS  289.91f          // 底盘旋转中心到轮子中心的距离 (单位：毫米, mm)
#define WHEEL_CIRCUMFERENCE 314.16f      // 轮子的周长 (单位：毫米, mm)，用于速度单位转换
#define SQRT_2_INV      0.70710678f      // 1/sqrt(2) 的预计算值，用于简化全向轮和舵轮的运动学计算
#define PI              3.1415926f       // 圆周率常量

// --- 3. 数据结构定义 ---

/**
 * @brief  单个轮子的目标指令数据结构
 * @note   该结构根据底盘类型的不同，成员变量可能会有差异。
 */
typedef struct {
    float vel;                            // 轮子的目标速度
                                          //  - 对于全向轮：通常指电机的转速 (RPM 或自定义单位)
                                          //  - 对于舵轮：通常指轮子的线速度 (mm/s)

#if defined(CHASSIS_TYPE_DUOLUN) || defined(CHASSIS_TYPE_TRIANGLE_DUOLUN)
    float target_angle;                   // 舵轮的目标转向角 (单位：度, °)
                                          //  仅在舵轮类型的底盘中使用。
#endif
} Wheel_Command_t;

// --- 4. 函数声明 ---

/**
 * @brief  底盘遥控指令处理函数
 * @param  vx: 底盘在X轴方向的目标线速度 (单位：mm/s)
 * @param  vy: 底盘在Y轴方向的目标线速度 (单位：mm/s)
 * @param  vr: 底盘绕Z轴的目标角速度 (单位：弧度/秒, rad/s)
 * @retval None
 * @note   这是底盘控制的主入口函数。它接收期望的速度指令，经过运动学解算，
 *         计算出每个轮子的具体执行指令（速度和角度），并最终发送给电机驱动。
 */
void cha_remote(float vx, float vy, float vr);

#endif // R1_CHASSIS_CHASSIS_DRIVER_H