//
// Created by 马皓然 on 2025/11/5.
//
// 文件名: chassis_driver.c
// 功能描述: 底盘驱动模块的实现文件。
//          包含了运动学逆解算算法、速度限幅、电机指令打包和发送等核心逻辑。
//          代码通过条件编译实现了对多种底盘类型的支持。
//
// Changed by 闫一航 on 2025/11/22
// 在马皓然代码的基础上添加了对于三角全向轮和三轮舵轮的代码的姐算
// 通过AI添加了代码的相应的注释
//

#include "chassis_driver.h"
#include <math.h>

// --- 1. 全局数据实例 ---

/**
 * @brief  存储所有轮子最终指令的数组
 * @note   该数组由运动学解算函数填充，然后由指令发送函数读取。
 *         其大小由 WHEEL_NUM 宏定义决定。
 */
static Wheel_Command_t wheel_data[WHEEL_NUM];

// --- 2. 工具函数 ---

#if defined(CHASSIS_TYPE_DUOLUN) || defined(CHASSIS_TYPE_TRIANGLE_DUOLUN)
/**
 * @brief  角度转换辅助函数
 * @param  x: 速度向量在X轴上的分量
 * @param  y: 速度向量在Y轴上的分量
 * @retval 转换后的角度 (单位：度, °)
 * @note   1. 该函数计算速度向量与Y轴正方向的夹角。
 *         2. 角度范围被归一化到 [-180.0, 180.0] 度之间。
 *         3. 此函数的角度定义是为了匹配舵轮电机的控制要求。
 */
static float arctan2y(float x, float y) {
    float theta_rad = 0.0f; // 中间变量，用于存储弧度值

    // 处理x为0的特殊情况，避免除零错误
    if (x == 0) {
        theta_rad = (y >= 0) ? PI/2.0f : -PI/2.0f;
    } else {
        // atanf(y/x) 计算出的是与X轴正方向的夹角
        theta_rad = atanf(y / x);

        // 根据x和y的正负，调整角度到正确的象限
        if (y < 0 && x < 0) theta_rad -= PI;
        if (y >= 0 && x < 0) theta_rad += PI;
    }

    // 将与X轴的夹角转换为与Y轴正方向的夹角
    // 1. theta_rad * 180.0f / PI 将弧度转换为度
    // 2. 90.0f - ... 实现坐标系转换
    float theta_deg = 90.0f - theta_rad * 180.0f / PI;

    // 将角度归一化到 [-180.0, 180.0] 范围
    while (theta_deg > 180.0f) theta_deg -= 360.0f;
    while (theta_deg < -180.0f) theta_deg += 360.0f;

    return theta_deg;
}
#endif

// --- 3. 运动学解算函数 ---

#ifdef CHASSIS_TYPE_QUANXIANGLUN
/**
 * @brief  四轮全向轮底盘的运动学逆解算
 * @param  motor_id: 电机编号 (0-3)
 * @param  vx: 底盘X轴方向线速度
 * @param  vy: 底盘Y轴方向线速度
 * @param  vr: 底盘绕Z轴角速度
 * @retval None
 * @note   1. 该函数根据底盘的整体运动指令，计算出每个全向轮的目标转速。
 *         2. 轮子布局为标准正方形，全向轮与坐标轴成45度角安装。
 *         3. 公式推导基于速度合成原理：轮缘速度 = 底盘质心速度 + 旋转带来的线速度。
 */
static void speed_decompose(int motor_id, float vx, float vy, float vr) {
    // 计算旋转运动在轮子处产生的线速度大小
    float vel_r = vr * CHASSIS_RADIUS;

    // 根据轮子位置，应用不同的速度合成公式
    switch (motor_id) {
        case 0: // 前左 (FL)
            wheel_data[motor_id].vel = vel_r - SQRT_2_INV * vx - SQRT_2_INV * vy;
            break;
        case 1: // 后左 (RL)
            wheel_data[motor_id].vel = vel_r + SQRT_2_INV * vx - SQRT_2_INV * vy;
            break;
        case 2: // 前右 (FR)
            wheel_data[motor_id].vel = vel_r - SQRT_2_INV * vx + SQRT_2_INV * vy;
            break;
        case 3: // 后右 (RR)
            wheel_data[motor_id].vel = vel_r + SQRT_2_INV * vx + SQRT_2_INV * vy;
            break;
        default: // 无效电机编号，速度设为0
            wheel_data[motor_id].vel = 0.0f;
            break;
    }
}

#elif defined(CHASSIS_TYPE_DUOLUN)
/**
 * @brief  四轮舵轮底盘的运动学逆解算
 * @param  motor_id: 电机编号 (0-3)
 * @param  vx: 底盘X轴方向线速度
 * @param  vy: 底盘Y轴方向线速度
 * @param  vr: 底盘绕Z轴角速度
 * @retval None
 * @note   1. 该函数为每个舵轮计算两个目标值：轮子的线速度和转向角度。
 *         2. 舵轮布局为标准正方形。
 *         3. 解算分为两步：
 *            a. 计算轮子在其安装点处的绝对速度向量 (vx_i, vy_i)。
 *            b. 将速度向量分解为大小 (vel) 和方向 (target_angle)。
 */
static void speed_decompose(int motor_id, float vx, float vy, float vr) {
    float vx_i, vy_i; // 轮子i在其安装点的速度向量分量
    // 计算轮子到旋转中心在X或Y轴上的距离
    float R = CHASSIS_RADIUS * SQRT_2_INV;

    // Step 1: 计算每个轮子的速度向量
    switch (motor_id) {
        case 0: // 左前
            vx_i = vx - vr * R;
            vy_i = vy - vr * R;
            break;
        case 1: // 左后
            vx_i = vx + vr * R;
            vy_i = vy - vr * R;
            break;
        case 2: // 右前
            vx_i = vx - vr * R;
            vy_i = vy + vr * R;
            break;
        case 3: // 右后
            vx_i = vx + vr * R;
            vy_i = vy + vr * R;
            break;
        default: // 无效电机编号
            vx_i = 0.0f;
            vy_i = 0.0f;
            break;
    }

    // Step 2: 计算速度大小和方向
    wheel_data[motor_id].vel = sqrtf(vx_i * vx_i + vy_i * vy_i);
    wheel_data[motor_id].target_angle = arctan2y(vx_i, vy_i);

    // 对轮子的线速度进行限幅
    if (wheel_data[motor_id].vel > SPEED_LIMIT_XY) {
        wheel_data[motor_id].vel = SPEED_LIMIT_XY;
    }
}

#elif defined(CHASSIS_TYPE_TRIANGLE_QUANXIANG)
/**
 * @brief  三角全向轮底盘的运动学逆解算
 * @param  motor_id: 电机编号 (0-2)
 * @param  vx: 底盘X轴方向线速度
 * @param  vy: 底盘Y轴方向线速度
 * @param  vr: 底盘绕Z轴角速度
 * @retval None
 * @note   1. 轮子呈等边三角形分布。
 *         2. 轮子0朝正前方(Y轴)，轮子1朝左后方(与Y轴夹角-120°)，轮子2朝右后方(与Y轴夹角+120°)。
 */
static void speed_decompose(int motor_id, float vx, float vy, float vr) {
    const float R = CHASSIS_RADIUS;       // 轮子到中心距离
    const float cos120 = -0.5f;           // cos(120°) 的值
    const float sin120 = sqrtf(3.0f) / 2.0f; // sin(120°) 的值
    float vel_r = vr * R;                 // 旋转带来的线速度

    switch (motor_id) {
        case 0: // 正前方轮子
            wheel_data[motor_id].vel = vy + vel_r;
            break;
        case 1: // 左后方轮子
            wheel_data[motor_id].vel = vy * cos120 - vx * sin120 + vel_r;
            break;
        case 2: // 右后方轮子
            wheel_data[motor_id].vel = vy * cos120 + vx * sin120 + vel_r;
            break;
        default: // 无效电机编号
            wheel_data[motor_id].vel = 0.0f;
            break;
    }
}

#elif defined(CHASSIS_TYPE_TRIANGLE_DUOLUN)
/**
 * @brief  三轮舵轮底盘的运动学逆解算
 * @param  motor_id: 电机编号 (0-2)
 * @param  vx: 底盘X轴方向线速度
 * @param  vy: 底盘Y轴方向线速度
 * @param  vr: 底盘绕Z轴角速度
 * @retval None
 * @note   1. 轮子呈等边三角形分布。
 */
static void speed_decompose(int motor_id, float vx, float vy, float vr) {
    const float R = CHASSIS_RADIUS;
    const float sin60 = sqrtf(3.0f) / 2.0f;
    const float cos60 = 0.5f;
    float x_i, y_i; // 轮子i的坐标
    float vx_i, vy_i; // 轮子i的速度向量

    // Step 1: 确定当前轮子的坐标
    switch (motor_id) {
        case 0: // 正前方
            x_i = 0.0f;
            y_i = R;
            break;
        case 1: // 左后方
            x_i = -R * sin60;
            y_i = -R * cos60;
            break;
        case 2: // 右后方
            x_i = R * sin60;
            y_i = -R * cos60;
            break;
        default:
            x_i = 0.0f;
            y_i = 0.0f;
            vx_i = 0.0f;
            vy_i = 0.0f;
            goto calc_end; // 直接跳转到计算结束，避免后续无效计算
    }

    // Step 2: 计算轮子的速度分量
    vx_i = vx - vr * y_i;
    vy_i = vy + vr * x_i;

calc_end:
    // Step 3: 计算轮子速度大小和角度
    wheel_data[motor_id].vel = sqrtf(vx_i * vx_i + vy_i * vy_i);
    wheel_data[motor_id].target_angle = arctan2y(vx_i, vy_i);

    // Step 4: 速度限幅
    if (wheel_data[motor_id].vel > SPEED_LIMIT_XY) {
        wheel_data[motor_id].vel = SPEED_LIMIT_XY;
    }
}

#else
/**
 * @brief  未知底盘类型的默认解算函数
 * @param  motor_id: 电机编号
 * @param  vx: 底盘X轴方向线速度
 * @param  vy: 底盘Y轴方向线速度
 * @param  vr: 底盘绕Z轴角速度
 * @retval None
 * @note   当没有定义任何已知的底盘类型时，编译此函数。
 *         函数将所有轮子指令设为0，起到安全保护作用。
 */
static void speed_decompose(int motor_id, float vx, float vy, float vr) {
    wheel_data[motor_id].vel = 0.0f;
#if defined(CHASSIS_TYPE_DUOLUN) || defined(CHASSIS_TYPE_TRIANGLE_DUOLUN)
    wheel_data[motor_id].target_angle = 0.0f;
#endif
}
#endif

// --- 4. 主速度控制函数 ---

/**
 * @brief  底盘遥控指令处理函数 (具体实现)
 * @param  vx: 底盘在X轴方向的目标线速度 (mm/s)
 * @param  vy: 底盘在Y轴方向的目标线速度 (mm/s)
 * @param  vr: 底盘绕Z轴的目标角速度 (rad/s)
 * @retval None
 * @note   函数执行流程：
 *         1. 速度限幅：对输入的vx和vy进行合成限幅，防止底盘超速。
 *         2. 运动学解算：调用与当前底盘类型匹配的解算函数，填充wheel_data数组。
 *         3. 指令发送：将wheel_data中的指令打包并发送给相应的电机驱动。
 */
void cha_remote(float vx, float vy, float vr) {
    float velx, vely, vela;

    // 1. 速度限幅 (仅对 XY 平面内的合速度进行限幅)
    //    目的是防止底盘在对角线运动时，合成速度超过硬件极限。
    float abs_xy_spd_sq = vx * vx + vy * vy; // 计算合速度的平方，避免开方运算，提高效率
    if (abs_xy_spd_sq > SPEED_LIMIT_XY * SPEED_LIMIT_XY) {
        // 如果合速度超限，则计算一个缩放比例
        float ratio = SPEED_LIMIT_XY / sqrtf(abs_xy_spd_sq);
        // 按比例缩小vx和vy，保持方向不变
        velx = vx * ratio;
        vely = vy * ratio;
    } else {
        // 速度在限幅范围内，直接赋值
        velx = vx;
        vely = vy;
    }
    vela = vr; // 旋转速度不与线速度联动限幅，单独处理

    // 2. 运动学解算
    //    遍历所有轮子，为每个轮子计算目标指令
    for (int i = 0; i < WHEEL_NUM; i++) {
        speed_decompose(i, velx, vely, vela);
    }

    // 3. 发送指令到电机
    //    根据不同的底盘类型，调用不同的指令发送函数
    for (int i = 0; i < WHEEL_NUM; i++) {
#if defined(CHASSIS_TYPE_QUANXIANGLUN) || defined(CHASSIS_TYPE_TRIANGLE_QUANXIANG)
        // 全向轮：仅需要发送转速指令
        Change_dji_speed(i, wheel_data[i].vel);
#elif defined(CHASSIS_TYPE_DUOLUN) || defined(CHASSIS_TYPE_TRIANGLE_DUOLUN)
        // 舵轮：需要同时发送转速和转向角指令
        Chassis_Send_Swerve_Command(i, wheel_data[i].vel, wheel_data[i].target_angle);
#endif
    }
}