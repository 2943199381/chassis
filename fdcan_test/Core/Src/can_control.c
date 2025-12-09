#include "can_control.h"
#include "fdcan.h"
#define M_PI 3.14159f

//CAN_HandleTypeDef hcan;

HAL_StatusTypeDef sendCANControlMotor(uint32_t moduleId, uint32_t motorId, float torque, float speed, float position,int8_t *motor_temperature, float *actual_torque, float *actual_speed, float *actual_position)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;

  // 配置CAN发送帧的扩展ID
  TxHeader.Identifier = ((moduleId & 0x3) << 27) |
(0 << 26)|// 下发标志
                   (0 << 24) |  // 数据内容：电机控制
                   (10 << 16) | // 控制模式10
                   ((motorId & 0xF) << 8) |
                   (1 << 12) | // motorStatus默认为1
                   (0 << 15) | // 超时标志
                   0;          // 预留

  // 配置CAN发送帧的其他属性
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;

  // 将浮点数转换为整数，以便在CAN帧中传输
  int32_t Pset = (int32_t)(position * 32768 / (2 * M_PI));
  int16_t Wset = (int16_t)(speed * 256.0f / (2 * M_PI));
  int16_t Tset = (int16_t)(torque * 256.0f);
  
  // 将转换后的整数数据填充到发送数据数组中
  TxData[0] = Pset & 0xFF;
  TxData[1] = (Pset >> 8) & 0xFF;
  TxData[2] = (Pset >> 16) & 0xFF;
  TxData[3] = (Pset >> 24) & 0xFF;
  TxData[4] = Wset & 0xFF;
  TxData[5] = (Wset >> 8) & 0xFF;
  TxData[6] = Tset & 0xFF;
  TxData[7] = (Tset >> 8) & 0xFF;

  // 发送CAN消息
  HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);

  // 检查发送状态
  if (status != HAL_OK)
  {
    return status;
  }

  // 等待接收响应，设置100ms超时
  uint32_t timeout = HAL_GetTick() + 100; // 100ms超时
  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0)
  {
    if (HAL_GetTick() > timeout)
    {
      return HAL_TIMEOUT;
    }
  }

  // 接收响应
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];
  // 从CAN接收FIFO中获取消息
  status = HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &RxHeader, RxData);

  if (status == HAL_OK)
  {
    // 解析接收到的数据
    uint8_t received_motor_id = RxHeader.Identifier >> 8 & 0x0F;
    // 验证接收到的数据是否匹配发送的请求
    if (received_motor_id != motorId)
    {
      return HAL_ERROR;
    }

    // 解析电机温度
    *motor_temperature = RxHeader.Identifier & 0xFF; // 取倒数第一个字节作为电机温度

    // 解析实际力矩、速度和位置
    uint16_t temp_torque = (RxData[6] | (RxData[7] << 8));
    uint16_t temp_speed = (RxData[4] | (RxData[5] << 8));
    uint32_t temp_position = (RxData[0] | (RxData[1] << 8) | (RxData[2] << 16) | (RxData[3] << 24));

    // 将整数数据转换回浮点数
    *actual_torque = (float)temp_torque / 256.0f;
    *actual_speed = (float)temp_speed / 256.0f * (2 * M_PI);
    *actual_position = (float)temp_position / 32768.0f * (2 * M_PI);
  }

  // 返回函数执行状态
  return status;
}


/**
 * @brief 发送CAN命令设置电机的位置和速度增益（Kpos和Kspd）
 * 
 * @param moduleId 模块ID (0-3)
 * @param motorId 电机ID (0-15)
 * @param Kpos 位置增益（单位：N·m/rad）
 * @param Kspd 速度增益（单位：N·m/(rad/s)）
 * @return HAL_StatusTypeDef 函数执行状态
 */
HAL_StatusTypeDef sendCANSetMotorKK(uint32_t moduleId, uint32_t motorId, float Kpos, float Kspd)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8] = {0};  // 初始化发送数据数组为0
  uint32_t TxMailbox;

  // 配置CAN发送帧的扩展ID
  TxHeader.Identifier = ((moduleId & 0x3) << 27) |
                   (0 << 26) |  // 下发标志
                   (0 << 24) |  // 数据内容：电机控制
                   (11 << 16) | // 控制模式11 - 设置KposKspd
                   ((motorId & 0xF) << 8) |
                   0; // 其余位无意义

  // 配置CAN发送帧的其他属性
  TxHeader.IdType = FDCAN_EXTENDED_ID;    // ID类型：扩展ID
  TxHeader.TxFrameType = FDCAN_DATA_FRAME; // 帧类型：数据帧
  TxHeader.DataLength = FDCAN_DLC_BYTES_8; // 数据长度（8字节）
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 错误状态指示器
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF; // 禁用比特率切换（经典CAN模式）
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;  // 帧格式：经典CAN
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 禁用Tx事件FIFO
  TxHeader.MessageMarker = 0;             // 报文标记（可选）

  // 将浮点数增益值转换为整数，以便在CAN帧中传输
  // 乘以1280是为了提高精度，同时保持在16位整数范围内
  uint16_t KspdSet = (uint16_t)(Kspd * 1280.0f);
  uint16_t KposSet = (uint16_t)(Kpos * 1280.0f);
  
  // 将转换后的整数数据填充到发送数据数组中
  TxData[0] = KspdSet & 0xFF;        // Kspd低8位
  TxData[1] = (KspdSet >> 8) & 0xFF; // Kspd高8位
  TxData[2] = KposSet & 0xFF;        // Kpos低8位
  TxData[3] = (KposSet >> 8) & 0xFF; // Kpos高8位
  // TxData[4]到TxData[7]保持为0，因为未使用

  // 发送CAN消息并返回发送状态
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, TxData);
}

int8_t motor_temperature[3];
float actual_torque[3],actual_speed[3],actual_position[3];

void motor_control(uint32_t motorId, float torque, float speed, float position, float Kpos, float Kspd)
{
  uint32_t moduleId = 3; // 模块ID，2位（范围0-3）默认情况下模块ID是3
        
  int8_t motor_temperature0;
  float actual_torque0, actual_speed0, actual_position0;
  printf("a%.3f\n",Kpos);
  sendCANSetMotorKK(moduleId, motorId, Kpos, Kspd); // 设置电机的KposKspd
  printf("b%.3f\n",Kpos);
  HAL_StatusTypeDef status = sendCANControlMotor(moduleId, motorId, torque, 
                                            speed, position,&motor_temperature0, &actual_torque0, &actual_speed0, &actual_position0);//设置电机的运动参数值
    
  motor_temperature[motorId] = motor_temperature0;
  actual_torque[motorId] = actual_torque0;
  actual_speed[motorId] = actual_speed0;
  actual_position[motorId] = actual_position0;
  if(status != HAL_OK) {
        
        //可以加上错误处理
  }
}