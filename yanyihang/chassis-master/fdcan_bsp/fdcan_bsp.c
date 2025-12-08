// R1_chassis
// Created by Lenovo on 2025/12/8 20 39.
//

#include "fdcan_bsp.h"

void FDCAN1_RxFilter_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0X1FFFFFFF;
    if(HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
    {
        Error_Handler();//进入硬件错误
    }

    if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO0中断
    {
        Error_Handler();//进入硬件错误
    }

    HAL_FDCAN_Start(&hfdcan1);
}
void FDCAN2_RxFilter_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0X1FFFFFFF;
    if(HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
    {
        Error_Handler();//进入硬件错误
    }

    if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO1中断
    {
        Error_Handler();//进入硬件错误
    }

    HAL_FDCAN_Start(&hfdcan2);
}

void FDCAN3_RxFilter_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;
    /* Configure Rx filter */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x00000000;
    sFilterConfig.FilterID2 = 0X1FFFFFFF;
    if(HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_ACCEPT_IN_RX_FIFO0,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
    {
        Error_Handler();//进入硬件错误
    }

    if(HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_BUS_OFF, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO0中断
    {
        Error_Handler();//进入硬件错误
    }

    HAL_FDCAN_Start(&hfdcan3);
}
/*
@brief FDCAN1发送函数
@param
	TxData 待发送数据
	id 帧ID
	len 待发送数据帧长度
	EXTflag = 1 使用扩展帧 EXTflag = 0 使用标准帧
@return
	0 发送成功
	1 发送失败
*/
uint8_t FDCAN1_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
	FDCAN_TxHeaderTypeDef TxMessage;

	TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */

	if(EXTflag)
		TxMessage.IdType			= FDCAN_EXTENDED_ID;	/* 扩展ID */
	else
		TxMessage.IdType			= FDCAN_STANDARD_ID;	/* 标准ID */

	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= len;					/* 设置数据长度 */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
	TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
	TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;	/* FDCAN格式 */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
	TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxMessage, TxData) != HAL_OK)
	{
		return 1;
	}

	return 0;
}

/*
@brief FDCAN2发送函数
@param
	TxData 待发送数据
	id 帧ID
	len 待发送数据帧长度
	EXTflag = 1 使用扩展帧 EXTflag = 0 使用标准帧
@return
	0 发送成功
	1 发送失败
*/
uint8_t FDCAN2_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
	FDCAN_TxHeaderTypeDef TxMessage;

	TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */

	if(EXTflag)
		TxMessage.IdType			= FDCAN_EXTENDED_ID;	/* 扩展ID */
	else
		TxMessage.IdType			= FDCAN_STANDARD_ID;	/* 标准ID */

	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= len;					/* 设置数据长度 */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
	TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
	TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;	/* FDCAN格式 */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
	TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxMessage, TxData) != HAL_OK)
	{
		return 1;
	}

	return 0;
}

/*
@brief FDCAN3发送函数
@param
	TxData 待发送数据
	id 帧ID
	len 待发送数据帧长度
	EXTflag = 1 使用扩展帧 EXTflag = 0 使用标准帧
@return
	0 发送成功
	1 发送失败
*/
uint8_t FDCAN3_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag)
{
	FDCAN_TxHeaderTypeDef TxMessage;

	TxMessage.Identifier 			= id;					/* 设置发送帧消息的ID */

	if(EXTflag)
		TxMessage.IdType			= FDCAN_EXTENDED_ID;	/* 扩展ID */
	else
		TxMessage.IdType			= FDCAN_STANDARD_ID;	/* 标准ID */

	TxMessage.TxFrameType 			= FDCAN_DATA_FRAME;		/* 数据帧 */
	TxMessage.DataLength 			= len;					/* 设置数据长度 */
	TxMessage.ErrorStateIndicator 	= FDCAN_ESI_ACTIVE;		/* 设置错误状态指 */
	TxMessage.BitRateSwitch 		= FDCAN_BRS_OFF;		/* 关闭可变波特率 */
	TxMessage.FDFormat 				= FDCAN_CLASSIC_CAN;	/* FDCAN格式/传统CAN */
	TxMessage.TxEventFifoControl 	= FDCAN_NO_TX_EVENTS;	/* 用于发送事件FIFO控制, 无发送事件*/
	TxMessage.MessageMarker 		= 0;					/* 用于复制到TX EVENT FIFO的消息Maker来识别消息状态，范围0-0xFF */

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &TxMessage, TxData) != HAL_OK)
	{
		return 1;
	}

	return 0;
}
//错误重启函数
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  //__HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);
	if(hfdcan->Instance == FDCAN1)
	{
		MX_FDCAN1_Init();
	}
	else if(hfdcan->Instance == FDCAN2)
	{
		MX_FDCAN2_Init();
	}
	else if(hfdcan->Instance == FDCAN3)
	{
		MX_FDCAN3_Init();
	}
	else
	{

	}
}

void Chassis_Send_Swerve_Command(uint8_t id,uint32_t speed,uint32_t angle)
{
	uint8_t TxMessage[24];
	id += 0x20;
	TxMessage[8]  = (speed >> 24) & 0xFF;  // 提取24~31位
	TxMessage[9]  = (speed >> 16) & 0xFF;  // 提取16~23位
	TxMessage[10] = (speed >> 8)  & 0xFF;  // 提取8~15位
	TxMessage[11] = speed & 0xFF;
	TxMessage[12]  = (angle >> 24) & 0xFF;  // 提取24~31位
	TxMessage[13]  = (angle >> 16) & 0xFF;  // 提取16~23位
	TxMessage[14] = (angle >> 8)  & 0xFF;  // 提取8~15位
	TxMessage[15] = angle & 0xFF;
	FDCAN3_Transmit(TxMessage, id, 24, 1);
}