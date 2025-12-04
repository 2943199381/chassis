#include "bsp_can.h"
#include "string.h"
/**************内部宏定义与重命名begin**************/

/**************内部宏定义与重命名end**************/

/**************内部变量与函数begin**************/
struct LocatorResult lcResult;
uint8_t LOCATOR_buff[64];
extern float MM;
/**************内部变量与函数end**************/

/**************外部接口begin**************/
uint8_t can_data_num_g=0;//结构体数组can_database_g的大小，在Hash_table_init(void)中更新
Can_Data can_database_g[]={//通信表，odrive和vesc的依赖项。新加ID时，在ID_NUMDEF中定义相应ID的意义
    //Data_type            Data_ID             *Data_ptr                                   		Data_length  	*MenuFunc   Channel		Fifo_num
//		{WRITE_ONLY,      vesc_motor1,         (uint8_t*)(&vesc_content_transform[1].u8_data),       4,          NULL,        2,		FDCAN_RX_FIFO0},
//		{WRITE_ONLY,      vesc_motor2,         (uint8_t*)(&vesc_content_transform[2].u8_data),       4,          NULL,        2,		FDCAN_RX_FIFO0},
//        {WRITE_ONLY,      vesc_motor3,         (uint8_t*)(&vesc_content_transform[3].u8_data),       4,          NULL,        2,		FDCAN_RX_FIFO0}
};
uint16_t hash_table[1000]={999};

void FDCAN_SendCurrent(float current)
{
    uint8_t can_data  [8];
    uint32_t *p_current = (uint32_t*)&current;
    
    // 构造CAN消息
    FDCAN_TxHeaderTypeDef tx_header = {
        .Identifier = 0x21,
        .IdType = FDCAN_STANDARD_ID,
        .TxFrameType = FDCAN_DATA_FRAME,
        .DataLength = FDCAN_DLC_BYTES_8,
        .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
        .BitRateSwitch = FDCAN_BRS_OFF,
        .FDFormat = FDCAN_CLASSIC_CAN,
        .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
        .MessageMarker = 0
    };
    
    // 复制浮点数到CAN数据域
    memcpy(can_data, p_current, 4);
    
    // 发送数据
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &tx_header, can_data);

}

/**************外部接口end**************/

FDCAN_TxHeaderTypeDef TxHeader1;
FDCAN_RxHeaderTypeDef RxHeader1;
FDCAN_TxHeaderTypeDef TxHeader2;
FDCAN_RxHeaderTypeDef RxHeader2;
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
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = 0x00000000;
	sFilterConfig.FilterID2 = 0X1FFFFFFF;
	if(HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	
	if(HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_ACCEPT_IN_RX_FIFO1,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE) != HAL_OK)//设置全局配置
	{
		Error_Handler();//进入硬件错误
	}
	
	if(HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK)//启动FIFO1中断
	{
		Error_Handler();//进入硬件错误
	}
	
	HAL_FDCAN_Start(&hfdcan3);
}

/*
1.函数功能：初始化通信hash表
2.入参：none
3.返回值：none
4.用法及调用要求：在使用vesc和odrive之前调用此函数
5.其它：
*/
void Hash_table_init(void){ 
	int i;
	can_data_num_g = sizeof(can_database_g) / sizeof(can_database_g[0]);
	for(i=0;i<1000;i++){
		hash_table[i] = 999;
	}
	for(i=0;i<can_data_num_g;i++){
		hash_table[can_database_g[i].Data_ID] = i;
	}
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan==&hfdcan1)
	{
			
	}
}


void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
	 if(hfdcan==&hfdcan1)
	{
		FDCAN_RxHeaderTypeDef rx_header;
			uint8_t rx_data[8];
			HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
			FDCAN_SendCurrent(MM);
	}
}