// R1_chassis
// Created by Lenovo on 2025/12/8 20 40.
//

#ifndef R1_CHASSIS_FDCAN_BSP_H
#define R1_CHASSIS_FDCAN_BSP_H

#include "main.h"
#include "fdcan.h"

void FDCAN1_RxFilter_Config(void);
void FDCAN2_RxFilter_Config(void);
void FDCAN3_RxFilter_Config(void);

uint8_t FDCAN1_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag);
uint8_t FDCAN2_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag);
uint8_t FDCAN3_Transmit(uint8_t *TxData, uint32_t id, uint32_t len, uint8_t EXTflag);

void Chassis_Send_Swerve_Command(uint8_t id,uint32_t speed,uint32_t angle);

#endif //R1_CHASSIS_FDCAN_BSP_H