/**
 ******************************************************************************
 * @file	 bsp_CAN.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2020/2/4
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _BSP_CAN_H
#define _BSP_CAN_H

#include "includes.h"

#define CAN_RC_DATA_Frame_0 0x131
#define CAN_RC_DATA_Frame_1 0x132

#define CAN_VTM_DATA_Frame_0 0x430
#define CAN_VTM_DATA_Frame_1 0x432

#define CAN_SYSTEM_RESET_CMD 0xAAA

#define CAN_PowerContol_ID 0x301

#define CAN_AERIAL_DATA_1 0x501
#define CAN_AERIAL_DATA_2 0x502
// void CAN_Device_Init(CAN_HandleTypeDef *_hcan);
void CAN_Device_Init(void);

void Send_Gimbal_Control_1(CAN_HandleTypeDef *hcan, int16_t Yaw, int16_t Pitch, int16_t Stir, int8_t ifFricOn);
void Send_Gimbal_Control_2(CAN_HandleTypeDef *_hcan, int16_t Mouse_X, int16_t Mouse_Y, int16_t KeyCode, int16_t MouseLeft, int16_t MouseRight);
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data);
void Send_VTM_Data(CAN_HandleTypeDef *_hcan, uint8_t *vtm_data);
void Send_Robot_Info(CAN_HandleTypeDef *_hcan, int8_t ID, uint16_t heatLimit, uint16_t heat, uint16_t bulletSpeed, uint16_t speed_limit,
					 uint16_t heatLimit2, uint16_t heat2, uint16_t speed_limit2, uint8_t gameStatus, uint16_t outpost_HP);
void Send_Reset_Command(CAN_HandleTypeDef *_hcan);
void Send_Power_Data(CAN_HandleTypeDef *_hcan, uint16_t Chassis_power_buffer, uint16_t Chassis_power_limit);
void Send_JudgeRxData(CAN_HandleTypeDef *_hcan, uint8_t *data);
void SendAerialData(CAN_HandleTypeDef *_hcan, float *X, float *Y, uint8_t *KeyBoard);
void float2u8array(float *FloatData, uint8_t *u8Array, uint8_t Key); // 浮点数转u8数组，Key为高低位变换

#endif
