/**
 ******************************************************************************
 * @file    bsp_CAN.c
 * @author  Wang Hongxi
 * @version V1.1.0
 * @date    2021/3/30
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "bsp_CAN.h"
#include "VTM_info.h"
#include "bsp_dwt.h"

uint8_t tempBuff[16] = {0};
int16_t TempPlanX1000;
int16_t TempPlanY1000;

/**
 * @Func		CAN_Device_Init
 * @Brief
 * @Param		CAN_HandleTypeDef* hcan
 * @Retval		None
 * @Date       2019/11/4
 **/
// void CAN_Device_Init(CAN_HandleTypeDef *_hcan)
// {
//     // ��ʼ��CAN������Ϊ������״̬ ��Ϊ�������� �����
//     CAN_FilterTypeDef can_filter_st;
//     can_filter_st.FilterActivation = ENABLE;
//     can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
//     can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
//     can_filter_st.FilterIdHigh = 0x0000;
//     can_filter_st.FilterIdLow = 0x0000;
//     can_filter_st.FilterMaskIdHigh = 0x0000;
//     can_filter_st.FilterMaskIdLow = 0x0000;
//     can_filter_st.FilterBank = 0;
//     can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

//     // CAN2��CAN1��FilterBank��ͬ
//     if (_hcan == &hcan2)
//     {
//         can_filter_st.SlaveStartFilterBank = 14;
//         can_filter_st.FilterBank = 14;
//     }

//     // CAN ��������ʼ��
//     HAL_CAN_ConfigFilter(_hcan, &can_filter_st);
//     // ����CAN
//     HAL_CAN_Start(_hcan);
//     // ����֪ͨ
//     HAL_CAN_ActivateNotification(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
// }
void CAN_Device_Init(void)
{
	// ��ʼ��CAN������Ϊ������״̬ ��Ϊ�������� �����
	CAN_FilterTypeDef can_filter_st;
	can_filter_st.FilterActivation = ENABLE;
	can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
	can_filter_st.FilterIdHigh = 0x0000;
	can_filter_st.FilterIdLow = 0x0000;
	can_filter_st.FilterMaskIdHigh = 0x0000;
	can_filter_st.FilterMaskIdLow = 0x0000;
	can_filter_st.FilterBank = 0;
	can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
	// CAN ��������ʼ��
	while (HAL_CAN_ConfigFilter(&hcan1, &can_filter_st) != HAL_OK)
	{
	}
	// ����CAN
	while (HAL_CAN_Start(&hcan1) != HAL_OK)
	{
	}
	// ����֪ͨ
	while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	}

	can_filter_st.SlaveStartFilterBank = 14;
	can_filter_st.FilterBank = 14;
	// CAN ��������ʼ��
	while (HAL_CAN_ConfigFilter(&hcan2, &can_filter_st) != HAL_OK)
	{
	}
	// ����CAN
	while (HAL_CAN_Start(&hcan2) != HAL_OK)
	{
	}
	// ����֪ͨ
	while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
	}
}

/**
 * @Func	    void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* _hcan)
 * @Brief      CAN���߽��ջص����� ���ڽ��յ������
 * @Param	    CAN_HandleTypeDef* _hcan
 * @Retval	    None
 * @Date       2019/11/4
 **/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *_hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	static uint8_t RC_Data_Buf[16];
	static uint8_t Follow_Data_Buf[8];
	static uint8_t MiniPC_CtrlFrame_Buf[8];
	static int16_t CAP_vol;

	HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &rx_header, rx_data);

	if (_hcan == &hcan1)
	{
		if (rx_header.StdId >= 0x201 && rx_header.StdId <= 0x204)
		{
			if (Chassis.ChassisMotor[rx_header.StdId - 0x201].msg_cnt++ <= 50)
			{
				get_moto_offset(&Chassis.ChassisMotor[rx_header.StdId - 0x201], rx_data);
			}
			else
			{
				get_moto_info(&Chassis.ChassisMotor[rx_header.StdId - 0x201], rx_data);
			}
			switch (rx_header.StdId)
			{
			case 0x201:
			{
				Detect_Hook(CHASSIS_MOTOR1_TOE);
				break;
			}
			case 0x202:
			{
				Detect_Hook(CHASSIS_MOTOR2_TOE);
				break;
			}
			case 0x203:
			{
				Detect_Hook(CHASSIS_MOTOR3_TOE);
				break;
			}
			case 0x204:
			{
				Detect_Hook(CHASSIS_MOTOR4_TOE);
				break;
			}
			}
		}
	}
	if (_hcan == &hcan2)
	{
		switch (rx_header.StdId)
		{
		// �����������ư巢����ң��������
		case CAN_RC_DATA_Frame_0:
			RC_Data_Buf[0] = rx_data[0];
			RC_Data_Buf[1] = rx_data[1];
			RC_Data_Buf[2] = rx_data[2];
			RC_Data_Buf[3] = rx_data[3];
			RC_Data_Buf[4] = rx_data[4];
			RC_Data_Buf[5] = rx_data[5];
			RC_Data_Buf[6] = rx_data[6];
			RC_Data_Buf[7] = rx_data[7];
			break;
		case CAN_RC_DATA_Frame_1:
			RC_Data_Buf[8] = rx_data[0];
			RC_Data_Buf[9] = rx_data[1];
			RC_Data_Buf[10] = rx_data[2];
			RC_Data_Buf[11] = rx_data[3];
			RC_Data_Buf[12] = rx_data[4];
			RC_Data_Buf[13] = rx_data[5];
			RC_Data_Buf[14] = rx_data[6];
			RC_Data_Buf[15] = rx_data[7];
			Callback_RC_Handle(&remote_control, RC_Data_Buf);
			break;
		case YAW_MOTOR_ID:
			Detect_Hook(GIMBAL_YAW_MOTOR_TOE);
			if (rx_data[6] != 0 && rx_data[7] != 0)
				get_RMD_info(&Gimbal.YawMotor, rx_data);
			break;
		case 0x151:
			tempBuff[0] = rx_data[0]; // CF_SOF
			tempBuff[1] = rx_data[1]; // POSX
			tempBuff[2] = rx_data[2]; // POSX
			tempBuff[3] = rx_data[3]; // POSY
			tempBuff[4] = rx_data[4]; // POSY
			tempBuff[5] = rx_data[5]; // YAW
			tempBuff[6] = rx_data[6]; // YAW
			tempBuff[7] = rx_data[7]; // planX 1
			// if (tempBuff[0] == 0x66 && tempBuff[7] == 0x88)
			// {
			Chassis.posX1000 = (int16_t)((rx_data[2] << 8) | rx_data[1]);
			Chassis.posY1000 = (int16_t)((rx_data[4] << 8) | rx_data[3]);
			Chassis.posZ1000 = (int16_t)((rx_data[6] << 8) | rx_data[5]);
			// }
			break;
		case 0x150:
			tempBuff[8] = rx_data[0];  // planX 1
			tempBuff[9] = rx_data[1];  // planY 1
			tempBuff[10] = rx_data[2]; // planY 1
			tempBuff[11] = rx_data[3]; // planX 2
			tempBuff[12] = rx_data[4]; // planX 2
			tempBuff[13] = rx_data[5]; // planY 2
			tempBuff[14] = rx_data[6]; // planY 2
			tempBuff[15] = rx_data[7]; // CF_EOF

			TempPlanX1000 = (int16_t)((tempBuff[8] << 8) | (tempBuff[7]));
			TempPlanY1000 = (int16_t)((tempBuff[10] << 8) | (tempBuff[9]));
			Chassis.PlanX1000 = (int16_t)((tempBuff[12] << 8) | (tempBuff[11]));
			Chassis.PlanY1000 = (int16_t)((tempBuff[14] << 8) | (tempBuff[13]));
			break;
		}
	}
}

// ͨ��CAN���߷���ң������Ϣ ��������δʹ��
void Send_RC_Data(CAN_HandleTypeDef *_hcan, uint8_t *rc_data)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = CAN_RC_DATA_Frame_0;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = rc_data[0];
	CAN_Send_Data[1] = rc_data[1];
	CAN_Send_Data[2] = rc_data[2];
	CAN_Send_Data[3] = rc_data[3];
	CAN_Send_Data[4] = rc_data[4];
	CAN_Send_Data[5] = rc_data[5];
	CAN_Send_Data[6] = rc_data[6];
	CAN_Send_Data[7] = rc_data[7];

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

	TX_MSG.StdId = CAN_RC_DATA_Frame_1;
	CAN_Send_Data[0] = rc_data[8];
	CAN_Send_Data[1] = rc_data[9];
	CAN_Send_Data[2] = rc_data[10];
	CAN_Send_Data[3] = rc_data[11];
	CAN_Send_Data[4] = rc_data[12];
	CAN_Send_Data[5] = rc_data[13];
	CAN_Send_Data[6] = rc_data[14];
	CAN_Send_Data[7] = rc_data[15];

	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_VTM_Data(CAN_HandleTypeDef *_hcan, uint8_t *vtm_data)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = CAN_VTM_DATA_Frame_0;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = vtm_data[0];
	CAN_Send_Data[1] = vtm_data[1];
	CAN_Send_Data[2] = vtm_data[2];
	CAN_Send_Data[3] = vtm_data[3];
	CAN_Send_Data[4] = vtm_data[4];
	CAN_Send_Data[5] = vtm_data[5];
	CAN_Send_Data[6] = vtm_data[6];
	CAN_Send_Data[7] = vtm_data[7];

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

	TX_MSG.StdId = CAN_VTM_DATA_Frame_1;
	CAN_Send_Data[0] = vtm_data[8];
	CAN_Send_Data[1] = vtm_data[9];
	CAN_Send_Data[2] = vtm_data[10];
	CAN_Send_Data[3] = vtm_data[11];
	CAN_Send_Data[4] = 0;
	CAN_Send_Data[5] = 0;
	CAN_Send_Data[6] = 0;
	CAN_Send_Data[7] = 0;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Robot_Info(CAN_HandleTypeDef *_hcan, int8_t ID, uint16_t heatLimit, uint16_t heat, uint16_t bulletSpeed, uint16_t speed_limit,
					 uint16_t heatLimit2, uint16_t heat2, uint16_t speed_limit2, uint8_t gameStatus, uint16_t outpost_HP)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = 0x133;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = ID;
	CAN_Send_Data[1] = (heatLimit >> 8) & 0xFF;
	CAN_Send_Data[2] = heatLimit & 0xFF;
	CAN_Send_Data[3] = (heat >> 8) & 0xFF;
	CAN_Send_Data[4] = heat & 0xFF;
	CAN_Send_Data[5] = (bulletSpeed >> 8) & 0xFF;
	CAN_Send_Data[6] = bulletSpeed & 0xFF;
	CAN_Send_Data[7] = speed_limit & 0xFF;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
	TX_MSG.StdId = 0x134;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;

	CAN_Send_Data[0] = (heatLimit2 >> 8) & 0xFF;
	CAN_Send_Data[1] = heatLimit2 & 0xFF;
	CAN_Send_Data[2] = (heat2 >> 8) & 0xFF;
	CAN_Send_Data[3] = heat2 & 0xFF;
	CAN_Send_Data[4] = speed_limit2 & 0xFF;
	CAN_Send_Data[5] = gameStatus;
	CAN_Send_Data[6] = (outpost_HP >> 8) & 0xFF;
	CAN_Send_Data[7] = outpost_HP & 0xFF;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_JudgeRxData(CAN_HandleTypeDef *_hcan, uint8_t *data)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = 0x235;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	for (uint8_t i = 0; i < 8; i++)
		CAN_Send_Data[i] = data[i];

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Reset_Command(CAN_HandleTypeDef *_hcan)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = CAN_SYSTEM_RESET_CMD;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void Send_Power_Data(CAN_HandleTypeDef *_hcan, uint16_t Chassis_power_buffer, uint16_t Chassis_power_limit)
{
	if (Chassis_power_limit >= 10240)
		Chassis_power_limit /= 256;
	if (Chassis_power_limit >= 200)
		Chassis_power_limit /= 5;
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;

	TX_MSG.StdId = 0x302;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = Chassis_power_buffer >> 8;
	CAN_Send_Data[1] = Chassis_power_buffer;
	CAN_Send_Data[2] = Chassis_power_limit >> 8;
	CAN_Send_Data[3] = Chassis_power_limit;
	CAN_Send_Data[4] = 0;
	CAN_Send_Data[5] = 0;
	CAN_Send_Data[6] = 0;
	CAN_Send_Data[7] = 0;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void SendAerialData(CAN_HandleTypeDef *_hcan, float *X, float *Y, uint8_t *KeyBoard)
{
	static CAN_TxHeaderTypeDef TX_MSG;
	static uint8_t CAN_Send_Data[8];
	uint32_t send_mail_box;
	static uint8_t XData[4];
	static uint8_t YData[4];

	float2u8array(X, XData, TRUE);
	float2u8array(Y, YData, TRUE);

	TX_MSG.StdId = CAN_AERIAL_DATA_1;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = XData[0];
	CAN_Send_Data[1] = XData[1];
	CAN_Send_Data[2] = XData[2];
	CAN_Send_Data[3] = XData[3];
	CAN_Send_Data[4] = YData[0];
	CAN_Send_Data[5] = YData[1];
	CAN_Send_Data[6] = YData[2];
	CAN_Send_Data[7] = YData[3];

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0)
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);

	TX_MSG.StdId = CAN_AERIAL_DATA_2;
	TX_MSG.IDE = CAN_ID_STD;
	TX_MSG.RTR = CAN_RTR_DATA;
	TX_MSG.DLC = 0x08;
	CAN_Send_Data[0] = map_interactivity.commd_keyboard;
	CAN_Send_Data[1] = 0;
	CAN_Send_Data[2] = 0;
	CAN_Send_Data[3] = 0;
	CAN_Send_Data[4] = 0;
	CAN_Send_Data[5] = 0;
	CAN_Send_Data[6] = 0;
	CAN_Send_Data[7] = 77;

	while (!((_hcan->State == HAL_CAN_STATE_READY) || (_hcan->State == HAL_CAN_STATE_LISTENING)))
	{
	}
	while (HAL_CAN_GetTxMailboxesFreeLevel(_hcan) == 0) // ����������䶼�����˾͵�һ�����ֱ������ĳ���������
	{
	}
	/* Check Tx Mailbox 1 status */
	if ((_hcan->Instance->TSR & CAN_TSR_TME0) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX0;
	}
	/* Check Tx Mailbox 1 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME1) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX1;
	}

	/* Check Tx Mailbox 2 status */
	else if ((_hcan->Instance->TSR & CAN_TSR_TME2) != 0U)
	{
		send_mail_box = CAN_TX_MAILBOX2;
	}
	HAL_CAN_AddTxMessage(_hcan, &TX_MSG, CAN_Send_Data, &send_mail_box);
}

void float2u8array(float *FloatData, uint8_t *u8Array, uint8_t Key)
{
	static uint8_t TempU8Arr[4];

	*(float *)TempU8Arr = *FloatData;
	if (Key == TRUE)
	{
		u8Array[0] = TempU8Arr[0];
		u8Array[1] = TempU8Arr[1];
		u8Array[2] = TempU8Arr[2];
		u8Array[3] = TempU8Arr[3];
	}
	else
	{
		u8Array[3] = TempU8Arr[0];
		u8Array[2] = TempU8Arr[1];
		u8Array[1] = TempU8Arr[2];
		u8Array[0] = TempU8Arr[3];
	}
}