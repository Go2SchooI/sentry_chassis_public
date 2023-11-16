#include "ui_task.h"

static void Show_UI_Init(void);
static void Show_UI(void);

static void Layer1(uint8_t operate);
static void Layer2(uint8_t operate);
static void Layer3(uint8_t operate);
static void Layer4(uint8_t operate);
static void Layer5(uint8_t operate);
static void Layer6(uint8_t operate);
static void Layer7(uint8_t operate);

graphic_data_struct_t graphic[7];
client_frame_t Char;
client_char_t Char2;
client_frame_t Char3;
client_frame_t Char4;
static float dt = 0, t = 0;
float uiCostTime = 0, uiTimeStamp = 0;
uint32_t UI_DWT_Count = 0, ui_DWT_Cost = 0;

int8_t flg = 0;

uint16_t remain_HP;
uint8_t hunt_buff;
uint8_t client_send_buff[30];
uint8_t client_send_buff_0[30];
uint16_t x[3], y[3], end_x[2], end_y[2];
uint16_t width;
uint8_t color;
uint16_t rect_startX = 706;
uint16_t rect_startY = 820;
uint16_t rect_endX = 1214;
uint16_t rect_endY = 840;
uint16_t HP_startX = 710;
uint16_t HP_startY = 830;
uint16_t HP_endX = 890;
uint16_t HP_endY = 830;
uint16_t HIT_startX = 890;
uint16_t HIT_startY = 830;
uint16_t HIT_endX = 890 - 100;
uint16_t HIT_endY = 830;
uint8_t HP_Width = 12;
uint8_t HIT_Width = 8;

uint16_t rect_Aim_startX = 0;
uint16_t rect_Aim_startY = 0;
uint16_t rect_Aim_endX = 0;
uint16_t rect_Aim_endY = 0;
uint16_t rect_Aimpre_startX = 0;
uint16_t rect_Aimpre_startY = 0;
uint16_t rect_Aimpre_endX = 0;
uint16_t rect_Aimpre_endY = 0;

uint16_t robot_state_startX[5] = {1550, 1550, 1550, 0, 0};
uint16_t robot_state_startY[5] = {590, 560, 530, 0, 0};
uint16_t robot_state_endX[5] = {1620, 1600, 1580, 0, 0};
uint16_t robot_state_endY[5] = {590, 560, 530, 0, 0};
uint8_t robot_width[3] = {0, 0, 0};
#if INFANTRY_ID == 1

uint16_t Start_x1[7] = {900, 850, 900, 945, 1000, 1050, 0};
uint16_t Start_y1[7] = {400, 400, 400, 400, 400, 400, 0};
uint16_t End_x1[7] = {1050, 850, 900, 945, 1000, 1050, 0};
uint16_t End_y1[7] = {400, 410, 410, 410, 410, 410, 0};

uint16_t Start_x2[7] = {750, 750, 850, 945, 1050, 1150, 0};
uint16_t Start_y2[7] = {350, 350, 350, 350, 350, 350, 0};
uint16_t End_x2[7] = {1150, 750, 850, 945, 1050, 1150, 0};
uint16_t End_y2[7] = {350, 360, 360, 360, 360, 360, 0};

uint16_t Center_x_15[5] = {960, 960, 960, 960, 960};
uint16_t Center_y_15[5] = {430, 420, 400, 290, 245};
uint16_t Radius_15[5] = {3, 3, 3, 3, 3}; // 3

uint16_t Center_x_30[5] = {945, 945, 945, 945, 945};
uint16_t Center_y_30[5] = {430, 420, 400, 290, 245};
uint16_t Radius_30[5] = {3, 3, 3, 3, 3}; // 3

#elif INFANTRY_ID == 2
uint16_t Start_x1[7] = {850, 850, 900, 945, 1000, 1050, 0};
uint16_t Start_y1[7] = {400, 400, 400, 400, 400, 400, 0};
uint16_t End_x1[7] = {1050, 850, 900, 945, 1000, 1050, 0};
uint16_t End_y1[7] = {400, 410, 410, 410, 410, 410, 0};

uint16_t Start_x2[7] = {750, 750, 850, 945, 1050, 1150, 0};
uint16_t Start_y2[7] = {350, 350, 350, 350, 350, 350, 0};
uint16_t End_x2[7] = {1150, 750, 850, 945, 1050, 1150, 0};
uint16_t End_y2[7] = {350, 360, 360, 360, 360, 360, 0};

uint16_t Center_x_15[5] = {960, 960, 960, 960, 960};
uint16_t Center_y_15[5] = {430, 420, 400, 290, 245};
uint16_t Radius_15[5] = {3, 3, 3, 3, 3}; // 3

uint16_t Center_x_30[5] = {945, 945, 945, 945, 945};
uint16_t Center_y_30[5] = {430, 420, 400, 290, 245};
uint16_t Radius_30[5] = {3, 3, 3, 3, 3}; // 3
#elif INFANTRY_ID == 3
uint16_t Start_x1[7] = {850, 850, 900, 945, 1000, 1050, 0};
uint16_t Start_y1[7] = {400, 400, 400, 400, 400, 400, 0};
uint16_t End_x1[7] = {1050, 850, 900, 945, 1000, 1050, 0};
uint16_t End_y1[7] = {400, 410, 410, 410, 410, 410, 0};

uint16_t Start_x2[7] = {750, 750, 850, 945, 1050, 1150, 0};
uint16_t Start_y2[7] = {350, 350, 350, 350, 350, 350, 0};
uint16_t End_x2[7] = {1150, 750, 850, 945, 1050, 1150, 0};
uint16_t End_y2[7] = {350, 360, 360, 360, 360, 360, 0};

uint16_t Center_x_15[5] = {960, 960, 960, 960, 960};
uint16_t Center_y_15[5] = {430, 420, 400, 290, 245};
uint16_t Radius_15[5] = {3, 3, 3, 3, 3}; // 3

uint16_t Center_x_30[5] = {945, 945, 945, 945, 945};
uint16_t Center_y_30[5] = {430, 420, 400, 290, 245};
uint16_t Radius_30[5] = {3, 3, 3, 3, 3}; // 3
#elif INFANTRY_ID == 4
uint16_t Start_x1[7] = {850, 850, 900, 945, 1000, 1050, 0};
uint16_t Start_y1[7] = {400, 400, 400, 400, 400, 400, 0};
uint16_t End_x1[7] = {1050, 850, 900, 945, 1000, 1050, 0};
uint16_t End_y1[7] = {400, 410, 410, 410, 410, 410, 0};

uint16_t Start_x2[7] = {750, 750, 850, 945, 1050, 1150, 0};
uint16_t Start_y2[7] = {350, 350, 350, 350, 350, 350, 0};
uint16_t End_x2[7] = {1150, 750, 850, 945, 1050, 1150, 0};
uint16_t End_y2[7] = {350, 360, 360, 360, 360, 360, 0};

uint16_t Center_x_15[5] = {960, 960, 960, 960, 960};
uint16_t Center_y_15[5] = {430, 420, 400, 290, 245};
uint16_t Radius_15[5] = {3, 3, 3, 3, 3}; // 3

uint16_t Center_x_30[5] = {945, 945, 945, 945, 945};
uint16_t Center_y_30[5] = {430, 420, 400, 290, 245};
uint16_t Radius_30[5] = {3, 3, 3, 3, 3}; // 3
#endif

void UI_Task(void)
{
	dt = DWT_GetDeltaT(&UI_DWT_Count);
	t += dt;
	uiTimeStamp = DWT_GetDeltaT(&ui_DWT_Cost);

	static uint8_t state = 0;

	switch (state)
	{
	case 0:
		state++;
		Show_UI_Init();
		break;

	case 1:
		state = 0;
		Show_UI();
		break;

	default:
		state = 0;
		break;
	}

	uiTimeStamp = DWT_GetDeltaT(&ui_DWT_Cost);
	uiCostTime = uiTimeStamp;
}

static void Show_UI_Init(void)
{
	static uint8_t state = 0;

	switch (state)
	{
	case 0:
		state++;
		Layer1(GRAPHIC_ADD);
		break;

	case 1:
		state++;
		Layer2(GRAPHIC_ADD);
		break;

	case 2:
		state++;
		Layer3(GRAPHIC_ADD);
		break;

	case 3:
		state++;
		Layer4(GRAPHIC_ADD);
		break;

	case 4:
		state++;
		Layer5(GRAPHIC_ADD);
		break;

	case 5:
		state++;
		Layer6(GRAPHIC_ADD);
		break;
	case 6:
		state = 0;
		Layer7(GRAPHIC_ADD);

		break;

	default:
		state = 0;
		break;
	}
}

static void Show_UI(void)
{
	static uint8_t state = 0;
	switch (state)
	{
	case 0:
		state++;
		Layer1(GRAPHIC_MODIFY);
		break;

	case 1:
		state++;
		Layer2(GRAPHIC_MODIFY);
		break;

	case 2:
		state++;
		Layer3(GRAPHIC_MODIFY);
		break;
	case 3:
		state++;
		Layer4(GRAPHIC_MODIFY);

		break;
	case 4:
		state = 0;
		if (flg > 10)
		{
			flg = 0;
		}
		Layer5(GRAPHIC_MODIFY);
		flg++;
		break;

	default:
		state = 0;
		break;
	}
}

static void Layer1(uint8_t operate)
{

	if (Gimbal_Data.isFricOn == 1)
	{
		robot_width[0] = 8;
	}
	else
	{
		robot_width[0] = 0;
	}
	if (Gimbal_Data.isLidOpen == 0)
	{
		robot_width[1] = 8;
	}
	else
	{
		robot_width[1] = 0;
	}
	if (Gimbal_Data.MiniPC_state == 1)
	{
		robot_width[2] = 8;
	}
	else
	{
		robot_width[2] = 0;
	}

	for (int i = 0; i < 3; i++)
	{

		Line_Generate(&graphic[i], 1, i, operate, robot_state_startX[i], robot_state_startY[i], robot_state_endX[i], robot_state_endY[i], robot_width[i], 4);
	}
	client_draw(JudgeUSART, 7, graphic, robot_state.robot_id, robot_state.robot_id + 256);
}
static void Layer2(uint8_t operate)
{
	memset(client_send_buff, 0, sizeof(client_send_buff));

	Line_Generate(&graphic[0], 2, 0, operate,
				  (uint16_t)(960 - 80 * arm_sin_f32(Chassis.FollowTheta / RADIAN_COEF)), (uint16_t)(540 + 80 * arm_cos_f32(Chassis.FollowTheta / RADIAN_COEF)),
				  (uint16_t)(960 - 150 * arm_sin_f32(Chassis.FollowTheta / RADIAN_COEF)), (uint16_t)(540 + 150 * arm_cos_f32(Chassis.FollowTheta / RADIAN_COEF)),
				  2, CLIENT_RedOrBlue);

	if (Gimbal_Data.myteam == RED)
	{
		switch (Gimbal_Data.tgt_ID)
		{
		case 0:
			remain_HP = 0;
			break;
		case 1:
			remain_HP = game_robot_HP.blue_1_robot_HP;
			break;
		case 2:
			remain_HP = game_robot_HP.blue_2_robot_HP;
			break;
		case 3:
			remain_HP = game_robot_HP.blue_3_robot_HP;
			break;
		case 4:
			remain_HP = game_robot_HP.blue_4_robot_HP;
			break;
		case 5:
			remain_HP = game_robot_HP.blue_5_robot_HP;
			break;
		default:
			break;
		}
	}
	else if (Gimbal_Data.myteam == BLUE)
	{
		switch (Gimbal_Data.tgt_ID)
		{
		case 0:
			remain_HP = 0;
			break;
		case 1:
			remain_HP = game_robot_HP.red_1_robot_HP;
			break;
		case 2:
			remain_HP = game_robot_HP.red_2_robot_HP;
			break;
		case 3:
			remain_HP = game_robot_HP.red_3_robot_HP;
			break;
		case 4:
			remain_HP = game_robot_HP.red_4_robot_HP;
			break;
		case 5:
			remain_HP = game_robot_HP.red_5_robot_HP;
			break;
		default:
			break;
		}
	}
	if ((event_data.event_type & (0x01 << 4)) == 1)
	{
		hunt_buff = 5;
	}
	else if ((event_data.event_type & (0x01 << 5)) == 1)
	{
		hunt_buff = 10;
	}
	else
	{
		hunt_buff = 0;
	}
	HP_endX = HP_startX + remain_HP;
	HIT_startX = HP_endX;
	HIT_endX = HIT_startX - power_heat_data_t.shooter_id1_17mm_cooling_heat / 10 *
								(10 + hunt_buff * (buff_musk.power_rune_buff & (0x01 << 3)));
	rect_Generate(&graphic[1], 2, 1, operate, rect_startX, rect_startY, rect_endX, rect_endY, 3, CLIENT_GREEN);
	Line_Generate(&graphic[2], 2, 2, operate, HP_startX, HP_startY, HP_endX, HP_endY, HP_Width, CLIENT_GREEN);
	Line_Generate(&graphic[3], 2, 3, operate, HIT_startX, HIT_startY, HIT_endX, HIT_endY, HIT_Width, CLIENT_RedOrBlue);
	rect_Generate(&graphic[4], 2, 4, operate, rect_startX, rect_startY, rect_endX, rect_endY, 3, CLIENT_GREEN);
	rect_Generate(&graphic[5], 2, 5, operate, rect_startX, rect_startY, rect_endX, rect_endY, 3, CLIENT_GREEN);
#if INFANTRY_ID == 4
	switch (Gimbal_Date.shooter_id)
	{
	case 0:
	{
		rect_Generate(&graphic[6], 2, 6, operate, 0, 0, 0, 0, 0, CLIENT_RedOrBlue);
	}
	case 1:
	{
		// shooter1
		rect_Generate(&graphic[6], 2, 6, operate, 450, 475, 570, 605, 2, CLIENT_RedOrBlue);
		break;
	}
	case 2:
	{
		// shooter2
		rect_Generate(&graphic[6], 2, 6, operate, 1460, 475, 1350, 605, 2, CLIENT_RedOrBlue);
	}
	}
#endif
	client_draw(JudgeUSART, 7, graphic, robot_state.robot_id, robot_state.robot_id + 256);
}
static void Layer3(uint8_t operate)
{
	uint8_t
		lens[3];

	memset(client_send_buff, 0, sizeof(client_send_buff));

	switch (Chassis.Mode)
	{
	case Follow_Mode:
		memcpy(client_send_buff, "Follow\n", strlen("Follow\n"));
		lens[0] = strlen("Follow\n");
		break;

	case Silence_Mode:
		memcpy(client_send_buff, "Silence\n", strlen("Silencen\n"));

		lens[0] = strlen("Silencen\n");
		break;

	case Spinning_Mode:
		memcpy(client_send_buff, "Spinning\n", strlen("Spinning\n"));
		lens[0] = strlen("Spinning\n");
		break;

	case Side_Mode:
		memcpy(client_send_buff, "Side\n", strlen("Side\n"));
		lens[0] = strlen("Side\n");
		break;
	}
	switch (Gimbal_Data.Gimbal_mode)
	{
	case 0:
	{
		lens[1] = strlen("NO_AIM\n");
		memcpy(client_send_buff + lens[0], "NO_AIM\n", strlen("NO_AIM\n"));
		break;
	}
	case 1:
	{
		lens[1] = strlen("AUTO_AIM\n");
		memcpy(client_send_buff + lens[0], "AUTO_AIM\n", strlen("AUTO_AIM\n"));
		break;
	}
	case 2:
	{
		lens[1] = strlen("RUNE_MIN\n");
		memcpy(client_send_buff + lens[0], "RUNE_MIN\n", strlen("RUNE_MIN\n"));
		break;
	}
	case 3:
	{
		lens[1] = strlen("MAX_RUNE\n");
		memcpy(client_send_buff + lens[0], "MAX_RUNE\n", strlen("MAX_RUNE\n"));
		break;
	}
	}
	if (Chassis.Mode == Follow_Mode)
	{
		// if (abs(Chassis.HeadingFlag) == 1)
		// {
		// 	lens[2] = strlen("Low\n");
		// 	memcpy(client_send_buff + lens[0] + lens[1], "Low\n", strlen("Low\n"));
		// }
		// else
		// {
		// 	lens[2] = strlen("High\n");
		// 	memcpy(client_send_buff + lens[0] + lens[1], "High\n", strlen("High\n"));
		// }
		;
	}
	else
	{
		lens[2] = strlen("No\n");
		memcpy(client_send_buff + lens[0] + lens[1], "No\n", strlen("No\n"));
	}
	if ((Chassis.Mode == Spinning_Mode) &&
		((remote_control.key_code & Key_SHIFT) || (remote_control.key_code & Key_CTRL)) &&
		((remote_control.key_code & Key_W) || (remote_control.key_code & Key_A) || (remote_control.key_code & Key_S) || (remote_control.key_code & Key_D)))
		client_draw_char(JudgeUSART, &Char2, 3, operate, 600, 580, 20, robot_state.robot_id, robot_state.robot_id + 256, client_send_buff, 30, CLIENT_Amaranth);
	else
		client_draw_char(JudgeUSART, &Char2, 3, operate, 600, 580, 20, robot_state.robot_id, robot_state.robot_id + 256, client_send_buff, 30, CLIENT_RedOrBlue);
}
static void Layer4(uint8_t operate)
{
	memset(client_send_buff, 0, sizeof(client_send_buff));

	memset(client_send_buff_0, 0, sizeof(client_send_buff_0));
	sprintf((char *)client_send_buff_0, "CAP %.1f\nPit %.1f\nPwr %.1f\n", ina226[0].Bus_Voltage, Gimbal.PitchMotor.AngleInDegree, Chassis.PowerControl.Power_Calculation_Clipping);
	client_draw_char_2(JudgeUSART, &Char3, 1, operate, 180, 600, 20, robot_state.robot_id, robot_state.robot_id + 256, client_send_buff_0, 30, CLIENT_RedOrBlue);
}
static void Layer5(uint8_t operate)
{

	memset(client_send_buff, 0, sizeof(client_send_buff));
	if (flg == 1)
	{
		int8_t a;
		uint8_t lens;
		memcpy(client_send_buff, "EnemyID ", strlen("EnemyID "));
		lens = strlen("EnemyID ");
		a = Gimbal_Data.tgt_ID + '0';
		memcpy(client_send_buff + lens, &a, 1);
		client_draw_char_2(JudgeUSART, &Char4, 4, operate, 500, 840, 20, robot_state.robot_id, robot_state.robot_id + 256, client_send_buff, 30, CLIENT_Yellow);
	}
	if (flg == 8)
	{
		static uint8_t length[5];
		memset(client_send_buff, 0, sizeof(client_send_buff));
		length[0] = strlen("Fric\n");
		memcpy(client_send_buff, "Fric\n", length[0]);
		length[1] = strlen("Lid\n");
		memcpy(client_send_buff + length[0], "Lid\n", length[1]);
		length[2] = strlen("PC\n");
		memcpy(client_send_buff + length[0] + length[1], "PC\n", length[2]);
		client_draw_char_2(JudgeUSART, &Char4, 5, GRAPHIC_ADD, 1550, 600, 20, robot_state.robot_id, robot_state.robot_id + 256, client_send_buff, 30, CLIENT_RedOrBlue);
	}
}
static void Layer6(uint8_t operate)
{
}
static void Layer7(uint8_t operate)
{
}
void UI_Buff_Generate(float a, float b, float c, uint8_t *data)
{
	uint8_t len[3];
	uint8_t float_data[10];
	uint8_t length = 0;

	memcpy(data, "Cap=", strlen("Cap="));
	length += strlen("Cap=");
	len[0] = float_to_char(a, float_data);
	memcpy(data + length, float_data, len[0]);
	length += len[0];

	memcpy(data + length, "Pit=", strlen("Pit="));
	length += strlen("Pit=");
	len[1] = float_to_char(b, float_data);
	memcpy(data + length, float_data, len[1]);
	length += len[1];

	memcpy(data + length, "Pwr=", strlen("Pwr="));
	length += strlen("Pwr=");
	len[2] = float_to_char(c, float_data);
	memcpy(data + length, float_data, len[2]);
	length += len[2];
}

uint8_t float_to_char(float floatdata, uint8_t *buffer)
{
	int32_t slope;
	int32_t temp;
	int8_t i, j;
	uint8_t len = 0;

	slope = (int32_t)(floatdata * 100 + 0.5f);
	if (slope < 0)
	{
		buffer[len] = '-';
		slope = -slope;
		len += 1;
	}
	temp = slope;

	for (i = 0; temp != 0; i++)
		temp /= 10;
	temp = slope;

	for (j = i; j >= 0; j--)
	{
		buffer[len + j] = temp % 10 + '0';
		temp /= 10;
		if ((i - j) == 1)
		{
			buffer[len + j - 1] = '.';
			j -= 1;
		}
	}
	len += i + 1;

	buffer[len] = '\n';
	len += 1;
	return len;
}
