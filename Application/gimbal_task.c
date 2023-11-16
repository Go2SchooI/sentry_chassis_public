#include "gimbal_task.h"
#include "remote_control.h"
#include "includes.h"
#include "math.h"

Gimbal_t Gimbal = {0};
Gimbal_Data_t Gimbal_Data = {0};
void Gimbal_Init(void)
{
	Gimbal.YawMotor.zero_offset = YAW_MOTOR_ZERO_OFFSET;
	Gimbal.PitchMotor.zero_offset = PITCH_MOTOR_ZERO_OFFSET;

	//    Gimbal.PitchMotor.Direction = NEGATIVE;
	//				Gimbal.YawMotor.Direction = NEGATIVE;

	if (robot_state.robot_id <= 7)
		Gimbal_Data.myteam = RED;
	else
		Gimbal_Data.myteam = BLUE;
}
