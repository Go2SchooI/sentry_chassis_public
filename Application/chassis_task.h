#ifndef _CHASSIS_TASK_H
#define _CHASSIS_TASK_H

#include "includes.h"
#include "kalman_filter.h"
#include "motor.h"

// #define Chassis_Use_IMU
#define Chassis_Vr_FFC_MAXOUT 800
#define Chassis_Vr_FCC_LPF 0.001
#define Chassis_Vr_C0 1
#define Chassis_Vr_C1 1
#define Chassis_Vr_C2 1
#define Change_Boundary 500

#define CHASSIS_TASK_PERIOD 2

#define UseAttitudeControl
#define GRAVUTYCENTER_ADJUSTMENT 1.0f;

#define POWER_GAIN 0.95f // 功率修正系数
#define LOWVOLTAGE_GAIN 0.7

#define ENABLE_SPINNING

#define FOLLOW_DEAD_BAND 10.0f

#define VELOCITY_RATIO 3
#define RC_STICK_ROTATE_RATIO 0.72
#define RC_MOUSE_ROTATE_RATIO 0.5f

#ifndef STD_RADIAN
#define STD_RADIAN(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif

#define wheel_radius 76.00f
#define pi 3.1415926f
#define Kx 0.250f
#define Ky 0.250f

#define YAW_REDUCTION_RATIO 1 / 1.0f // YAW轴电机的传动系统的减速比

#define YAW_REDUCTION_CORRECTION_ANGLE 90.0f

#ifdef ARM_MATH_DSP
#define user_cos arm_cos_f32
#define user_sin arm_sin_f32
#else
#define user_cos cosf
#define user_sin sinf
#endif

#define FOLLOW_THETA_LEN 200

typedef struct
{
  float FollowTheta;
  float TimeStamp_ms;
} thetaFrame_t;

typedef struct
{
  float PowerScale;                 /*功率范围*/
  float Capacitor_Voltage;          /*电容电压*/
  float Power_Limit;                /*功率限制*/
  float Power_Calculation;          /*限幅之前的功率*/
  float Power_Calculation_Clipping; /*限幅之后的实际功率*/
  float Power_correction_gain;      /*功率修正系数（同POWER_GAIN，防止机器人超功率）*/
  float LowVoltage_Gain;            /*电压修正系数（防止电容电压过低）*/
} Chassis_PowerControl_t;

typedef struct
{
  int16_t vx;
  int16_t vy;
  int32_t theta;

  int16_t FollowPosition[3];
  float SpinningPosition[3];

  uint8_t FollowTargetStatus;
  uint8_t IsSpinning;

  uint8_t MiniPC_Update;

  PID_t PID_Follow;
} MiniPC_ControlFrame;

typedef struct _Chassis_t
{
  int16_t Vx, Vy, Vr; /*XY轴速度与角速度 */
  int16_t LastVr;
  float VxTransfer, VyTransfer; /*用于云台坐标系与底盘坐标系的转换*/
  float AngularVelocity;        /*角速度*/

  float VelocityRatio;      /*调整小陀螺在总功率中的占比*/
  float rcStickRotateRatio; /*摇杆运动与电机间的比例系数*/
  float rcMouseRotateRatio; /*鼠标运动与电机间的比例系数*/

  KalmanFilter_t ChassisMotionEst;
  float V1, V2, V3, V4;

  float Vx_is_Chassis, Vy_is_Chassis; /*底盘坐标系下反解出的车速度*/
  float Vx_is_Body, Vy_is_Body;       /*云台坐标系下反解出的车速度*/
  float Ax_Chassis, Ay_Chassis;       /*底盘坐标系下在X,Y方向上的加速度*/
  float Ax_Body, Ay_Body;             /*云台坐标系下在X,Y方向上的加速度*/
  float V1_is, V2_is, V3_is, V4_is;   /*反解出四个轮子的速度*/
  float Vx_is, Vy_is, Vr_is;
  float VxTransfer_is, VyTransfer_is;

  float Position[2];
  float V_Position[2];
  float Velocity[2];
  float Accel[2];

  float x;
  float y;

  uint8_t status;
  uint8_t Mode;

  int YawCorrectionScale; /*YAW修正后的范围*/
  uint32_t ChassisSwitchTick;

  float GravityCenter_Adjustment; /*云台重心修正*/
  float Theta;
  float TotalTheta;
  float FollowTheta;

  float Heading;
  float cap_energy;
  float energy_percentage;

  Chassis_PowerControl_t PowerControl;

  Motor_t ChassisMotor[4];
  PID_t RotateFollow;

  TD_t ChassisVxTD;
  TD_t ChassisVyTD;
  TD_t SpinningTD;

  int16_t FollowXVelocity1000;
  int16_t FollowYVelocity1000;
  float FollowXVelocity;
  float FollowYVelocity;
  float FollowCoef;
  uint8_t geto_des;
  uint8_t FlagFollow;
  uint8_t IsSpining;

  PID_t SpinningValid;
  int16_t posX1000;
  int16_t posY1000;
  int16_t posZ1000;
  float posX;
  float posY;
  float posZ;
  float spinnig_center[2];
  int16_t PlanX1000;
  int16_t PlanY1000;
  float PlanX;
  float PlanY;
  thetaFrame_t thetaFrame[FOLLOW_THETA_LEN];
} Chassis_t;

enum
{
  Follow_Mode = 0, // 跟随
  Spinning_Mode,   // 小陀螺
  Side_Mode,       // 横着走
  Silence_Mode,    // 静止模式
  Count_Mode,
  Debug_Mode,
};

enum
{
  stay_in_place,
  spin_in_place,
  go_to_des,
  reach_des,
  go_back_start,
  reach_start,
  go_to_area,
  reach_area,
};

void Chassis_Control(void);
void Chassis_Init(void);
void Callback_Follow_Handle(MiniPC_ControlFrame *MiniPC_CtrlFrame, uint8_t *buff);
void Insert_thetaFrame(thetaFrame_t *theta_frame, float follow_theta, uint32_t time_stamp_ms);
uint16_t Find_thetaFrame(thetaFrame_t *theta_frame, uint32_t match_time_stamp_ms);
void SendMapData(void);
void AerialKeyBoardCmd(void);

extern Chassis_t Chassis;
extern MiniPC_ControlFrame MiniPC_CtrlFrame;
extern uint8_t aimassist_online;

#endif
