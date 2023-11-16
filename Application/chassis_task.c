#include "chassis_task.h"

uint16_t outpost_HP, sentry_HP;

uint8_t count_press = 0;
static float dt = 0,
             t = 0;

float ChassisMotionEst_F[36] = {
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1};
float ChassisMotionEst_P[36];
float ChassisMotionEst_Pinit[36] = {
    1000, 0.1, 0.1, 0.1, 0.1, 0.1,
    0.1, 10000, 0.1, 0.1, 0.1, 0.1,
    0.1, 0.1, 10000, 0.1, 0.1, 0.1,
    0.1, 0.1, 0.1, 1000, 0.1, 0.1,
    0.1, 0.1, 0.1, 0.1, 10000, 0.1,
    0.1, 0.1, 0.1, 0.1, 0.1, 10000};
float ChassisMotionEst_Sigma[2] = {100, 100};
float ChassisMotionEst_Q[36] = {
    10, 0, 0, 0, 0, 0,
    0, 1000, 0, 0, 0, 0,
    0, 0, 10, 0, 0, 0,
    0, 0, 0, 10, 0, 0,
    0, 0, 0, 0, 1000, 0,
    0, 0, 0, 0, 0, 10};
float ChassisMotionEst_R[16] = {
    100000, 0, 0, 0,
    0, 100, 0, 0,
    0, 0, 100000, 0,
    0, 0, 0, 100};
float ChassisMotionEst_H[24] = {
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1};
float ChiSquare;
float xhat_data_obsv[6];

static void Chassis_Get_Theta(void);     // 获取底盘与云台偏角
static void Chassis_Set_Mode(void);      // 设置底盘运动模式
static void Chassis_Get_CtrlValue(void); // 处理来自摇杆与键盘的控制数据
static void Chassis_Set_Control(void);   // 底盘运动解算以及PID计算
static void Send_Chassis_Current(void);  // 发送底盘电机控制电流
static float Max_4(float num1, float num2, float num3, float num4);
static void abs_control(void);
static void Chassis_Power_Limit(void);
static void Chassis_Power_Exp(void); // 计算可能的底盘功率
static void Velocity_MAXLimit(void); // 最大速度限制
static void ChassisMotionEst_Init(void);
static void ChassisMotionEst_Update(float dt);

void Chassis_Init(void)
{
    Chassis.VelocityRatio = VELOCITY_RATIO;

    Chassis.rcStickRotateRatio = 0;

    Chassis.GravityCenter_Adjustment = GRAVUTYCENTER_ADJUSTMENT;

    Chassis.PowerControl.Power_correction_gain = POWER_GAIN;
    Chassis.PowerControl.LowVoltage_Gain = LOWVOLTAGE_GAIN;

    TD_Init(&Chassis.ChassisVxTD, 1000, 0.01f);
    TD_Init(&Chassis.ChassisVyTD, 1000, 0.01f);

    Chassis.TotalTheta = (Gimbal.YawMotor.total_angle - Gimbal.YawMotor.zero_offset) * ENCODERCOEF * YAW_REDUCTION_RATIO;

    Chassis.Theta = loop_float_constrain((Chassis.TotalTheta + Gimbal.YawMotor.offset_angle * ENCODERCOEF * YAW_REDUCTION_RATIO), -180, 180);

    Chassis.YawCorrectionScale = 0;

    // 电机速度环PID计算
    for (uint8_t i = 0; i < 4; i++)
    {
        PID_Init(
            &Chassis.ChassisMotor[i].PID_Velocity, 16384, 16384, 0, 15, 30, 0, 500, 100, 0.005, 0, 1, Integral_Limit | OutputFilter);
        Chassis.ChassisMotor[i].Max_Out = 12000.0f;
    }
    // 底盘跟随云台PID初始化
    PID_Init(&Chassis.RotateFollow, 300, 100, 0, 8, 0, 0, 0,
             0, 0, 0, 1,
             Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter);

    PID_Init(&Chassis.SpinningValid, 50, 30, 0, 0.1, 0, 0, 0,
             0, 0, 0, 1,
             Integral_Limit | Derivative_On_Measurement | OutputFilter | DerivativeFilter);

    TD_Init(&Chassis.SpinningTD, 100000, 0.001);

    Matrix_Init(&Chassis.ChassisMotionEst.H, 4, 6, (float *)Chassis.ChassisMotionEst.H_data);
    Matrix_Init(&Chassis.ChassisMotionEst.HT, 6, 4, (float *)Chassis.ChassisMotionEst.HT_data);
    ChassisMotionEst_Init();
    Chassis.IsSpining = 0;
}

static void ChassisMotionEst_Init(void)
{
    // Chassis.ChassisMotionEst.UseAutoAdjustment = TRUE;
    Kalman_Filter_Init(&Chassis.ChassisMotionEst, 6, 0, 4);
    Chassis.ChassisMotionEst.MeasurementMap[0] = 2;
    Chassis.ChassisMotionEst.MeasurementMap[1] = 3;
    Chassis.ChassisMotionEst.MeasurementMap[2] = 5;
    Chassis.ChassisMotionEst.MeasurementMap[3] = 6;
    Chassis.ChassisMotionEst.MeasurementDegree[0] = 1;
    Chassis.ChassisMotionEst.MeasurementDegree[1] = 1;
    Chassis.ChassisMotionEst.MeasurementDegree[2] = 1;
    Chassis.ChassisMotionEst.MeasurementDegree[3] = 1;
    // Chassis.ChassisMotionEst.MatR_DiagonalElements[0] = 100;
    // Chassis.ChassisMotionEst.MatR_DiagonalElements[1] = 100;
    // Chassis.ChassisMotionEst.MatR_DiagonalElements[2] = 100;
    // Chassis.ChassisMotionEst.MatR_DiagonalElements[3] = 100;
    memcpy(Chassis.ChassisMotionEst.F_data, ChassisMotionEst_F, sizeof(ChassisMotionEst_F));
    memcpy(Chassis.ChassisMotionEst.P_data, ChassisMotionEst_Pinit, sizeof(ChassisMotionEst_Pinit));
    memcpy(Chassis.ChassisMotionEst.Q_data, ChassisMotionEst_Q, sizeof(ChassisMotionEst_Q));
    memcpy(Chassis.ChassisMotionEst.R_data, ChassisMotionEst_R, sizeof(ChassisMotionEst_R));
    memcpy(Chassis.ChassisMotionEst.H_data, ChassisMotionEst_H, sizeof(ChassisMotionEst_H));
    // Chassis.ChassisMotionEst.User_Func0_f = ChassisMotionEst_Tuning;
    // Chassis.ChassisMotionEst.User_Func3_f = ChassisMotionEst_ChiSquare_Test;
}

void Chassis_Control(void)
{
    dt = DWT_GetDeltaT(&Chassis_DWT_Count);
    t += dt;
    ChassisMotionEst_Update(dt);
    // 获取底盘与云台偏角
    Chassis_Get_Theta();
    // 设置底盘运动模式
    Chassis_Set_Mode();
    // 处理来自摇杆与键盘的控制数据
    Chassis_Get_CtrlValue();
    // 底盘运动解算以及PID计算
    Chassis_Set_Control();
    // 发送底盘电机控制电流
    Send_Chassis_Current();
    // 发送云台手数据
    SendAerialData(&hcan2, &TempAerialX, &TempAerialY, &map_interactivity.commd_keyboard);
}

static void Chassis_Get_Theta(void)
{
    // 底盘与云台偏角获取
    Chassis.TotalTheta = (Gimbal.YawMotor.total_angle - Gimbal.YawMotor.zero_offset) * ENCODERCOEF * YAW_REDUCTION_RATIO;

    Chassis.Theta = loop_float_constrain((Chassis.TotalTheta +
                                          Gimbal.YawMotor.offset_angle * ENCODERCOEF * YAW_REDUCTION_RATIO),
                                         -180, 180);

    if (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE))
    {
        Chassis.TotalTheta = 0;
        Chassis.Theta = 0;
    }

    Chassis.FollowTheta = float_deadband(Chassis.Theta, -0.005f, 0.005f);
    Insert_thetaFrame(Chassis.thetaFrame, Chassis.FollowTheta, INS_GetTimeline());
}

static void Chassis_Get_CtrlValue(void)
{
    static float Temp_Vx, Temp_Vy; // 速度的期望值
    static float tempVal;
    static float RC = 0.000001f;

    // 功率限制与实际值不符时
    if (robot_state.chassis_power_limit >= 10240) // 数据包体积的最大值
        robot_state.chassis_power_limit /= 256;
    // 功率限幅
    if (robot_state.chassis_power_limit > 120) // 数据包的体积的最小值
        robot_state.chassis_power_limit = 120;
    // 功率最低限制
    if (robot_state.chassis_power_limit == 0)
        robot_state.chassis_power_limit = 45;

    if (robot_state.chassis_power_limit != 0)
        Chassis.VelocityRatio = float_constrain(robot_state.chassis_power_limit / 10.0f + 2, 3, 7); // 通过power_limit 调整机器人跑得快慢

    Temp_Vx = 0;
    Temp_Vy = 0;
    Temp_Vx += remote_control.ch3;
    Temp_Vy += remote_control.ch4 == -660 ? 0 : remote_control.ch4; // ch4 != -660 ?

    if (remote_control.switch_left == Switch_Up)
    {
        Temp_Vx *= 2;
        Temp_Vy *= 2;
    }

    if (fabsf(Temp_Vx) > 1e-3f) // 上升部分 1e-3f ~~ 0
    {
        if (Chassis.Vx * Temp_Vx < 0)
            Chassis.Vx = 0;

        tempVal = (Temp_Vx - Chassis.Vx) / (RC + dt);
        if (tempVal > Vx_k)
            tempVal = Vx_k;
        else if (tempVal < -Vx_k)
            tempVal = -Vx_k;
        Chassis.Vx += tempVal * dt;
    }
    else
    {
        tempVal = (Temp_Vx - Chassis.Vx) / (0.01f + dt);
        Chassis.Vx += tempVal * dt;
    }

    if (fabsf(Temp_Vy) > 1e-3f)
    {
        if (Chassis.Vy * Temp_Vy < 0)
            Chassis.Vy = 0;
        tempVal = (Temp_Vy - Chassis.Vy) / (RC + dt);
        if (tempVal > Vy_k)
            tempVal = Vy_k;
        else if (tempVal < -Vy_k)
            tempVal = -Vy_k;
        Chassis.Vy += tempVal * dt;
    }
    else
    {
        tempVal = (Temp_Vy - Chassis.Vy) / (0.01f + dt);
        Chassis.Vy += tempVal * dt;
    }

    Chassis.Vx = float_constrain(Chassis.Vx, -400.0f, 400.0f);
    Chassis.Vy = float_constrain(Chassis.Vy, -400.0f, 400.0f);
}

static void Chassis_Set_Mode(void)
{
    static uint16_t LastKeyCode = 0;

    if (is_TOE_Error(CHASSIS_MOTOR1_TOE) && is_TOE_Error(CHASSIS_MOTOR2_TOE) && is_TOE_Error(CHASSIS_MOTOR3_TOE) && is_TOE_Error(CHASSIS_MOTOR4_TOE))
        Chassis.Mode = Silence_Mode;

    // 拨杆切换运动模式
    switch (remote_control.switch_right)
    {
    case Switch_Up: //
        Chassis.Mode = Spinning_Mode;
        //		Chassis.FollowCoef = 0;
        break;

    case Switch_Middle: // All from Keyboard?
        Chassis.Mode = Follow_Mode;
        Chassis.FollowCoef = 0;
        break;

    case Switch_Down: // All from Keyboard?
        Chassis.Mode = Follow_Mode;
        Chassis.FollowCoef = 0;
        break;
    }

    AerialKeyBoardCmd();

    last_game_status = game_status.game_progress;
    LastKeyCode = remote_control.key_code;
}

static void Chassis_Set_Control(void)
{
    static float tempVr, posX100, posY100;
    static float spining_distance, spinnig_center0_temp, spinnig_center1_temp, move_ratio;
    static uint32_t in_pos_count, center_count, edge_count, data_novalid_count, reach_count;
    static uint8_t thetaFrameNum, is_velocity;

    Chassis.PlanX = Chassis.PlanX * 0.1 / (0.1 + dt) + Chassis.PlanX1000 / 10.0f * dt / (0.1 + dt); // 0.0002
    Chassis.PlanY = Chassis.PlanY * 0.1 / (0.1 + dt) + Chassis.PlanY1000 / 10.0f * dt / (0.1 + dt);
    Chassis.posX = Chassis.posX * 0.002 / (0.002 + dt) + Chassis.posX1000 / 10.0f * dt / (0.002 + dt);
    Chassis.posY = Chassis.posY * 0.002 / (0.002 + dt) + Chassis.posY1000 / 10.0f * dt / (0.002 + dt);
    Chassis.posZ = Chassis.posZ1000 / 1000.0f;

    switch (Chassis.Mode)
    {
    case Follow_Mode:
    FOLLOW:
        if (fabsf(Chassis.FollowTheta) < 0.005f) // 判断是否正对头部
            Chassis.Vr = remote_control.ch1 * Chassis.rcStickRotateRatio;
        else
        {
            if (remote_control.switch_left == Switch_Up)
                Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, 0.0f);
            else
            {
                Chassis.Vr = PID_Calculate(&Chassis.RotateFollow, Chassis.FollowTheta, 0.0f) +
                             remote_control.ch1 * Chassis.rcStickRotateRatio;
            }
        }
        Chassis.VxTransfer = user_cos(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vx + Chassis.FollowXVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
                             user_sin(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vy + Chassis.FollowYVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60));
        Chassis.VyTransfer = -user_sin(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vx + Chassis.FollowXVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
                             user_cos(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vy + Chassis.FollowYVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60));

        if (Chassis.Mode == Spinning_Mode)
        {
            Chassis.Vr *= 2;
            tempTime_Sprint = USER_GetTick();
        }
        break;

    case Silence_Mode:
        Chassis.Vr = 0;
        Chassis.VxTransfer = 0;
        Chassis.VyTransfer = 0;
        break;

    case Spinning_Mode:
        if (robot_state.robot_id <= 7) // red
        {
            outpost_HP = game_robot_HP.red_outpost_HP;
            sentry_HP = game_robot_HP.red_7_robot_HP;
        }
        else
        {
            outpost_HP = game_robot_HP.blue_outpost_HP;
            sentry_HP = game_robot_HP.blue_7_robot_HP;
        }

        arm_sqrt_f32((Chassis.posX1000 - Chassis.PlanX) * (Chassis.posX1000 - Chassis.PlanX) +
                         (Chassis.posY1000 - Chassis.PlanY) * (Chassis.posY1000 - Chassis.PlanY),
                     &spining_distance);

        if (game_status.game_progress != 4 && game_status.game_progress != 9)
            Chassis.status = stay_in_place;
        else if (game_status.game_progress == 9)
            Chassis.status = spin_in_place;
        else
        {
            if (Chassis.status == stay_in_place || game_status.game_progress == 9 || (??? && Chassis.status == reach_des))
                Chassis.status = go_to_des;

            if (?? && Chassis.status == go_to_des)
                Chassis.status = reach_des;

            if (outpost_HP <= 400 || (map_interactivity))
                Chassis.status = back_to_start;

            if(??? && Chassis.status == back_to_start;)
                Chassis.status = reach_start;

            if (map_interactivity)
                Chassis.status = go_to_area;

            if (?? && Chassis.status == goto_area;)
                Chassis.status = reach_area;
        }

        if (Chassis.status == go_to_des || Chassis.status == back_to_start || Chassis.status == go_to_area;) // 听从雷达导航，包括去外面、回原地、去云台手
        {
            tempVr = 0.0f; // 0
            is_velocity = 1;
            Chassis.spinnig_center[0] = Chassis.PlanX;
            Chassis.spinnig_center[1] = Chassis.PlanY;
            move_ratio = 6.6f;

            Chassis.Vr = Chassis.Vr * 0.2f / (0.2f + dt) + tempVr * dt / (0.2f + dt);
            Chassis.Vr = float_constrain(Chassis.Vr, 0.0f, 1000.0f);

            SpinningValidVx = PID_Calculate(&Chassis.SpinningValid, Chassis.posX, Chassis.spinnig_center[0]) * is_velocity;
            SpinningValidVy = PID_Calculate(&Chassis.SpinningValid, Chassis.posY, Chassis.spinnig_center[1]) * is_velocity;

            thetaFrameNum = Find_thetaFrame(Chassis.thetaFrame, (uint32_t)(INS_GetTimeline() - debugvalue));
            preFollowTheta = Chassis.thetaFrame[thetaFrameNum].FollowTheta / RADIAN_COEF;
            SpinningValidTheta = STD_RADIAN(-preFollowTheta + Chassis.posZ);

            Chassis.VxTransfer = user_cos(SpinningValidTheta) * (Chassis.Vx + SpinningValidVx * 10.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
                                 user_sin(SpinningValidTheta) * (Chassis.Vy + SpinningValidVy * 10.0f * 10.0f / wheel_radius / 2 / (pi / 60));
            Chassis.VyTransfer = -user_sin(SpinningValidTheta) * (Chassis.Vx + SpinningValidVx * 10.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
                                 user_cos(SpinningValidTheta) * (Chassis.Vy + SpinningValidVy * 10.0f * 10.0f / wheel_radius / 2 / (pi / 60));

            Chassis.VxTransfer *= move_ratio;
            Chassis.VyTransfer *= move_ratio;
        }
        else if (Chassis.status == stay_in_place || Chassis.status == reach_des) // 停留原地，到达雷达
        {
            ;
        }
        else // 原地陀螺包括读不到比赛状态、到达原地、到达云台手
        {
            // arm_sqrt_f32((Chassis.posX1000 - Chassis.spinnig_center[0]) * (Chassis.posX1000 - Chassis.spinnig_center[0]) +
            //                  (Chassis.posY1000 - Chassis.spinnig_center[1]) * (Chassis.posY1000 - Chassis.spinnig_center[1]),
            //              &spining_distance);

            if (spining_distance < 600.0f)
                center_count++;
            else
                center_count = 0;

            if (center_count > 2500 && spining_distance < 300.0f)
            {
                tempVr = 700.0f;
                is_velocity = 0;
            }
            else if (spining_distance < 1500.0f)
            {
                tempVr = 450.0f;
                is_velocity = 1;
            }

            if (spining_distance > 1500.0f && spining_distance < 5000.0f)
                edge_count++;
            else
                edge_count = 0;

            if (edge_count > 2000)
            {
                tempVr = 450.0f;
                is_velocity = 1;
                Chassis.spinnig_center[0] = Chassis.posX;
                Chassis.spinnig_center[1] = Chassis.posY;
            }

            if (spining_distance > 10000.0f)
                data_novalid_count++;
            else
                data_novalid_count = 0;

            if (data_novalid_count > 500)
            {
                tempVr = 700.0f;
                is_velocity = 0;
            }
        }

        Chassis.posX1000 = int16_deadband(Chassis.posX1000, -50, 50);
        Chassis.posY1000 = int16_deadband(Chassis.posY1000, -50, 50);

        break;

    case Side_Mode:
        Chassis.Vr = 0;
        Chassis.FollowTheta = 0.0f;
        Chassis.VxTransfer = user_cos(-Chassis.FollowTheta / RADIAN_COEF) * Chassis.Vx +
                             user_sin(-Chassis.FollowTheta / RADIAN_COEF) * Chassis.Vy;
        Chassis.VyTransfer = -user_sin(-Chassis.FollowTheta / RADIAN_COEF) * Chassis.Vx +
                             user_cos(-Chassis.FollowTheta / RADIAN_COEF) * Chassis.Vy;
        break;
    case Count_Mode:
        // if (game_status.game_progress == 4)
        // {
        // if (Last_game_status != 4 && game_status.game_progress == 4 && method != 1 && method != 2 && method != 3 && walktodes != 1)
        // {
        //     walktodes = 1;
        //     method = 4;
        // }

        //                if ((method == 1 || method == 2 || method == 3 || method == 4) && walktodes == 1)
        //                {
        //                    Chassis.Vr = 0;
        //                    Chassis.VyTransfer = PID_Calculate(&MiniPC_CtrlFrame.PID_Follow, Chassis.Position[1], 304.0f);
        //                }

        //                if (!walktodes)
        //                    Chassis.Vr = 0;

        //                if (fabsf(Chassis.Position[1] - 304.0f) < 5.0f)
        //                    ReachCount++;
        user_count_time += dt;
        //        if (aimassist_online == 1 && lostaim == 0)
        //        {
        aimssistLoseCount = 0;
        if (ReachFinished == 0)
        {
            Chassis.FollowCoef = 1;
            if (Chassis.geto_des == 0)
            {
                if (Chassis.IsSpining)
                {
                    // tempVr = 220.0f;
                    Chassis.Vr = 220.0f;
                }
                else
                {
                    Chassis.Vr = 0.0f;
                }
            }
            if (Chassis.geto_des == 1)
            {
                ReachCount++;
            }
            if (ReachCount > 9)
            {
                ReachFinished = 1;
                ReachCount = 0;
            }
        }
        else // ReachFinished == 1
        {
            Chassis.FollowCoef = 0;
            if (Chassis.geto_des == 1)
            {
                Chassis.Vr = 750.0f;
            }
            else // Chassis.geto_des == 0
            {
                ReachCount++;
            }
            if (ReachCount > 5)
            {
                ReachFinished = 0;
                // Chassis.Vr = 0;
                ReachCount = 0;
            }
        }
        //        }
        //        else
        //        {
        //            aimssistLoseCount++;
        //			  Chassis.FollowCoef = 0;
        //        }

        //        if (aimssistLoseCount > 10000)
        //        {
        //            tempVr = 650.0f;
        //            Chassis.Vr = TD_Calculate(&Chassis.SpinningTD, tempVr);
        //            lostaim = 1;
        //			Chassis.FollowCoef = 0;
        //    }

        Chassis.FollowXVelocity = (Chassis.FollowXVelocity1000 / 10.0f) * dt / (dt + 0.005f) + Chassis.FollowXVelocity * 0.005f / (dt + 0.005f);
        Chassis.FollowXVelocity = float_constrain(Chassis.FollowXVelocity, -20.25f, 20.25f);
        Chassis.FollowYVelocity = (Chassis.FollowYVelocity1000 / 10.0f) * dt / (dt + 0.005f) + Chassis.FollowYVelocity * 0.005f / (dt + 0.005f);
        Chassis.FollowYVelocity = float_constrain(Chassis.FollowYVelocity, -20.25f, 20.25f);
        Chassis.VxTransfer = user_cos(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vx + Chassis.FollowXVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
                             user_sin(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vy + Chassis.FollowYVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60));
        Chassis.VyTransfer = -user_sin(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vx + Chassis.FollowXVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
                             user_cos(-Chassis.FollowTheta / RADIAN_COEF) * (Chassis.Vy + Chassis.FollowYVelocity * Chassis.FollowCoef * 14.0f * 10.0f / wheel_radius / 2 / (pi / 60));
        last_game_status = game_status.game_progress;
        break;
    }

    if (is_TOE_Error(GIMBAL_YAW_MOTOR_TOE)) // 如果YAW轴电机掉线（云台无法旋转），只有底盘旋转带动枪管旋转
        Chassis.Vr = remote_control.ch1 * Chassis.rcStickRotateRatio;

    // Chassis.posX1000 = int16_deadband(Chassis.posX1000, -100, 100);
    // Chassis.posY1000 = int16_deadband(Chassis.posY1000, -100, 100);
    // Chassis.posX = Chassis.posX * 0.002 / (0.002 + dt) + Chassis.posX1000 / 10 * dt / (0.002 + dt);
    // Chassis.posY = Chassis.posY * 0.002 / (0.002 + dt) + Chassis.posY1000 / 10 * dt / (0.002 + dt);
    // Chassis.posZ = Chassis.posZ1000 / 1000.0f;
    // float SpinningValidVx = PID_Calculate(&Chassis.SpinningValid, Chassis.posX, 0.0f);
    // float SpinningValidVy = PID_Calculate(&Chassis.SpinningValid, Chassis.posY, 0.0f);

    // thetaFrameNum = Find_thetaFrame(Chassis.thetaFrame, (uint32_t)(INS_GetTimeline() - debugvalue));
    // preFollowTheta = Chassis.thetaFrame[thetaFrameNum].FollowTheta / RADIAN_COEF;
    // SpinningValidTheta = STD_RADIAN(-preFollowTheta + Chassis.posZ);

    // Chassis.VxTransfer = user_cos(SpinningValidTheta) * (Chassis.Vx + SpinningValidVx * 10.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
    //                      user_sin(SpinningValidTheta) * (Chassis.Vy + SpinningValidVy * 10.0f * 10.0f / wheel_radius / 2 / (pi / 60));
    // Chassis.VyTransfer = -user_sin(SpinningValidTheta) * (Chassis.Vx + SpinningValidVx * 10.0f * 10.0f / wheel_radius / 2 / (pi / 60)) +
    //                      user_cos(SpinningValidTheta) * (Chassis.Vy + SpinningValidVy * 10.0f * 10.0f / wheel_radius / 2 / (pi / 60));

    Chassis.V1 = -(-Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + Gimbal_Position_Modification * Chassis.Vr * 10;
    Chassis.V2 = -(Chassis.VxTransfer - Chassis.VyTransfer) * Chassis.VelocityRatio + Gimbal_Position_Modification * Chassis.Vr * 10;
    Chassis.V3 = -(Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + Chassis.Vr * 10;
    Chassis.V4 = -(-Chassis.VxTransfer + Chassis.VyTransfer) * Chassis.VelocityRatio + Chassis.Vr * 10;

    // 云台重心不一样的修正
    Chassis.V3 *= Chassis.GravityCenter_Adjustment;
    Chassis.V4 *= Chassis.GravityCenter_Adjustment;

    Velocity_MAXLimit();

    Motor_Speed_Calculate(&Chassis.ChassisMotor[0], Chassis.ChassisMotor[0].Velocity_RPM, Chassis.V1);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[1], Chassis.ChassisMotor[1].Velocity_RPM, Chassis.V2);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[2], Chassis.ChassisMotor[2].Velocity_RPM, Chassis.V3);
    Motor_Speed_Calculate(&Chassis.ChassisMotor[3], Chassis.ChassisMotor[3].Velocity_RPM, Chassis.V4);

    if (!isnormal(Chassis.ChassisMotor[0].Output))
    {
        Chassis.ChassisMotor[0].Output = 0.0f;
        nanCount++;
    }
    if (!isnormal(Chassis.ChassisMotor[1].Output))
    {
        Chassis.ChassisMotor[1].Output = 0.0f;
        nanCount++;
    }
    if (!isnormal(Chassis.ChassisMotor[2].Output))
    {
        Chassis.ChassisMotor[2].Output = 0.0f;
        nanCount++;
    }
    if (!isnormal(Chassis.ChassisMotor[3].Output))
    {
        Chassis.ChassisMotor[3].Output = 0.0f;
        nanCount++;
    }

    if (Chassis.Vx != 0 || Chassis.Vy != 0)
        time_temp = USER_GetTick();

    Chassis_Power_Exp(); // 计算可能的底盘功率
                         // 功率限制
    Chassis_Power_Limit();
}

static void Send_Chassis_Current(void) /*发送底盘电机控制电流*/
{
    static uint32_t count = 0;

    if (is_TOE_Error(RC_TOE) && is_TOE_Error(VTM_TOE)) // RC——遥控器接收器  VTM——图传
    {
        if (Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0) == HAL_OK)
            HAL_IWDG_Refresh(&hiwdg);
        ;
    }
    else
    {
        // if (Send_Motor_Current_1_4(&hcan1,
        //                            Chassis.ChassisMotor[0].Output, Chassis.ChassisMotor[1].Output,
        //                            Chassis.ChassisMotor[2].Output, Chassis.ChassisMotor[3].Output) == HAL_OK)
        if (Send_Motor_Current_1_4(&hcan1, 0, 0, 0, 0) == HAL_OK)
            HAL_IWDG_Refresh(&hiwdg);
        ;
    }
    // 计算实际底盘功率
    Chassis.PowerControl.Power_Calculation_Clipping = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        Chassis.PowerControl.Power_Calculation_Clipping += ina226[0].Bus_Voltage * fabsf(Chassis.ChassisMotor[i].Real_Current * Chassis.ChassisMotor[i].Velocity_RPM);
    }
    Chassis.PowerControl.Power_Calculation_Clipping *= 0.0000001732f;

    if (JudgeRxValid || count % 100 == 0)
    {
        Send_JudgeRxData(&hcan2, JudgeRxData);
        JudgeRxValid = 0;
    }

    count++;
}

static void ChassisMotionEst_Update(float dt)
{
    static float sigmaSqrt[2];
    /*
     0  1  2  3  4  5
     6  7  8  9 10 11
    12 13 14 15 16 17
    18 19 20 21 22 23
    24 25 26 27 28 29
    30 31 32 33 34 35
    */
    for (uint8_t i = 0; i < 2; i++)
        sigmaSqrt[i] = ChassisMotionEst_Sigma[i] * ChassisMotionEst_Sigma[i];

    Chassis.ChassisMotionEst.F_data[1] = dt;
    Chassis.ChassisMotionEst.F_data[2] = dt * dt * 0.5f;
    Chassis.ChassisMotionEst.F_data[8] = dt;
    Chassis.ChassisMotionEst.F_data[22] = dt;
    Chassis.ChassisMotionEst.F_data[23] = dt * dt * 0.5f;
    Chassis.ChassisMotionEst.F_data[29] = dt;

    ChassisMotionEst_Q[0] = dt * dt * dt * dt * dt * dt / 36.0f * sigmaSqrt[X];
    ChassisMotionEst_Q[1] = dt * dt * dt * dt * dt / 12.0f * sigmaSqrt[X];
    ChassisMotionEst_Q[2] = dt * dt * dt * dt / 6.0f * sigmaSqrt[X];
    ChassisMotionEst_Q[6] = dt * dt * dt * dt * dt / 12.0f * sigmaSqrt[X];
    ChassisMotionEst_Q[7] = dt * dt * dt * dt / 4.0f * sigmaSqrt[X];
    ChassisMotionEst_Q[8] = dt * dt * dt / 2.0f * sigmaSqrt[X];
    ChassisMotionEst_Q[12] = dt * dt * dt * dt / 6.0f * sigmaSqrt[X];
    ChassisMotionEst_Q[13] = dt * dt * dt / 2.0f * sigmaSqrt[X];
    ChassisMotionEst_Q[14] = dt * dt * sigmaSqrt[X];

    ChassisMotionEst_Q[21] = dt * dt * dt * dt * dt * dt / 36.0f * sigmaSqrt[Y];
    ChassisMotionEst_Q[22] = dt * dt * dt * dt * dt / 12.0f * sigmaSqrt[Y];
    ChassisMotionEst_Q[23] = dt * dt * dt * dt / 6.0f * sigmaSqrt[Y];
    ChassisMotionEst_Q[27] = dt * dt * dt * dt * dt / 12.0f * sigmaSqrt[Y];
    ChassisMotionEst_Q[28] = dt * dt * dt * dt / 4.0f * sigmaSqrt[Y];
    ChassisMotionEst_Q[29] = dt * dt * dt / 2.0f * sigmaSqrt[Y];
    ChassisMotionEst_Q[33] = dt * dt * dt * dt / 6.0f * sigmaSqrt[Y];
    ChassisMotionEst_Q[34] = dt * dt * dt / 2.0f * sigmaSqrt[Y];
    ChassisMotionEst_Q[35] = dt * dt * sigmaSqrt[Y];

    Chassis.V1_is = ((Chassis.ChassisMotor[0].Velocity_RPM) * 2 * pi / 60) * wheel_radius / 10 / 14.0f; // ��λ cm/s
    Chassis.V2_is = ((Chassis.ChassisMotor[1].Velocity_RPM) * 2 * pi / 60) * wheel_radius / 10 / 14.0f;
    Chassis.V3_is = ((Chassis.ChassisMotor[2].Velocity_RPM) * 2 * pi / 60) * wheel_radius / 10 / 14.0f;
    Chassis.V4_is = ((Chassis.ChassisMotor[3].Velocity_RPM) * 2 * pi / 60) * wheel_radius / 10 / 14.0f;

    Chassis.Vx_is = (Chassis.V2_is + Chassis.V3_is - Chassis.V1_is - Chassis.V4_is) * Kx;
    Chassis.Vy_is = (Chassis.V1_is + Chassis.V2_is - Chassis.V3_is - Chassis.V4_is) * Ky;

    // Chassis.VxTransfer_is = Chassis.Vx_is * cosf(-Chassis.FollowTheta / RADIAN_COEF) + Chassis.Vy_is * sinf(-Chassis.FollowTheta / RADIAN_COEF);
    // Chassis.VyTransfer_is = Chassis.Vy_is * cosf(-Chassis.FollowTheta / RADIAN_COEF) - Chassis.Vx_is * sinf(-Chassis.FollowTheta / RADIAN_COEF);

    Chassis.ChassisMotionEst.MeasuredVector[0] = Chassis.Vx_is;
    Chassis.ChassisMotionEst.MeasuredVector[1] = BMI088.Accel[0] * 100.0f;
    Chassis.ChassisMotionEst.MeasuredVector[2] = Chassis.Vy_is;
    Chassis.ChassisMotionEst.MeasuredVector[3] = BMI088.Accel[1] * 100.0f;

    Kalman_Filter_Update(&Chassis.ChassisMotionEst);

    for (uint8_t i = 0; i < Chassis.ChassisMotionEst.xhatSize; i++)
    {
        if (!isnormal(Chassis.ChassisMotionEst.xhat_data[i]))
            Chassis.ChassisMotionEst.xhat_data[i] = 0;
        if (!isnormal(Chassis.ChassisMotionEst.FilteredValue[i]))
            Chassis.ChassisMotionEst.FilteredValue[i] = 0;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        Chassis.Position[i] = Chassis.ChassisMotionEst.FilteredValue[i * 3];
        Chassis.Velocity[i] = Chassis.ChassisMotionEst.FilteredValue[i * 3 + 1];
        Chassis.Accel[i] = Chassis.ChassisMotionEst.FilteredValue[i * 3 + 2];
    }

    Chassis.V_Position[0] += Chassis.Vx_is * dt;
    Chassis.V_Position[1] += Chassis.Vy_is * dt;
}

static float Max_4(float num1, float num2, float num3, float num4)
{
    static float max_num = 0;

    max_num = fabsf(num1);
    if (fabsf(num2) > max_num)
        max_num = fabsf(num2);
    if (fabsf(num3) > max_num)
        max_num = fabsf(num3);
    if (fabsf(num4) > max_num)
        max_num = fabsf(num4);

    return max_num;
}

void Chassis_Power_Limit(void)
{
    static float coef[4] = {1, 1, 1, 1};

    // 电容电压小于14.5V时
    if (ina226[0].Bus_Voltage < 14.5f)
    {
        Chassis.PowerControl.Power_Limit = Chassis.PowerControl.LowVoltage_Gain * robot_state.chassis_power_limit;
    }
    else
    {
        // 计算修正后的功率限制
        Chassis.PowerControl.Power_Limit = Chassis.PowerControl.Power_correction_gain * robot_state.chassis_power_limit;

        // 缓冲能量充足时，可放宽限制
        if (power_heat_data_t.chassis_power_buffer > 30)
            Chassis.PowerControl.Power_Limit = 1.5f * Chassis.PowerControl.Power_correction_gain * robot_state.chassis_power_limit;
    }

    // 底盘功率大于功率限制时
    if (Chassis.PowerControl.Power_Calculation > Chassis.PowerControl.Power_Limit)
    {
        Chassis.PowerControl.PowerScale = Chassis.PowerControl.Power_Limit / Chassis.PowerControl.Power_Calculation;
        if (Chassis.Mode == Spinning_Mode)
            Chassis.PowerControl.PowerScale = Chassis.PowerControl.PowerScale * 0.95;
    }
    else
        Chassis.PowerControl.PowerScale = 1.0f;

    // 急停时无功率限制
    if ((USER_GetTick() - time_temp < 500 && USER_GetTick() - time_temp > 5) || (USER_GetTick() - tempTime_Sprint < 500 && USER_GetTick() - tempTime_Sprint > 5))
        Chassis.PowerControl.PowerScale = 1.0f;

    for (uint8_t i = 0; i < 4; i++)
    {
        Chassis.ChassisMotor[i].Output *= Chassis.PowerControl.PowerScale * coef[i];
    }
}

// 计算可能底盘功率
static void Chassis_Power_Exp(void)
{
    if (is_TOE_Error(CAP_TOE))         // CAP是电容
        ina226[0].Bus_Voltage = 24.0f; // 电容掉线，默认为24v;

    Chassis.PowerControl.Power_Calculation = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        Chassis.PowerControl.Power_Calculation += ina226[0].Bus_Voltage * fabsf(Chassis.ChassisMotor[i].Output * Chassis.ChassisMotor[i].Velocity_RPM);
    }
    Chassis.PowerControl.Power_Calculation *= 0.0000001732f;
    if (!isnormal(Chassis.PowerControl.Power_Calculation))
        Chassis.PowerControl.Power_Calculation = 0.0f;
}

static void Velocity_MAXLimit(void)
{
    static float TempMaxVelocity;
    if (Max_4(fabsf(Chassis.V1), fabsf(Chassis.V2), fabsf(Chassis.V3), fabsf(Chassis.V4)) > MAX_RPM)
    {
        TempMaxVelocity = Max_4(fabsf(Chassis.V1), fabsf(Chassis.V2), fabsf(Chassis.V3), fabsf(Chassis.V4));
        Chassis.V1 *= MAX_RPM / TempMaxVelocity;
        Chassis.V2 *= MAX_RPM / TempMaxVelocity;
        Chassis.V3 *= MAX_RPM / TempMaxVelocity;
        Chassis.V4 *= MAX_RPM / TempMaxVelocity;
    }
}

void Insert_thetaFrame(thetaFrame_t *theta_frame, float follow_theta, uint32_t time_stamp_ms)
{
    memmove(theta_frame + 1, theta_frame, (FOLLOW_THETA_LEN - 1) * sizeof(thetaFrame_t));
    theta_frame[0].TimeStamp_ms = time_stamp_ms;
    theta_frame[0].FollowTheta = follow_theta;
}

uint16_t Find_thetaFrame(thetaFrame_t *theta_frame, uint32_t match_time_stamp_ms)
{
    uint16_t num = 0;
    float min_time_error = fabsf((float)(theta_frame[0].TimeStamp_ms) - match_time_stamp_ms);

    for (uint16_t i = 0; i < FOLLOW_THETA_LEN; i++)
    {
        if (fabsf((float)(theta_frame[i].TimeStamp_ms) - match_time_stamp_ms) < min_time_error)
        {
            min_time_error = fabsf((float)(theta_frame[i].TimeStamp_ms) - match_time_stamp_ms);
            num = i;
        }
    }
    return num;
}

void AerialKeyBoardCmd(void)
{
    switch (map_interactivity.commd_keyboard)
    {
    case 'Q':                          // 吊射高地
        if (robot_state.robot_id <= 7) // red
        {
            TempAerialX = 7.89f;
            TempAerialY = 14.46f;
        }
        else
        {
            TempAerialX = 21.84f;
            TempAerialY = 2.58f;
        }
        break;
    case 'E':                          // 工程银矿处
        if (robot_state.robot_id <= 7) // red
        {
            TempAerialX = 9.28f;
            TempAerialY = 12.10f;
        }
        else
        {
            TempAerialX = 19.17f;
            TempAerialY = 5.02f;
        }
        break;
    default:
        TempAerialX = map_interactivity.target_position_x;
        TempAerialY = map_interactivity.target_position_y;
    }
}