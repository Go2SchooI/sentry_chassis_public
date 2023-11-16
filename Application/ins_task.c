#include "ins_task.h"
#include "QuaternionAHRS.h"
#include "includes.h"
#include "GravityEstimateKF.h"
#include "tim.h"

PID_t TempCtrl = {0};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;

void INS_Init(void)
{
    // 卡尔曼滤波器初始化
    gEstimateKF_Init(0.01, 100000);

    // imu heat init
    // IMU_PWM_Init();
    PID_Init(&TempCtrl, 2000, 1200, 0, 500, 80, 0, 0, 0, 0, 0, 0,
             DerivativeFilter | Integral_Limit | Trapezoid_Intergral); // IMU温度控制用
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
}

void INS_Task(void)
{
    static uint32_t count = 0;
    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);

        for (uint8_t i = 0; i < 3; i++) // 姿态解算
        {
            AHRS.Accel[i] = BMI088.Accel[i];
            AHRS.Gyro[i] = BMI088.Gyro[i];
        }

        gEstimateKF_Update(BMI088.Gyro[X], BMI088.Gyro[Y], BMI088.Gyro[Z], BMI088.Accel[X], BMI088.Accel[Y], BMI088.Accel[Z], dt);
        Quaternion_AHRS_UpdateIMU(BMI088.Gyro[X], BMI088.Gyro[Y], BMI088.Gyro[Z], gVec[X], gVec[Y], gVec[Z], dt);
        InsertQuaternionFrame(&QuaternionBuffer, AHRS.q, USER_GetTick() / 1000.0f);

        Get_EulerAngle(AHRS.q);

        if (GlobalDebugMode == INS_DEBUG) // DEBUG时 打印数据
        {
            if (ins_debug_mode == 0)
                Serial_Debug(&huart1, 1, AHRS.Yaw, AHRS.Pitch, AHRS.Roll, 0, 0, 0);
            if (ins_debug_mode == 1)
                Serial_Debug(&huart1, 1, gVec[0], gVec[1], gVec[2], AHRS.Accel[X], AHRS.Accel[Y], AHRS.Accel[Z]);
            if (ins_debug_mode == 2)
                Serial_Debug(&huart1, 1, atan2f(gVec[0], gVec[2]) * RADIAN_COEF, atan2f(gVec[1], gVec[2]) * RADIAN_COEF, atan2f(AHRS.Accel[X], AHRS.Accel[Z]) * RADIAN_COEF, atan2f(AHRS.Accel[Y], AHRS.Accel[Z]) * RADIAN_COEF, 0, 0);
        }
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
        if (GlobalDebugMode == IMU_HEAT_DEBUG)
            Serial_Debug(&huart1, 1, RefTemp, BMI088.Temperature, TempCtrl.Output / 1000.0f, TempCtrl.Pout / 1000.0f, TempCtrl.Iout / 1000.0f, TempCtrl.Dout / 1000.0f);
    }

    if ((count % 1000) == 0)
    {
        // 200hz
    }

    count++;
}

void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp);

    TIM_Set_PWM(&htim10, TIM_CHANNEL_1, float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}
