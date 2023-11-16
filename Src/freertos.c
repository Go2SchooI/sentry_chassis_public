/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "includes.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t count_ui = 0;
uint8_t count_shoot = 0;
uint16_t enemy_outpost_HP = 1500;
/* USER CODE END Variables */
osThreadId GimbalTaskHandle;
osThreadId INSTaskHandle;
osThreadId DetectTaskHandle;
osThreadId PowerMeasureTasHandle;
osThreadId UITaskHandle;
osThreadId ChassisTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartGimbalTask(void const *argument);       /*��̨����*/
void StartINSTask(void const *argument);          /*��̬��������*/
void StartDetectTask(void const *argument);       /*�����˳�*/
void StartPowerMeasureTask(void const *argument); /*���ʲ���*/
void StartUITask(void const *argument);           /*UI����*/
void StartChassisTask(void const *argument);      /*��������*/

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of GimbalTask */
  osThreadDef(GimbalTask, StartGimbalTask, osPriorityHigh, 0, 512);
  GimbalTaskHandle = osThreadCreate(osThread(GimbalTask), NULL);

  /* definition and creation of INSTask */
  osThreadDef(INSTask, StartINSTask, osPriorityAboveNormal, 0, 512);
  INSTaskHandle = osThreadCreate(osThread(INSTask), NULL);

  /* definition and creation of DetectTask */
  osThreadDef(DetectTask, StartDetectTask, osPriorityBelowNormal, 0, 512);
  DetectTaskHandle = osThreadCreate(osThread(DetectTask), NULL);

  /* definition and creation of PowerMeasureTas */
  osThreadDef(PowerMeasureTas, StartPowerMeasureTask, osPriorityNormal, 0, 512);
  PowerMeasureTasHandle = osThreadCreate(osThread(PowerMeasureTas), NULL);

  /* definition and creation of UITask */
  osThreadDef(UITask, StartUITask, osPriorityNormal, 0, 512);
  UITaskHandle = osThreadCreate(osThread(UITask), NULL);

  /* definition and creation of ChassisTask */
  osThreadDef(ChassisTask, StartChassisTask, osPriorityAboveNormal, 0, 512);
  ChassisTaskHandle = osThreadCreate(osThread(ChassisTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
 * @brief  Function implementing the GimbalTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void const *argument)
{
  /* USER CODE BEGIN StartGimbalTask */
  Gimbal_Init(); // ���̴���������ҪYAW����
  osDelay(1000);
  /* Infinite loop */
  for (;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartINSTask */
/**
 * @brief Function implementing the INSTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartINSTask */
void StartINSTask(void const *argument)
{
  /* USER CODE BEGIN StartINSTask */
  INS_Init();
  /* Infinite loop */
  for (;;)
  {
    INS_Task(); // ��������

    osDelay(INS_TASK_PERIOD);
  }
  /* USER CODE END StartINSTask */
}

/* USER CODE BEGIN Header_StartDetectTask */
/**
 * @brief Function implementing the DetectTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDetectTask */
void StartDetectTask(void const *argument) // ����Detect ���ٷ���
{
  /* USER CODE BEGIN StartDetectTask */
  Detect_Task_Init();
  /* Infinite loop */
  for (;;)
  {
    HAL_IWDG_Refresh(&hiwdg);
    if ((remote_control.key_code & Key_F) && (remote_control.key_code & Key_E))
    {
      resetCount++;
      if (resetCount * DETECT_TASK_PERIOD > 1500)
      {
        __set_FAULTMASK(1);
        HAL_NVIC_SystemReset();
      }
    }
    else
      resetCount = 0;

    Detect_Task();
    osDelay(DETECT_TASK_PERIOD);
  }
  /* USER CODE END StartDetectTask */
}

/* USER CODE BEGIN Header_StartPowerMeasureTask */
/**
 * @brief Function implementing the PowerMeasureTas thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartPowerMeasureTask */
void StartPowerMeasureTask(void const *argument) // ����
{
  /* USER CODE BEGIN StartPowerMeasureTask */
  //  INA226_Init(INA226_ADDR1);
  /* Infinite loop */
  for (;;)
  {
    //    INA226_Read_Registers(INA226_ADDR1);
    osDelay(300000);
  }
  /* USER CODE END StartPowerMeasureTask */
}

/* USER CODE BEGIN Header_StartUITask */
/**
 * @brief Function implementing the UITask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUITask */
void StartUITask(void const *argument) // UI
{
  /* USER CODE BEGIN StartUITask */
  /* Infinite loop */
  for (;;)
  {
    // UI_Task();
    osDelay(UI_TASK_PERIOD);
  }
  /* USER CODE END StartUITask */
}

/* USER CODE BEGIN Header_StartChassisTask */
/**
 * @brief Function implementing the ChassisTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartChassisTask */
void StartChassisTask(void const *argument)
{
  /* USER CODE BEGIN StartChassisTask */
  osDelay(1000);
  Chassis_Init();
  /* Infinite loop */
  for (;;)
  {

    Chassis_Control();

    if (RC_Update)
    {
      if (resetCount < 2000 && !((remote_control.key_code & Key_R) && (remote_control.key_code & Key_E)))
      {
        Send_RC_Data(&hcan1, RC_Data_Buffer);
        Send_RC_Data(&hcan2, RC_Data_Buffer);
      }
      RC_Update = 0;
    }

    if (Shoot_Updata == 1 || count_shoot > 5)
    {
      if (robot_state.robot_id <= 7) // red
        enemy_outpost_HP = game_robot_HP.blue_outpost_HP;
      else
        enemy_outpost_HP = game_robot_HP.red_outpost_HP;

      Send_Robot_Info(&hcan2, robot_state.robot_id,
                      robot_state.shooter_id1_17mm_cooling_limit,
                      power_heat_data_t.shooter_id1_17mm_cooling_heat,
                      (uint16_t)(shoot_data.bullet_speed * 10),
                      robot_state.shooter_id1_17mm_speed_limit,
                      robot_state.shooter_id2_17mm_cooling_limit,
                      power_heat_data_t.shooter_id2_17mm_cooling_heat,
                      robot_state.shooter_id2_17mm_speed_limit,
                      (uint8_t)game_status.game_progress,
                      enemy_outpost_HP);

      Shoot_Updata = 0;
      count_shoot = 0;
    }
    count_shoot++;
    osDelay(CHASSIS_TASK_PERIOD);
  }
  /* USER CODE END StartChassisTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
