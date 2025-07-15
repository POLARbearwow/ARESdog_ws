/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cybergear.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static motor_pid angle_pid;
static motor_pid speed_pid;
static motor_pid current_pid;

int16_t pidcalculate_angle(motor_pid *pid, MI_Motor *Motor, int16_t measure)
{ // pid解算第一环角度环函数
  pid->Last_error = pid->error;
  pid->error = measure * 7.75f - Motor->Angle; // what's the coefficient of measure
  pid->Ierror += pid->Ki * pid->error;
  return (pid->Kp * pid->error) + (pid->Ierror) + (pid->Kd * (pid->error - pid->Last_error));
}

int16_t pidcalculate_speed(motor_pid *pid, MI_Motor *Motor, int16_t measure)
{ // pid解算第二环�?�度环函�??
  pid->Last_error = pid->error;
  pid->error = measure - Motor->Speed;
  pid->Ierror += pid->Ki * pid->error;
  return (pid->Kp * pid->error) + (pid->Ierror) + (pid->Kd * (pid->error - pid->Last_error));
}

int16_t pidcalculate_current(motor_pid *pid, MI_Motor *Motor, int16_t measure)
{ // pid解算第二环�?�度环函�??
  pid->Last_error = pid->error;
  pid->error = measure - Motor->Torque;
  pid->Ierror += pid->Ki * pid->error;
  return (pid->Kp * pid->error) + (pid->Ierror) + (pid->Kd * (pid->error - pid->Last_error));
}

void MPC_Init(MPC_Controller *mpc, float Kp, float Ki, float Kd, float setpoint) {
  mpc->Kp = Kp;
  mpc->Ki = Ki;
  mpc->Kd = Kd;
  mpc->setpoint = setpoint;
  mpc->integral = 0.0f;
  mpc->previous_error = 0.0f;
}

float MPC_Compute(MPC_Controller *mpc, float measurement) {
  float error = mpc->setpoint - measurement;
  mpc->integral += error;
  float derivative = error - mpc->previous_error;
  mpc->previous_error = error;
  return mpc->Kp * error + mpc->Ki * mpc->integral + mpc->Kd * derivative;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 机械参数配置（根据实际机器人调整�?
#define L1 0.09f
#define L2 0.224f
#define L3 0.224f
#define L4 0.09f
#define CONTROL_CYCLE 10 // 控制周期10ms
#define TRAJ_POINTS 30   // 轨迹点数
#define BACK_TRAJ_POINTS 30
#define P_MOTOR  10
#define D_MOTOR  1
#define F_MOTOR  7
#define Torque 10
#include <math.h>

typedef struct
{
  float x;
  float z;
} Point2D;

volatile float  g_target_theta1;
volatile float  g_target_theta4;

// 逆运动学计算 (X-Z平面)
uint8_t inverse_kinematic(float x, float z, float *theta1, float *theta4)
{
  float l0 = sqrtf(x * x + z * z);

  if (fabsf(L3 - L4) >= l0 || l0 >= (L3 + L4))
  {
    return 0; // 返回错误标志
  }

  float theta_inside = acosf((L4 * L4 + l0 * l0 - L3 * L3) / (2 * L4 * l0));
  float theta = atan2f(z, x);

  *theta1 = theta + theta_inside;
  *theta4 = theta - theta_inside;
  return 1; // 成功标志
}

void emergency_stop(void);
void emergency_stop(void)
{
  motor_controlmode(&mi_motor[0], 0, 0, 0, 0, 0);
  motor_controlmode(&mi_motor[1], 0, 0, 0, 0, 0);
}

// 摆线轨迹生成
void generate_cycloid(Point2D start, Point2D end, float height, Point2D *traj, int num_points)
{
  float delta_x = end.x - start.x;
  float delta_z = end.z - start.z; // 起点与落点可能存在高度差

  for (int i = 0; i < num_points; ++i)
  {
    float t = 2 * M_PI * i / (num_points - 1);

    // X方向摆线
    traj[i].x = start.x + delta_x * (t - sinf(t)) / (2 * M_PI);

    traj[i].z = start.z + delta_z * (i / (float)num_points) + height * (1 - cosf(t)) / 2; // 关键修改：height取反
  }
}

typedef enum
{
  FORWARD,
  PAUSE_END,  // 终点停留
  BACKWARD,   // 返回起点
  PAUSE_START // 起点停留
} GaitState;

void MotorControlTask()
{
  static uint8_t traj_index = 0;
  static Point2D forward_trajectory[TRAJ_POINTS];
  static Point2D backward_trajectory[BACK_TRAJ_POINTS];
  static GaitState gait_state = FORWARD;
  static uint32_t state_start_time = 0;
  static uint8_t initialized = 0;
  static uint32_t last_tick = 0;

  // 初始化轨迹（仅运行一次）
  if (!initialized)
  {
    Point2D start = {-0.10f, -0.20f};
    Point2D end = {0.10f, -0.2f};
    generate_cycloid(start, end, 0.08f, forward_trajectory, TRAJ_POINTS);
    generate_cycloid(end, start, -0.03f, backward_trajectory, BACK_TRAJ_POINTS);
    initialized = 1;
  }


  if (HAL_GetTick() - last_tick < CONTROL_CYCLE)
    return;

  last_tick = HAL_GetTick();

  switch (gait_state)
  {
  case FORWARD:
  {
    if(traj_index >= TRAJ_POINTS) traj_index = TRAJ_POINTS - 1;
    // 正向执行轨迹
    Point2D current = forward_trajectory[traj_index];

    // 逆运动学解算
    float theta1, theta4;
    if (inverse_kinematic(current.x, current.z, &theta1, &theta4))
    {
      g_target_theta1 = -theta1;
      g_target_theta4 = theta4 + M_PI;
      motor_controlmode(&mi_motor[0], Torque, -theta1, 1, P_MOTOR, D_MOTOR);
      motor_controlmode(&mi_motor[1], Torque, theta4 + M_PI, 1, P_MOTOR, D_MOTOR);
    }

    // 更新索引
    if (++traj_index >= TRAJ_POINTS)
    {
      gait_state = PAUSE_END;
      traj_index = 0; // 重置为0以便反向轨迹从头开始
      state_start_time = HAL_GetTick();
      // traj_index = TRAJ_POINTS - 1; //  单摆线
    }
    break;
  }

  case PAUSE_END:
  {

    if (HAL_GetTick() - state_start_time > 500) // 设置占空比？
    {
      gait_state = BACKWARD;
      traj_index = 0; //
    }
    break;
  }

  case BACKWARD:
  {

    if(traj_index >= BACK_TRAJ_POINTS) traj_index = BACK_TRAJ_POINTS - 1;

    // 逆向执行轨迹返回
    Point2D current = backward_trajectory[traj_index];

    // 逆运动学解算
    float theta1, theta4;
    if (inverse_kinematic(current.x, current.z, &theta1, &theta4))
    {
      g_target_theta1 = -theta1;
      g_target_theta4 = theta4 + M_PI;
      motor_controlmode(&mi_motor[0], Torque, -theta1, 1, P_MOTOR, D_MOTOR);
      motor_controlmode(&mi_motor[1], Torque, theta4 + M_PI, 1, P_MOTOR, D_MOTOR);
    }

    // 更新索引
    if (++traj_index >= BACK_TRAJ_POINTS)
    {
      gait_state = PAUSE_START;
      traj_index = 0;  // 重置索引
      state_start_time = HAL_GetTick();
    }
    break;
  }

  case PAUSE_START:
  {
    
    if (HAL_GetTick() - state_start_time > 500)
    {
      gait_state = FORWARD;
      traj_index = 0; // 重置索引
    }
    break;
  }
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  can_filter_Init();

  // angle_pid.Kp = 0.2;
  // angle_pid.Ki = 0;
  // angle_pid.Kd = 0;
  // speed_pid.Kp = 0.5;
  // speed_pid.Ki = 0;
  // speed_pid.Kd = 0.01;
  // current_pid.Kp = 1;
  // current_pid.Ki = 0;
  // current_pid.Kd = 0.1;

  init_cybergear(&mi_motor[1], 0x02, 0);
  HAL_Delay(100);
  init_cybergear(&mi_motor[0], 0x01, 0);
  HAL_Delay(100);

  MPC_Controller mpc;
  MPC_Init(&mpc, 1.0f, 0.1f, 0.01f, 0.0f); // 根据需要调整参数
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // #endregion
  while (1)
  {
    MotorControlTask();
    HAL_Delay(1); // 释放CPU资源
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
