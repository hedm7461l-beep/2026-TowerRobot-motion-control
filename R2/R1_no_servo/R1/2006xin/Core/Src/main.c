/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Three-omni chassis + two M2006 + suction integration
  ******************************************************************************
  * Hardware:
  *   STM32F405RGT6
  *   CAN1: 3 x M3508/C620 (ESC IDs 1,2,3)
  *         M2006/C610 #1 (ESC ID 4)
  *         M2006/C610 #2 (ESC ID 5)
  *   USART2: HC-05, 115200 8N1
  *   PB0/PB2: suction driver inputs (same pins as xipanxin project)
  *   PB1: additional active-high 24 V solenoid driver input
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct
{
  volatile uint16_t angle;
  volatile int16_t speed;
  volatile int16_t current;
  volatile uint8_t temperature;
  volatile uint32_t last_rx_tick;
} MotorFeedback_t;

typedef struct
{
  float kp;
  float ki;
  float kd;
  float error;
  float last_error;
  float integral;
  float integral_max;
  float output;
  float output_max;
} PID_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Three C620 ESC IDs must be 1, 2 and 3. */
#define CHASSIS_MOTOR_COUNT       3U

/* C610 #1 must be ID4; C610 #2 must be ID5. */
#define M2006_1_ESC_ID            4U
#define M2006_2_ESC_ID            5U

/* Keep the tested chassis PID values from dipanxin. */
#define MOTOR1_KP                 3.0f
#define MOTOR1_KI                 0.2f
#define MOTOR2_KP                 3.0f
#define MOTOR2_KI                 0.2f
#define MOTOR3_KP                 3.0f
#define MOTOR3_KI                 0.09285f

#define CHASSIS_MAX_RPM           2000.0f
#define CHASSIS_CURRENT_MAX       15000.0f
#define PID_INTEGRAL_MAX          2000.0f
#define SQRT3_DIV_2               0.8660254f
#define JOYSTICK_DEADBAND         5

/* C610 tested current. Increase gradually only after checking the mechanism. */
#define M2006_RUN_CURRENT         1000
#define M2006_1_DIRECTION         1
#define M2006_2_DIRECTION         1

#define CONTROL_PERIOD_MS         10U
#define CONTROL_TIMEOUT_MS        500U
#define ESC_STARTUP_TIME_MS       500U

/* Suction output pins copied from xipanxin_STLink_fixed. */
#define SUCTION_IN1_PORT          GPIOB
#define SUCTION_IN1_PIN           GPIO_PIN_0
#define SUCTION_IN2_PORT          GPIOB
#define SUCTION_IN2_PIN           GPIO_PIN_2

/* The standalone valve project used PB0, but PB0 is already used by suction.
 * The additional solenoid valve is therefore moved to the free PB1 pin.
 */
#define VALVE_PORT                GPIOB
#define VALVE_PIN                 GPIO_PIN_1

/* Motor key state bits. */
#define KEY_M2006_1_FORWARD       0x01U
#define KEY_M2006_1_REVERSE       0x02U
#define KEY_M2006_2_FORWARD       0x04U
#define KEY_M2006_2_REVERSE       0x08U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Index 0..4 corresponds to ESC ID 1..5. */
static MotorFeedback_t motor_feedback[5];
static PID_t chassis_pid[CHASSIS_MOTOR_COUNT];

/* Bluetooth joystick values: -100..100. */
static volatile int8_t bt_vx = 0;
static volatile int8_t bt_vy = 0;
static volatile int8_t bt_vw = 0;
static volatile uint8_t motor_key_state = 0;
static volatile uint8_t valve_pressed = 0;
static volatile uint32_t last_control_rx_tick = 0;

/* USART2 one-byte interrupt receiver. */
static uint8_t bt_rx_byte = 0;

/* Bluetooth Debugger frame parser:
 * A5 | key_bits | vx | vy | vw | checksum | 5A
 * Seven bool variables occupy one byte. The first six retain their original
 * meanings and bool7 controls the additional valve while it is held.
 * checksum = low 8 bits of key_bits + vx + vy + vw.
 */
static uint8_t bt_parse_state = 0;
static uint8_t bt_frame_index = 0;
static uint8_t bt_frame_data[5];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void CAN1_UserStart(void);
static HAL_StatusTypeDef CAN_SendMotorFrame(uint16_t std_id,
                                            int16_t c1,
                                            int16_t c2,
                                            int16_t c3,
                                            int16_t c4);
static void CAN_SendAllCurrents(int16_t chassis_c1,
                                int16_t chassis_c2,
                                int16_t chassis_c3,
                                int16_t m2006_c1,
                                int16_t m2006_c2);
static void PID_InitAll(void);
static float PID_Calculate(PID_t *pid, float target, float feedback);
static void Omni_Mix(float vx, float vy, float w,
                     float *motor1, float *motor2, float *motor3);
static void LimitWheelTargets(float *motor1, float *motor2, float *motor3);
static int8_t ApplyJoystickDeadband(int8_t value);
static int16_t GetM2006Current(uint8_t forward_mask,
                               uint8_t reverse_mask,
                               int16_t direction);
static void BT_ProcessButton(uint8_t command);
static void BT_ProcessReceivedByte(uint8_t byte);
static void Suction_GPIO_Init(void);
static void Suction_On(void);
static void Suction_Off(void);
static void Valve_On(void);
static void Valve_Off(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void CAN1_UserStart(void)
{
  CAN_FilterTypeDef filter = {0};

  /* Accept all standard CAN frames into FIFO0. */
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1,
                                   CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

static HAL_StatusTypeDef CAN_SendMotorFrame(uint16_t std_id,
                                            int16_t c1,
                                            int16_t c2,
                                            int16_t c3,
                                            int16_t c4)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint8_t data[8];
  uint32_t mailbox;

  tx_header.StdId = std_id;
  tx_header.ExtId = 0;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8;
  tx_header.TransmitGlobalTime = DISABLE;

  data[0] = (uint8_t)(((uint16_t)c1 >> 8) & 0xFFU);
  data[1] = (uint8_t)((uint16_t)c1 & 0xFFU);
  data[2] = (uint8_t)(((uint16_t)c2 >> 8) & 0xFFU);
  data[3] = (uint8_t)((uint16_t)c2 & 0xFFU);
  data[4] = (uint8_t)(((uint16_t)c3 >> 8) & 0xFFU);
  data[5] = (uint8_t)((uint16_t)c3 & 0xFFU);
  data[6] = (uint8_t)(((uint16_t)c4 >> 8) & 0xFFU);
  data[7] = (uint8_t)((uint16_t)c4 & 0xFFU);

  return HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}

static void CAN_SendAllCurrents(int16_t chassis_c1,
                                int16_t chassis_c2,
                                int16_t chassis_c3,
                                int16_t m2006_c1,
                                int16_t m2006_c2)
{
  /* 0x200 slots control ESC IDs 1..4. ID4 is M2006 #1. */
  (void)CAN_SendMotorFrame(0x200U,
                           chassis_c1,
                           chassis_c2,
                           chassis_c3,
                           m2006_c1);

  /* 0x1FF slots control ESC IDs 5..8. ID5 is M2006 #2. */
  (void)CAN_SendMotorFrame(0x1FFU, m2006_c2, 0, 0, 0);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef rx_header;
  uint8_t data[8];
  uint32_t index;

  if (hcan->Instance != CAN1)
  {
    return;
  }

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                           &rx_header, data) != HAL_OK)
  {
    return;
  }

  if ((rx_header.IDE == CAN_ID_STD) &&
      (rx_header.RTR == CAN_RTR_DATA) &&
      (rx_header.DLC == 8U) &&
      (rx_header.StdId >= 0x201U) &&
      (rx_header.StdId <= 0x205U))
  {
    index = rx_header.StdId - 0x201U;
    motor_feedback[index].angle =
        (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    motor_feedback[index].speed =
        (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor_feedback[index].current =
        (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor_feedback[index].temperature = data[6];
    motor_feedback[index].last_rx_tick = HAL_GetTick();
  }
}

static void PID_InitAll(void)
{
  uint32_t i;

  chassis_pid[0].kp = MOTOR1_KP;
  chassis_pid[0].ki = MOTOR1_KI;
  chassis_pid[1].kp = MOTOR2_KP;
  chassis_pid[1].ki = MOTOR2_KI;
  chassis_pid[2].kp = MOTOR3_KP;
  chassis_pid[2].ki = MOTOR3_KI;

  for (i = 0; i < CHASSIS_MOTOR_COUNT; i++)
  {
    chassis_pid[i].kd = 0.0f;
    chassis_pid[i].error = 0.0f;
    chassis_pid[i].last_error = 0.0f;
    chassis_pid[i].integral = 0.0f;
    chassis_pid[i].integral_max = PID_INTEGRAL_MAX;
    chassis_pid[i].output = 0.0f;
    chassis_pid[i].output_max = CHASSIS_CURRENT_MAX;
  }
}

static float PID_Calculate(PID_t *pid, float target, float feedback)
{
  /* At a complete stop, remove residual integral to prevent motor humming. */
  if ((target > -0.5f) && (target < 0.5f) &&
      (feedback > -20.0f) && (feedback < 20.0f))
  {
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
    return 0.0f;
  }

  pid->error = target - feedback;
  pid->integral += pid->error;

  if (pid->integral > pid->integral_max)
  {
    pid->integral = pid->integral_max;
  }
  if (pid->integral < -pid->integral_max)
  {
    pid->integral = -pid->integral_max;
  }

  pid->output = pid->kp * pid->error +
                pid->ki * pid->integral +
                pid->kd * (pid->error - pid->last_error);
  pid->last_error = pid->error;

  if (pid->output > pid->output_max)
  {
    pid->output = pid->output_max;
  }
  if (pid->output < -pid->output_max)
  {
    pid->output = -pid->output_max;
  }

  return pid->output;
}

static void Omni_Mix(float vx, float vy, float w,
                     float *motor1, float *motor2, float *motor3)
{
  /* Same 90/210/330-degree wheel arrangement as the tested chassis project.
   * vx: forward positive, vy: left positive, w: CCW positive.
   */
  *motor1 = -vx + w;
  *motor2 = 0.5f * vx - SQRT3_DIV_2 * vy + w;
  *motor3 = 0.5f * vx + SQRT3_DIV_2 * vy + w;
}

static void LimitWheelTargets(float *motor1, float *motor2, float *motor3)
{
  float abs1 = (*motor1 >= 0.0f) ? *motor1 : -*motor1;
  float abs2 = (*motor2 >= 0.0f) ? *motor2 : -*motor2;
  float abs3 = (*motor3 >= 0.0f) ? *motor3 : -*motor3;
  float maximum = abs1;
  float scale;

  if (abs2 > maximum)
  {
    maximum = abs2;
  }
  if (abs3 > maximum)
  {
    maximum = abs3;
  }

  if (maximum > CHASSIS_MAX_RPM)
  {
    scale = CHASSIS_MAX_RPM / maximum;
    *motor1 *= scale;
    *motor2 *= scale;
    *motor3 *= scale;
  }
}

static int8_t ApplyJoystickDeadband(int8_t value)
{
  if ((value >= -JOYSTICK_DEADBAND) &&
      (value <= JOYSTICK_DEADBAND))
  {
    return 0;
  }
  return value;
}

static int16_t GetM2006Current(uint8_t forward_mask,
                               uint8_t reverse_mask,
                               int16_t direction)
{
  uint8_t keys = motor_key_state;
  uint8_t forward_pressed = keys & forward_mask;
  uint8_t reverse_pressed = keys & reverse_mask;

  /* Pressing both directions is treated as stop. */
  if ((forward_pressed != 0U) && (reverse_pressed == 0U))
  {
    return (int16_t)(direction * M2006_RUN_CURRENT);
  }
  if ((reverse_pressed != 0U) && (forward_pressed == 0U))
  {
    return (int16_t)(-direction * M2006_RUN_CURRENT);
  }
  return 0;
}

static void BT_ProcessButton(uint8_t command)
{
  uint8_t valid_motion_command = 1U;

  switch (command)
  {
    /* Button 1: M2006 #1 forward, press A / release a. */
    case 'A':
      motor_key_state |= KEY_M2006_1_FORWARD;
      break;
    case 'a':
      motor_key_state &= (uint8_t)~KEY_M2006_1_FORWARD;
      break;

    /* Button 2: M2006 #1 reverse, press B / release b. */
    case 'B':
      motor_key_state |= KEY_M2006_1_REVERSE;
      break;
    case 'b':
      motor_key_state &= (uint8_t)~KEY_M2006_1_REVERSE;
      break;

    /* Button 3: M2006 #2 forward, press C / release c. */
    case 'C':
      motor_key_state |= KEY_M2006_2_FORWARD;
      break;
    case 'c':
      motor_key_state &= (uint8_t)~KEY_M2006_2_FORWARD;
      break;

    /* Button 4: M2006 #2 reverse, press D / release d. */
    case 'D':
      motor_key_state |= KEY_M2006_2_REVERSE;
      break;
    case 'd':
      motor_key_state &= (uint8_t)~KEY_M2006_2_REVERSE;
      break;

    /* Button 5/6: suction is latched; release data is not required. */
    case 'E':
      Suction_On();
      valid_motion_command = 0U;
      break;
    case 'F':
      Suction_Off();
      valid_motion_command = 0U;
      break;

    /* Optional ASCII fallback for the additional valve. */
    case 'G':
      valve_pressed = 1U;
      Valve_On();
      break;
    case 'g':
      valve_pressed = 0U;
      Valve_Off();
      break;

    /* Optional emergency stop command. */
    case 'X':
      bt_vx = 0;
      bt_vy = 0;
      bt_vw = 0;
      motor_key_state = 0;
      valve_pressed = 0U;
      Valve_Off();
      break;

    default:
      valid_motion_command = 0U;
      break;
  }

  if (valid_motion_command != 0U)
  {
    last_control_rx_tick = HAL_GetTick();
  }
}

static void BT_ProcessReceivedByte(uint8_t byte)
{
  switch (bt_parse_state)
  {
    case 0:
      if (byte == 0xA5U)
      {
        bt_frame_index = 0;
        bt_parse_state = 1;
      }
      else
      {
        BT_ProcessButton(byte);
      }
      break;

    case 1:
      bt_frame_data[bt_frame_index++] = byte;
      if (bt_frame_index >= 5U)
      {
        bt_parse_state = 2;
      }
      break;

    case 2:
      if ((byte == 0x5AU) &&
          ((uint8_t)(bt_frame_data[0] +
                     bt_frame_data[1] +
                     bt_frame_data[2] +
                     bt_frame_data[3]) == bt_frame_data[4]))
      {
        /* bit0..bit3 directly represent the four held motor buttons. */
        motor_key_state = bt_frame_data[0] & 0x0FU;

        /* Suction commands are latched. If both are pressed, OFF wins. */
        if ((bt_frame_data[0] & 0x20U) != 0U)
        {
          Suction_Off();
        }
        else if ((bt_frame_data[0] & 0x10U) != 0U)
        {
          Suction_On();
        }

        /* bool7: additional solenoid valve, held ON only while pressed. */
        valve_pressed = ((bt_frame_data[0] & 0x40U) != 0U) ? 1U : 0U;
        if (valve_pressed != 0U)
        {
          Valve_On();
        }
        else
        {
          Valve_Off();
        }

        bt_vx = (int8_t)bt_frame_data[1];
        bt_vy = (int8_t)bt_frame_data[2];
        bt_vw = (int8_t)bt_frame_data[3];
        last_control_rx_tick = HAL_GetTick();
      }
      bt_parse_state = 0;
      bt_frame_index = 0;
      break;

    default:
      bt_parse_state = 0;
      bt_frame_index = 0;
      break;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    BT_ProcessReceivedByte(bt_rx_byte);
    (void)HAL_UART_Receive_IT(&huart2, &bt_rx_byte, 1);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    bt_parse_state = 0;
    bt_frame_index = 0;
    valve_pressed = 0U;
    Valve_Off();
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    (void)HAL_UART_Receive_IT(&huart2, &bt_rx_byte, 1);
  }
}

static void Suction_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio_init = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB,
                    SUCTION_IN1_PIN | SUCTION_IN2_PIN | VALVE_PIN,
                    GPIO_PIN_RESET);

  gpio_init.Pin = SUCTION_IN1_PIN | SUCTION_IN2_PIN | VALVE_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_PULLDOWN;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio_init);
}

static void Suction_On(void)
{
  /* Same polarity as the tested suction project: IN1=1, IN2=0. */
  HAL_GPIO_WritePin(SUCTION_IN2_PORT, SUCTION_IN2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SUCTION_IN1_PORT, SUCTION_IN1_PIN, GPIO_PIN_SET);
}

static void Suction_Off(void)
{
  HAL_GPIO_WritePin(SUCTION_IN1_PORT, SUCTION_IN1_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SUCTION_IN2_PORT, SUCTION_IN2_PIN, GPIO_PIN_RESET);
}

static void Valve_On(void)
{
  /* Same active-high polarity as the tested diancifa project. */
  HAL_GPIO_WritePin(VALVE_PORT, VALVE_PIN, GPIO_PIN_SET);
}

static void Valve_Off(void)
{
  HAL_GPIO_WritePin(VALVE_PORT, VALVE_PIN, GPIO_PIN_RESET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t last_control_tick;
  uint32_t startup_tick;
  uint32_t zero_send_tick;
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  Suction_GPIO_Init();
  Suction_Off();
  Valve_Off();

  PID_InitAll();
  CAN1_UserStart();

  if (HAL_UART_Receive_IT(&huart2, &bt_rx_byte, 1) != HAL_OK)
  {
    Error_Handler();
  }

  /* Give all ESCs 500 ms to start while repeatedly transmitting zero current. */
  startup_tick = HAL_GetTick();
  zero_send_tick = startup_tick;
  while ((HAL_GetTick() - startup_tick) < ESC_STARTUP_TIME_MS)
  {
    if ((HAL_GetTick() - zero_send_tick) >= CONTROL_PERIOD_MS)
    {
      zero_send_tick = HAL_GetTick();
      CAN_SendAllCurrents(0, 0, 0, 0, 0);
    }
  }

  last_control_tick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    if ((now - last_control_tick) >= CONTROL_PERIOD_MS)
    {
      int8_t vx_command;
      int8_t vy_command;
      int8_t vw_command;
      float vx;
      float vy;
      float vw;
      float target1;
      float target2;
      float target3;
      int16_t chassis_current1;
      int16_t chassis_current2;
      int16_t chassis_current3;
      int16_t m2006_current1;
      int16_t m2006_current2;

      last_control_tick = now;

      /* Bluetooth failsafe: stop every motor if no valid motion data for 500 ms.
       * The suction output intentionally remains latched until button 6 is used.
       */
      if ((now - last_control_rx_tick) > CONTROL_TIMEOUT_MS)
      {
        bt_vx = 0;
        bt_vy = 0;
        bt_vw = 0;
        motor_key_state = 0;
        valve_pressed = 0U;
        Valve_Off();
      }

      vx_command = ApplyJoystickDeadband(bt_vx);
      vy_command = ApplyJoystickDeadband(bt_vy);
      vw_command = ApplyJoystickDeadband(bt_vw);

      vx = ((float)vx_command / 100.0f) * CHASSIS_MAX_RPM;
      vy = ((float)vy_command / 100.0f) * CHASSIS_MAX_RPM;
      vw = ((float)vw_command / 100.0f) * CHASSIS_MAX_RPM;

      Omni_Mix(vx, vy, vw, &target1, &target2, &target3);
      LimitWheelTargets(&target1, &target2, &target3);

      chassis_current1 = (int16_t)PID_Calculate(
          &chassis_pid[0], target1, (float)motor_feedback[0].speed);
      chassis_current2 = (int16_t)PID_Calculate(
          &chassis_pid[1], target2, (float)motor_feedback[1].speed);
      chassis_current3 = (int16_t)PID_Calculate(
          &chassis_pid[2], target3, (float)motor_feedback[2].speed);

      m2006_current1 = GetM2006Current(KEY_M2006_1_FORWARD,
                                       KEY_M2006_1_REVERSE,
                                       M2006_1_DIRECTION);
      m2006_current2 = GetM2006Current(KEY_M2006_2_FORWARD,
                                       KEY_M2006_2_REVERSE,
                                       M2006_2_DIRECTION);

      CAN_SendAllCurrents(chassis_current1,
                          chassis_current2,
                          chassis_current3,
                          m2006_current1,
                          m2006_current2);
    }
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Same 8 MHz HSE / 168 MHz clock arrangement as the tested 2006 project. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  Suction_Off();
  Valve_Off();
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
