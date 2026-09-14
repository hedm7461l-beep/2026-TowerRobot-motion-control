/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Complete three-omni-wheel robot controller
  *
  * Hardware:
  *   - STM32F405RGT6, 8 MHz HSE, 168 MHz SYSCLK
  *   - HC-05 on USART2, 115200 8N1, BTMCU 8-byte ValuePack
  *   - C620 + M3508 x3, ESC IDs 1/2/3
  *   - C610 + M2006 x3, ESC IDs 4/5/6
  *     (claw/lift/suction-lift position control)
  *   - SG90 on PA6 / TIM3_CH1
  *   - Solenoid-valve MOS module signal on PB0
  *   - Vacuum-pump L298N IN1/IN2 on PB1/PB2
  *   - Suction lift uses C610 ID6 + M2006 with encoder position holding
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  volatile uint16_t angle;
  volatile uint16_t last_angle;
  volatile int32_t total_angle;
  volatile int16_t speed;
  volatile int16_t current;
  volatile uint8_t temperature;
  volatile uint8_t angle_initialized;
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

typedef enum
{
  AUTO_IDLE = 0,
  AUTO_FENCE_DOWN,
  AUTO_WAIT_PUMP,
  AUTO_HOLD_OBJECT,
  AUTO_WAIT_RETRACT,
  AUTO_FENCE_UP
} AutoState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Control periods and Bluetooth fail-safe. BTMCU sends one frame every 50 ms. */
#define CAN_CONTROL_PERIOD_MS       10U
#define SERVO_CONTROL_PERIOD_MS     20U
#define BT_LOSS_TIMEOUT_MS          300U
#define MOTOR_FEEDBACK_TIMEOUT_MS   100U

/*
 * BTMCU ValuePack selected in the supplied .pro file:
 * A5 | bool[0..15] | BYTE0 | BYTE1 | BYTE2 | checksum(data bytes) | 5A
 */
#define BT_PACKET_HEAD              0xA5U
#define BT_PACKET_TAIL              0x5AU
#define BT_PACKET_DATA_SIZE         5U
#define BT_PACKET_SIZE              8U
#define BT_AXIS_DEADZONE            5
#define BT_AXIS_FULL_SCALE          100.0f

/* bool12/13 control the C610 ID6 + M2006 suction lift. */
#define BT_KEY_CLAW_CLOSE           (1U << 0)
#define BT_KEY_CLAW_OPEN            (1U << 1)
#define BT_KEY_LIFT_UP              (1U << 2)
#define BT_KEY_LIFT_DOWN            (1U << 3)
#define BT_KEY_PUMP_ON              (1U << 4)
#define BT_KEY_PUMP_OFF             (1U << 5)
#define BT_KEY_CYLINDER             (1U << 6)
#define BT_KEY_FENCE_DOWN           (1U << 7)
#define BT_KEY_FENCE_UP             (1U << 8)

#define BT_KEY_AUTO_CAPTURE         (1U << 9)
#define BT_KEY_AUTO_RELEASE         (1U << 10)
#define BT_KEY_EMERGENCY_STOP       (1U << 11)
#define BT_KEY_SUCTION_LIFT_UP       (1U << 12)
#define BT_KEY_SUCTION_LIFT_DOWN     (1U << 13)
#define BT_MANUAL_OUTPUT_KEYS       (BT_KEY_PUMP_ON | BT_KEY_PUMP_OFF | \
                                     BT_KEY_CYLINDER | BT_KEY_FENCE_DOWN | \
                                     BT_KEY_FENCE_UP)

/* Chassis: keep the three independently adjustable gains from dipanxin. */
#define MOTOR1_KP                  3.0f
#define MOTOR1_KI                  0.2f
#define MOTOR1_KD                  0.0f
#define MOTOR2_KP                  3.0f
#define MOTOR2_KI                  0.2f
#define MOTOR2_KD                  0.0f
#define MOTOR3_KP                  3.0f
#define MOTOR3_KI                  0.25f
#define MOTOR3_KD                  0.0f
#define CHASSIS_MAX_RPM             2000.0f
#define C620_CURRENT_LIMIT          15000.0f
#define PID_INTEGRAL_LIMIT          2000.0f
#define CHASSIS_STOP_RPM_THRESHOLD  20
#define CHASSIS_BRAKE_CURRENT_LIMIT 5000.0f
#define SQRT3_OVER_2                0.8660254f

/* Change one sign to -1.0f if that input axis or installed wheel is reversed. */
#define CHASSIS_VX_SIGN             1.0f
#define CHASSIS_VY_SIGN             1.0f
#define CHASSIS_VW_SIGN             1.0f
#define MOTOR1_DIRECTION            1.0f
#define MOTOR2_DIRECTION            1.0f
#define MOTOR3_DIRECTION            1.0f

/*
 * M2006 position control. C610 reports the rotor encoder as 0..8191 and the
 * M2006 P36 gearbox ratio is 36:1. All mechanism angles below are output-shaft
 * degrees relative to the position seen immediately after power-up.
 */
#define M2006_ENCODER_COUNTS_PER_REV  8192.0f
#define M2006_GEAR_RATIO              36.0f
#define M2006_COUNTS_PER_OUTPUT_DEG   \
        (M2006_ENCODER_COUNTS_PER_REV * M2006_GEAR_RATIO / 360.0f)
#define CLAW_FEEDBACK_INDEX           3U
#define LIFT_FEEDBACK_INDEX           4U
#define SUCTION_LIFT_FEEDBACK_INDEX   5U

/* Claw: bool0 selects this closed angle; bool1 returns to the power-up zero. */
#define CLAW_CLOSE_OUTPUT_DEG          55.0f
#define CLAW_POSITION_KP               0.040f
#define CLAW_SPEED_KD                  0.35f
#define CLAW_CURRENT_LIMIT             3000.0f
#define CLAW_HOLD_BIAS_CURRENT         2000.0f

/*
 * Lift: bool2/bool3 ramp the target while held, then the target is retained.
 * Default travel is one output-shaft revolution; set this to the real travel.
 */
#define LIFT_MIN_OUTPUT_DEG             0.0f
#define LIFT_MAX_OUTPUT_DEG             800.0f
#define LIFT_TARGET_SPEED_DEG_PER_SEC   300.0f  //速度
#define LIFT_POSITION_KP                0.045f
#define LIFT_SPEED_KD                   0.35f
#define LIFT_CURRENT_LIMIT              5000.0f
#define LIFT_GRAVITY_HOLD_CURRENT       1000.0f

/*
 * Suction lift on C610 ID6 + M2006: bool12/bool13 change the target while
 * held. Releasing both buttons retains the target and position current.
 */
#define SUCTION_LIFT_MIN_OUTPUT_DEG             0.0f
#define SUCTION_LIFT_MAX_OUTPUT_DEG             800.0f
#define SUCTION_LIFT_TARGET_SPEED_DEG_PER_SEC   300.0f
#define SUCTION_LIFT_POSITION_KP                0.045f
#define SUCTION_LIFT_SPEED_KD                   0.35f
#define SUCTION_LIFT_CURRENT_LIMIT              5000.0f
#define SUCTION_LIFT_GRAVITY_HOLD_CURRENT       1000.0f

/* Change a sign to -1 if the installed mechanism moves backwards. */
#define CLAW_CLOSE_SIGN                 1
#define LIFT_UP_SIGN                    1
#define SUCTION_LIFT_UP_SIGN             1

/* SG90 settings: TIM3 runs at 1 MHz, so compare value equals pulse width us. */
#define SERVO_MIN_US               600U
#define SERVO_CENTER_US            1500U
#define SERVO_MAX_US               2400U
#define SERVO_STEP_US              10U

/* Test without the mechanical linkage first; swap these two if reversed. */
#define FENCE_UP_US                SERVO_MIN_US
#define FENCE_DOWN_US              SERVO_MAX_US

/* Automatic suction sequence timing. */
#define PUMP_LEAD_TIME_MS          300U
#define CYLINDER_RETRACT_TIME_MS   800U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* Feedback indices 0..5 correspond to ESC IDs 1..6. */
MotorFeedback_t motor_feedback[6];
PID_t chassis_pid[3];
volatile uint32_t can_receive_count = 0U;

/* BTMCU receive state. Packet layout is documented above. */
static uint8_t uart_rx_byte = 0U;
static volatile int8_t bt_vx = 0;
static volatile int8_t bt_vy = 0;
static volatile int8_t bt_vw = 0;
static volatile uint16_t bt_key_bits = 0U;
static volatile uint32_t last_bt_packet_tick = 0U;
static volatile uint32_t bt_packet_sequence = 0U;
static volatile uint32_t bt_valid_packet_count = 0U;
static volatile uint32_t bt_error_packet_count = 0U;
static volatile uint8_t bt_link_valid = 0U;
static uint8_t bt_rx_index = 0U;
static uint8_t bt_rx_packet[BT_PACKET_SIZE];
static uint16_t last_applied_keys = 0U;

/* Mechanism state. Claw buttons select positions; lift buttons ramp a target. */
static volatile int8_t lift_command = 0;
static volatile int8_t suction_lift_command = 0;
static volatile int8_t servo_manual_direction = 0;
static volatile uint8_t mechanism_outputs_enabled = 1U;
static uint8_t claw_closed_requested = 0U;
static uint8_t claw_position_ready = 0U;
static uint8_t lift_position_ready = 0U;
static uint8_t suction_lift_position_ready = 0U;
static int32_t claw_zero_counts = 0;
static int32_t claw_target_counts = 0;
static int32_t lift_zero_counts = 0;
static int32_t lift_target_counts = 0;
static float lift_target_output_deg = 0.0f;
static int32_t suction_lift_zero_counts = 0;
static int32_t suction_lift_target_counts = 0;
static float suction_lift_target_output_deg = 0.0f;

static uint16_t servo_pulse_us = SERVO_CENTER_US;
static volatile AutoState_t auto_state = AUTO_IDLE;
static volatile uint32_t auto_state_tick = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void CAN1_Start(void);
static void PID_InitAll(void);
static void PID_Reset(PID_t *pid);
static float PID_Calculate(PID_t *pid, float target, float feedback);
static int16_t Chassis_BrakeCurrent(PID_t *pid,
                                    uint8_t feedback_index,
                                    uint32_t now);
static void Omni_Mix(float vx, float vy, float w,
                     float speed_limit,
                     float *motor1, float *motor2, float *motor3);
static float ClampFloat(float value, float minimum, float maximum);
static int32_t OutputDegreesToCounts(float degrees);
static int16_t Position_Current(int32_t target_counts,
                                int32_t actual_counts,
                                int16_t rotor_speed,
                                float kp, float kd,
                                float feedforward,
                                float current_limit);
static void Mechanism_Task(uint32_t now,
                           int16_t *claw_current,
                           int16_t *lift_current,
                           int16_t *suction_lift_current);
static int8_t Bluetooth_FilterAxis(int8_t value);
static uint8_t Bluetooth_ValidatePacket(const uint8_t *packet);
static void Bluetooth_DecodePacket(const uint8_t *packet);
static void Bluetooth_ApplyPacket(uint16_t keys, uint32_t now);
static void DJI_SendCurrent_0x200(int16_t id1, int16_t id2,
                                 int16_t id3, int16_t id4);
static void DJI_SendCurrent_0x1FF(int16_t id5, int16_t id6,
                                 int16_t id7, int16_t id8);
static void Pump_On(void);
static void Pump_Off(void);
static void Valve_On(void);
static void Valve_Off(void);
static void Communication_Failsafe(void);
static void Emergency_Stop(void);
static void Servo_Task(void);
static uint8_t Servo_MoveToward(uint16_t target_us);
static void Automatic_Task(uint32_t now);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float ClampFloat(float value, float minimum, float maximum)
{
  if (value > maximum)
  {
    return maximum;
  }
  if (value < minimum)
  {
    return minimum;
  }
  return value;
}

static int32_t OutputDegreesToCounts(float degrees)
{
  float counts = degrees * M2006_COUNTS_PER_OUTPUT_DEG;

  if (counts >= 0.0f)
  {
    return (int32_t)(counts + 0.5f);
  }
  return (int32_t)(counts - 0.5f);
}

static int16_t Position_Current(int32_t target_counts,
                                int32_t actual_counts,
                                int16_t rotor_speed,
                                float kp, float kd,
                                float feedforward,
                                float current_limit)
{
  float position_error = (float)(target_counts - actual_counts);
  float output = kp * position_error
               - kd * (float)rotor_speed
               + feedforward;

  output = ClampFloat(output, -current_limit, current_limit);
  if (output >= 0.0f)
  {
    return (int16_t)(output + 0.5f);
  }
  return (int16_t)(output - 0.5f);
}

static void Mechanism_Task(uint32_t now,
                           int16_t *claw_current,
                           int16_t *lift_current,
                           int16_t *suction_lift_current)
{
  int32_t claw_position;
  int32_t lift_position;
  int32_t suction_lift_position;
  int16_t claw_speed;
  int16_t lift_speed;
  int16_t suction_lift_speed;
  uint32_t claw_feedback_tick;
  uint32_t lift_feedback_tick;
  uint32_t suction_lift_feedback_tick;
  uint8_t claw_encoder_ready;
  uint8_t lift_encoder_ready;
  uint8_t suction_lift_encoder_ready;
  uint8_t outputs_enabled;
  int8_t local_lift_command;
  int8_t local_suction_lift_command;
  uint8_t claw_feedback_fresh;
  uint8_t lift_feedback_fresh;
  uint8_t suction_lift_feedback_fresh;
  float claw_feedforward;
  float lift_feedforward;
  float suction_lift_feedforward;

  __disable_irq();
  claw_position = motor_feedback[CLAW_FEEDBACK_INDEX].total_angle;
  lift_position = motor_feedback[LIFT_FEEDBACK_INDEX].total_angle;
  suction_lift_position =
      motor_feedback[SUCTION_LIFT_FEEDBACK_INDEX].total_angle;
  claw_speed = motor_feedback[CLAW_FEEDBACK_INDEX].speed;
  lift_speed = motor_feedback[LIFT_FEEDBACK_INDEX].speed;
  suction_lift_speed = motor_feedback[SUCTION_LIFT_FEEDBACK_INDEX].speed;
  claw_feedback_tick = motor_feedback[CLAW_FEEDBACK_INDEX].last_rx_tick;
  lift_feedback_tick = motor_feedback[LIFT_FEEDBACK_INDEX].last_rx_tick;
  suction_lift_feedback_tick =
      motor_feedback[SUCTION_LIFT_FEEDBACK_INDEX].last_rx_tick;
  claw_encoder_ready =
      motor_feedback[CLAW_FEEDBACK_INDEX].angle_initialized;
  lift_encoder_ready =
      motor_feedback[LIFT_FEEDBACK_INDEX].angle_initialized;
  suction_lift_encoder_ready =
      motor_feedback[SUCTION_LIFT_FEEDBACK_INDEX].angle_initialized;
  outputs_enabled = mechanism_outputs_enabled;
  local_lift_command = lift_command;
  local_suction_lift_command = suction_lift_command;
  __enable_irq();

  claw_feedback_fresh =
      ((claw_feedback_tick != 0U) &&
       ((now - claw_feedback_tick) <= MOTOR_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
  lift_feedback_fresh =
      ((lift_feedback_tick != 0U) &&
       ((now - lift_feedback_tick) <= MOTOR_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;
  suction_lift_feedback_fresh =
      ((suction_lift_feedback_tick != 0U) &&
       ((now - suction_lift_feedback_tick) <=
        MOTOR_FEEDBACK_TIMEOUT_MS)) ? 1U : 0U;

  /* The first valid encoder position after power-up is the relative zero. */
  if ((claw_position_ready == 0U) &&
      (claw_encoder_ready != 0U) &&
      (outputs_enabled != 0U))
  {
    claw_zero_counts = claw_position;
    claw_target_counts = claw_zero_counts;
    if (claw_closed_requested != 0U)
    {
      claw_target_counts +=
          CLAW_CLOSE_SIGN * OutputDegreesToCounts(CLAW_CLOSE_OUTPUT_DEG);
    }
    claw_position_ready = 1U;
  }

  if ((lift_position_ready == 0U) &&
      (lift_encoder_ready != 0U) &&
      (outputs_enabled != 0U))
  {
    lift_zero_counts = lift_position;
    lift_target_output_deg = LIFT_MIN_OUTPUT_DEG;
    lift_target_counts = lift_zero_counts +
        LIFT_UP_SIGN * OutputDegreesToCounts(lift_target_output_deg);
    lift_position_ready = 1U;
  }

  if ((suction_lift_position_ready == 0U) &&
      (suction_lift_encoder_ready != 0U) &&
      (outputs_enabled != 0U))
  {
    suction_lift_zero_counts = suction_lift_position;
    suction_lift_target_output_deg = SUCTION_LIFT_MIN_OUTPUT_DEG;
    suction_lift_target_counts = suction_lift_zero_counts +
        SUCTION_LIFT_UP_SIGN *
        OutputDegreesToCounts(suction_lift_target_output_deg);
    suction_lift_position_ready = 1U;
  }

  /* Pressing lift up/down changes the target; release freezes that target. */
  if ((lift_position_ready != 0U) &&
      (lift_feedback_fresh != 0U) &&
      (outputs_enabled != 0U))
  {
    lift_target_output_deg +=
        (float)local_lift_command * LIFT_TARGET_SPEED_DEG_PER_SEC *
        ((float)CAN_CONTROL_PERIOD_MS / 1000.0f);
    lift_target_output_deg = ClampFloat(lift_target_output_deg,
                                        LIFT_MIN_OUTPUT_DEG,
                                        LIFT_MAX_OUTPUT_DEG);
    lift_target_counts = lift_zero_counts +
        LIFT_UP_SIGN * OutputDegreesToCounts(lift_target_output_deg);
  }

  /* The ID6 suction lift uses the same hold-to-move position strategy. */
  if ((suction_lift_position_ready != 0U) &&
      (suction_lift_feedback_fresh != 0U) &&
      (outputs_enabled != 0U))
  {
    suction_lift_target_output_deg +=
        (float)local_suction_lift_command *
        SUCTION_LIFT_TARGET_SPEED_DEG_PER_SEC *
        ((float)CAN_CONTROL_PERIOD_MS / 1000.0f);
    suction_lift_target_output_deg =
        ClampFloat(suction_lift_target_output_deg,
                   SUCTION_LIFT_MIN_OUTPUT_DEG,
                   SUCTION_LIFT_MAX_OUTPUT_DEG);
    suction_lift_target_counts = suction_lift_zero_counts +
        SUCTION_LIFT_UP_SIGN *
        OutputDegreesToCounts(suction_lift_target_output_deg);
  }

  *claw_current = 0;
  *lift_current = 0;
  *suction_lift_current = 0;

  if ((outputs_enabled != 0U) &&
      (claw_position_ready != 0U) &&
      (claw_feedback_fresh != 0U))
  {
    claw_feedforward = 0.0f;
    if (claw_closed_requested != 0U)
    {
      claw_feedforward =
          (float)CLAW_CLOSE_SIGN * CLAW_HOLD_BIAS_CURRENT;
    }
    *claw_current = Position_Current(claw_target_counts,
                                     claw_position,
                                     claw_speed,
                                     CLAW_POSITION_KP,
                                     CLAW_SPEED_KD,
                                     claw_feedforward,
                                     CLAW_CURRENT_LIMIT);
  }

  if ((outputs_enabled != 0U) &&
      (lift_position_ready != 0U) &&
      (lift_feedback_fresh != 0U))
  {
    lift_feedforward = 0.0f;
    if (lift_target_output_deg > (LIFT_MIN_OUTPUT_DEG + 0.5f))
    {
      lift_feedforward =
          (float)LIFT_UP_SIGN * LIFT_GRAVITY_HOLD_CURRENT;
    }
    *lift_current = Position_Current(lift_target_counts,
                                     lift_position,
                                     lift_speed,
                                     LIFT_POSITION_KP,
                                     LIFT_SPEED_KD,
                                     lift_feedforward,
                                     LIFT_CURRENT_LIMIT);
  }

  if ((outputs_enabled != 0U) &&
      (suction_lift_position_ready != 0U) &&
      (suction_lift_feedback_fresh != 0U))
  {
    suction_lift_feedforward = 0.0f;
    if (suction_lift_target_output_deg >
        (SUCTION_LIFT_MIN_OUTPUT_DEG + 0.5f))
    {
      suction_lift_feedforward =
          (float)SUCTION_LIFT_UP_SIGN *
          SUCTION_LIFT_GRAVITY_HOLD_CURRENT;
    }
    *suction_lift_current =
        Position_Current(suction_lift_target_counts,
                         suction_lift_position,
                         suction_lift_speed,
                         SUCTION_LIFT_POSITION_KP,
                         SUCTION_LIFT_SPEED_KD,
                         suction_lift_feedforward,
                         SUCTION_LIFT_CURRENT_LIMIT);
  }
}

static int8_t Bluetooth_FilterAxis(int8_t value)
{
  if ((value >= -BT_AXIS_DEADZONE) && (value <= BT_AXIS_DEADZONE))
  {
    return 0;
  }
  if (value > 100)
  {
    return 100;
  }
  if (value < -100)
  {
    return -100;
  }
  return value;
}

static uint8_t Bluetooth_ValidatePacket(const uint8_t *packet)
{
  uint8_t checksum = 0U;
  uint8_t i;

  if ((packet[0] != BT_PACKET_HEAD) ||
      (packet[BT_PACKET_SIZE - 1U] != BT_PACKET_TAIL))
  {
    return 0U;
  }

  for (i = 1U; i <= BT_PACKET_DATA_SIZE; i++)
  {
    checksum = (uint8_t)(checksum + packet[i]);
  }
  return (checksum == packet[BT_PACKET_SIZE - 2U]) ? 1U : 0U;
}

static void Bluetooth_DecodePacket(const uint8_t *packet)
{
  bt_key_bits = (uint16_t)(((uint16_t)packet[2] << 8) | packet[1]);
  bt_vx = Bluetooth_FilterAxis((int8_t)packet[3]);
  bt_vy = Bluetooth_FilterAxis((int8_t)packet[4]);
  bt_vw = Bluetooth_FilterAxis((int8_t)packet[5]);
  last_bt_packet_tick = HAL_GetTick();
  bt_link_valid = 1U;
  bt_packet_sequence++;
  bt_valid_packet_count++;
}

static void CAN1_Start(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

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

static void DJI_PackCurrent(uint8_t *data, uint8_t index, int16_t current)
{
  uint16_t raw = (uint16_t)current;
  data[index] = (uint8_t)((raw >> 8) & 0xFFU);
  data[index + 1U] = (uint8_t)(raw & 0xFFU);
}

static void DJI_SendCurrent_0x200(int16_t id1, int16_t id2,
                                 int16_t id3, int16_t id4)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t data[8] = {0};
  uint32_t mailbox = 0U;

  header.StdId = 0x200U;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  header.TransmitGlobalTime = DISABLE;

  DJI_PackCurrent(data, 0U, id1);
  DJI_PackCurrent(data, 2U, id2);
  DJI_PackCurrent(data, 4U, id3);
  DJI_PackCurrent(data, 6U, id4);

  (void)HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

static void DJI_SendCurrent_0x1FF(int16_t id5, int16_t id6,
                                 int16_t id7, int16_t id8)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t data[8] = {0};
  uint32_t mailbox = 0U;

  header.StdId = 0x1FFU;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8U;
  header.TransmitGlobalTime = DISABLE;

  DJI_PackCurrent(data, 0U, id5);
  DJI_PackCurrent(data, 2U, id6);
  DJI_PackCurrent(data, 4U, id7);
  DJI_PackCurrent(data, 6U, id8);

  (void)HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
}

static void PID_InitAll(void)
{
  uint8_t i;

  chassis_pid[0].kp = MOTOR1_KP;
  chassis_pid[0].ki = MOTOR1_KI;
  chassis_pid[0].kd = MOTOR1_KD;
  chassis_pid[1].kp = MOTOR2_KP;
  chassis_pid[1].ki = MOTOR2_KI;
  chassis_pid[1].kd = MOTOR2_KD;
  chassis_pid[2].kp = MOTOR3_KP;
  chassis_pid[2].ki = MOTOR3_KI;
  chassis_pid[2].kd = MOTOR3_KD;

  for (i = 0U; i < 3U; i++)
  {
    chassis_pid[i].error = 0.0f;
    chassis_pid[i].last_error = 0.0f;
    chassis_pid[i].integral = 0.0f;
    chassis_pid[i].integral_max = PID_INTEGRAL_LIMIT;
    chassis_pid[i].output = 0.0f;
    chassis_pid[i].output_max = C620_CURRENT_LIMIT;
  }
}

static void PID_Reset(PID_t *pid)
{
  pid->error = 0.0f;
  pid->last_error = 0.0f;
  pid->integral = 0.0f;
  pid->output = 0.0f;
}

static float PID_Calculate(PID_t *pid, float target, float feedback)
{
  pid->error = target - feedback;
  pid->integral += pid->error;
  pid->integral = ClampFloat(pid->integral,
                             -pid->integral_max,
                             pid->integral_max);

  pid->output = pid->kp * pid->error
              + pid->ki * pid->integral
              + pid->kd * (pid->error - pid->last_error);
  pid->last_error = pid->error;
  pid->output = ClampFloat(pid->output,
                           -pid->output_max,
                           pid->output_max);
  return pid->output;
}

/*
 * A zero current command only lets a wheel coast.  When the joystick returns
 * to its centre, command zero speed instead so every chassis motor actively
 * brakes.  The small speed threshold prevents hunting after the wheel stops,
 * and the separate current limit keeps the stop from being excessively hard.
 */
static int16_t Chassis_BrakeCurrent(PID_t *pid,
                                    uint8_t feedback_index,
                                    uint32_t now)
{
  int16_t speed;
  float brake_current;

  if ((motor_feedback[feedback_index].last_rx_tick == 0U) ||
      ((now - motor_feedback[feedback_index].last_rx_tick) >
       MOTOR_FEEDBACK_TIMEOUT_MS))
  {
    PID_Reset(pid);
    return 0;
  }

  speed = motor_feedback[feedback_index].speed;
  if ((speed >= -CHASSIS_STOP_RPM_THRESHOLD) &&
      (speed <= CHASSIS_STOP_RPM_THRESHOLD))
  {
    PID_Reset(pid);
    return 0;
  }

  brake_current = PID_Calculate(pid, 0.0f, (float)speed);
  brake_current = ClampFloat(brake_current,
                             -CHASSIS_BRAKE_CURRENT_LIMIT,
                             CHASSIS_BRAKE_CURRENT_LIMIT);
  return (int16_t)brake_current;
}

/* Three wheels are installed at 90, 210 and 330 degrees. */
static void Omni_Mix(float vx, float vy, float w,
                     float speed_limit,
                     float *motor1, float *motor2, float *motor3)
{
  *motor1 = MOTOR1_DIRECTION * (-vx + w);
  *motor2 = MOTOR2_DIRECTION *
            (0.5f * vx - SQRT3_OVER_2 * vy + w);
  *motor3 = MOTOR3_DIRECTION *
            (0.5f * vx + SQRT3_OVER_2 * vy + w);

  *motor1 = ClampFloat(*motor1, -speed_limit, speed_limit);
  *motor2 = ClampFloat(*motor2, -speed_limit, speed_limit);
  *motor3 = ClampFloat(*motor3, -speed_limit, speed_limit);
}

static void Pump_On(void)
{
  HAL_GPIO_WritePin(PUMP_IN2_GPIO_Port, PUMP_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PUMP_IN1_GPIO_Port, PUMP_IN1_Pin, GPIO_PIN_SET);
}

static void Pump_Off(void)
{
  HAL_GPIO_WritePin(PUMP_IN1_GPIO_Port, PUMP_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PUMP_IN2_GPIO_Port, PUMP_IN2_Pin, GPIO_PIN_RESET);
}

static void Valve_On(void)
{
  HAL_GPIO_WritePin(VALVE_SIG_GPIO_Port, VALVE_SIG_Pin, GPIO_PIN_SET);
}

static void Valve_Off(void)
{
  HAL_GPIO_WritePin(VALVE_SIG_GPIO_Port, VALVE_SIG_Pin, GPIO_PIN_RESET);
}

static void Communication_Failsafe(void)
{
  bt_vx = 0;
  bt_vy = 0;
  bt_vw = 0;
  lift_command = 0;
  suction_lift_command = 0;
  servo_manual_direction = 0;
  auto_state = AUTO_IDLE;
  Pump_Off();
  Valve_Off();
}

static void Emergency_Stop(void)
{
  Communication_Failsafe();
  mechanism_outputs_enabled = 0U;
  claw_closed_requested = 0U;
  claw_position_ready = 0U;
  lift_position_ready = 0U;
  suction_lift_position_ready = 0U;
}

static void Bluetooth_ApplyPacket(uint16_t keys, uint32_t now)
{
  uint16_t rising_keys = (uint16_t)(keys & (uint16_t)(~last_applied_keys));

  /* Emergency has the highest priority. It remains active while the bit is 1. */
  if ((keys & BT_KEY_EMERGENCY_STOP) != 0U)
  {
    Emergency_Stop();
    last_applied_keys = keys;
    return;
  }

  mechanism_outputs_enabled = 1U;

  /* Claw commands are edge-triggered position selections, not hold-to-run. */
  if (((rising_keys & BT_KEY_CLAW_CLOSE) != 0U) &&
      ((keys & BT_KEY_CLAW_OPEN) == 0U))
  {
    claw_closed_requested = 1U;
    if (claw_position_ready != 0U)
    {
      claw_target_counts = claw_zero_counts +
          CLAW_CLOSE_SIGN * OutputDegreesToCounts(CLAW_CLOSE_OUTPUT_DEG);
    }
  }
  else if (((rising_keys & BT_KEY_CLAW_OPEN) != 0U) &&
           ((keys & BT_KEY_CLAW_CLOSE) == 0U))
  {
    claw_closed_requested = 0U;
    if (claw_position_ready != 0U)
    {
      claw_target_counts = claw_zero_counts;
    }
  }

  /* Lift remains hold-to-move; releasing both buttons holds the new target. */
  if (((keys & BT_KEY_LIFT_UP) != 0U) &&
      ((keys & BT_KEY_LIFT_DOWN) == 0U))
  {
    lift_command = 1;
  }
  else if (((keys & BT_KEY_LIFT_DOWN) != 0U) &&
           ((keys & BT_KEY_LIFT_UP) == 0U))
  {
    lift_command = -1;
  }
  else
  {
    lift_command = 0;
  }

  /* ID6 suction lift: release freezes the target and keeps position holding. */
  if (((keys & BT_KEY_SUCTION_LIFT_UP) != 0U) &&
      ((keys & BT_KEY_SUCTION_LIFT_DOWN) == 0U))
  {
    suction_lift_command = 1;
  }
  else if (((keys & BT_KEY_SUCTION_LIFT_DOWN) != 0U) &&
           ((keys & BT_KEY_SUCTION_LIFT_UP) == 0U))
  {
    suction_lift_command = -1;
  }
  else
  {
    suction_lift_command = 0;
  }

  /* Optional automatic buttons are edge-triggered. */
  if ((rising_keys & BT_KEY_AUTO_CAPTURE) != 0U)
  {
    servo_manual_direction = 0;
    Valve_Off();
    Pump_Off();
    auto_state = AUTO_FENCE_DOWN;
  }
  else if ((rising_keys & BT_KEY_AUTO_RELEASE) != 0U)
  {
    servo_manual_direction = 0;
    Valve_Off();
    auto_state_tick = now;
    auto_state = AUTO_WAIT_RETRACT;
  }
  else
  {
    /* Any manual pump/valve/servo action cancels an automatic sequence. */
    if ((keys & BT_MANUAL_OUTPUT_KEYS) != 0U)
    {
      auto_state = AUTO_IDLE;
    }

    if (auto_state == AUTO_IDLE)
    {
      /* OFF wins if ON and OFF are pressed together. */
      if ((keys & BT_KEY_PUMP_OFF) != 0U)
      {
        Pump_Off();
      }
      else if ((keys & BT_KEY_PUMP_ON) != 0U)
      {
        Pump_On();
      }

      if ((keys & BT_KEY_CYLINDER) != 0U)
      {
        Valve_On();
      }
      else
      {
        Valve_Off();
      }

      if (((keys & BT_KEY_FENCE_DOWN) != 0U) &&
          ((keys & BT_KEY_FENCE_UP) == 0U))
      {
        servo_manual_direction = 1;
      }
      else if (((keys & BT_KEY_FENCE_UP) != 0U) &&
               ((keys & BT_KEY_FENCE_DOWN) == 0U))
      {
        servo_manual_direction = -1;
      }
      else
      {
        servo_manual_direction = 0;
      }
    }
  }

  last_applied_keys = keys;
}

static uint8_t Servo_MoveToward(uint16_t target_us)
{
  if (servo_pulse_us < target_us)
  {
    if ((uint16_t)(target_us - servo_pulse_us) > SERVO_STEP_US)
    {
      servo_pulse_us = (uint16_t)(servo_pulse_us + SERVO_STEP_US);
    }
    else
    {
      servo_pulse_us = target_us;
    }
  }
  else if (servo_pulse_us > target_us)
  {
    if ((uint16_t)(servo_pulse_us - target_us) > SERVO_STEP_US)
    {
      servo_pulse_us = (uint16_t)(servo_pulse_us - SERVO_STEP_US);
    }
    else
    {
      servo_pulse_us = target_us;
    }
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pulse_us);
  return (servo_pulse_us == target_us) ? 1U : 0U;
}

static void Servo_Task(void)
{
  if (auto_state == AUTO_FENCE_DOWN)
  {
    if (Servo_MoveToward(FENCE_DOWN_US) != 0U)
    {
      Pump_On();
      auto_state_tick = HAL_GetTick();
      auto_state = AUTO_WAIT_PUMP;
    }
  }
  else if (auto_state == AUTO_FENCE_UP)
  {
    if (Servo_MoveToward(FENCE_UP_US) != 0U)
    {
      auto_state = AUTO_IDLE;
    }
  }
  else if (servo_manual_direction > 0)
  {
    (void)Servo_MoveToward(FENCE_DOWN_US);
  }
  else if (servo_manual_direction < 0)
  {
    (void)Servo_MoveToward(FENCE_UP_US);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pulse_us);
  }
}

static void Automatic_Task(uint32_t now)
{
  if ((auto_state == AUTO_WAIT_PUMP) &&
      ((now - auto_state_tick) >= PUMP_LEAD_TIME_MS))
  {
    Valve_On();
    auto_state = AUTO_HOLD_OBJECT;
  }
  else if ((auto_state == AUTO_WAIT_RETRACT) &&
           ((now - auto_state_tick) >= CYLINDER_RETRACT_TIME_MS))
  {
    Pump_Off();
    auto_state = AUTO_FENCE_UP;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint8_t index;
  uint16_t new_angle;
  int32_t angle_delta;

  if (hcan->Instance != CAN1)
  {
    return;
  }
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
  {
    return;
  }

  can_receive_count++;

  if ((header.IDE == CAN_ID_STD) &&
      (header.StdId >= 0x201U) &&
      (header.StdId <= 0x206U) &&
      (header.DLC == 8U))
  {
    index = (uint8_t)(header.StdId - 0x201U);
    new_angle = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    if (motor_feedback[index].angle_initialized == 0U)
    {
      motor_feedback[index].last_angle = new_angle;
      motor_feedback[index].total_angle = 0;
      motor_feedback[index].angle_initialized = 1U;
    }
    else
    {
      angle_delta = (int32_t)new_angle -
                    (int32_t)motor_feedback[index].last_angle;
      if (angle_delta > 4096)
      {
        angle_delta -= 8192;
      }
      else if (angle_delta < -4096)
      {
        angle_delta += 8192;
      }
      motor_feedback[index].total_angle += angle_delta;
      motor_feedback[index].last_angle = new_angle;
    }

    motor_feedback[index].angle = new_angle;
    motor_feedback[index].speed =
        (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor_feedback[index].current =
        (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor_feedback[index].temperature = data[6];
    motor_feedback[index].last_rx_tick = HAL_GetTick();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (bt_rx_index == 0U)
    {
      if (uart_rx_byte == BT_PACKET_HEAD)
      {
        bt_rx_packet[0] = uart_rx_byte;
        bt_rx_index = 1U;
      }
    }
    else
    {
      bt_rx_packet[bt_rx_index] = uart_rx_byte;
      bt_rx_index++;

      if (bt_rx_index >= BT_PACKET_SIZE)
      {
        if (Bluetooth_ValidatePacket(bt_rx_packet) != 0U)
        {
          Bluetooth_DecodePacket(bt_rx_packet);
          bt_rx_index = 0U;
        }
        else
        {
          bt_error_packet_count++;
          bt_rx_index = 0U;

          /* If the last byte is a new head, retain it for quick resync. */
          if (uart_rx_byte == BT_PACKET_HEAD)
          {
            bt_rx_packet[0] = uart_rx_byte;
            bt_rx_index = 1U;
          }
        }
      }
    }

    if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U) != HAL_OK)
    {
      bt_link_valid = 0U;
      Communication_Failsafe();
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    bt_rx_index = 0U;
    bt_link_valid = 0U;
    Communication_Failsafe();
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U);
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
  uint32_t now;
  uint32_t last_can_control_tick;
  uint32_t last_servo_control_tick;
  uint32_t last_processed_bt_sequence;
  uint8_t bt_failsafe_active;
  uint8_t chassis_was_neutral;
  /* USER CODE END 1 */

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  Pump_Off();
  Valve_Off();
  PID_InitAll();

  servo_pulse_us = SERVO_CENTER_US;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pulse_us);
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  CAN1_Start();
  if (HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1U) != HAL_OK)
  {
    Error_Handler();
  }

  last_can_control_tick = HAL_GetTick();
  last_servo_control_tick = HAL_GetTick();
  last_processed_bt_sequence = 0U;
  bt_failsafe_active = 1U;
  chassis_was_neutral = 1U;
  /* USER CODE END 2 */

  while (1)
  {
    int8_t vx_command;
    int8_t vy_command;
    int8_t vw_command;
    uint16_t key_snapshot;
    uint32_t packet_tick_snapshot;
    uint32_t packet_sequence_snapshot;
    uint8_t link_valid_snapshot;
    float vx;
    float vy;
    float vw;
    float speed_limit;
    float target1;
    float target2;
    float target3;
    int16_t current1;
    int16_t current2;
    int16_t current3;
    int16_t claw_current;
    int16_t lift_current;
    int16_t suction_lift_current;

    now = HAL_GetTick();

    __disable_irq();
    key_snapshot = bt_key_bits;
    packet_tick_snapshot = last_bt_packet_tick;
    packet_sequence_snapshot = bt_packet_sequence;
    link_valid_snapshot = bt_link_valid;
    __enable_irq();

    if ((link_valid_snapshot == 0U) ||
        ((now - packet_tick_snapshot) > BT_LOSS_TIMEOUT_MS))
    {
      if (bt_failsafe_active == 0U)
      {
        Communication_Failsafe();
        last_applied_keys = 0U;
        bt_failsafe_active = 1U;
      }
    }
    else
    {
      bt_failsafe_active = 0U;
      if (packet_sequence_snapshot != last_processed_bt_sequence)
      {
        Bluetooth_ApplyPacket(key_snapshot, now);
        last_processed_bt_sequence = packet_sequence_snapshot;
      }
    }

    if ((now - last_can_control_tick) >= CAN_CONTROL_PERIOD_MS)
    {
      last_can_control_tick = now;

      __disable_irq();
      vx_command = bt_vx;
      vy_command = bt_vy;
      vw_command = bt_vw;
      __enable_irq();

      speed_limit = CHASSIS_MAX_RPM;
      vx = CHASSIS_VX_SIGN *
           ((float)vx_command / BT_AXIS_FULL_SCALE) * speed_limit;
      vy = CHASSIS_VY_SIGN *
           ((float)vy_command / BT_AXIS_FULL_SCALE) * speed_limit;
      vw = CHASSIS_VW_SIGN *
           ((float)vw_command / BT_AXIS_FULL_SCALE) * speed_limit;
      Omni_Mix(vx, vy, vw, speed_limit, &target1, &target2, &target3);

      if ((vx_command == 0) && (vy_command == 0) && (vw_command == 0))
      {
        /* Clear the old driving integral once, then actively brake all wheels. */
        if (chassis_was_neutral == 0U)
        {
          PID_Reset(&chassis_pid[0]);
          PID_Reset(&chassis_pid[1]);
          PID_Reset(&chassis_pid[2]);
        }
        chassis_was_neutral = 1U;

        current1 = Chassis_BrakeCurrent(&chassis_pid[0], 0U, now);
        current2 = Chassis_BrakeCurrent(&chassis_pid[1], 1U, now);
        current3 = Chassis_BrakeCurrent(&chassis_pid[2], 2U, now);
      }
      else
      {
        chassis_was_neutral = 0U;
        if ((motor_feedback[0].last_rx_tick == 0U) ||
            ((now - motor_feedback[0].last_rx_tick) >
             MOTOR_FEEDBACK_TIMEOUT_MS))
        {
          PID_Reset(&chassis_pid[0]);
          current1 = 0;
        }
        else
        {
          current1 = (int16_t)PID_Calculate(&chassis_pid[0], target1,
                                            (float)motor_feedback[0].speed);
        }

        if ((motor_feedback[1].last_rx_tick == 0U) ||
            ((now - motor_feedback[1].last_rx_tick) >
             MOTOR_FEEDBACK_TIMEOUT_MS))
        {
          PID_Reset(&chassis_pid[1]);
          current2 = 0;
        }
        else
        {
          current2 = (int16_t)PID_Calculate(&chassis_pid[1], target2,
                                            (float)motor_feedback[1].speed);
        }

        if ((motor_feedback[2].last_rx_tick == 0U) ||
            ((now - motor_feedback[2].last_rx_tick) >
             MOTOR_FEEDBACK_TIMEOUT_MS))
        {
          PID_Reset(&chassis_pid[2]);
          current3 = 0;
        }
        else
        {
          current3 = (int16_t)PID_Calculate(&chassis_pid[2], target3,
                                            (float)motor_feedback[2].speed);
        }
      }

      Mechanism_Task(now,
                     &claw_current,
                     &lift_current,
                     &suction_lift_current);

      /* ID1/2/3 = chassis C620, ID4 = claw C610. */
      DJI_SendCurrent_0x200(current1, current2, current3, claw_current);
      /* ID5 = claw lift C610; ID6 = suction lift C610. */
      DJI_SendCurrent_0x1FF(lift_current, suction_lift_current, 0, 0);
    }

    if ((now - last_servo_control_tick) >= SERVO_CONTROL_PERIOD_MS)
    {
      last_servo_control_tick = now;
      Servo_Task();
    }

    Automatic_Task(now);
  }
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

  /* 8 MHz HSE / 8 * 336 / 2 = 168 MHz; PLLQ = 7 gives 48 MHz. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8U;
  RCC_OscInitStruct.PLL.PLLN = 336U;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7U;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  Emergency_Stop();
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */
