#include <Arduino.h>
#include <ESP32Encoder.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/string.h>

// ===================== PIN DEFINITIONS =====================

#define MOTOR_PWM_BASE 16
#define MOTOR_DIR_BASE 4
#define ENCODER_BASE_A 33
#define ENCODER_BASE_B 32

#define MOTOR_PWM_L1 5
#define MOTOR_DIR_L1 17
#define ENCODER_L1_A 35
#define ENCODER_L1_B 34

#define MOTOR_PWM_L2 22
#define MOTOR_DIR_L2 21
#define ENCODER_L2_A 39
#define ENCODER_L2_B 36

// ===================== CPR VALUES =====================

#define CPR_BASE 120000.0
#define CPR_L1 37500.0
#define CPR_L2 120000.0

// ===================== PWM CONFIG =====================

const int pwmFreq = 20000;
const int pwmRes = 8;
const int chBase = 0;
const int chL1 = 1;
const int chL2 = 2;

const int fixedPWM = 100;
const float angleTolerance = 2.0;

// ===================== ENCODERS =====================

ESP32Encoder encBase;
ESP32Encoder encL1;
ESP32Encoder encL2;

// ===================== TARGETS =====================

float targetBase = 0;
float targetL1 = 0;
float targetL2 = 0;

bool activeBase = false;
bool activeL1 = false;
bool activeL2 = false;

// ===================== ✅ SAFETY =====================

unsigned long lastCmdTime = 0;

// === RCCHECK ===
#define RCCHECK(fn)       \
  {                       \
    rcl_ret_t rc = fn;    \
    if (rc != RCL_RET_OK) \
    {                     \
      while (1)           \
        ;                 \
    }                     \
  }

// ===================== ROS CORE =====================

rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t manipulator_cmd_sub;
rcl_publisher_t debug_pub;

std_msgs__msg__Int16 manipulator_cmd_msg;
std_msgs__msg__String debug_msg;

void debug_log(const char *msg)
{
  debug_msg.data.data = (char *)msg;
  debug_msg.data.size = strlen(msg);
  debug_msg.data.capacity = debug_msg.data.size + 1;
  rcl_publish(&debug_pub, &debug_msg, NULL);
}

// ===================== UTILS =====================

float getAngle(long count, float cpr)
{
  return (count / cpr) * 360.0;
}

void motorDrive(int channel, int dirPin, int pwm)
{
  bool dir = pwm >= 0;
  digitalWrite(dirPin, dir);
  ledcWrite(channel, abs(pwm));
}

bool updateMotor(ESP32Encoder &enc, int channel, int dirPin,
                 float target, float cpr, const char *name)
{
  float angle = getAngle(enc.getCount(), cpr);
  float error = target - angle;
  bool done = fabs(error) <= angleTolerance;

  if (!done)
  {
    int pwm = (error > 0) ? fixedPWM : -fixedPWM;
    motorDrive(channel, dirPin, pwm);
  }
  else
  {
    motorDrive(channel, dirPin, 0);
  }

  return done;
}

void manipulator_cmd_callback(const void *msgin)
{
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  int cmd = (int)msg->data;

  lastCmdTime = millis();

  switch (cmd)
  {

  case 0: // ✅ EMERGENCY STOP
    motorDrive(chBase, MOTOR_DIR_BASE, 0);
    motorDrive(chL1, MOTOR_DIR_L1, 0);
    motorDrive(chL2, MOTOR_DIR_L2, 0);
    debug_log("STOP");
    break;

  case 1:
    targetBase += 5;
    activeBase = true;
    debug_log("Base taget+5");
    break;

  case 2:
    targetBase -= 5;
    activeBase = true;
    debug_log("Base taget-5");
    break;

  case 3:
    targetL1 += 5;
    activeL1 = true;
    debug_log("Link1 taget+5");
    break;

  case 4:
    targetL1 -= 5;
    activeL1 = true;
    debug_log("Link1 taget-5");
    break;

  case 5:
    targetL2 += 5;
    activeL2 = true;
    debug_log("Link2 taget+5");
    break;

  case 6:
    targetL2 -= 5;
    activeL2 = true;
    debug_log("Link2 taget-5");
    break;

  default:
    debug_log("Invalid command");
    return;
  }
}

// ===================== SETUP =====================

void setup()
{

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  encBase.attachHalfQuad(ENCODER_BASE_A, ENCODER_BASE_B);
  encL1.attachHalfQuad(ENCODER_L1_A, ENCODER_L1_B);
  encL2.attachHalfQuad(ENCODER_L2_A, ENCODER_L2_B);

  encBase.clearCount();
  encL1.clearCount();
  encL2.clearCount();

  pinMode(MOTOR_DIR_BASE, OUTPUT);
  pinMode(MOTOR_DIR_L1, OUTPUT);
  pinMode(MOTOR_DIR_L2, OUTPUT);

  ledcSetup(chBase, pwmFreq, pwmRes);
  ledcSetup(chL1, pwmFreq, pwmRes);
  ledcSetup(chL2, pwmFreq, pwmRes);

  ledcAttachPin(MOTOR_PWM_BASE, chBase);
  ledcAttachPin(MOTOR_PWM_L1, chL1);
  ledcAttachPin(MOTOR_PWM_L2, chL2);

  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "manipulator_node", "", &support);

  RCCHECK(rclc_publisher_init_default(
      &debug_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "manipulator_cmd"));

  RCCHECK(rclc_subscription_init_default(
      &manipulator_cmd_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "manipulator_debug"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  RCCHECK((
      &executor,
      &manipulator_cmd_sub,
      &manipulator_cmd_msg,
      &manipulator_cmd_callback,
      ON_NEW_DATA));
  debug_log("Manipulator ready!");
}

// ===================== LOOP =====================

void loop()
{

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  if (millis() - lastCmdTime > 100)
  {
    motorDrive(chBase, MOTOR_DIR_BASE, 0);
    motorDrive(chL1, MOTOR_DIR_L1, 0);
    motorDrive(chL2, MOTOR_DIR_L2, 0);
  }

  if (activeBase && updateMotor(encBase, chBase, MOTOR_DIR_BASE, targetBase, CPR_BASE, "Base"))
    activeBase = false;

  if (activeL1 && updateMotor(encL1, chL1, MOTOR_DIR_L1, targetL1, CPR_L1, "L1"))
    activeL1 = false;

  if (activeL2 && updateMotor(encL2, chL2, MOTOR_DIR_L2, targetL2, CPR_L2, "L2"))
    activeL2 = false;

  // Read encoder angles and print angle + error (relative to 0)
  float angleBase = getAngle(encBase.getCount(), CPR_BASE);
  float angleL1 = getAngle(encL1.getCount(), CPR_L1);
  float angleL2 = getAngle(encL2.getCount(), CPR_L2);

  char buf[100];
  snprintf(buf, sizeof(buf), "Base: %.2f | L1: %.2f | L2: %.2f | Err(B,L1,L2): %.2f, %.2f, %.2f",
           angleBase, angleL1, angleL2,
           -angleBase, -angleL1, -angleL2);
  debug_log(buf);
  delay(10);
}
