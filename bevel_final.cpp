#include <Arduino.h>
#include <ESP32Encoder.h>
//#include <ESP32Servo.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>

// ================== PIN DEFINITIONS ==================
#define MOTOR_PWM1 14
#define MOTOR_DIR1 13
#define MOTOR_PWM2 4
#define MOTOR_DIR2 16
#define GRIP_DIR 17
#define GRIP_PWM 5

#define ENCODER1_A 34
#define ENCODER1_B 35
#define ENCODER2_A 39
#define ENCODER2_B 36

#define LIMIT_LIFT_UP 25
#define LIMIT_LIFT_DOWN 26

// ================== CONSTANTS ==================
#define CPR 6500.0
#define GEAR_RATIO 1.0

const int pwmFreq = 20000;
const int pwmRes = 8;
const int ch1 = 0;
const int ch2 = 1;
const int ch3 = 2;
const int fixedPWM = 100;
const int gripperPWM = 255

// Gripper timeout (ms)
const unsigned long GRIP_TIMEOUT = 800;

// ================== GLOBALS ==================
ESP32Encoder enc1, enc2;

float targetBevel = 0, targetLift = 0;
float currentBevel = 0, currentLift = 0;
float currentAngleL = 0, currentAngleR = 0;

bool movingBevel = false;
bool movingLift = false;
bool movingGripper = false;

int bevelDir = 1;
int liftDir = 1;
int gripDir = 1;
unsigned long gripStartTime = 0;

// ================== micro-ROS ==================
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t bevel_cmd_sub;
rcl_subscription_t grip_cmd_sub;

rcl_publisher_t pub_bevel_angle;
rcl_publisher_t pub_lift_angle;
rcl_publisher_t pub_limit_hit;

std_msgs__msg__Int16 bevel_cmd_msg;
std_msgs__msg__Int16 grip_cmd_msg;
std_msgs__msg__Float32 msg_bevel_angle;
std_msgs__msg__Float32 msg_lift_angle;
std_msgs__msg__Bool msg_limit_hit;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { while (1); } }

// ================== HELPERS ==================
float getJointAngle(long counts)
{
  return (counts / (CPR * GEAR_RATIO * 5.56)) * 360.0;
}

float getAveragedAngle()
{
  currentAngleL = getJointAngle(enc1.getCount());
  currentAngleR = getJointAngle(enc2.getCount());
  return currentAngleR;
}

void stopMotors()
{
  ledcWrite(ch1, 0);
  ledcWrite(ch2, 0);
}

void stopGripper()
{
  ledcWrite(ch3, 0);
}

void driveMotors(int pwm, bool sameDirection, int dir)
{
  digitalWrite(MOTOR_DIR1, dir >= 0);
  digitalWrite(MOTOR_DIR2, sameDirection ? (dir >= 0) : (dir < 0));
  ledcWrite(ch1, abs(pwm));
  ledcWrite(ch2, abs(pwm));
}

void driveGripper(int pwm, int dir)
{
  digitalWrite(GRIP_DIR, dir >= 0);
  ledcWrite(ch3, abs(pwm));
}

// ================== CALLBACKS ==================
void bevel_cmd_callback(const void *msgin)
{
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;

  currentBevel = getAveragedAngle();
  currentLift = currentBevel;

  movingBevel = false;
  movingLift = false;

  int cmd = msg->data;

  if (cmd & (1 << 0)) { targetBevel = currentBevel + 10; bevelDir = 1; movingBevel = true; }
  if (cmd & (1 << 1)) { targetBevel = currentBevel - 10; bevelDir = -1; movingBevel = true; }
  if (cmd & (1 << 2)) { targetLift  = currentLift  + 10; liftDir  = 1; movingLift  = true; }
  if (cmd & (1 << 3)) { targetLift  = currentLift  - 10; liftDir  = -1; movingLift  = true; }
}

void grip_cmd_callback(const void *msgin)
{
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;

  int cmd = msg->data;

  if (cmd & (1 << 4)) {      // Close
      gripDir = 1;
      movingGripper = true;
      gripStartTime = millis();
  }
  if (cmd & (1 << 5)) {      // Open
      gripDir = -1;
      movingGripper = true;
      gripStartTime = millis();
  }
}

// ================== SETUP ==================
void setup()
{
  Serial.begin(115200);
  delay(2000);

  // Encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc1.attachHalfQuad(ENCODER1_A, ENCODER1_B);
  enc2.attachHalfQuad(ENCODER2_A, ENCODER2_B);
  enc1.clearCount();
  enc2.clearCount();

  // Motors
  pinMode(MOTOR_DIR1, OUTPUT);
  pinMode(MOTOR_DIR2, OUTPUT);
  pinMode(GRIP_DIR, OUTPUT);

  // Limit switches
  pinMode(LIMIT_LIFT_UP, INPUT_PULLUP);
  pinMode(LIMIT_LIFT_DOWN, INPUT_PULLUP);

  // LEDC setup
  ledcSetup(ch1, pwmFreq, pwmRes);
  ledcSetup(ch2, pwmFreq, pwmRes);
  ledcSetup(ch3, pwmFreq, pwmRes);

  ledcAttachPin(MOTOR_PWM1, ch1);
  ledcAttachPin(MOTOR_PWM2, ch2);
  ledcAttachPin(GRIP_PWM, ch3);

  // micro-ROS init
  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "bevel_node", "", &support));

  // Publishers
  RCCHECK(rclc_publisher_init_default(&pub_bevel_angle, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "bevel_angle"));
  RCCHECK(rclc_publisher_init_default(&pub_lift_angle, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "lift_angle"));
  RCCHECK(rclc_publisher_init_default(&pub_limit_hit, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "limit_hit"));

  // Subscribers
  RCCHECK(rclc_subscription_init_default(&bevel_cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "bevel_cmd"));
  RCCHECK(rclc_subscription_init_default(&grip_cmd_sub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "grip_cmd"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor,
    &bevel_cmd_sub, &bevel_cmd_msg, &bevel_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor,
    &grip_cmd_sub, &grip_cmd_msg, &grip_cmd_callback, ON_NEW_DATA));
}

// ================== LOOP ==================
void loop()
{
  // Handle ROS callbacks
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Limit switches
  if (!digitalRead(LIMIT_LIFT_UP) && movingLift && liftDir > 0)
  {
    enc1.clearCount();
    enc2.clearCount();
    stopMotors();
    movingLift = movingBevel = false;
  }

  currentBevel = getAveragedAngle();
  currentLift = currentBevel;

  int pwmBevel = 0;
  int pwmLift = 0;

  if (movingBevel && abs(currentBevel - targetBevel) > 1.0)
    pwmBevel = fixedPWM * bevelDir;
  if (movingLift && abs(currentLift - targetLift) > 1.0)
    pwmLift = fixedPWM * liftDir;

  if (pwmBevel && pwmLift)
    driveMotors(pwmLift, false, liftDir);
  else if (pwmBevel)
    driveMotors(pwmBevel, true, bevelDir);
  else if (pwmLift)
    driveMotors(pwmLift, false, liftDir);
  else
    stopMotors();
  
  // === Gripper Control with timeout ===
  if (movingGripper)
  {
      if (millis() - gripStartTime > GRIP_TIMEOUT)
      {
          movingGripper = false;
          stopGripper();
      }
      else
      {
          driveGripper(gripperPWM, gripDir);
      }
  }
  else
  {
      stopGripper();
  }

  // Publish feedback
  msg_bevel_angle.data = currentBevel;
  msg_lift_angle.data = currentLift;
  msg_limit_hit.data = !digitalRead(LIMIT_LIFT_UP) || !digitalRead(LIMIT_LIFT_DOWN);

  rcl_publish(&pub_bevel_angle, &msg_bevel_angle, NULL);
  rcl_publish(&pub_lift_angle, &msg_lift_angle, NULL);
  rcl_publish(&pub_limit_hit, &msg_limit_hit, NULL);
}
