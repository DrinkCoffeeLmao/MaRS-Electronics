#include <Arduino.h>
#include <ESP32Encoder.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

// ==================== PIN DEFINITIONS ====================
#define MOTOR_PWM_LEFT 14
#define MOTOR_DIR_LEFT 13
#define ENC_LEFT_A 35  //E-5 Pin in PCB
#define ENC_LEFT_B 34

#define MOTOR_PWM_RIGHT 4
#define MOTOR_DIR_RIGHT 16

// ==================== CONSTANTS ====================
#define CPR 6500.0

const int pwmFreq = 20000;
const int pwmRes = 8;
const int chLeft = 0;
const int chRight = 1;

const int fixedPWM = 100;
const float angleTolerance = 2.0;

// ==================== OBJECTS ====================
ESP32Encoder encLeft;

// Target storage
float targetLift = 0;
float targetRotate = 0;

bool doLift = false;
bool doRotate = false;

// ROS handles
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_subscription_t subLift, subRotate;
rcl_publisher_t pubAngle;

std_msgs__msg__Float32 msgLift;
std_msgs__msg__Float32 msgRotate;
std_msgs__msg__Float32 msgAngle;

// ==================== HELPERS ====================
float getAngle(long count)
{
  return (count / CPR) * 360.0;
}

void motorLeft(int pwm)
{
  bool dir = pwm >= 0;
  digitalWrite(MOTOR_DIR_LEFT, dir);
  ledcWrite(chLeft, abs(pwm));
}

void motorRight(int pwm)
{
  bool dir = pwm >= 0;
  digitalWrite(MOTOR_DIR_RIGHT, dir);
  ledcWrite(chRight, abs(pwm));
}

// ==================== CALLBACKS ====================
void cbLift(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  targetLift = msg->data;
  doLift = true;
  doRotate = false; // disable rotate
}

void cbRotate(const void *msgin)
{
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)msgin;
  targetRotate = msg->data;
  doRotate = true;
  doLift = false; // disable lift
}

// ==================== TIMER ====================
void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  (void)last_call_time;

  msgAngle.data = getAngle(encLeft.getCount());
  rcl_publish(&pubAngle, &msgAngle, NULL);
}

// ==================== SETUP ====================
void setup()
{
  Serial.begin(115200);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encLeft.attachHalfQuad(ENC_LEFT_A, ENC_LEFT_B);
  encLeft.clearCount();

  pinMode(MOTOR_DIR_LEFT, OUTPUT);
  pinMode(MOTOR_DIR_RIGHT, OUTPUT);

  ledcSetup(chLeft, pwmFreq, pwmRes);
  ledcSetup(chRight, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_PWM_LEFT, chLeft);
  ledcAttachPin(MOTOR_PWM_RIGHT, chRight);

  // micro-ROS
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);

  rclc_node_init_default(&node, "bevel_lift_rotate", "", &support);

  // Publisher
  rclc_publisher_init_default(
      &pubAngle, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "angle");

  // Subscribers
  rclc_subscription_init_default(
      &subLift, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "lift");

  rclc_subscription_init_default(
      &subRotate, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "rotate");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);

  rclc_executor_init(&executor, &support.context, 3, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &subLift, &msgLift, cbLift, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subRotate, &msgRotate, cbRotate, ON_NEW_DATA);
}

// ==================== LOOP ====================
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  float angle = getAngle(encLeft.getCount());

  // ========= LIFT MODE =========
  if (doLift)
  {
    float err = targetLift - angle;

    if (fabs(err) < angleTolerance)
    {
      motorLeft(0);
      motorRight(0);
      doLift = false;
    }
    else
    {
      int pwm = (err > 0) ? fixedPWM : -fixedPWM;
      motorLeft(pwm);
      motorRight(pwm);
    }
  }

  // ========= ROTATE MODE =========
  if (doRotate)
  {
    float err = targetRotate - angle;

    if (fabs(err) < angleTolerance)
    {
      motorLeft(0);
      motorRight(0);
      doRotate = false;
    }
    else
    {
      int pwm = (err > 0) ? fixedPWM : -fixedPWM;
      motorLeft(pwm);
      motorRight(-pwm); // opposite for rotation
    }
  }

  delay(10);
}
