// ================= MULTI-STEERING + micro-ROS =================
#include <Arduino.h>
#include <ESP32Encoder.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/string.h>

// ================= ENCODERS =================
ESP32Encoder FrontLeft;
ESP32Encoder FrontRight;
ESP32Encoder BackLeft;
ESP32Encoder BackRight;

// ================= MOTOR PINS =================
#define MOTOR_PWM_FL 16
#define MOTOR_DIR_FL 4

#define MOTOR_PWM_FR 5
#define MOTOR_DIR_FR 17

#define MOTOR_PWM_BL 19
#define MOTOR_DIR_BL 18

#define MOTOR_PWM_BR 22
#define MOTOR_DIR_BR 21

// ================= ENCODER CPR =================
#define COUNTS_PER_REV_FL 200000.0
#define COUNTS_PER_REV_FR 200000.0
#define COUNTS_PER_REV_BL 120000.0
#define COUNTS_PER_REV_BR 120000.0

// ================= CONTROL =================
const int fixedPWM = 100;
const float angleTolerance = 1.0;

// ================= TARGETS =================
float targetFL = 0;
float targetFR = 0;
float targetBL = 0;
float targetBR = 0;

// ================= FLAGS =================
bool activeFL = false;
bool activeFR = false;
bool activeBL = false;
bool activeBR = false;

// ================= PWM =================
const int pwmFreq = 20000;
const int pwmRes = 8;
const int chFL = 0;
const int chFR = 1;
const int chBL = 2;
const int chBR = 3;

// ================= ROS OBJECTS =================
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;
rcl_timer_t timer;

rcl_subscription_t subFL, subFR, subBL, subBR;
rcl_publisher_t pubAngles;

std_msgs__msg__Float32 msgFL, msgFR, msgBL, msgBR;
std_msgs__msg__String msgAngles;

char angle_buffer[120];

// ================= HELPERS =================
float getAngle(long count, float cpr)
{
  return (count / cpr) * 360.0;
}

void motorDrive(int channel, int dirPin, int pwm)
{
  digitalWrite(dirPin, pwm >= 0);
  ledcWrite(channel, abs(pwm));
}

bool updateMotor(ESP32Encoder &enc, int channel, int dirPin,
                 float target, float cpr)
{
  float angle = getAngle(enc.getCount(), cpr);
  float err = target - angle;

  if (fabs(err) > angleTolerance)
  {
    motorDrive(channel, dirPin, (err > 0) ? fixedPWM : -fixedPWM);
    return false;
  }
  else
  {
    motorDrive(channel, dirPin, 0);
    return true;
  }
}

// ================= ROS CALLBACKS =================
void cbFL(const void *msgin)
{
  targetFL = ((std_msgs__msg__Float32 *)msgin)->data;
  activeFL = true;
}

void cbFR(const void *msgin)
{
  targetFR = ((std_msgs__msg__Float32 *)msgin)->data;
  activeFR = true;
}

void cbBL(const void *msgin)
{
  targetBL = ((std_msgs__msg__Float32 *)msgin)->data;
  activeBL = true;
}

void cbBR(const void *msgin)
{
  targetBR = ((std_msgs__msg__Float32 *)msgin)->data;
  activeBR = true;
}

// ================= TIMER (PUBLISH ANGLES) =================
void timer_callback(rcl_timer_t *, int64_t)
{
  float fl = getAngle(FrontLeft.getCount(), COUNTS_PER_REV_FL);
  float fr = getAngle(FrontRight.getCount(), COUNTS_PER_REV_FR);
  float bl = getAngle(BackLeft.getCount(), COUNTS_PER_REV_BL);
  float br = getAngle(BackRight.getCount(), COUNTS_PER_REV_BR);

  snprintf(angle_buffer, sizeof(angle_buffer),
           "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
           fl, fr, bl, br);

  msgAngles.data.data = angle_buffer;
  msgAngles.data.size = strlen(angle_buffer);
  msgAngles.data.capacity = sizeof(angle_buffer);

  rcl_publish(&pubAngles, &msgAngles, NULL);
}

// ================= SETUP =================
void setup()
{
  Serial.begin(115200);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  FrontLeft.attachHalfQuad(32, 33);
  FrontRight.attachHalfQuad(27, 26);
  BackLeft.attachHalfQuad(35, 34);
  BackRight.attachHalfQuad(39, 36);

  FrontLeft.clearCount();
  FrontRight.clearCount();
  BackLeft.clearCount();
  BackRight.clearCount();

  pinMode(MOTOR_DIR_FL, OUTPUT);
  pinMode(MOTOR_DIR_FR, OUTPUT);
  pinMode(MOTOR_DIR_BL, OUTPUT);
  pinMode(MOTOR_DIR_BR, OUTPUT);

  ledcSetup(chFL, pwmFreq, pwmRes);
  ledcSetup(chFR, pwmFreq, pwmRes);
  ledcSetup(chBL, pwmFreq, pwmRes);
  ledcSetup(chBR, pwmFreq, pwmRes);

  ledcAttachPin(MOTOR_PWM_FL, chFL);
  ledcAttachPin(MOTOR_PWM_FR, chFR);
  ledcAttachPin(MOTOR_PWM_BL, chBL);
  ledcAttachPin(MOTOR_PWM_BR, chBR);

  // ================= micro-ROS =================
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "multi_steering", "", &support);

  rclc_publisher_init_default(
      &pubAngles, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "steer/angles");

  rclc_subscription_init_default(&subFL, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "fl/target");
  rclc_subscription_init_default(&subFR, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "fr/target");
  rclc_subscription_init_default(&subBL, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "bl/target");
  rclc_subscription_init_default(&subBR, &node,
                                 ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "br/target");

  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);

  rclc_executor_init(&executor, &support.context, 5, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &subFL, &msgFL, cbFL, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subFR, &msgFR, cbFR, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subBL, &msgBL, cbBL, ON_NEW_DATA);
  rclc_executor_add_subscription(&executor, &subBR, &msgBR, cbBR, ON_NEW_DATA);
}

// ================= LOOP =================
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  if (activeFL && updateMotor(FrontLeft, chFL, MOTOR_DIR_FL, targetFL, COUNTS_PER_REV_FL))
    activeFL = false;

  if (activeFR && updateMotor(FrontRight, chFR, MOTOR_DIR_FR, targetFR, COUNTS_PER_REV_FR))
    activeFR = false;

  if (activeBL && updateMotor(BackLeft, chBL, MOTOR_DIR_BL, targetBL, COUNTS_PER_REV_BL))
    activeBL = false;

  if (activeBR && updateMotor(BackRight, chBR, MOTOR_DIR_BR, targetBR, COUNTS_PER_REV_BR))
    activeBR = false;

  delay(10);
}
