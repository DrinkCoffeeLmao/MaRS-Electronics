#include <Arduino.h>
#include <ESP32Encoder.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/string.h>
#include <string.h>
#include <math.h>

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
#define MOTOR_DIR_BR 23

// ================= CPR =================
#define COUNTS_PER_REV_FL 200000.0
#define COUNTS_PER_REV_FR 200000.0
#define COUNTS_PER_REV_BL 200000.0
#define COUNTS_PER_REV_BR 200000.0

// ================= CONTROL =================
const int fixedPWM = 150;
const float angleTolerance = 2.0;

// ================= PWM =================
const int pwmFreq = 20000;
const int pwmRes = 8;
const int chFL = 0, chFR = 1, chBL = 2, chBR = 3;

// ================= micro-ROS =================
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t steering_sub;
rcl_publisher_t debug_pub;

std_msgs__msg__Int16 steering_msg;
std_msgs__msg__String debug_msg;
char debug_buf[256];

unsigned long last_cmd_time = 0;

// ================= SPOT STATE =================
bool spot_active = false;

float targetFL, targetFR, targetBL, targetBR;

// ================= MACROS =================
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) while (1); }

// ================= DEBUG =================
void debug_log(const char *text)
{
  strncpy(debug_buf, text, sizeof(debug_buf) - 1);
  debug_buf[sizeof(debug_buf) - 1] = '\0';
  debug_msg.data.data = debug_buf;
  debug_msg.data.size = strlen(debug_buf);
  debug_msg.data.capacity = sizeof(debug_buf);
  rcl_publish(&debug_pub, &debug_msg, NULL);
}
float getAngle(long count, float cpr)
{
  return (count / cpr) * 360.0;
}
void publishAngles()
{
  float fl = getAngle(FrontLeft.getCount(), COUNTS_PER_REV_FL);
  float fr = getAngle(FrontRight.getCount(), COUNTS_PER_REV_FR);
  float bl = getAngle(BackLeft.getCount(), COUNTS_PER_REV_BL);
  float br = getAngle(BackRight.getCount(), COUNTS_PER_REV_BR);

  char buf[120];
  snprintf(buf, sizeof(buf),
           "FL: %.2f | FR: %.2f | BL: %.2f | BR: %.2f",
           fl, fr, bl, br);

  debug_log(buf);
}


// ================= HELPERS =================


void motorDrive(int channel, int dirPin, int pwm)
{
  digitalWrite(dirPin, pwm >= 0);
  ledcWrite(channel, abs(pwm));
}

bool updateMotor(ESP32Encoder &enc, int ch, int dir,
                 float target, float cpr, const char *name)
{
  float angle = getAngle(enc.getCount(), cpr);
  float error = target - angle;

  if (fabs(error) > angleTolerance)
  {
    motorDrive(ch, dir, (error > 0) ? fixedPWM : -fixedPWM);
    return false;
  }
  else
  {
    motorDrive(ch, dir, 0);
    return true;
  }
}

// ================= ROS CALLBACK =================
void steering_cmd_callback(const void *msgin)
{
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  int cmd = msg->data;
  last_cmd_time = millis();

  switch (cmd)
  {
  case 0:
    motorDrive(chFL, MOTOR_DIR_FL, 0);
    motorDrive(chFR, MOTOR_DIR_FR, 0);
    motorDrive(chBL, MOTOR_DIR_BL, 0);
    motorDrive(chBR, MOTOR_DIR_BR, 0);
    spot_active = false;
    break;

  // -------- MANUAL CONTROL --------
  case 1: motorDrive(chFL, MOTOR_DIR_FL, fixedPWM); break;
  case 2: motorDrive(chFL, MOTOR_DIR_FL, -fixedPWM); break;
  case 3: motorDrive(chFR, MOTOR_DIR_FR, fixedPWM); break;
  case 4: motorDrive(chFR, MOTOR_DIR_FR, -fixedPWM); break;
  case 5: motorDrive(chBL, MOTOR_DIR_BL, fixedPWM); break;
  case 6: motorDrive(chBL, MOTOR_DIR_BL, -fixedPWM); break;
  case 7: motorDrive(chBR, MOTOR_DIR_BR, fixedPWM); break;
  case 8: motorDrive(chBR, MOTOR_DIR_BR, -fixedPWM); break;
  case 25:motorDrive(chFL, MOTOR_DIR_FL, fixedPWM);motorDrive(chFR, MOTOR_DIR_FR, fixedPWM); break;
  case 50:motorDrive(chFL, MOTOR_DIR_FL, -fixedPWM);motorDrive(chFR, MOTOR_DIR_FR, -fixedPWM); break;

  // -------- SPOT TURN CW 45 DEG --------
  case 10:
  {
    targetFL = getAngle(FrontLeft.getCount(), COUNTS_PER_REV_FL) + 45.0;
    targetFR = getAngle(FrontRight.getCount(), COUNTS_PER_REV_FR) - 45.0;
    targetBL = getAngle(BackLeft.getCount(), COUNTS_PER_REV_BL) + 45.0;
    targetBR = getAngle(BackRight.getCount(), COUNTS_PER_REV_BR) - 45.0;
    spot_active = true;
    debug_log("SPOT CW 45 DEG");
    break;
  }

  // -------- SPOT TURN CCW 45 DEG --------
  case 11:
  {
    targetFL = getAngle(FrontLeft.getCount(), COUNTS_PER_REV_FL) - 45.0;
    targetFR = getAngle(FrontRight.getCount(), COUNTS_PER_REV_FR) + 45.0;
    targetBL = getAngle(BackLeft.getCount(), COUNTS_PER_REV_BL) - 45.0;
    targetBR = getAngle(BackRight.getCount(), COUNTS_PER_REV_BR) + 45.0;
    spot_active = true;
    debug_log("SPOT CCW 45 DEG");
    break;
  }
  }
}

// ================= PWM SETUP =================
void setupPWM()
{
  ledcSetup(chFL, pwmFreq, pwmRes);
  ledcSetup(chFR, pwmFreq, pwmRes);
  ledcSetup(chBL, pwmFreq, pwmRes);
  ledcSetup(chBR, pwmFreq, pwmRes);

  ledcAttachPin(MOTOR_PWM_FL, chFL);
  ledcAttachPin(MOTOR_PWM_FR, chFR);
  ledcAttachPin(MOTOR_PWM_BL, chBL);
  ledcAttachPin(MOTOR_PWM_BR, chBR);
}

// ================= SETUP =================
void setup()
{
  ESP32Encoder::useInternalWeakPullResistors = puType::up;

  FrontLeft.attachHalfQuad(33, 32);
  FrontRight.attachHalfQuad(26, 27);
  BackLeft.attachHalfQuad(35, 34);
  BackRight.attachHalfQuad(39, 36);

  pinMode(MOTOR_DIR_FL, OUTPUT);
  pinMode(MOTOR_DIR_FR, OUTPUT);
  pinMode(MOTOR_DIR_BL, OUTPUT);
  pinMode(MOTOR_DIR_BR, OUTPUT);

  setupPWM();

  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "steering_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &debug_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "steer_debug"));

  RCCHECK(rclc_subscription_init_default(
      &steering_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "steer_cmd"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &steering_sub, &steering_msg,
      &steering_cmd_callback, ON_NEW_DATA));

  debug_log("STEERING NODE READY");
}

// ================= LOOP =================
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // ---- SPOT MODE ----
  if (spot_active)
  {
    bool d1 = updateMotor(FrontLeft, chFL, MOTOR_DIR_FL, targetFL, COUNTS_PER_REV_FL, "FL");
    bool d2 = updateMotor(FrontRight, chFR, MOTOR_DIR_FR, targetFR, COUNTS_PER_REV_FR, "FR");
    bool d3 = updateMotor(BackLeft, chBL, MOTOR_DIR_BL, targetBL, COUNTS_PER_REV_BL, "BL");
    bool d4 = updateMotor(BackRight, chBR, MOTOR_DIR_BR, targetBR, COUNTS_PER_REV_BR, "BR");

    publishAngles();   // ✅ CONTINUOUS ANGLE PRINT

    if (d1 && d2 && d3 && d4)
    {
      spot_active = false;
      debug_log("SPOT DONE");
    }

    delay(50);         // limit spam
    return;
  }

  // ---- NORMAL MODE ----
  publishAngles();     // ✅ CONTINUOUS ANGLE PRINT

  if (millis() - last_cmd_time > 100)
  {
    motorDrive(chFL, MOTOR_DIR_FL, 0);
    motorDrive(chFR, MOTOR_DIR_FR, 0);
    motorDrive(chBL, MOTOR_DIR_BL, 0);
    motorDrive(chBR, MOTOR_DIR_BR, 0);
  }

  delay(50);           // ~20 Hz logging
}

