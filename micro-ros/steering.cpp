#include <Arduino.h>
#include <ESP32Encoder.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/string.h>
#include <string.h>

// --- Encoders ---
ESP32Encoder FrontLeft;
ESP32Encoder FrontRight;
ESP32Encoder BackLeft;
ESP32Encoder BackRight;

// --- Motor pins ---
#define MOTOR_PWM_FL 16
#define MOTOR_DIR_FL 4

#define MOTOR_PWM_FR 5
#define MOTOR_DIR_FR 17

#define MOTOR_PWM_BL 22
#define MOTOR_DIR_BL 23

#define MOTOR_PWM_BR 14
#define MOTOR_DIR_BR 13

// --- Encoder CPR ---
#define COUNTS_PER_REV_FL 200000.0
#define COUNTS_PER_REV_FR 200000.0
#define COUNTS_PER_REV_BL 20000.0
#define COUNTS_PER_REV_BR 20000.0

// --- Control params ---
const int fixedPWM = 75;
const float angleTolerance = 2.0;
const float stepAngle = 5.0;

// --- PWM ---
const int pwmFreq = 20000;
const int pwmRes = 8;
const int chFL = 0, chFR = 1, chBL = 2, chBR = 3;

// --- micro-ROS ---
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t steering_sub;
rcl_publisher_t debug_pub;

std_msgs__msg__Int16 steering_msg;
static std_msgs__msg__String debug_msg;
static char debug_buf[256];

unsigned long last_cmd_time = 0;
#define RCCHECK(fn)       \
  {                       \
    rcl_ret_t rc = fn;    \
    if (rc != RCL_RET_OK) \
    {                     \
      while (1)           \
        ;                 \
    }                     \
  }

// --- Debug publisher ---
void debug_log(const char *text)
{
  strncpy(debug_buf, text, sizeof(debug_buf) - 1);
  debug_buf[sizeof(debug_buf) - 1] = '\0';

  debug_msg.data.data = debug_buf;
  debug_msg.data.size = strlen(debug_buf);
  debug_msg.data.capacity = sizeof(debug_buf);

  rcl_publish(&debug_pub, &debug_msg, NULL);
}

// --- Helpers ---
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

// --- ROS callback ---
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
    break;

  case 1:
    motorDrive(chFL, MOTOR_DIR_FL, fixedPWM);
    debug_log("FrontLeft()++");
    break;

  case 2:
    motorDrive(chFL, MOTOR_DIR_FL, -fixedPWM);
    debug_log("FrontLeft()--");
    break;

  case 3:
    motorDrive(chFR, MOTOR_DIR_FR, fixedPWM);
    debug_log("FrontRight()--");
    break;
  case 4:
    motorDrive(chFR, MOTOR_DIR_FR, -fixedPWM);
    debug_log("FrontRight()--");
    break;

  case 5:
    motorDrive(chBL, MOTOR_DIR_BL, fixedPWM);
    debug_log("BackLeft())++");
    break;
  case 6:
    motorDrive(chBL, MOTOR_DIR_BL, -fixedPWM);
    debug_log("BackLeft()--");
    break;

  case 7:
    motorDrive(chBR, MOTOR_DIR_BR, fixedPWM);
    debug_log("BackRight()++");
    break;
  case 8:
    motorDrive(chBR, MOTOR_DIR_BR, -fixedPWM);
    debug_log("BackRight()--");
    break;

  case 25:
    motorDrive(chFL, MOTOR_DIR_FL, -fixedPWM);
    motorDrive(chFR, MOTOR_DIR_FR, fixedPWM);
    motorDrive(chBL, MOTOR_DIR_BL, -fixedPWM);
    motorDrive(chBR, MOTOR_DIR_BR, fixedPWM);

    if (FrontRight.getCount() == 45.000000)
    {
      motorDrive(chFL, MOTOR_DIR_FL, 0);
      motorDrive(chFR, MOTOR_DIR_FR, 0);
      motorDrive(chBL, MOTOR_DIR_BL, 0);
      motorDrive(chBR, MOTOR_DIR_BR, 0);
    }
    break;

  case 50:
    motorDrive(chFL, MOTOR_DIR_FL, fixedPWM);
    motorDrive(chFR, MOTOR_DIR_FR, -fixedPWM);
    motorDrive(chBL, MOTOR_DIR_BL, fixedPWM);
    motorDrive(chBR, MOTOR_DIR_BR, -fixedPWM);
    break;

  default:
    debug_log("Invalid steering command");
    return;
  }

  char buf[64];
  snprintf(buf, sizeof(buf), "Steering cmd: %d", cmd);
  debug_log(buf);
}

// --- PWM setup ---
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

// --- SETUP ---
void setup()
{
  // Encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  FrontLeft.attachHalfQuad(26, 27);
  FrontRight.attachHalfQuad(33, 32);
  BackLeft.attachHalfQuad(34, 35);
  BackRight.attachHalfQuad(39, 36);

  FrontLeft.clearCount();
  FrontRight.clearCount();
  BackLeft.clearCount();
  BackRight.clearCount();

  // Motors
  pinMode(MOTOR_DIR_FL, OUTPUT);
  pinMode(MOTOR_DIR_FR, OUTPUT);
  pinMode(MOTOR_DIR_BL, OUTPUT);
  pinMode(MOTOR_DIR_BR, OUTPUT);
  setupPWM();

  // micro-ROS
  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "steering_node", "", &support));

  RCCHECK(rclc_publisher_init_default(
      &debug_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "steering_debug"));

  RCCHECK(rclc_subscription_init_default(
      &steering_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "steering_cmd"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &steering_sub, &steering_msg, &steering_cmd_callback, ON_NEW_DATA));

  debug_log("Steering node initialized!");
}

// --- LOOP ---
void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  unsigned long count = BackRight.getCount();

  Serial.print(count);

  float Fl = getAngle(FrontLeft.getCount(), COUNTS_PER_REV_FL);
  float Fr = getAngle(FrontRight.getCount(), COUNTS_PER_REV_FR);
  float Bl = getAngle(BackLeft.getCount(), COUNTS_PER_REV_BL);
  float Br = getAngle(BackRight.getCount(), COUNTS_PER_REV_BR);

  char buf[100];
  snprintf(buf, sizeof(buf), "FrontLeft: %.2f | FrontRight: %.2f | BackLeft: %.2f |BackRight: %.2f| Err(FL,FR,BL,BR): %.2f, %.2f, %.2f,%.2f",
           Fl, Fr, Bl, Br,
           -Fl, -Fr, -Bl, -Br);
  debug_log(buf);

  delay(100);
}
