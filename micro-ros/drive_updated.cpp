#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>

#include <string.h> // for strlen, strncpy, snprintf

#define PWM_FREQ 5000
#define PWM_RES 8

#define DIR_FL 4
#define PWM_FL 16
#define CH_FL 0

#define DIR_FR 17
#define PWM_FR 5
#define CH_FR 1

#define DIR_BL 18
#define PWM_BL 19
#define CH_BL 2

#define DIR_BR 21
#define PWM_BR 22
#define CH_BR 3

int drive_speed = 30;
int current_speed = 0;
int current_dir = 0; // -1 = backward, 0 = stopped, +1 = forward

const int RAMP_STEP = 5;
const int RAMP_DELAY = 25;

class Motor
{
  int dir_pin, pwm_pin, ch;

public:
  Motor(int dir, int pwm, int ch_num)
  {
    dir_pin = dir;
    pwm_pin = pwm;
    ch = ch_num;
    pinMode(dir_pin, OUTPUT);
    ledcSetup(ch, PWM_FREQ, PWM_RES);
    ledcAttachPin(pwm_pin, ch);
    ledcWrite(ch, 0);
  }
  void forward() { digitalWrite(dir_pin, HIGH); }
  void backward() { digitalWrite(dir_pin, LOW); }
  void setSpeed(int val) { ledcWrite(ch, constrain(val, 0, 255)); }
  void stop() { ledcWrite(ch, 0); }
};

// === Motor Objects ===
Motor mFL(DIR_FL, PWM_FL, CH_FL);
Motor mFR(DIR_FR, PWM_FR, CH_FR);
Motor mBL(DIR_BL, PWM_BL, CH_BL);
Motor mBR(DIR_BR, PWM_BR, CH_BR);

// === Drive Controls ===
void applySpeedToDrives(int pwm)
{
  mFL.setSpeed(pwm);
  mFR.setSpeed(pwm);
  mBL.setSpeed(pwm);
  mBR.setSpeed(pwm);
}

void setDriveDirection(int dir)
{
  if (dir > 0)
  {
    mFL.forward();
    mFR.forward();
    mBL.forward();
    mBR.forward();
  }
  else if (dir < 0)
  {
    mFL.backward();
    mFR.backward();
    mBL.backward();
    mBR.backward();
  }
}

void smoothRampTo(int target_speed)
{
  if (target_speed < 0)
    target_speed = 0;
  if (target_speed > 255)
    target_speed = 255;

  while (current_speed < target_speed)
  {
    current_speed += RAMP_STEP;
    if (current_speed > target_speed)
      current_speed = target_speed;
    applySpeedToDrives(current_speed);
    delay(RAMP_DELAY);
  }

  while (current_speed > target_speed)
  {
    current_speed -= RAMP_STEP;
    if (current_speed < target_speed)
      current_speed = target_speed;
    applySpeedToDrives(current_speed);
    delay(RAMP_DELAY);
  }
}

void smoothStopDrive()
{
  smoothRampTo(0);
  current_dir = 0;
  applySpeedToDrives(0);
}

void driveForward()
{
  if (current_dir == -1 && current_speed > 0)
    smoothStopDrive();

  setDriveDirection(+1);
  current_dir = 1;
  smoothRampTo(drive_speed);
}

void driveBackward()
{
  if (current_dir == 1 && current_speed > 0)
    smoothStopDrive();

  setDriveDirection(-1);
  current_dir = -1;
  smoothRampTo(drive_speed);
}

void stopDrive()
{
  smoothStopDrive();
}

// === Spot Turn Controls ===
void spotTurnLeft()
{
  // Left motors backward, right motors forward
  mFL.backward();
  mBL.backward();
  mFR.forward();
  mBR.forward();

  applySpeedToDrives(drive_speed);
}

void spotTurnRight()
{
  // Left motors forward, right motors backward
  mFL.forward();
  mBL.forward();
  mFR.backward();
  mBR.backward();

  applySpeedToDrives(drive_speed);
}

void stopTurn()
{
  applySpeedToDrives(0);
}

// === micro-ROS Setup ===
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t debug;
rclc_executor_t executor;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      while (1)                  \
        ;                        \
    }                            \
  }

typedef std_msgs__msg__Int32 CommandMsg;
CommandMsg msg;

// Debug publisher setup
static std_msgs__msg__String debug_msg;
static char debug_buf[256];

void publish_debug(const char *text)
{
  strncpy(debug_buf, text, sizeof(debug_buf) - 1);
  debug_buf[sizeof(debug_buf) - 1] = '\0';

  debug_msg.data.data = debug_buf;
  debug_msg.data.size = strlen(debug_buf);
  debug_msg.data.capacity = sizeof(debug_buf);

  rcl_publish(&debug, &debug_msg, NULL);
}

// === Callback ===
void rover_cmd(const void *msgin)
{
  const CommandMsg *cmd_msg = (const CommandMsg *)msgin;
  int32_t cmd = cmd_msg->data;

  char info[128];
  snprintf(info, sizeof(info), "Callback triggered, cmd: %d", (int)cmd);
  publish_debug(info);

  switch (cmd)
  {
  case 1:
    driveForward();
    publish_debug("→ driveForward()");
    break;
  case 2:
    driveBackward();
    publish_debug("→ driveBackward()");
    break;
  case 3:
    stopDrive();
    publish_debug("→ stopDrive()");
    break;
  case 4:
    spotTurnLeft();
    publish_debug("→ spotTurnLeft()");
    break;
  case 5:
    spotTurnRight();
    publish_debug("→ spotTurnRight()");
    break;
  case 6:
    stopTurn();
    publish_debug("→ stopTurn()");
    break;
  case 7:
    drive_speed = min(255, drive_speed + 5);
    snprintf(info, sizeof(info), "Speed increased to %d", drive_speed);
    publish_debug(info);
    break;
  case 8:
    drive_speed = max(0, drive_speed - 5);
    snprintf(info, sizeof(info), "Speed decreased to %d", drive_speed);
    publish_debug(info);
    break;
  default:
    snprintf(info, sizeof(info), "Unknown command: %d", (int)cmd);
    publish_debug(info);
    break;
  }
}

// === Setup ===
void setup()
{
  Serial.begin(115200);
  delay(500);

  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_rover", "", &support));

  // Subscriber: rover_cmd (Int32)
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "rover_cmd"));

  // Publisher: rover_debug (String)
  RCCHECK(rclc_publisher_init_default(
      &debug, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "rover_debug"));

  debug_msg.data.data = debug_buf;
  debug_msg.data.size = 0;
  debug_msg.data.capacity = sizeof(debug_buf);

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &rover_cmd, ON_NEW_DATA));

  publish_debug("Node initialized. Waiting for commands...");
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
