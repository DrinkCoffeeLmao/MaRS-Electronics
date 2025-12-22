#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int16.h>

#include <string.h> // for strlen, strncpy, snprintf

#define PWM_FREQ 5000
#define PWM_RES 8

#define DIR_L 2
#define PWM_L 4
#define CH_L 0

#define DIR_R 13
#define PWM_R 14
#define CH_R 1

#define DIR_M 23
#define PWM_M 21
#define CH_M 2

#define DIR_S 32
#define PWM_S 25
#define CH_S 3

#define LM1 22
volatile int flag1;
int drive_speed = 0;
const int motor_pwm = 150;
int current_dir = 0; // -1 = backward, 0 = stopped, +1 = forward

unsigned long last_cmd = 0;

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
Motor mFL(DIR_L, PWM_L, CH_L);
Motor mFR(DIR_R, PWM_R, CH_R);
Motor mM(DIR_M, PWM_M, CH_M);
Motor mS(DIR_S, PWM_S, CH_S);

// === Drive Controls ===
void applySpeedToDrives(int pwm)
{
  mFL.setSpeed(pwm);
  mFR.setSpeed(pwm);
}

void setDriveDirection(int dir)
{
  if (dir > 0)
  {
    mFL.forward();
    mFR.forward();
  }
  else if (dir < 0)
  {
    mFL.backward();
    mFR.backward();
  }
}

void setManipDirection(int dir)
{
  if (dir > 0)
    mM.forward();
  else if (dir < 0)
    mM.backward();
}

void setScoopDirection(int dir)
{
  if (dir > 0)
    mS.forward();
  else if (dir < 0)
    mS.backward();
}
void driveForward()
{
  setDriveDirection(+1);
  current_dir = 1;
  applySpeedToDrives(constrain(drive_speed, 0, 255));
}

void driveBackward()
{
  setDriveDirection(-1);
  current_dir = -1;
  applySpeedToDrives(constrain(-drive_speed, 0, 255));
}

void manipUP()
{
  setManipDirection(1);
  current_dir = 1;
  mM.setSpeed(motor_pwm);
}

void manipDown()
{
  setManipDirection(-1);
  current_dir = -1;
  mM.setSpeed(motor_pwm);
}

void scoopOpen()
{
  setScoopDirection(1);
  current_dir = 1;
  mS.setSpeed(200);
}

void scoopClose()
{
  setScoopDirection(-1);
  current_dir = -1;
  mS.setSpeed(200);
}

void stopDrive()
{
  mFL.setSpeed(0);
  mFR.setSpeed(0);
  mS.setSpeed(0);
  mM.setSpeed(0);
}

// === Spot Turn Controls ===
void spotTurnLeft()
{
  // Left motors backward, right motors forward
  mFL.backward();
  mFR.forward();

  applySpeedToDrives(constrain(drive_speed, 0, 255));
}

void spotTurnRight()
{
  // Left motors forward, right motors backward
  mFL.forward();
  mFR.backward();

  applySpeedToDrives(constrain(drive_speed, 0, 255));
}

void stopTurn()
{
  stopDrive();
}

void limitingswitch(){
 flag1=digitalRead(LM1);
}
// === micro-ROS Setup ===
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t debug;
rclc_executor_t executor;
rcl_publisher_t limit_pub;

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      while (1)                  \
        ;                        \
    }                            \
  }

typedef std_msgs__msg__Int16 CommandMsg;
std_msgs__msg__Int16 limit_msg;
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
  last_cmd = millis();
  char info[128];
  snprintf(info, sizeof(info), "Callback triggered, cmd: %d", (int)cmd);
  publish_debug(info);

  switch (cmd)
  {
  case 1:
    drive_speed = min(255, drive_speed + 5);
    snprintf(info, sizeof(info), "Speed increased to %d", drive_speed);
    publish_debug(info);
    if (drive_speed > 0)
    {
      driveForward();
      snprintf(info, sizeof(info), "Speed increased to %d", drive_speed);
      publish_debug("→ driveForward()");
    }
    else
      driveBackward();
    break;
  case 2:
    drive_speed = max(-255, drive_speed - 5);
    snprintf(info, sizeof(info), "Speed decreased to %d", drive_speed);
    publish_debug(info);

    if (drive_speed < 0)
    {
      driveBackward();
      snprintf(info, sizeof(info), "Speed increased to %d", drive_speed);
      publish_debug("→ driveBackward()");
    }
    else
      driveForward();

    break;
  case 3:
    stopDrive();
    publish_debug("→ stopDrive()");
    drive_speed = 0;
    break;
  case 4:
    drive_speed = 200;
    spotTurnRight();
    publish_debug("→SportTurnRight ()");
    break;
  case 5:
    drive_speed = 200;
    spotTurnLeft();
    publish_debug("→SportTurnLeft ()");
    break;
  case 6:
    stopTurn();
    publish_debug("→ stopTurn()");
    break;

  case 10:
    manipUP();
    publish_debug("Manipulator up");
    break;

  case 9:
  while(!flag1){
    mM.setSpeed(0);
    publish_debug("Switch pressed");
    return;
  }
    manipDown();
    publish_debug("Manipulator down");
  break;

  case 7:
    scoopOpen();
    publish_debug("Scoop open");
    break;

  case 8:
    scoopClose();
    publish_debug("Scoop close");
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
  pinMode(LM1, INPUT_PULLUP);

  set_microros_serial_transports(Serial);
  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_rover", "", &support));

  // Subscriber: rover_cmd (Int32)
  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "rover_cmd"));

  // Publisher: rover_debug (String)
  RCCHECK(rclc_publisher_init_default(
      &debug, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "kutty_rover_debug"));
  
  RCCHECK(rclc_publisher_init_default(
    &limit_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "limit_switch"));

  debug_msg.data.data = debug_buf;
  debug_msg.data.size = 0;
  debug_msg.data.capacity = sizeof(debug_buf);

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &rover_cmd, ON_NEW_DATA));

  publish_debug("Node initialized. Waiting for commands...");
}

void loop()
{
  if (millis() - last_cmd > 100)
  {
    mM.setSpeed(0);
    mS.setSpeed(0);
  }
  limitingswitch();
  limit_msg.data = flag1;   // 1 = hit, 0 = free
  rcl_publish(&limit_pub, &limit_msg, NULL);
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  delay(10);
}
