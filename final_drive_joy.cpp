// Drive Code for joy stick


#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <ESP32Encoder.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int16.h>
#include <rmw_microros/time_sync.h>
#include <nav_msgs/msg/odometry.h> // << NEW

#include <string.h>

ESP32Encoder FrontLeft;
ESP32Encoder FrontRight;
ESP32Encoder BackLeft;
ESP32Encoder BackRight;

#define PWM_FREQ 5000
#define PWM_RES 8

// #define DIR_FL 4
// #define PWM_FL 16
// #define CH_FL 0

// #define DIR_FR 18
// #define PWM_FR 19
// #define CH_FR 1

// #define DIR_BL 17
// #define PWM_BL 5
// #define CH_BL 2

// #define DIR_BR 21
// #define PWM_BR 22
// #define CH_BR 3


#define DIR_FL 4
#define PWM_FL 16
#define CH_FL 0

#define DIR_FR 14
#define PWM_FR 25
#define CH_FR 1

#define DIR_BL 18
#define PWM_BL 19
#define CH_BL 2

#define DIR_BR 21
#define PWM_BR 22
#define CH_BR 3

#define CPR_FL 120000.0
#define CPR_FR 120000.0
#define CPR_BL 120000.0
#define CPR_BR 120000.0

// ODOMETRY CONSTANTS
#define WHEEL_RADIUS 22 // 3.3cm example
#define WHEEL_BASE 31.6 // Distance between left & right wheels
#define DT 0.1          // same as loop time (100ms)

// Pose storage
float robot_x = 0;
float robot_y = 0;
float robot_yaw = 0;

int drive_speed = 0;
int current_dir = 0;
unsigned long last_cmd_time = 0;

int failure_count = 0;
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

Motor mFL(DIR_FL, PWM_FL, CH_FL);
Motor mFR(DIR_FR, PWM_FR, CH_FR);
Motor mBL(DIR_BL, PWM_BL, CH_BL);
Motor mBR(DIR_BR, PWM_BR, CH_BR);

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

void stopDrive()
{
  current_dir = 0;
  applySpeedToDrives(0);
}

void spotTurnLeft()
{
  mFL.backward();
  mBL.backward();
  mFR.forward();
  mBR.forward();
  applySpeedToDrives(constrain(100, 0, 255));
}

void spotTurnRight()
{
  mFL.forward();
  mBL.forward();
  mFR.backward();
  mBR.backward();
  applySpeedToDrives(constrain(100, 0, 255));
}

void stopTurn()
{
  applySpeedToDrives(0);
}

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_subscription_t subscriber;
rcl_publisher_t debug;

// NEW: ODOMETRY PUBLISHER
rcl_publisher_t odom_pub;
nav_msgs__msg__Odometry odom_msg;

rclc_executor_t executor;

#define RCCHECK(fn) \
  {                \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
      delay(1000); \
      ESP.restart(); \
    } \
  }

typedef std_msgs__msg__Int16 CommandMsg;
CommandMsg msg;

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

float countToRadians(long count, float cpr)
{
  return (count / cpr) * 2.0f * PI;
}

void publish_odometry()
{
  long fl = FrontLeft.getCount();
  long fr = FrontRight.getCount();

  // Convert to radians
  float th_l = countToRadians(fl, CPR_FL);
  float th_r = countToRadians(fr, CPR_FR);

  // Convert to wheel linear distance
  float dist_l = th_l * WHEEL_RADIUS;
  float dist_r = th_r * WHEEL_RADIUS;

  float d_center = (dist_l + dist_r) / 2.0;
  float d_theta = (dist_r - dist_l) / WHEEL_BASE;

  // Update pose
  robot_x += d_center * cos(robot_yaw);
  robot_y += d_center * sin(robot_yaw);
  robot_yaw += d_theta;

  // Fill Odom msg
  odom_msg.pose.pose.position.x = robot_x;
  odom_msg.pose.pose.position.y = robot_y;
  odom_msg.pose.pose.position.z = 0;

  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = sin(robot_yaw / 2.0);
  odom_msg.pose.pose.orientation.w = cos(robot_yaw / 2.0);

  odom_msg.pose.covariance[0] = 0.01;
  odom_msg.pose.covariance[7] = 0.01;
  odom_msg.pose.covariance[35] = 0.02;
  odom_msg.twist.covariance[0] = 0.01;
  odom_msg.twist.covariance[7] = 0.01;
  odom_msg.twist.covariance[35] = 0.02;

  odom_msg.header.frame_id.data = (char *)"odom";
  odom_msg.child_frame_id.data = (char *)"base_link";

  // timestamp
  int64_t now = rmw_uros_epoch_nanos();
  odom_msg.header.stamp.sec = now / 1000000000LL;
  odom_msg.header.stamp.nanosec = now % 1000000000LL;

  // Velocities
  odom_msg.twist.twist.linear.x = d_center / DT;
  odom_msg.twist.twist.angular.z = d_theta / DT;

  rcl_publish(&odom_pub, &odom_msg, NULL);
}

void rover_cmd(const void *msgin)
{
  const CommandMsg *cmd_msg = (const CommandMsg *)msgin;
  int32_t cmd = cmd_msg->data;

  last_cmd_time = millis();

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

  default:
    snprintf(info, sizeof(info), "Unknown command: %d", (int)cmd);
    publish_debug(info);
    break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  FrontLeft.attachHalfQuad(26, 27);
  FrontRight.attachHalfQuad(32, 33);
  BackLeft.attachHalfQuad(39, 36);
  BackRight.attachHalfQuad(32, 33);

  FrontLeft.clearCount();
  FrontRight.clearCount();
  BackLeft.clearCount();
  BackRight.clearCount();

  set_microros_serial_transports(Serial);
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) 
    {
     delay(100);
    }
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_rover", "", &support));

  RCCHECK(rclc_subscription_init_default(
      &subscriber, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "rover_cmd"));

  RCCHECK(rclc_publisher_init_default(
      &debug, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "rover_debug"));

  debug_msg.data.data = debug_buf;
  debug_msg.data.size = 0;
  debug_msg.data.capacity = sizeof(debug_buf);

  // NEW: ODOMETRY PUBLISHER
  RCCHECK(rclc_publisher_init_default(
      &odom_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
      "drive_odom"));

  memset(odom_msg.pose.covariance, 0, sizeof(odom_msg.pose.covariance));
  memset(odom_msg.twist.covariance, 0, sizeof(odom_msg.twist.covariance));

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &rover_cmd, ON_NEW_DATA));

  publish_debug("Node initialized. Waiting for commands...");

  // Wait for time sync with agent
  while (!rmw_uros_epoch_synchronized())
  {
    rmw_uros_sync_session(1000);
    delay(100);
  }
}

void loop()
{
  if(RMW_RET_OK == rmw_uros_ping_agent(100, 1))
  {
  failure_count = 0;
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

  char buf[150];
  snprintf(buf, sizeof(buf),
           "FL:%lld | FR:%lld | BL:%lld | BR:%lld",
           (long long)FrontLeft.getCount(),
           (long long)FrontRight.getCount(),
           (long long)BackLeft.getCount(),
           (long long)BackRight.getCount());

  publish_debug(buf);

  // NEW: publish odom every cycle
  publish_odometry();

  delay(100);
  }
  else if (RMW_RET_OK != rmw_uros_ping_agent(100, 1))
  {
    failure_count ++;
    if(failure_count >= 30)
    {
      ESP.restart();
    }
    delay(10);
  }
}
