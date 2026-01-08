// Correncted Link1 code: with reset


#include <Arduino.h>
#include <ESP32Encoder.h>

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32_multi_array.h>


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// === Pin Definitions ===
#define MOTOR_PWM_BASE 16
#define MOTOR_DIR_BASE 4
#define ENCODER_BASE_A 33
#define ENCODER_BASE_B 32

#define MOTOR_PWM_L1 19
#define MOTOR_DIR_L1 18
#define ENCODER_L1_A 35
#define ENCODER_L1_B 34

#define MOTOR_PWM_L2 22
#define MOTOR_DIR_L2 21
#define ENCODER_L2_A 39
#define ENCODER_L2_B 36

// === Limit Switch Pins ===
#define LIMIT_L1 14
#define LIMIT_L2 27

// === Encoder CPRs ===
#define CPR_BASE 120000.0
#define CPR_L1 37500.0
#define CPR_L2 120000.0

// === PWM Config ===
const int pwmFreq = 20000;
const int pwmRes = 8;
const int chBase = 0;
const int chL1 = 1;
const int chL2 = 2;

const int fixedPWM_L1 = 150;
const int fixedPWM_L2 = 255;
const int fixedPWM_BASE = 200;

unsigned long last_cmd_time = 0;

int failure_count = 0;
float yaw_limit;
bool flag=false;
float yaw_mapped;



// === Direction tracking for limits ===
int moveDirL1 = 0;
int moveDirL2 = 0;

// === Encoder Objects ===
ESP32Encoder encBase;
ESP32Encoder encL1;
ESP32Encoder encL2;


// ----------- IMU -----------
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// === micro-ROS Entities ===
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t manipulator_cmd_sub;
rcl_publisher_t debug_pub;


// ------- IMU------
rcl_publisher_t imu_pub;
rcl_timer_t imu_timer;
std_msgs__msg__Float32MultiArray imu_msg;

std_msgs__msg__Int16 manipulator_cmd_msg;
std_msgs__msg__Int16 debug_msg;

// === RCCHECK ===
#define RCCHECK(fn) \
  {                \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
      delay(1000); \
      ESP.restart(); \
    } \
  }

// IMU___ CALLBACK FUNCTION
// ----------- IMU Timer Callback -----------
double mapRange(double value,
                double in_min, double in_max,
                double out_min, double out_max)
{
    return (value - in_min) * (out_max - out_min)
           / (in_max - in_min)
           + out_min;
}

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer == NULL) return;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float yaw = euler.x();

  // Capture yaw reference ONCE when limit is hit
  if (!digitalRead(LIMIT_L1))
  {
      yaw_limit = yaw;
  }

  float yaw_mapped_local = 0.0;

 
  // if(yaw_limit<270){
  // yaw_mapped_local = mapRange(yaw, yaw_limit, yaw_limit + 90, 0, 90);
  // }
  // else{
  //   if(yaw_limit<=yaw && yaw <= 360){     
  //     yaw_mapped_local = mapRange(yaw, yaw_limit, yaw_limit + 90, 0, 90);
  //   }

  //   else if(0<=yaw && yaw <= ((yaw_limit+90.0)%360)){
  //     yaw += 360;
  //   yaw_mapped_local = mapRange(yaw, yaw_limit, yaw_limit + 90, 0, 90);

  //   }

  //}

    if((yaw_limit>270) && ((0<=yaw) && (yaw <= (int((yaw_limit+float(90.0)))%360)))){

      yaw += 360;
      
    }
         yaw_mapped_local = mapRange(yaw, yaw_limit, yaw_limit + 90, 0, 90);
    



  imu_msg.data.data[0] = euler.x();
  imu_msg.data.data[1] = yaw_mapped_local;
  imu_msg.data.size = 2;

  rcl_publish(&imu_pub, &imu_msg, NULL);
}




// === Debug Publisher ===
void debug_log(int16_t value)
{
    debug_msg.data = value;
    rcl_publish(&debug_pub, &debug_msg, NULL);
}

// === Encoder angle helper ===
float getAngle(long count, float cpr)
{
  return (count / cpr*2.8) * 360.0;
}

// === Motor driver ===
void motorDrive(int channel, int dirPin, int pwm)
{
  bool dir = pwm >= 0;
  digitalWrite(dirPin, dir);
  ledcWrite(channel, abs(pwm));
}

// === Limit switch handling ===
void handleLimitSwitches()
{
  // L1: block negative motion
  if (!digitalRead(LIMIT_L1) && (moveDirL1 > 0) )
  {
    encL2.clearCount(); //l2 slot being used for l1
    motorDrive(chL1, MOTOR_DIR_L1, 0);
    debug_log(10); // L1 limit hit indicator
    float angleL2 = 0;

  }

  // L2: block positive motion
  if (!digitalRead(LIMIT_L2) && moveDirL2 > 0)
  {
    encL1.clearCount();
    motorDrive(chL2, MOTOR_DIR_L2, 0);
    debug_log(-2); // L2 limit hit indicator
  }
}

// === Manipulator Command Callback ===
void manipulator_cmd_callback(const void *msgin)
{
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  int cmd = msg->data;
  last_cmd_time = millis();

  // Stop all motors
  motorDrive(chBase, MOTOR_DIR_BASE, 0);
  motorDrive(chL1, MOTOR_DIR_L1, 0);
  motorDrive(chL2, MOTOR_DIR_L2, 0);

  moveDirL1 = 0;
  moveDirL2 = 0;

  // --- L1 ---
  if (cmd & (1 << 0)) { moveDirL1 = +1; motorDrive(chL1, MOTOR_DIR_L1, +fixedPWM_L1); }
  if (cmd & (1 << 1)) { moveDirL1 = -1; motorDrive(chL1, MOTOR_DIR_L1, -fixedPWM_L1); }

  // --- L2 ---
  if (cmd & (1 << 2)) { moveDirL2 = +1; motorDrive(chL2, MOTOR_DIR_L2, +fixedPWM_L2); }
  if (cmd & (1 << 3)) { moveDirL2 = -1; motorDrive(chL2, MOTOR_DIR_L2, -fixedPWM_L2); }

  // --- Base ---
  if (cmd & (1 << 4)) motorDrive(chBase, MOTOR_DIR_BASE, +fixedPWM_BASE);
  if (cmd & (1 << 5)) motorDrive(chBase, MOTOR_DIR_BASE, -fixedPWM_BASE);
}

// === Setup ===
void setup()
{
  Serial.begin(115200);
  delay(2000);


  // IMU Init
   if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    // while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

  // Limit switches
  pinMode(LIMIT_L1, INPUT_PULLUP);
  pinMode(LIMIT_L2, INPUT_PULLUP);

  // Encoders
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encBase.attachHalfQuad(ENCODER_BASE_A, ENCODER_BASE_B);
  encL1.attachHalfQuad(ENCODER_L1_A, ENCODER_L1_B);
  encL2.attachHalfQuad(ENCODER_L2_A, ENCODER_L2_B);

  encBase.clearCount();
  encL1.clearCount();
  encL2.clearCount();

  // Motor pins
  pinMode(MOTOR_DIR_BASE, OUTPUT);
  pinMode(MOTOR_DIR_L1, OUTPUT);
  pinMode(MOTOR_DIR_L2, OUTPUT);

  ledcSetup(chBase, pwmFreq, pwmRes);
  ledcSetup(chL1, pwmFreq, pwmRes);
  ledcSetup(chL2, pwmFreq, pwmRes);

  ledcAttachPin(MOTOR_PWM_BASE, chBase);
  ledcAttachPin(MOTOR_PWM_L1, chL1);
  ledcAttachPin(MOTOR_PWM_L2, chL2);

  // micro-ROS
  set_microros_serial_transports(Serial);

  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) 
  {
     delay(100);
  }

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "manipulator_node", "", &support));

  // IMU 
  rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "imu_orientation"
  );

  // Allocate message memory: IMU
  imu_msg.data.capacity = 3;
  imu_msg.data.size = 0;
  imu_msg.data.data = (float*) malloc(3 * sizeof(float));

  // IMU timer
    rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(50), imu_timer_callback);



  RCCHECK(rclc_publisher_init_default(
      &debug_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "manipulator_debug"));

  RCCHECK(rclc_subscription_init_default(
      &manipulator_cmd_sub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "manipulator_cmd"));

  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    rclc_executor_add_timer(&executor, &imu_timer);

  RCCHECK(rclc_executor_add_subscription(
      &executor, &manipulator_cmd_sub,
      &manipulator_cmd_msg,
      &manipulator_cmd_callback,
      ON_NEW_DATA));

  //debug_log(1); // Startup indicator: 1 = ready
}

// === Loop ===
void loop()
{
  if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) 
  {
    failure_count = 0;
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  handleLimitSwitches();

  // Timeout stop
  if (millis() - last_cmd_time > 500)
  {
    motorDrive(chBase, MOTOR_DIR_BASE, 0);
    motorDrive(chL1, MOTOR_DIR_L1, 0);
    motorDrive(chL2, MOTOR_DIR_L2, 0);
  }

  // Encoder debug - publish L2 angle as int16
  float angleL2 = getAngle(encL2.getCount(), CPR_L2);
  int16_t angleL2_int = (int16_t)angleL2; // Convert float to int16
  debug_log(angleL2_int);

  delay(10);
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
