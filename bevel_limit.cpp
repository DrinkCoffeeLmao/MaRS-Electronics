// bevel with  limit switch imu: Reset ESP 

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/float32_multi_array.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// === Pin Definitions ===
#define MOTOR_PWM1 14
#define MOTOR_DIR1 13
#define MOTOR_PWM2 4
#define MOTOR_DIR2 16

#define ENCODER1_A 34
#define ENCODER1_B 35
#define ENCODER2_A 39
#define ENCODER2_B 36

float yaw_limit;

// === Limit Switch Pins ===
#define LIMIT_LIFT_UP 25
#define LIMIT_LIFT_DOWN 26

// === Encoder CPR ===
#define CPR 6500.0
#define GEAR_RATIO 1.0

// === PWM Config ===
const int pwmFreq = 20000;
const int pwmRes = 8;
const int ch1 = 0;
const int ch2 = 1;
const int fixedPWM = 100;

unsigned long last_cmd_time = 0;
int failure_count = 0;

// === Encoder Objects ===
ESP32Encoder enc1, enc2;

// === Movement variables ===
float targetBevel = 0, targetLift = 0;
float currentBevel = 0, currentLift = 0;
float currentAngleL = 0, currentAngleR = 0;  // Individual encoder angles
bool movingBevel = false, movingLift = false;
int bevelDir = 1, liftDir = 1;

// ----------- IMU -----------
Adafruit_BNO055 bno = Adafruit_BNO055(55);
rcl_publisher_t imu_pub;
rcl_timer_t imu_timer;
std_msgs__msg__Float32MultiArray imu_msg;

// === micro-ROS Entities ===
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t bevel_cmd_sub;
rcl_publisher_t pub_bevel_angle;
rcl_publisher_t pub_lift_angle;
rcl_publisher_t pub_limit_hit;
rcl_publisher_t limit1_pub;
rcl_publisher_t limit2_pub;

std_msgs__msg__Int16 limit1_msg;
std_msgs__msg__Int16 limit2_msg;


std_msgs__msg__Int16 bevel_cmd_msg;
std_msgs__msg__Float32 msg_bevel_angle;
std_msgs__msg__Float32 msg_lift_angle;
std_msgs__msg__Bool msg_limit_hit;

// === RCCHECK ===
#define RCCHECK(fn) \
  {                \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
      delay(1000); \
      ESP.restart(); \
    } \
  }
double mapRange(double value,
                double in_min, double in_max,
                double out_min, double out_max)
{
    return (value - in_min) * (out_max - out_min)
           / (in_max - in_min)
           + out_min;
}



// IMU___ CALLBACK FUNCTION
// ----------- IMU Timer Callback -----------
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

     float yaw  =  euler.z();
      if (!digitalRead(LIMIT_LIFT_UP))
  {
      yaw_limit = yaw;
  }

  float yaw_mapped_local = 0.0;

    imu_msg.data.data[0] = yaw;
    imu_msg.data.size = 1;

        if((yaw_limit>270) && ((0<=yaw) && (yaw <= (int((yaw_limit+float(90.0)))%360)))){

      yaw += 360;
      
    }
         yaw_mapped_local = mapRange(yaw, yaw_limit, yaw_limit + 90, 0, 90);
    



  imu_msg.data.data[0] = euler.z();
  imu_msg.data.data[1] = yaw_mapped_local;
  imu_msg.data.data[2] = euler.x();

  imu_msg.data.size = 3;

    rcl_publish(&imu_pub, &imu_msg, NULL);


  }
}

// === Helper: Convert encoder counts to angle ===
float getJointAngle(long counts)
{
    return (counts / (CPR * GEAR_RATIO*5.56)) * 360.0;
}

// === Helper: Get averaged angle from both encoders ===
float getAveragedAngle()
{
    long encCountL = enc1.getCount();
    long encCountR = enc2.getCount();

    currentAngleL = getJointAngle(encCountL);
    currentAngleR = getJointAngle(encCountR);

    // return (currentAngleL + currentAngleR) / 2.0;
    return (currentAngleR);

}

// === Stop both motors ===
void stopMotors()
{
    ledcWrite(ch1, 0);
    ledcWrite(ch2, 0);
}

// === Drive both motors ===
// sameDirection = true → Bevel rotation
// sameDirection = false → Lift movement (opposite direction)
void driveMotors(int pwm, bool sameDirection, int dir)
{
    // Motor1 direction
    digitalWrite(MOTOR_DIR1, dir >= 0);

    // Motor2 direction depends on same/opposite
    if (sameDirection) {
        digitalWrite(MOTOR_DIR2, dir >= 0);  // same → Bevel
    } else {
        digitalWrite(MOTOR_DIR2, dir < 0);   // opposite → Lift
    }

    ledcWrite(ch1, abs(pwm));
    ledcWrite(ch2, abs(pwm));
}

// === Callback: Bitmask Command ===
// Bit 0 → Bevel rotate +
// Bit 1 → Bevel rotate -
// Bit 2 → Lift +
// Bit 3 → Lift -
void bevel_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
    int cmd = msg->data;
    last_cmd_time = millis();

    // Get current averaged angle from both encoders
    currentBevel = getAveragedAngle();
    currentLift = currentBevel; // same motors, logical separate variable

    movingBevel = false;
    movingLift = false;

    if (cmd & (1 << 0)) { // Bevel +
        targetBevel = currentBevel + 10;
        bevelDir = 1;
        movingBevel = true;
    }
    if (cmd & (1 << 1)) { // Bevel -
        targetBevel = currentBevel - 10;
        bevelDir = -1;
        movingBevel = true;
    }
    if (cmd & (1 << 2)) { // Lift +
        targetLift = currentLift + 10;
        liftDir = 1;
        movingLift = true;
    }
    if (cmd & (1 << 3)) { // Lift -
        targetLift = currentLift - 10;
        liftDir = -1;
        movingLift = true;
    }
}

// === Setup ===
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


    // IMU Init
   if (!bno.begin()) {
    Serial.println("BNO055 not detected!");
    while (1);
  }
  bno.setExtCrystalUse(true);
  delay(1000);

    // Motor pins
    pinMode(MOTOR_DIR1, OUTPUT);
    pinMode(MOTOR_DIR2, OUTPUT);

    pinMode(LIMIT_LIFT_DOWN, INPUT_PULLUP);
    pinMode(LIMIT_LIFT_UP, INPUT_PULLUP);

    ledcSetup(ch1, pwmFreq, pwmRes);
    ledcSetup(ch2, pwmFreq, pwmRes);
    ledcAttachPin(MOTOR_PWM1, ch1);
    ledcAttachPin(MOTOR_PWM2, ch2);

    // micro-ROS init
    set_microros_serial_transports(Serial);

  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) 
  {
     delay(100);
  }

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "bevel_node", "", &support));

     // IMU 
  rclc_publisher_init_default(
    &imu_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "bevel_orientation"
  );
  // Allocate message memory: IMU
  imu_msg.data.capacity = 3;
  imu_msg.data.size = 0;
  imu_msg.data.data = (float*) malloc(3 * sizeof(float));

  // IMU timer
    rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(50), imu_timer_callback);


    // Publishers
    RCCHECK(rclc_publisher_init_default(
        &pub_bevel_angle, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "bevel_angle"));
    RCCHECK(rclc_publisher_init_default(
        &pub_lift_angle, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "lift_angle"));
     RCCHECK(rclc_publisher_init_default(
        &pub_limit_hit, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "limit_hit"));
        RCCHECK(rclc_publisher_init_default(
    &limit1_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "limit_switch_1"));

RCCHECK(rclc_publisher_init_default(
    &limit2_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
    "limit_switch_2"));


    // Subscriber
    RCCHECK(rclc_subscription_init_default(
        &bevel_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "bevel_cmd"));

    // Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &bevel_cmd_sub, &bevel_cmd_msg, &bevel_cmd_callback, ON_NEW_DATA));
            rclc_executor_add_timer(&executor, &imu_timer);

}

// === Main Loop ===
void loop()
{
 if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) 
  {
    failure_count = 0; 
    // Handle ROS callbacks
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Limit switch handling
    if(!digitalRead(LIMIT_LIFT_UP) && movingLift == true && liftDir > 0)
    {
        enc1.clearCount();
        enc2.clearCount();
        stopMotors();
        movingLift = false;
        movingBevel = false;
        currentBevel = 0;
        currentLift = 0;
        targetBevel = 0;
        targetLift = 0;

      limit1_msg.data = 1;
      rcl_publish(&limit1_pub, &limit1_msg, NULL);



    }
    else if(!digitalRead(LIMIT_LIFT_DOWN) && movingLift == true && liftDir < 0)
    {
        stopMotors();
        movingLift = false;
                movingBevel = false;

        limit2_msg.data =1;
        rcl_publish(&limit2_pub, &limit2_msg, NULL);
    }

    else
    {
        // No limit hit

      limit1_msg.data = 0;
      rcl_publish(&limit1_pub, &limit1_msg, NULL);
        limit2_msg.data =0;
        rcl_publish(&limit2_pub, &limit2_msg, NULL);

    }

    if(!digitalRead(LIMIT_LIFT_UP)){
      limit1_msg.data = 1;
      rcl_publish(&limit1_pub, &limit1_msg, NULL);

    }
    else if(!digitalRead(LIMIT_LIFT_DOWN))    {
        limit2_msg.data =1;
        rcl_publish(&limit2_pub, &limit2_msg, NULL);
    }
    else{
            limit1_msg.data = 0;
      rcl_publish(&limit1_pub, &limit1_msg, NULL);
        limit2_msg.data =0;
        rcl_publish(&limit2_pub, &limit2_msg, NULL);
    }

    // Get current averaged angle from both encoders
    currentBevel = getAveragedAngle();
    currentLift = currentBevel; // logical lift

    int pwmBevel = 0;
    int pwmLift = 0;

    // Bevel movement
    if (movingBevel && abs(currentBevel - targetBevel) > 1.0)
        pwmBevel = fixedPWM * bevelDir;

    // Lift movement
    if (movingLift && abs(currentLift - targetLift) > 1.0)
        pwmLift = fixedPWM * liftDir;

    // Apply motors
    if (pwmBevel != 0 && pwmLift != 0)
    {
        // Both commands → lift has priority in opposite direction
        driveMotors(pwmLift, false, liftDir); // opposite directions → lift
    }
    else if (pwmBevel != 0)
    {
        driveMotors(pwmBevel, true, bevelDir); // same direction → rotate
    }
    else if (pwmLift != 0)
    {
        driveMotors(pwmLift, false, liftDir); // opposite → lift
    }
    else
    {
        stopMotors();
    }
  
    // Publish encoder feedback (averaged angles)
    msg_bevel_angle.data = currentBevel;
    msg_lift_angle.data = currentLift;
    rcl_publish(&pub_bevel_angle, &msg_bevel_angle, NULL);
    rcl_publish(&pub_lift_angle, &msg_lift_angle, NULL);
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
