// // ************** Steer Code wtih limiting swithches and esp reset
#include <Arduino.h>
// #include <ESP32Encoder.h> // --- DISABLED: Replaced with Limit Switch Logic ---

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/string.h>
#include <string.h>

// --- Encoders REMOVED ---
// ESP32Encoder FrontLeft;
// ESP32Encoder FrontRight;
// ...

// --- Motor pwins (UNCHANGED) ---
#define MOTOR_PWM_FL 16
#define MOTOR_DIR_FL 4

#define MOTOR_PWM_FR 5
#define MOTOR_DIR_FR 17

#define MOTOR_PWM_BL 22
#define MOTOR_DIR_BL 23

#define MOTOR_PWM_BR 19
#define MOTOR_DIR_BR 18

// --- NEW: Limit Switch Pins (Repurposed Encoder Pins) ---
// Assumption: First pin is LIMIT_STRAIGHT, Second pin is LIMIT_TURN
// LOGIC: Low (GND) = Switch Hit, High = Switch Open

// Front Left
#define PIN_FL_LIM_STRAIGHT  32
#define PIN_FL_LIM_TURN      33

// Front Right
#define PIN_FR_LIM_STRAIGHT 25 //26 //33
#define PIN_FR_LIM_TURN     15 //27 //32

// Back Left (Note: Pins 34, 35 need external 10k Pull-up resistors!)
#define PIN_BL_LIM_STRAIGHT 34
#define PIN_BL_LIM_TURN     35

// Back Right (Note: Pins 39, 36 need external 10k Pull-up resistors!)
#define PIN_BR_LIM_STRAIGHT 39
#define PIN_BR_LIM_TURN     36

// --- Control params ---
const int fixedPWM = 150;
const int spotfixedPWM =100;
// const float angleTolerance = 2.0; // Not used with switches
// const float stepAngle = 5.0;      // Not used with switches

// --- PWM ---
const int pwmFreq = 20000;
const int pwmRes = 8;
const int chFL = 0, chFR = 1, chBL = 2, chBR = 3;

int failure_count = 0;

// --- micro-ROS ---
rcl_node_t node;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;

rcl_subscription_t steering_sub;
rcl_publisher_t debug_pub;
rcl_publisher_t status_pub;


std_msgs__msg__Int16 steering_msg;
std_msgs__msg__Int16 status_msg;

static std_msgs__msg__String debug_msg;
static char debug_buf[256];

unsigned long last_cmd_time = 0;
#define RCCHECK(fn) \
  {                \
    rcl_ret_t rc = fn; \
    if (rc != RCL_RET_OK) { \
      delay(1000); \
      ESP.restart(); \
    } \
  }


int status = 0;
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

void status_publish(int16_t status)
{
  status_msg.data = status;
  rcl_publish(&status_pub, &status_msg, NULL);
}

// --- Helper: Basic Motor Drive (UNCHANGED) ---
// Handles direction pin and ABS(pwm) for speed
void motorDrive(int channel, int dirPin, int pwm)
{
  bool dir = pwm >= 0;
  digitalWrite(dirPin, dir);
  ledcWrite(channel, abs(pwm));
}

// --- NEW Helper: Safe Motor Drive with Limits ---
// Checks the relevant limit switch before moving.
// Direction Logic:
//   Positive PWM (> 0)  = Moving towards TURN position.
//   Negative PWM (< 0)  = Moving towards STRAIGHT position.
//   Switch Logic: LOW = HIT (Stop), HIGH = OPEN (Move)
// void safeMotorDrive(int channel, int dirPin, int pwm, int pinStraight, int pinTurn)
// {
//   // Read switches
//   bool hitStraight = (digitalRead(pinStraight) == LOW);
//   bool hitTurn     = (digitalRead(pinTurn) == LOW);

//   // 1. If trying to Turn (Positive PWM) AND Turn Switch is HIT -> STOP
//   if (pwm > 0 && hitTurn) {
//     motorDrive(channel, dirPin, 0); 
//     return;
//   }

//   // 2. If trying to go Straight (Negative PWM) AND Straight Switch is HIT -> STOP
//   if (pwm < 0 && hitStraight) {
//     motorDrive(channel, dirPin, 0);
//     return;
//   }

//   // 3. Otherwise, path is clear, move motor
//   motorDrive(channel, dirPin, pwm);
// }

// --- ROS callback ---
void steer_cmd_callback(const void *msgin)
{
  const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
  int cmd = msg->data;

  last_cmd_time = millis();
  
  // Vars for auto-loops
  bool fl_done, fr_done, bl_done, br_done;

  switch (cmd)
  {
  case 0: // STOP ALL
    motorDrive(chFL, MOTOR_DIR_FL, 0);
    motorDrive(chFR, MOTOR_DIR_FR, 0);
    motorDrive(chBL, MOTOR_DIR_BL, 0);
    motorDrive(chBR, MOTOR_DIR_BR, 0);
    break;

  // --- MANUAL COMMANDS (Updated with Limit Safety) ---
  case 1: // FL Move Turn (+)
    // safeMotorDrive(chFL, MOTOR_DIR_FL, -fixedPWM, PIN_FL_LIM_STRAIGHT, PIN_FL_LIM_TURN);
      motorDrive(chFL, MOTOR_DIR_FL,  -fixedPWM);
    debug_log("FL++ (Turn)");
    break;

  case 2: // FL Move Straight (-)
    // safeMotorDrive(chFL, MOTOR_DIR_FL, fixedPWM, PIN_FL_LIM_STRAIGHT, PIN_FL_LIM_TURN);
    motorDrive(chFL, MOTOR_DIR_FL, fixedPWM);
    debug_log("FL-- (Straight)");
    break;

  case 3: // FR Move Turn (+)
    // safeMotorDrive(chFR, MOTOR_DIR_FR, fixedPWM, PIN_FR_LIM_STRAIGHT, PIN_FR_LIM_TURN);
    motorDrive(chFR, MOTOR_DIR_FR, fixedPWM);
    debug_log("FR++ (Turn)");
    break;
  case 4: // FR Move Straight (-)
    // safeMotorDrive(chFR, MOTOR_DIR_FR, -fixedPWM, PIN_FR_LIM_STRAIGHT, PIN_FR_LIM_TURN);
    motorDrive(chFR, MOTOR_DIR_FR, -fixedPWM);
    debug_log("FR-- (Straight)");
    break;

  case 5: // BL Move Turn (+)
    // safeMotorDrive(chBL, MOTOR_DIR_BL, fixedPWM, PIN_BL_LIM_STRAIGHT, PIN_BL_LIM_TURN);
    motorDrive(chBL, MOTOR_DIR_BL, fixedPWM);
    debug_log("BL++ (Turn)");
    break;
  case 6: // BL Move Straight (-)
    // safeMotorDrive(chBL, MOTOR_DIR_BL, -fixedPWM, PIN_BL_LIM_STRAIGHT, PIN_BL_LIM_TURN);
    motorDrive(chBL, MOTOR_DIR_BL, -fixedPWM);
    debug_log("BL-- (Straight)");
    break;

  // --- AUTOMATED MODES ---

  // CASE 8: AUTO SPOT TURN
  // Goal: Move all motors Positive (+) until they hit their TURN limit switch.
  case 8:
    debug_log("CMD: Spot Turn...");
    status  = 1;
    while(true)
        {
        //rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    // Check if switches are hit (LOW = Hit)
       bool fl_hit = (digitalRead(PIN_FL_LIM_TURN) == LOW); 
       bool fr_hit = (digitalRead(PIN_FR_LIM_TURN) == LOW);
       bool bl_hit = (digitalRead(PIN_BL_LIM_TURN) == LOW);
       bool br_hit = (digitalRead(PIN_BR_LIM_TURN) == LOW);

       // Move individual motors if NOT hit
       if(!fl_hit) motorDrive(chFL, MOTOR_DIR_FL, spotfixedPWM); else motorDrive(chFL, MOTOR_DIR_FL, 0);
       if(!fr_hit) motorDrive(chFR, MOTOR_DIR_FR, -spotfixedPWM); else motorDrive(chFR, MOTOR_DIR_FR, 0);
       if(!bl_hit) motorDrive(chBL, MOTOR_DIR_BL, spotfixedPWM); else motorDrive(chBL, MOTOR_DIR_BL, 0);
       if(!br_hit) motorDrive(chBR, MOTOR_DIR_BR, -spotfixedPWM); else motorDrive(chBR, MOTOR_DIR_BR, 0);

       // Break loop only when ALL are hit
       if(fl_hit && fr_hit && bl_hit && br_hit) {
         debug_log("Spot Turn Complete");
         break;
       }

       status_publish(status);
    }
        status  = 2;
status_publish(status);
    break;
    

  // CASE 7: AUTO STRAIGHT
  // Goal: Move all motors Negative (-) until they hit their STRAIGHT limit switch.
  case 7:
    debug_log("CMD: Reverse spot turn...");
    status = 3;
    while(true)
    {
      // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
       // Check if switches are hit (LOW = Hit)
       bool fl_hit = (digitalRead(PIN_FL_LIM_STRAIGHT) == LOW);
       bool fr_hit = (digitalRead(PIN_FR_LIM_STRAIGHT) == LOW);
       bool bl_hit = (digitalRead(PIN_BL_LIM_STRAIGHT) == LOW);
       bool br_hit = (digitalRead(PIN_BR_LIM_STRAIGHT) == LOW);

       // Move individual motors if NOT hit (Note the negative PWM)
       if(!fl_hit) motorDrive(chFL, MOTOR_DIR_FL, -spotfixedPWM); 
       else {
        motorDrive(chFL, MOTOR_DIR_FL, 0);
        debug_log("FL Reached");
      }
       if(!fr_hit) motorDrive(chFR, MOTOR_DIR_FR, spotfixedPWM); 
       else 
       { motorDrive(chFR, MOTOR_DIR_FR, 0);
        debug_log("FR Reached ");
       }
       if(!bl_hit) motorDrive(chBL, MOTOR_DIR_BL, -spotfixedPWM);
        else 
        {motorDrive(chBL, MOTOR_DIR_BL, 0);
          debug_log("BL Reached ");}
       if(!br_hit) motorDrive(chBR, MOTOR_DIR_BR, spotfixedPWM); 
       else
       { motorDrive(chBR, MOTOR_DIR_BR, 0);
        debug_log("BR Reached ");}

       // Break loop only when ALL are hit
       if(fl_hit && fr_hit && bl_hit && br_hit) {
         debug_log("Straight Position Complete");
         break;
       }
       status_publish(status);
    }

    status = 0;
    status_publish(status);
    break;

  // Manual Pairs (Safety Added)
  case 50:
     // safeMotorDrive(chFL, MOTOR_DIR_FL, fixedPWM, PIN_FL_LIM_STRAIGHT, PIN_FL_LIM_TURN);
     // safeMotorDrive(chFR, MOTOR_DIR_FR, fixedPWM, PIN_FR_LIM_STRAIGHT, PIN_FR_LIM_TURN);
     motorDrive(chFR, MOTOR_DIR_FR, fixedPWM);
     motorDrive(chFL, MOTOR_DIR_FL,  fixedPWM);

     break;
  case 25:
     // safeMotorDrive(chFL, MOTOR_DIR_FL, -fixedPWM, PIN_FL_LIM_STRAIGHT, PIN_FL_LIM_TURN);
     // safeMotorDrive(chFR, MOTOR_DIR_FR, -fixedPWM, PIN_FR_LIM_STRAIGHT, PIN_FR_LIM_TURN);
     motorDrive(chFR, MOTOR_DIR_FR, -fixedPWM);
     motorDrive(chFL, MOTOR_DIR_FL, -fixedPWM);
     break;

  default:
    motorDrive(chFL, MOTOR_DIR_FL, 0);
    motorDrive(chFR, MOTOR_DIR_FR, 0);
    motorDrive(chBL, MOTOR_DIR_BL, 0);
    motorDrive(chBR, MOTOR_DIR_BR, 0);
    debug_log("Invalid steering command");
    return;
  }

  char buf[64];
  snprintf(buf, sizeof(buf), "Steering cmd: %d executed", cmd);
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
  // --- CHANGED: Setup Pins as Limit Switches (INPUT_PULLUP) ---
  // FL
  pinMode(PIN_FL_LIM_STRAIGHT, INPUT_PULLUP);
  pinMode(PIN_FL_LIM_TURN,     INPUT_PULLUP);
  // FR
  pinMode(PIN_FR_LIM_STRAIGHT, INPUT_PULLUP);
  pinMode(PIN_FR_LIM_TURN,     INPUT_PULLUP);
  // BL
  pinMode(PIN_BL_LIM_STRAIGHT, INPUT_PULLUP); 
  pinMode(PIN_BL_LIM_TURN,     INPUT_PULLUP);
  // BR
  pinMode(PIN_BR_LIM_STRAIGHT, INPUT_PULLUP); 
  pinMode(PIN_BR_LIM_TURN,     INPUT_PULLUP);

  // Motors
  pinMode(MOTOR_DIR_FL, OUTPUT);
  pinMode(MOTOR_DIR_FR, OUTPUT);
  pinMode(MOTOR_DIR_BL, OUTPUT);
  pinMode(MOTOR_DIR_BR, OUTPUT);
  setupPWM();

  // micro-ROS
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  while (RMW_RET_OK != rmw_uros_ping_agent(100, 1)) 
    {
     delay(100);
    }
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

 RCCHECK(rclc_publisher_init_default(
      &status_pub, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
      "steer_status"));
  

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &steering_sub, &steering_msg, &steer_cmd_callback, ON_NEW_DATA));

  debug_log("Steering node initialized (Limit Switches)!");
}

// --- LOOP ---
void loop()
{
  if (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) 
  {
  failure_count = 0;
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // --- CHANGED: Debug output to show Limit Switch States (1=Open, 0=Hit) ---
  // This helps you verify if wiring is correct.
  char buf[150];
  snprintf(buf, sizeof(buf), 
    "Limits(Str|Trn) FL:%d|%d FR:%d|%d BL:%d|%d BR:%d|%d",
    digitalRead(PIN_FL_LIM_STRAIGHT), digitalRead(PIN_FL_LIM_TURN),
    digitalRead(PIN_FR_LIM_STRAIGHT), digitalRead(PIN_FR_LIM_TURN),
    digitalRead(PIN_BL_LIM_STRAIGHT), digitalRead(PIN_BL_LIM_TURN),
    digitalRead(PIN_BR_LIM_STRAIGHT), digitalRead(PIN_BR_LIM_TURN)
  );

  // Limit debug rate to avoid flooding
  static unsigned long last_print = 0;
  if (millis() - last_print > 500) {
    debug_log(buf);
    last_print = millis();
  }

  status_publish(status);

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
