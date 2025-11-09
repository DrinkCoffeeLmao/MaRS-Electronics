#include <Arduino.h>
#include <ESP32Encoder.h>

#include <ESP32Servo.h>

// === Motor and Servo Pins ===
#define PWM_LEFT 14
#define DIR_LEFT 13
#define PWM_RIGHT 4
#define DIR_RIGHT 16
#define SERVO_PIN 18

// === Encoder Pins ===
#define ENC_A 32
#define ENC_B 33

// === Objects ===
ESP32Encoder encLeft;
Servo manipServo;

// === Motor Parameters ===
#define COUNTS_PER_REV 6500.0
#define GEAR_RATIO 1.0
#define ANGLE_TOLERANCE 2.0
#define FIXED_PWM 80
#define PWM_FREQ 20000
#define PWM_RES 8

// === PWM Channels ===
#define CH_LEFT 1
#define CH_RIGHT 1

// === Control Variables ===
float targetAngle = 0;
float currentAngle = 0;
bool moving = false;
bool liftMode = true; // true = lift, false = rotate
int moveDir = 1;      // direction multiplier (+1 or -1)

// === Convert encoder counts → joint angle ===
float getJointAngle(long counts)
{
  return (counts / (COUNTS_PER_REV * GEAR_RATIO)) * 360.0;
}

// === Motor Control ===
void driveMotors(int pwm, bool sameDirection, int dir)
{

  Serial.printf(" Driving motors | PWM: %d | SameDir: %d | Dir: %d\n", pwm, sameDirection, dir);
  int effectivePWM = pwm * dir;

  if (sameDirection)
  {  Serial.printf(" hi");
    // Lift: both motors same direction
    digitalWrite(DIR_LEFT, effectivePWM >= 0);
    digitalWrite(DIR_RIGHT, effectivePWM >= 0);
  }
  else
  {
    Serial.printf(" hi");
    // Rotate: opposite directions
    digitalWrite(DIR_LEFT, effectivePWM >= 0);
    digitalWrite(DIR_RIGHT, effectivePWM < 0);
  }
Serial.printf(" hi");
  ledcWrite(CH_LEFT, abs(effectivePWM));
  ledcWrite(CH_RIGHT, abs(effectivePWM));
}

void stopMotors()
{
  Serial.printf(" hi -stop motor");
  ledcWrite(CH_LEFT, 0);
  ledcWrite(CH_RIGHT, 0);
}

// === PWM Setup ===
void setupPWM()
{


  Serial.printf(" hello");
  ledcSetup(CH_LEFT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_LEFT, CH_LEFT);

  ledcSetup(CH_RIGHT, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_RIGHT, CH_RIGHT);
  ledcWrite(CH_LEFT, 0);
  ledcWrite(CH_RIGHT, 0);
}

// === Setup ===
void setup()
{
  Serial.begin(9600);
  Serial.println("🦾 ESP32 Manipulator — Lift + Rotate + Servo (±Angle)");

  pinMode(DIR_LEFT, OUTPUT);
  pinMode(DIR_RIGHT, OUTPUT);
  setupPWM();

  // Encoder setup
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encLeft.attachHalfQuad(ENC_A, ENC_B);
  encLeft.clearCount();

  // Servo setup
  manipServo.attach(SERVO_PIN, 500, 2500);
  manipServo.write(0);

  Serial.println("Commands:");
  Serial.println("  l <deg>    → Lift arm (+ up, - down)");
  Serial.println("  r <deg>    → Rotate arm (+CW, -CCW)");
  Serial.println("  s <deg>    → Servo move (0–180°)");
  stopMotors();
}

// === Loop ===
void loop()
{
  if (Serial.available())
  {
    String cmd = Serial.readStringUntil(' ');
    float value = Serial.parseFloat();

    cmd.trim();

    if (cmd.equalsIgnoreCase("r"))
    {
      encLeft.clearCount();
      targetAngle = fabs(value);
      
      moveDir = (value >= 0) ? 1 : -1;
      liftMode = true;
      moving = true;
      Serial.printf("roatate %.1f° (dir=%d)\n", value, moveDir);
    }
    else if (cmd.equalsIgnoreCase("l"))
    {

      encLeft.clearCount();
      targetAngle = fabs(value);
      moveDir = (value >= 0) ? 1 : -1;
      liftMode = false;
      moving = true;
      Serial.printf("lift %.1f° (dir=%d)\n", value, moveDir);
    }
    else if (cmd.equalsIgnoreCase("s"))
    {

      stopMotors();
      int servoAngle = constrain((int)value, 0, 180);
      manipServo.write(servoAngle);
    
      Serial.printf("🧭 Servo moved to %d°\n", servoAngle);
     // stopMotors();
    }
//  while (Serial.available())
//       Serial.read(); // flush buffer
  }

  // === Motion control ===
  if (moving)
  {
    long encCount = encLeft.getCount();
    currentAngle = fabs(getJointAngle(encCount));

    if (currentAngle < targetAngle - ANGLE_TOLERANCE)
    {
      driveMotors(FIXED_PWM, liftMode, moveDir);
    }
    else
    {
      stopMotors();
      moving = false;
      Serial.printf(" Completed %.2f° | Target %.2f°\n", currentAngle, targetAngle);
    }

    Serial.printf("Current: %.2f° | Target: %.2f° | Dir: %d\n", currentAngle, targetAngle, moveDir);
  }

delay(50);
}
