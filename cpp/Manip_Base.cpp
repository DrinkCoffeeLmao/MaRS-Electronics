//Base, Link1, Link2 
#include <Arduino.h>
#include <ESP32Encoder.h>

// --- Encoders ---
ESP32Encoder encoderBase;
ESP32Encoder encoderL1;
ESP32Encoder encoderL2;

// --- Motor pins ---
#define MOTOR_PWM_BASE 16
#define MOTOR_DIR_BASE 4

#define MOTOR_PWM_L1 5
#define MOTOR_DIR_L1 17

#define MOTOR_PWM_L2 22
#define MOTOR_DIR_L2 21

// --- Encoder CPR (counts per revolution) ---
#define COUNTS_PER_REV_BASE 120000.0
#define COUNTS_PER_REV_L1 37500.0
#define COUNTS_PER_REV_L2 120000.0

// --- Fixed PWM and tolerance ---
const int fixedPWM = 100;
const float angleTolerance = 2.0; // degrees

// --- Target angles ---
float targetBase = 0;
float targetL1 = 0;
float targetL2 = 0;

// --- Active flags ---
bool activeBase = false;
bool activeL1 = false;
bool activeL2 = false;

// --- LEDC PWM setup ---
const int pwmFreq = 20000; // 20 kHz
const int pwmRes = 8;      // 8-bit resolution
const int chBase = 0;
const int chL1 = 1;
const int chL2 = 2;

// --- Helper: convert encoder counts to angle ---
float getAngle(long count, float cpr)
{
  return (count / cpr) * 360.0;
}

// --- Helper: drive motor with fixed PWM ---
void motorDrive(int channel, int dirPin, int pwm)
{
  bool dir = pwm >= 0;
  digitalWrite(dirPin, dir);
  ledcWrite(channel, abs(pwm));
}

// --- Update one motor, return true when done ---
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
    motorDrive(channel, dirPin, 0); // stop motor
  }

  Serial.printf("%s | Angle: %.2f | Target: %.2f | Error: %.2f\n",
                name, angle, target, error);

  return done;
}

void setupPWM()
{
  ledcSetup(chBase, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_PWM_BASE, chBase);

  ledcSetup(chL1, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_PWM_L1, chL1);

  ledcSetup(chL2, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_PWM_L2, chL2);
}

void setup()
{
  Serial.begin(115200);

  // --- Encoders setup ---
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoderBase.attachHalfQuad(32, 33);
  encoderL1.attachHalfQuad(35, 34);
  encoderL2.attachHalfQuad(36, 39);

  encoderBase.clearCount();
  encoderL1.clearCount();
  encoderL2.clearCount();

  // --- Motor setup ---
  pinMode(MOTOR_DIR_BASE, OUTPUT);
  pinMode(MOTOR_DIR_L1, OUTPUT);
  pinMode(MOTOR_DIR_L2, OUTPUT);
  setupPWM();

  Serial.println("Send: id angle");
}

void loop()
{
  // --- Read serial command ---
  if (Serial.available())
  {
    int id = Serial.parseInt();
    float angle = Serial.parseFloat();

    if (id == 1)
    {
      targetBase = angle;
      activeBase = true;
      Serial.printf("New target BASE: %.2f°\n", targetBase);
    }
    else if (id == 2)
    {
      targetL1 = angle;
      activeL1 = true;
      Serial.printf("New target L1: %.2f°\n", targetL1);
    }
    else if (id == 3)
    {
      targetL2 = angle;
      activeL2 = true;
      Serial.printf("New target L2: %.2f°\n", targetL2);
    }
    else
    {
      Serial.println("Invalid motor ID! Use 1, 2, or 3.");
    }

    while (Serial.available())
      Serial.read(); // flush buffer
  }

  // --- Update all active motors in parallel ---
  if (activeBase)
  {
    if (updateMotor(encoderBase, chBase, MOTOR_DIR_BASE, targetBase, COUNTS_PER_REV_BASE, "Base"))
    {
      Serial.println("Base reached target.");
      activeBase = false;
    }
  }

  if (activeL1)
  {
    if (updateMotor(encoderL1, chL1, MOTOR_DIR_L1, targetL1, COUNTS_PER_REV_L1, "Link1"))
    {
      Serial.println("Link1 reached target.");
      activeL1 = false;
    }
  }

  if (activeL2)
  {
    if (updateMotor(encoderL2, chL2, MOTOR_DIR_L2, targetL2, COUNTS_PER_REV_L2, "Link2"))
    {
      Serial.println("Link2 reached target.");
      activeL2 = false;
    }
  }

  delay(100); // 10 Hz update rate
}
