//Multi-Steering Code
#include <Arduino.h>
#include <ESP32Encoder.h>

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

#define MOTOR_PWM_BL 19
#define MOTOR_DIR_BL 18

#define MOTOR_PWM_BR 22
#define MOTOR_DIR_BR 21

// --- Encoder CPR ---
#define COUNTS_PER_REV_FL 200000.0
#define COUNTS_PER_REV_FR 200000.0
#define COUNTS_PER_REV_BL 120000.0
#define COUNTS_PER_REV_BR 120000.0

// --- Fixed PWM and tolerance ---
const int fixedPWM = 100;
const float angleTolerance = 2.0;

// --- Target angles ---
float targetFL = 0;
float targetFR = 0;
float targetBL = 0;
float targetBR = 0;

// --- Active flags ---
bool activeFL = false;
bool activeFR = false;
bool activeBL = false;
bool activeBR = false;

// --- LEDC ---
const int pwmFreq = 20000;
const int pwmRes = 8;
const int chFL = 0;
const int chFR = 1;
const int chBL = 2;
const int chBR = 3;

// --- Convert encoder counts to degrees ---
float getAngle(long count, float cpr)
{
  return (count / cpr) * 360.0;
}

// --- Motor drive ---
void motorDrive(int channel, int dirPin, int pwm)
{
  bool dir = pwm >= 0;
  digitalWrite(dirPin, dir);
  ledcWrite(channel, abs(pwm));
}

// --- Update motor ---
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

  Serial.printf("%s | Angle: %.2f | Target: %.2f | Error: %.2f\n",
                name, angle, target, error);

  return done;
}

// --- Apply one command ---
void applyCommand(int id, float angle)
{
  if (id == 1)
  {
    targetFL = angle;
    activeFL = true;
    Serial.printf("New target BASE: %.2f°\n", targetFL);
  }
  else if (id == 2)
  {
    targetFR = angle;
    activeFR = true;
    Serial.printf("New target L1: %.2f°\n", targetFR);
  }
  else if (id == 3)
  {
    targetBL = angle;
    activeBL = true;
    Serial.printf("New target L2: %.2f°\n", targetBL);
  }
  else if (id == 4)
  {
    targetBR = angle;
    activeBR = true;
    Serial.printf("New target L2: %.2f°\n", targetBR);
  }
  else
  {
    Serial.println("Invalid motor ID!");
  }
}

void setupPWM()
{
  ledcSetup(chFL, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_PWM_FL, chFL);

  ledcSetup(chFR, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_PWM_FR, chFR);

  ledcSetup(chBL, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_PWM_BL, chBL);

  ledcSetup(chBR, pwmFreq, pwmRes);
  ledcAttachPin(MOTOR_PWM_BR, chBR);
}

void setup()
{
  Serial.begin(9600);

  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  FrontLeft.attachHalfQuad(33, 32);
  FrontRight.attachHalfQuad(26, 27);
  BackLeft.attachHalfQuad(35, 34);
  BackRight.attachHalfQuad(39, 36);

  FrontLeft.clearCount();
  FrontRight.clearCount();
  BackLeft.clearCount();
  BackRight.clearCount();

  pinMode(MOTOR_DIR_FL, OUTPUT);
  pinMode(MOTOR_DIR_FR, OUTPUT);
  pinMode(MOTOR_DIR_BL, OUTPUT);
  pinMode(MOTOR_DIR_BR, OUTPUT);

  setupPWM();
  Serial.println("Send: id1 angle1 id2 angle2");
}

void loop()
{
  // --- Read TWO input commands ---
  if (Serial.available())
  {
    int id1 = Serial.parseInt();
    float angle1 = Serial.parseFloat();

    int id2 = Serial.parseInt();
    float angle2 = Serial.parseFloat();

    applyCommand(id1, angle1);
    applyCommand(id2, angle2);

    while (Serial.available())
      Serial.read();
  }

  // --- Update all motors ---
  if (activeFL)
    if (updateMotor(FrontLeft, chFL, MOTOR_DIR_FL, targetFL, COUNTS_PER_REV_FL, "FrontLeft"))
      activeFL = false;

  if (activeFR)
    if (updateMotor(FrontRight, chFR, MOTOR_DIR_FR, targetFR, COUNTS_PER_REV_FR, "FrontRight"))
      activeFR = false;

  if (activeBL)
    if (updateMotor(BackLeft, chBL, MOTOR_DIR_BL, targetBL, COUNTS_PER_REV_BL, "BackLeft"))
      activeBL = false;

  if (activeBR)
    if (updateMotor(BackRight, chBR, MOTOR_DIR_BR, targetBR, COUNTS_PER_REV_BR, "BackRight"))
      activeBR = false;

  // --- Print current angles of all motors ---
  Serial.printf("FL: %.2f°, FR: %.2f°, BL: %.2f°, BR: %.2f°\n",
                getAngle(FrontLeft.getCount(), COUNTS_PER_REV_FL),
                getAngle(FrontRight.getCount(), COUNTS_PER_REV_FR),
                getAngle(BackLeft.getCount(), COUNTS_PER_REV_BL),
                getAngle(BackRight.getCount(), COUNTS_PER_REV_BR));

  delay(100);
}
