#include <Arduino.h>

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

#define SDIR_FL 23
#define SPWM_FL 13
#define SCH_FL 4

#define SDIR_FR 14
#define SPWM_FR 25
#define SCH_FR 5

int drive_speed = 30;
int steer_speed = 140;

int current_speed = 0;      // actual PWM currently applied to drive motors
int current_dir = 0;        // -1 = backward, 0 = stopped, +1 = forward

const int RAMP_STEP = 5;    // change in PWM per ramp step
const int RAMP_DELAY = 25;  // ms between ramp steps

class Motor {
  int dir_pin, pwm_pin, ch;
public:
  Motor(int dir, int pwm, int ch_num) {
    dir_pin = dir; pwm_pin = pwm; ch = ch_num;
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
Motor sFL(SDIR_FL, SPWM_FL, SCH_FL);
Motor sFR(SDIR_FR, SPWM_FR, SCH_FR);

void applySpeedToDrives(int pwm) {
  mFL.setSpeed(pwm);
  mFR.setSpeed(pwm);
  mBL.setSpeed(pwm);
  mBR.setSpeed(pwm);
}

void setDriveDirection(int dir) {
  if (dir > 0) {
    mFL.forward(); mFR.forward(); mBL.forward(); mBR.forward();
  } else if (dir < 0) {
    mFL.backward(); mFR.backward(); mBL.backward(); mBR.backward();
  }
}

void smoothRampTo(int target_speed) {
  if (target_speed < 0) target_speed = 0;
  if (target_speed > 255) target_speed = 255;
  while (current_speed < target_speed) {
    current_speed += RAMP_STEP;
    if (current_speed > target_speed) current_speed = target_speed;
    applySpeedToDrives(current_speed);
    delay(RAMP_DELAY);
  }
  while (current_speed > target_speed) {
    current_speed -= RAMP_STEP;
    if (current_speed < target_speed) current_speed = target_speed;
    applySpeedToDrives(current_speed);
    delay(RAMP_DELAY);
  }
}

void smoothStopDrive() {
  smoothRampTo(0);
  current_dir = 0;
  applySpeedToDrives(0);
}

void driveForward() {
  if (current_dir == -1 && current_speed > 0) {
    smoothStopDrive();
  }
  setDriveDirection(+1);
  current_dir = 1;
  smoothRampTo(drive_speed);
}

void driveBackward() {
  if (current_dir == 1 && current_speed > 0) {
    smoothStopDrive();
  }
  setDriveDirection(-1);
  current_dir = -1;
  smoothRampTo(drive_speed);
}

void stopDrive() {
  smoothStopDrive();
}

void steerLeft() {
  sFL.backward(); sFR.forward();
  sFL.setSpeed(steer_speed); sFR.setSpeed(steer_speed);
}

void steerRight() {
  sFL.forward(); sFR.backward();
  sFL.setSpeed(steer_speed); sFR.setSpeed(steer_speed);
}

void stopSteer() {
  sFL.stop(); sFR.stop();
}

void setup() {
  Serial.begin(9600);
  delay(500);
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    switch (cmd) {
      case 'f': driveForward(); break;
      case 'b': driveBackward(); break;
      case 's': stopDrive(); break;
      case 'l': steerLeft(); break;
      case 'r': steerRight(); break;
      case 'q': stopSteer(); break;
      case '+':
        drive_speed = min(255, drive_speed + 5);
        if (current_dir != 0) smoothRampTo(drive_speed);
        Serial.print("drive_speed: "); Serial.println(drive_speed);
        break;
      case '-':
        drive_speed = max(0, drive_speed - 5);
        if (current_dir != 0) smoothRampTo(drive_speed);
        Serial.print("drive_speed: "); Serial.println(drive_speed);
        break;
      default: break;
    }
  }
  delay(10);
}
