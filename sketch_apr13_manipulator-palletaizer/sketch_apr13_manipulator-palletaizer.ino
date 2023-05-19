#include <ServoSmooth.h>
#include <TimerMs.h>
#include <math.h>

#define SERVO_AMOUNT 4

#define SERVO1_PIN 10
#define SERVO2_PIN 9
#define SERVO3_PIN 8
#define SERVO4_PIN 7

#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500

ServoSmooth servos[SERVO_AMOUNT];

TimerMs tmr(1000, 0, 0);

int servosPins[SERVO_AMOUNT] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};
bool servosDir[SERVO_AMOUNT] = {false, false, false, false};
int servosPos[SERVO_AMOUNT] = {0, 0, 0, 0};
int servosSpeed[SERVO_AMOUNT] = {90, 90, 90, 90};
float servosAccel[SERVO_AMOUNT] = {0.3, 0.3, 0.3, 0.3};

float* ikServosDeg = new float[SERVO_AMOUNT]{0};

int robotState = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Setup");
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].attach(10, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    //servos[i].smoothStart();
    servos[i].setSpeed(servosSpeed[i]);
    servos[i].setAccel(servosAccel[i]);
    servos[i].setMaxAngle(360);
    servos[i].setDirection(servosDir[i]);
    servos[i].setAutoDetach(true);
  }
  // Установить на среднуюю позицию
  //servos[0].setTarget(GetPulseWidthFromDeg(180));
  servos[0].setTargetDeg(180);
  //delay(1000);
  Serial.println("Start");
  tmr.setTime(1000);
  tmr.start();
}

float* Manipulator_IK(float x, float y, float z) {
  float a1 = atan(x / y);
  //Serial.println("a1_rad: " + String(a1));
  //Serial.println("a1_deg: " + String(degrees(a1)));
  float a2 = 0;
  float a3 = 0;
  float *ik = new float[SERVO_AMOUNT - 1]{0};
  ik[0] = 180 + degrees(a1), ik[1] = degrees(a2), ik[2] = degrees(a3);
  Serial.println("ik[0]: " + String(ik[0]));
  return ik;
}

void loop() {
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].tick(); // Здесь происходит движение серво по встроенному таймеру!
    //servos[i].tickManual(); // Двигаем все сервы. Такой вариант эффективнее отдельных тиков
  }

  if (tmr.tick() && true) {
    Serial.println("Tmr finish");
    if (robotState == 0) {
      servosPos[0] = 0;
      tmr.setTime(3000);
      robotState = 1;
    } else if (robotState == 1) {
      servosPos[0] = round(54.1);
      tmr.setTime(3000);
      robotState = 2;
    } else if (robotState == 2) {
      ikServosDeg = Manipulator_IK(30, 50, 0);
      servosPos[0] = ikServosDeg[0] * 1.4;
      tmr.setTime(3000);
      robotState = 3;
    } else if (robotState == 3) {
      servosPos[0] = round(305.91);
      tmr.setTime(3000);
      robotState = 4;
    } else if (robotState == 4) {
      servosPos[0] = 360;
      tmr.setTime(1000);
      robotState = 0;
    }
    Serial.println("servPos: " + String(servosPos[0]) + "\t" + "robotState: " + String(robotState));
    //servos[0].setTarget(GetPulseWidthFromDeg(servosPos[0]));
    servos[0].setTargetDeg(servosPos[0]);
    //delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
  }
}

int GetPulseWidthFromDeg(float deg) {
  return map((int) deg, 0, 360, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

void Turn1(int x, int y) {
  int angle;
  int tan0 = y / x;
  int tan1;
  int d;
  int k = 2;
  if (x >= 0) {
    tan0 = tan;
    d = 0;
  } else {
    tan0 = abs(tan1);
    d = -180;
  }
  angle = (180 - atan(tan1) * 180 / M_PI) * k + d;
  servos[0].setTarget(map(angle, 0, 360, 500, 2500));
  //srv1.writeMicroseconds(map(angle, 0, 360, 500, 2500));
}