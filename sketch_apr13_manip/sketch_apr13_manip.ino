#include <ServoSmooth.h>
#include <TimerMs.h>
#include <math.h>

#define SERVO_AMOUNT 4

#define SERVO1_PIN 10
#define SERVO2_PIN 9
#define SERVO3_PIN 8
#define SERVO4_PIN 7

ServoSmooth servos[SERVO_AMOUNT];

TimerMs tmr(1000, 0, 0);

int servosPins[SERVO_AMOUNT] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};
int servosPos[SERVO_AMOUNT] = {0, 0, 0, 0};

int robotState = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Setup");
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].attach(10, 500, 2500);
    servos[i].smoothStart();
    servos[i].setSpeed(90);
    servos[i].setAccel(0.2);
    servos[i].setAutoDetach(true);
  }
  servos[0].setTarget(map(180, 0, 360, 500, 2500));
  delay(1000);
  Serial.println("Start");
  tmr.setTime(1000);
  tmr.start();
}

void loop() {
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].tick(); // Здесь происходит движение серво по встроенному таймеру!
    //servos[i].tickManual(); // Двигаем все сервы. Такой вариант эффективнее отдельных тиков
  }

  if (tmr.tick()) {
    Serial.println("Tmr finish");
    if (robotState == 0) {
      servosPos[0] = 0;
      tmr.setTime(1500);
      robotState = 1;
    } else if (robotState == 1) {
      servosPos[0] = 90;
      tmr.setTime(1500);
      robotState = 2;
    } else if (robotState == 2) {
      servosPos[0] = 180;
      tmr.setTime(2000);
      robotState = 3;
    } else if (robotState == 3) {
      servosPos[0] = 270;
      tmr.setTime(1500);
      robotState = 4;
    } else if (robotState == 4) {
      servosPos[0] = 360;
      tmr.setTime(1500);
      robotState = 0;
    } else {
      tmr.setTime(100);
      robotState = 0;
    }
    Serial.println("servPos: " + String(servosPos[0]) + "\t" + "robotState: " + String(robotState));
    servos[1].setTarget(map(servosPos[0], 0, 360, 500, 2500));
  }
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
  //srv1.writeMicroseconds(map(angle, 0, 360, 500, 2500));
}