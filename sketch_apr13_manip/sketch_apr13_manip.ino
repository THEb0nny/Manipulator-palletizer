#include <ServoSmooth.h>
#include <TimerMs.h>
#include <math.h>

#define SERVO_AMOUNT 2

ServoSmooth servo1;
//ServoSmooth servos[SERVO_AMOUNT];

TimerMs tmr(1000, 0, 0);

int serv1Pos = 0;
int robotState = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Setup");
  servo1.attach(10, 500, 2500);
  //servo1.setTarget(map(180, 0, 360, 500, 2500));
  //servo1.smoothStart();
  servo1.writeMicroseconds(map(180, 0, 360, 500, 2500));
  servo1.setSpeed(120);
  servo1.setAccel(0.2);
  //servos[0].attach(10, 500, 2500);
  //servos[1].attach(7, 500, 2500);
  //servos[2].attach(8, 500, 2500);
  //servos[3].attach(7, 500, 2500);
  //servos[0].setSpeed(10);
  //servos[0].setAccel(0.1);
  //servos[1].setSpeed(10);
  //servos[1].setAccel(0.1);
  delay(1000);
  Serial.println("Start");
  tmr.setTime(1000);
  tmr.start();
}

void loop() {
  servo1.tick(); // Здесь происходит движение серво по встроенному таймеру!
  /*
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].tick(); // Здесь происходит движение серво по встроенному таймеру!
    //servos[i].tickManual(); // Двигаем все сервы. Такой вариант эффективнее отдельных тиков
  }
  */
  if (tmr.tick()) {
    Serial.println("Tmr finish");
    if (robotState == 0) {
      serv1Pos = 0;
      tmr.setTime(1500);
      robotState = 1;
    } else if (robotState == 1) {
      serv1Pos = 90;
      tmr.setTime(1500);
      robotState = 2;
    } else if (robotState == 2) {
      serv1Pos = 180;
      tmr.setTime(2000);
      robotState = 3;
    } else if (robotState == 3) {
      serv1Pos = 270;
      tmr.setTime(1500);
      robotState = 4;
    } else if (robotState == 4) {
      serv1Pos = 360;
      tmr.setTime(1500);
      robotState = 0;
    } else {
      tmr.setTime(100);
      robotState = 0;
    }
    Serial.println("serv1Pos: " + String(serv1Pos) + "\t" + "robotState: " + String(robotState));
    servo1.setTarget(map(serv1Pos, 0, 360, 500, 2500));
  }

  /*
  Turn1(10, 5);
  delay(200);
  srv3.writeMicroseconds(map(170, 0, 360, 500, 2500));
  delay(2000);
  srv2.writeMicroseconds(map(170, 0, 360, 500, 2500));
  delay(500);
  srv4.writeMicroseconds(map(360, 0, 360, 500, 2500));
  delay(500);
  srv2.writeMicroseconds(map(360, 0, 360, 500, 2500));
  */
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