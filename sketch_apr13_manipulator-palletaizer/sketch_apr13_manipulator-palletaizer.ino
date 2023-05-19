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
#define SERVO_START_POS 90

ServoSmooth servos[SERVO_AMOUNT](360); // Создаём объекты серв с указанием, что максимальный угол 360

TimerMs tmr(1000, 0, 0);
TimerMs tmrDebug(200, 1, 0);

int servosPins[SERVO_AMOUNT] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};
int servosStartupPos[SERVO_AMOUNT] = {180, 180, 180, 180}; // Позиция серв при старте перед тем
bool servosDir[SERVO_AMOUNT] = {true, false, false, false};
int servosPos[SERVO_AMOUNT] = {0, 0, 0, 0};
int servosSpeed[SERVO_AMOUNT] = {90, 90, 90, 90};
float servosAccel[SERVO_AMOUNT] = {0.3, 0.3, 0.3, 0.3};

int robotState = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Setup manipulator");
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].attach(servosPins[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE, servosStartupPos[i]);
    servos[i].smoothStart(); // Смягчает движение серво из неизвестной позиции к стартовой. БЛОКИРУЮЩАЯ НА 1 СЕК!
    servos[i].setSpeed(servosSpeed[i]); // Установить скорость
    servos[i].setAccel(servosAccel[i]); // Установить ускорение
    servos[i].setDirection(servosDir[i]); // Установить направление вращения
    servos[i].setAutoDetach(true); // Автоотключение при достижении целевого угла (по умолчанию включено)
  }
  Serial.println("Start work");
  tmr.setTime(2000);
  tmr.start();
}

float* Manipulator_IK(float x, float y, float z) {
  float a1 = atan(x / y);
  //Serial.println("a1_rad: " + String(a1));
  //Serial.println("a1_deg: " + String(degrees(a1)));
  float a2 = 0;
  float a3 = 0;
  float *ik = new float[SERVO_AMOUNT - 1]{0};
  ik[0] = 90 + degrees(a1), ik[1] = degrees(a2), ik[2] = degrees(a3);
  Serial.println("ik[0]: " + String(ik[0]));
  return ik;
}

void loop() {
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].tick(); // Здесь происходит движение серво по встроенному таймеру!
    //servos[i].tickManual(); // Двигаем все сервы. Такой вариант эффективнее отдельных тиков
  }
  if (tmrDebug.tick()) {
    Serial.println(servos[0].getCurrentDeg());
    if (servos[0].tick()) Serial.println("done");
  }
  
  if (tmr.tick() && true) {
    float* ikServosDeg = new float[SERVO_AMOUNT]{0};
    Serial.println("Tmr finish");
    tmr.setTime(2000);
    if (robotState == 0) {
      servosPos[0] = 0;
      tmr.setTime(2000);
      robotState = 1;
    } else if (robotState == 1) {
      servosPos[0] = round(54.1);
      tmr.setTime(2000);
      robotState = 2;
    } else if (robotState == 2) {
      ikServosDeg = Manipulator_IK(30, 50, 0);
      servosPos[0] = round(ikServosDeg[0]);
      tmr.setTime(2000);
      robotState = 3;
    } else if (robotState == 3) {
      servosPos[0] = round(305.91);
      tmr.setTime(2000);
      robotState = 4;
    } else if (robotState == 4) {
      servosPos[0] = 360;
      tmr.setTime(2000);
      robotState = 0;
    }
    Serial.println("serv1Pos: " + String(servosPos[0]) + "\t" + "robotState: " + String(robotState));
    servos[0].setTargetDeg(servosPos[0]);
    delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
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
