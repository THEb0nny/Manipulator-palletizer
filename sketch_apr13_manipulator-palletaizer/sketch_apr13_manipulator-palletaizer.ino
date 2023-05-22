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
TimerMs tmrPrint(300, 0, 0);

int servosPins[SERVO_AMOUNT] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN};
int servosStartupPos[SERVO_AMOUNT] = {180, 180, 180, 180}; // Позиция серв при старте перед тем
bool servosDir[SERVO_AMOUNT] = {true, false, false, false};
bool servosAutoDetach[SERVO_AMOUNT] = {true, true, true, true}; // Массив с значениями об автоотключении при достижении угла для сервоприводов
int servosSpeed[SERVO_AMOUNT] = {90, 90, 90, 90};
float servosAccel[SERVO_AMOUNT] = {0.3, 0.3, 0.3, 0.3};
//int servosPos[SERVO_AMOUNT] = {0, 0, 0, 0};

int robotState = 0; // Переменная для хранения состояния робота

void setup() {
  Serial.begin(115200);
  Serial.println("Setup manipulator");
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].attach(servosPins[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE, servosStartupPos[i]);
    servos[i].smoothStart(); // Смягчает движение серво из неизвестной позиции к стартовой. БЛОКИРУЮЩАЯ НА 1 СЕК!
    servos[i].setSpeed(servosSpeed[i]); // Установить скорость
    servos[i].setAccel(servosAccel[i]); // Установить ускорение
    servos[i].setDirection(servosDir[i]); // Установить направление вращения
    servos[i].setAutoDetach(servosAutoDetach[i]); // Автоотключение при достижении целевого угла (по умолчанию включено)
  }
  Serial.println("Start work");
  tmr.setTime(2000);
  tmr.start();
  tmrPrint.start(); // Старт таймера печати
}

float* Manipulator_IK(float x, float y, float z) {
  float a1 = atan(y / x);
  //Serial.println("a1_rad: " + String(a1));
  //Serial.println("a1_deg: " + String(degrees(a1)));
  float a2 = 0;
  float a3 = 0;
  float *ik = new float[3]{0};
  ik[0] = degrees(a1), ik[1] = degrees(a2), ik[2] = degrees(a3);
  //Serial.println("ik[0]: " + String(ik[0]));
  return ik;
}

void loop() {
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].tick(); // Здесь происходит движение серво по встроенному таймеру!
    //servos[i].tickManual(); // Двигаем все сервы. Такой вариант эффективнее отдельных тиков
  }
  
  if (tmrPrint.tick()) {
    String strOut = "";
    strOut += String(servos[0].getCurrentDeg());
    if (servos[0].tick()) strOut += "\t > performed\t\t";
    else strOut += "\t not performed\t";
    strOut += "state: " + String(robotState);
    Serial.println(strOut);
  }

  if (servos[0].tick()) { // Сервопривод 0 занял позицию?
    //float* ikServosDeg = new float[3]{0};
    if (robotState == 0) {
      servos[0].setTargetDeg(0);
      robotState = 1;
    } else if (robotState == 1) {
      servos[0].setTargetDeg(round(54.1));
      robotState = 2;
    } else if (robotState == 2) {
      float* ikServosDeg = new float[3]{0};
      ikServosDeg = Manipulator_IK(0, 15, 0);
      servos[0].setTargetDeg(round(ikServosDeg[0]));
      delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
      //servos[0].setTargetDeg(180);
      robotState = 3;
    } else if (robotState == 3) {
      servos[0].setTargetDeg(round(305.91));
      robotState = 4;
    } else if (robotState == 4) {
      servos[0].setTargetDeg(round(360));
      robotState = 0;
    }
    //delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
  }

  /*
  if (tmr.tick() && true) {
    float* ikServosDeg = new float[SERVO_AMOUNT]{0};
    Serial.println("Tmr finish");
    if (robotState == 0) {
      servos[0].setTargetDeg(0);
      tmr.setTime(3000);
      robotState = 1;
    } else if (robotState == 1) {
      servos[0].setTargetDeg(round(54.1));
      tmr.setTime(2000);
      robotState = 2;
    } else if (robotState == 2) {
      ikServosDeg = Manipulator_IK(30, 50, 0);
      servos[0].setTargetDeg(round(ikServosDeg[0]));
      tmr.setTime(2000);
      robotState = 3;
    } else if (robotState == 3) {
      servos[0].setTargetDeg(round(305.91));
      tmr.setTime(2000);
      robotState = 4;
    } else if (robotState == 4) {
      servos[0].setTargetDeg(round(360));
      tmr.setTime(2000);
      robotState = 0;
    }
    
    delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
  }
  */
}
