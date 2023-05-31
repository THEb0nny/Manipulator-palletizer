#include <ServoSmooth.h>
#include <TimerMs.h>
#include <math.h>

#define SERVO_AMOUNT 4 // Всего сервоприводов

#define SERVO1_PIN 10 // Пин сервопривода 1 в основании
#define SERVO2_PIN 9 // Пин сервопривода 2
#define SERVO3_PIN 8 // Пин сервопривода 3
#define SERVO4_PIN 7 // Пин сервопривода 4

#define SERVO_MIN_PULSE 500 // Минимальное значение имульса управления сервоприводами
#define SERVO_MAX_PULSE 2500 // Максимальное значение импульса управления сервоприводами

#define SERVO_RANGE_POS_PERFORMED 2 // Значение в каком диапазоне подтверждается занятие позиции сервоприводом

ServoSmooth servos[SERVO_AMOUNT](360); // Создаём объекты серв с указанием, что максимальный угол 360

TimerMs tmr(1000, 0, 0); // Вспомогательный таймер
TimerMs tmrPrint(300, 0, 0); // Таймер для печати инфы в интервале

int servosPins[SERVO_AMOUNT] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN}; // Массив значений пинов подключения сервоприводов
int servosStartupPos[SERVO_AMOUNT] = {180, 180, 180, 180}; // Позиция серв при старте перед тем
bool servosDir[SERVO_AMOUNT] = {true, false, false, false}; // Массив значений о инверсии сервопривода
bool servosAutoDetach[SERVO_AMOUNT] = {true, true, true, true}; // Массив с значениями об автоотключении при достижении угла для сервоприводов
int servosSpeed[SERVO_AMOUNT] = {90, 90, 90, 90}; // Массив значений скоростей для сервоприводов
float servosAccel[SERVO_AMOUNT] = {0.3, 0.3, 0.3, 0.3}; // Массив значений ускорения сервоприводов

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
  ik[0] = 90 + degrees(a1), ik[1] = degrees(a2), ik[2] = degrees(a3);
  //Serial.println("ik[0]: " + String(ik[0]));
  return ik;
}

void loop() {
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].tick(); // Здесь происходит движение серво по встроенному таймеру!
    //servos[i].tickManual(); // Двигаем все сервы. Такой вариант эффективнее отдельных тиков
  }
  
  if (tmrPrint.tick() || servos[0].tick()) {
    String strOut = "";
    strOut += "targetDeg: " + String(servos[0].getTargetDeg()) + "\t";
    strOut += "currentDeg: " + String(servos[0].getCurrentDeg());
    if (servos[0].tick()) strOut += "\t performed\t\t";
    else strOut += "\t not performed\t\t";
    strOut += "state: " + String(robotState);
    Serial.println(strOut);
  }

  if (servos[0].tick() && CheckServoPosPerformed(servos[0])) { // Сервопривод 0 занял позицию?
    if (robotState == 0) {
      servos[0].setTargetDeg(0);
      robotState = 1;
    } else if (robotState == 1) {
      servos[0].setTargetDeg(round(54.1));
      robotState = 2;
    } else if (robotState == 2) {
      float* ikServosDeg = Manipulator_IK(0, 15, 0); // Чтобы получать массив из функции нельзя отдельно создавать массив с выделением памяти, а потом записывать в него значение, т.к. будет утечка памяти!
      servos[0].setTargetDeg(round(ikServosDeg[0]));
      delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
      robotState = 3;
    } else if (robotState == 3) {
      servos[0].setTargetDeg(round(305.91));
      robotState = 4;
    } else if (robotState == 4) {
      servos[0].setTargetDeg(round(360));
      robotState = 0;
    }
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

// Фукнкция, которая проверяет занял ли серво позицию, находится в диапазоне +-
bool CheckServoPosPerformed(ServoSmooth servo) {
  if ((servo.getTargetDeg() - SERVO_RANGE_POS_PERFORMED) <= servo.getCurrentDeg() && servo.getCurrentDeg() <= (servo.getTargetDeg() + SERVO_RANGE_POS_PERFORMED)) {
    return true;
  }
  return false;
}
