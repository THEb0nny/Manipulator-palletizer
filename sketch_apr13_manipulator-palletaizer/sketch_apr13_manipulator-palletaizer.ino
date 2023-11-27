// https://github.com/GyverLibs/ServoSmooth
// https://github.com/GyverLibs/GParser

#include <ServoSmooth.h>
#include <TimerMs.h>
#include <GParser.h>
#include <math.h>

#define SERVO_AMOUNT 4 // Всего сервоприводов манипулятора включая пневматический захват

#define SERVO1_PIN 10 // Пин сервопривода 1 в основании
#define SERVO2_PIN 9 // Пин сервопривода 2
#define SERVO3_PIN 8 // Пин сервопривода 3
#define SERVO_CLAW_PIN 7 // Пин сервопривода 4 для пневматического захвата

#define SERVO_MIN_PULSE 500 // Минимальное значение имульса управления сервоприводами
#define SERVO_MAX_PULSE 2500 // Максимальное значение импульса управления сервоприводами

#define SERVO_RANGE_POS_PERFORMED 2 // Значение в каком диапазоне подтверждается занятие позиции сервоприводом

ServoSmooth servos[SERVO_AMOUNT](360); // Создаём объекты серв с указанием, что максимальный угол 360
ServoSmooth claw_servo(360); // Создаём объект серво пневматического захвата

TimerMs tmrPrint(500); // Таймер для печати инфы в интервале

int servosPins[SERVO_AMOUNT] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO_CLAW_PIN}; // Массив значений пинов подключения сервоприводов
int servosStartupPos[SERVO_AMOUNT] = {180, 180, 180, 180}; // Позиция серв при старте
bool servosDir[SERVO_AMOUNT] = {true, false, false, false}; // Массив значений о инверсии сервопривода
bool servosAutoDetach[SERVO_AMOUNT] = {true, true, true, true}; // Массив с значениями об автоотключении при достижении угла для сервоприводов
int servosSpeed[SERVO_AMOUNT] = {90, 90, 90, 90}; // Массив значений скоростей для сервоприводов
float servosAccel[SERVO_AMOUNT] = {0.3, 0.3, 0.3, 0.3}; // Массив значений ускорения сервоприводов

int robotState = 0; // Переменная для хранения состояния робота

float j1 = 0, j2 = 0, j3 = 0;
float j1_old = 180, j2_old = 180, j3_old = 180;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // Позволяет задать время ожидания данных
  Serial.println("Setup manipulator");
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].attach(servosPins[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE, servosStartupPos[i]); // Подключаем сервопривод в указанном пине, с указанием максимальных и минимальных длин импульса, а также с указанием стартового положения
    servos[i].setSpeed(servosSpeed[i]); // Установить скорость
    servos[i].setAccel(servosAccel[i]); // Установить ускорение
    servos[i].setDirection(servosDir[i]); // Установить направление вращения
    servos[i].setAutoDetach(servosAutoDetach[i]); // Автоотключение при достижении целевого угла (по умолчанию включено)
    servos[i].smoothStart(); // Смягчает движение серво из неизвестной позиции к стартовой. БЛОКИРУЮЩАЯ НА 1 СЕК! НЕ работает?
  }
  Serial.println("Start work"); // Сообщение о старте
  tmrPrint.setPeriodMode(); // Установить в режиме периода таймер печати
  tmrPrint.start(); // Запускаем таймер печати в режиме интервала
}

void loop() {
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].tick(); // Здесь происходит движение серво по встроенному таймеру!
    //servos[i].tickManual(); // Двигаем все сервы. Такой вариант эффективнее отдельных тиков
  }

  if (robotState == 0) {
    ParseFromSerialInputValues(true); // Парсим данные по Serial
    if (j1 != j1_old) {
      servos[0].setTargetDeg(round(j1)); // Устанавливаем значение для сервопривода
      j1_old = j1; // Переписываем в переменную для хранения старой позиции j1
      robotState = 1; // Установить режим работы
    }
  } else if (robotState == 1) {
    if (servos[0].tick() && CheckServoPosPerformed(servos[0])) { // Проверка, что сервопривод занял позицию
      robotState = 0; // Установить режим ожидания новых значений после завершения работы
    }
    if (tmrPrint.tick() || servos[0].tick()) {
      Serial.println("targDeg: " + String(servos[0].getTargetDeg()) + "\t" + "currDeg: " + String(servos[0].getCurrentDeg()) + "\t" + "state: " + String(robotState));
    }
  }

  /*
  if (servos[0].tick() && CheckServoPosPerformed(servos[0])) { // Сервопривод 0 занял позицию?
    if (robotState == 0) {
      ParseFromSerialInputValues(true); // Парсим данные по Serial
      servos[0].setTargetDeg(0);
      robotState = 1;
      //tmr.setTime(2000);
    } else if (robotState == 2) {
      float* ikServosDeg = Manipulator_IK(10, 15, 0); // Чтобы получать массив из функции нельзя отдельно создавать массив с выделением памяти, а потом записывать в него значение, т.к. будет утечка памяти!
      servos[0].setTargetDeg(round(ikServosDeg[0] * 0.666));
      delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
      robotState = 3;
    }
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

// Функция решения обратной задачи кинематики
float* Manipulator_IK(float x, float y, float z) {
  float a1 = atan(y / x);
  //Serial.println("a1_rad: " + String(a1));
  Serial.println("a1_deg: " + String(degrees(a1)));
  float a2 = 0;
  float a3 = 0;
  float *ik = new float[3]{0};
  ik[0] = 90 + degrees(a1), ik[1] = degrees(a2), ik[2] = degrees(a3);
  Serial.println("ik[0]: " + String(ik[0]));
  return ik;
}

// Парсинг значений из Serial
void ParseFromSerialInputValues(bool debug) {
  if (Serial.available() > 2) { // Если что-то прислали
    char inputStr[64]; // Массив символов для записи из Serial
    int amount = Serial.readBytesUntil(';', inputStr, 64); // Считать посимвольно до символа конца пакета точки с запятой и записать количество полученных байт в переменную
    inputStr[amount] = NULL; // Если отправляющее устройство не отправит нулевой символ, то он не запишется в буффер и вывод строк будет некорректным, решение дописать вручную и т.о. закрываем строку
    GParser data(inputStr, ','); // Парсим массив символов по символу запятой
    int am = data.split(); // Получаем количество данных, внимание, ломает строку!
    for (int i = 0; i < am; i++) {
      String tmpStr = data[i];
      tmpStr.replace(" ", ""); // Удалить пробел, если он был введёт по ошибке
      tmpStr.trim(); // Удаление ведущими и конечные пробелы
      char tmpCharArr[tmpStr.length()];
      tmpStr.toCharArray(tmpCharArr, tmpStr.length() + 1);
      if (debug) Serial.println(String(i) + ") " + tmpStr); // Вывести начальную строку
      GParser data2(tmpCharArr, ':'); // Парсим массив символов по символу запятой
      int am2 = data2.split(); // Получаем количество данных, внимание, ломает строку!
      if (am2 > 1) { // Если существует не только ключ, а ещё и значение
        String key = data2[0]; // Ключ - первое значение
        String value = data2[1]; // Значение - второе, или data.getInt(1), чтобы получить целое число
        if (debug) Serial.println("key: " + key + ", value: " + String(value)); // Вывод
        // Присваивание значений
        if (key.equals("j1")) {
          j1 = value.toInt();
        }
      }
    }
    if (debug) Serial.println(); // Перевод на новую строку для разделения значений, которые были введены
  }
}