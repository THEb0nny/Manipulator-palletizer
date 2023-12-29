// https://github.com/GyverLibs/ServoSmooth
// https://github.com/GyverLibs/ServoSmooth

//#include <ServoSmooth.h>
#include <Servo.h>
#include <TimerMs.h>
#include <GParser.h>
#include <math.h>

#define DEBUG_IK true
#define DEBUG_DEG_TO_SERVO_POS true

#define SERVO_AMOUNT 4 // Всего сервоприводов

#define SERVO1_PIN 10 // Пин сервопривода 1 в основании
#define SERVO2_PIN 9 // Пин сервопривода 2
#define SERVO3_PIN 8 // Пин сервопривода 3
#define SERVO4_PIN 7 // Пин сервопривода 4 для пневматического захвата

#define SERVO_MIN_PULSE 500 // Минимальное значение имульса управления сервоприводами
#define SERVO_MAX_PULSE 2500 // Максимальное значение импульса управления сервоприводами

//#define SERVO_RANGE_POS_PERFORMED 2 // Значение в каком диапазоне подтверждается занятие позиции сервоприводом

#define SERVO_DEG_OFFSET 38.5714286 // Отсуп на угол для разворота серво на математический разворот 180

#define MECHANICAL_MULTIPLIER_J1 1.4

Servo s1, s2, s3; // Объекты сервоприводов

//ServoSmooth servos[SERVO_AMOUNT](360); // Создаём объекты серв с указанием, что максимальный угол 360
TimerMs tmrPrint(500); // Таймер для печати инфы в интервале

int servosPins[SERVO_AMOUNT] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN, SERVO4_PIN}; // Массив значений пинов подключения сервоприводов
int servosStartupPos[SERVO_AMOUNT] = {180, 0, 45, 180}; // Позиция серв при старте перед тем
//bool servosDir[SERVO_AMOUNT] = {true, false, false, false}; // Массив значений о инверсии сервопривода
//bool servosAutoDetach[SERVO_AMOUNT] = {false, true, true, true}; // Массив с значениями об автоотключении при достижении угла для сервоприводов
//int servosSpeed[SERVO_AMOUNT] = {90, 90, 90, 90}; // Массив значений скоростей для сервоприводов
//float servosAccel[SERVO_AMOUNT] = {0.3, 0.3, 0.3, 0.3}; // Массив значений ускорения сервоприводов

int robotState = 0; // Переменная для хранения состояния робота

float x = 0, y = 0, z = 0;
float x_prev = 0, y_prev = 0, z_prev = 0;
float j1 = 0, j2 = 0, j3 = 0;
float j1r = 0, j2r = 0, j3r = 0;
float j1_prev = 0, j2_prev = 0, j3_prev = 0;
float j1r_prev = 0, j2r_prev = 0, j3r_prev = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // Позволяет задать время ожидания данных
  Serial.println("Setup manipulator");
  s1.attach(servosPins[0], SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  s2.attach(servosPins[1], SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  s3.attach(servosPins[2], SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  s1.writeMicroseconds(map(servosStartupPos[0], 0, 360, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  s2.writeMicroseconds(map(servosStartupPos[1], 0, 360, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  s3.writeMicroseconds(map(servosStartupPos[2], 0, 360, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
  /*
  for (byte i = 0; i < SERVO_AMOUNT; i++) {
    servos[i].attach(servosPins[i], SERVO_MIN_PULSE, SERVO_MAX_PULSE, servosStartupPos[i]); // Подключаем сервопривод в указанном пине, с указанием максимальных и минимальных длин импульса, а также с указанием стартового положения
    servos[i].smoothStart(); // Смягчает движение серво из неизвестной позиции к стартовой. БЛОКИРУЮЩАЯ НА 1 СЕК! НЕ работает?
    servos[i].setSpeed(servosSpeed[i]); // Установить скорость
    servos[i].setAccel(servosAccel[i]); // Установить ускорение
    servos[i].setDirection(servosDir[i]); // Установить направление вращения
    servos[i].setAutoDetach(servosAutoDetach[i]); // Автоотключение при достижении целевого угла (по умолчанию включено)
  }
  Serial.println("Start work"); // Сообщение о старте
  tmrPrint.setPeriodMode(); // Установить в режиме периода таймер печати
  tmrPrint.start(); // Запускаем таймер печати в режиме интервала
  */
  Serial.println("Start work"); // Сообщение о старте
  tmrPrint.setPeriodMode(); // Установить в режиме периода таймер печати
  tmrPrint.start(); // Запускаем таймер печати в режиме интервала
}

void loop() {
  if (robotState == 0) { // Режим ожидания новых значений
    ParseFromSerialInputValues(true);
    // Если какое-то из значений изменилось после ввода
    if (j1 != j1_prev) {
      s1.writeMicroseconds(map(convr(j1), 0, 360, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      Serial.println(convr(j1));
      j1_prev = j1; // Переписываем в переменную для хранения старой позиции j1
      //robotState = 1;
    } else if (j1r != j1r_prev) {
      s1.writeMicroseconds(map(j1r, 0, 360, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      j1r_prev = j1r;
      //robotState = 1;
    } else if (j2 != j2_prev) {
      s2.writeMicroseconds(map(j2, 0, 360, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      j2_prev = j2;
      //robotState = 1;
    } else if (j3 != j3_prev) {
      s3.writeMicroseconds(map(j3, 0, 360, SERVO_MIN_PULSE, SERVO_MAX_PULSE));
      j3_prev = j3;
      //robotState = 1;
    } else if (x != x_prev || y != y_prev || z != z_prev) {
      float* ikServosDeg = Manipulator_IK(x, y, z); // Чтобы получать массив из функции нельзя отдельно создавать массив с выделением памяти, а потом записывать в него значение, т.к. будет утечка памяти!
      int alphaJ1 = DegToServoJ1Pos(ikServosDeg[0]);
      s1.writeMicroseconds(map(alphaJ1, 0, 360, SERVO_MAX_PULSE, SERVO_MIN_PULSE));
      // robotState = 1;
      delete[] ikServosDeg; // Удалить память под массив из функции Manipulator_IK
      x_prev = x;
      y_prev = y;
      z_prev = z;
    }
  } else if (robotState == 1) { // Режим ожидания, когда сервоприводы займут позицию
    
  }
  
  /*
    for (byte i = 0; i < SERVO_AMOUNT; i++) {
      servos[i].tick(); // Здесь происходит движение серво по встроенному таймеру!
      //servos[i].tickManual(); // Двигаем все сервы. Такой вариант эффективнее отдельных тиков
    }

    if (robotState == 0) {
      ParseFromSerialInputValues(true); // Парсим данные по Serial
      if (j1 != j1_old) {
        servos[0].writeMicroseconds(j1); // Устанавливаем значение для сервопривода
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
/*
bool CheckServoPosPerformed(ServoSmooth servo) {
  if ((servo.getTargetDeg() - SERVO_RANGE_POS_PERFORMED) <= servo.getCurrentDeg() && servo.getCurrentDeg() <= (servo.getTargetDeg() + SERVO_RANGE_POS_PERFORMED)) {
    return true;
  }
  return false;
}
*/

int convr(int deg) {
    float d1 = 72;
    float value = 54;
    float cf = (float)deg/90;
    return (int)(value * cf - d1);
    //конвертатор
}

// Функция, которая расчитывает какой угол дать сервоприводу в J1, чтобы он занял тот самый угол, которая рассчитывается для манипулятора a1
int DegToServoJ1Pos(float deg) {
  if (deg < -SERVO_DEG_OFFSET) deg = 360 + deg; // Если уходим за -SERVO_DEG_OFFSET, то сервопривод так повернуться не может, тогда нужно пересчитать
  float convert = 180 - ((90 - deg) * MECHANICAL_MULTIPLIER_J1); // Если нужно изменить вращение манипулятора, тогда 180 - меняем знак
  if (DEBUG_DEG_TO_SERVO_POS) Serial.println("convert J1: " + String(convert));
  return round(convert); // Вернуть значение округлённым по математическим правилам
}

// Функция решения обратной задачи кинематики
float* Manipulator_IK(float x, float y, float z) {
  float a1 = atan2(y, x);
  float a2 = 0;
  float a3 = 0;
  if (DEBUG_IK) Serial.println("a1_rad: " + String(a1) + ", a1_deg: " + String(degrees(a1)));
  float *ik = new float[3]{0};
  ik[0] = degrees(a1), ik[1] = degrees(a2), ik[2] = degrees(a3);
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
        } else if (key.equals("j1r")) {
          j1r = value.toInt();
        } else if (key.equals("j2")) {
          j2 = value.toInt();
        } else if (key.equals("j3")) {
          j3 = value.toInt();
        } else if (key.equals("x")) {
          x = value.toInt();
        } else if (key.equals("y")) {
          y = value.toInt();
        } else if (key.equals("z")) {
          z = value.toInt();
        }
      }
    }
    if (debug) Serial.println(); // Перевод на новую строку для разделения значений, которые были введены
  }
}
