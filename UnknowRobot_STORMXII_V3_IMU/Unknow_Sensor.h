#ifndef UNKNOW_SENSOR_H
#define UNKNOW_SENSOR_H

#include <Arduino.h>
#include <EEPROM.h>

/* ==================================================================================================
 *  Unknow_Sensor.h — เซนเซอร์เส้น หน้า (F) / หลัง (B) 16 ตัว และคู่กลาง (C) 2 ตัว
 *
 *   1) ขา / ตัวแปร
 *   2) ค่าตั้ง (Config)
 *   3) อ่านค่าดิบ (Mux)
 *   4) ค่าคาลิเบรต (LightValue_xxx)
 *   5) อ่านค่าที่คาลิเบรตแล้ว (0 - 1000)
 *   6) คาลิเบรตอัตโนมัติ / EEPROM
 *   7) Serial debug
 * ================================================================================================== */


/* ==================================================================================================
 *  1) ขา / ตัวแปร
 * ================================================================================================== */

#define NUM_SENSORS 16
#define S0 14
#define S1 16
#define S2 15
#define S3 13
#define S3B 17
#define SIG A0
#define SIGB A1

#define BLACK 0
#define WHITE 1
#define CCL 0  // เซนเซอร์กลางซ้าย C[0]
#define CCR 1  // เซนเซอร์กลางขวา C[1]

uint8_t F_PIN[NUM_SENSORS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
uint8_t B_PIN[NUM_SENSORS] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };

int F[NUM_SENSORS], B[NUM_SENSORS], C[2];
int F_Ref[NUM_SENSORS], B_Ref[NUM_SENSORS], C_Ref[2];
int minValueF[NUM_SENSORS], maxValueF[NUM_SENSORS];
int minValueB[NUM_SENSORS], maxValueB[NUM_SENSORS];
int minValueC[2], maxValueC[2];


/* ==================================================================================================
 *  2) ค่าตั้ง (Config)
 * ================================================================================================== */

// ช่วงกรองค่าหลังคาลิเบรต: ต่ำกว่า MIN → 0, สูงกว่า MAX → 1000
int FRONT_MIN = 80;
int FRONT_MAX = 900;
int CENTER_MIN = 0;
int CENTER_MAX = 1000;
int BACK_MIN = 80;
int BACK_MAX = 90;

int Ref = 500;        // ค่าแบ่งขาว/ดำ เซนเซอร์หน้า-หลัง
int RefC = 500;       // ค่าแบ่งขาว/ดำ เซนเซอร์คู่กลาง
int LineColor = 0;    // BLACK / WHITE
int TrackLineCH = 0;  // จำนวนเซนเซอร์ที่ใช้จับแยก

void clampSensorValueF(int x, int y) {
  FRONT_MIN = x;
  FRONT_MAX = y;
}

void clampSensorValueC(int x, int y) {
  CENTER_MIN = x;
  CENTER_MAX = y;
}

void clampSensorValueB(int x, int y) {
  BACK_MIN = x;
  BACK_MAX = y;
}

void TrackLineColor(int Col) {
  LineColor = Col;
}

void SwitchLine() {
  LineColor = !LineColor;
}

void RefLineValue(int x) {
  Ref = x;
}

void RefCenterLineValue(int x) {
  RefC = x;
}

void SetSensorTrackLine_CH(int x) {
  TrackLineCH = x;
}


/* ==================================================================================================
 *  3) อ่านค่าดิบ (Mux)
 * ================================================================================================== */

inline void muxSelectF(uint8_t ch) {
  digitalWrite(S0, (ch >> 0) & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3, (ch >> 3) & 1);
}

inline void muxSelectB(uint8_t ch) {
  digitalWrite(S0, (ch >> 0) & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3B, (ch >> 3) & 1);
}

int muxReadF(uint8_t ch) {
  muxSelectF(ch);
  delayMicroseconds(3);
  return analogRead(SIG);
}

int muxReadB(uint8_t ch) {
  muxSelectB(ch);
  delayMicroseconds(3);
  return analogRead(SIGB);
}

uint16_t read_sensorA(int sensor) {
  return muxReadF(sensor);
}

uint16_t read_sensorB(int sensor) {
  return muxReadB(sensor);
}

void ReadF() {
  for (int i = 0; i < NUM_SENSORS; i++) F[i] = read_sensorA(F_PIN[i]);
}

void ReadB() {
  for (int i = 0; i < NUM_SENSORS; i++) B[i] = read_sensorB(B_PIN[i]);
}

void ReadC() {
  C[0] = analogRead(A6);
  C[1] = analogRead(A7);
}

void ReadSensorRaw() {
  ReadF();
  ReadB();
  ReadC();
}


/* ==================================================================================================
 *  4) ค่าคาลิเบรต — คัดลอกบรรทัดที่ CalibrateSensor() พิมพ์ออกมาไปวางใน setup()
 * ================================================================================================== */

void LightValue_FrontSensor(
  uint16_t minF0, uint16_t minF1, uint16_t minF2, uint16_t minF3,
  uint16_t minF4, uint16_t minF5, uint16_t minF6, uint16_t minF7,
  uint16_t minF8, uint16_t minF9, uint16_t minF10, uint16_t minF11,
  uint16_t minF12, uint16_t minF13, uint16_t minF14, uint16_t minF15,
  uint16_t maxF0, uint16_t maxF1, uint16_t maxF2, uint16_t maxF3,
  uint16_t maxF4, uint16_t maxF5, uint16_t maxF6, uint16_t maxF7,
  uint16_t maxF8, uint16_t maxF9, uint16_t maxF10, uint16_t maxF11,
  uint16_t maxF12, uint16_t maxF13, uint16_t maxF14, uint16_t maxF15) {
  const uint16_t mins[NUM_SENSORS] = { minF0, minF1, minF2, minF3, minF4, minF5, minF6, minF7,
                                       minF8, minF9, minF10, minF11, minF12, minF13, minF14, minF15 };
  const uint16_t maxs[NUM_SENSORS] = { maxF0, maxF1, maxF2, maxF3, maxF4, maxF5, maxF6, maxF7,
                                       maxF8, maxF9, maxF10, maxF11, maxF12, maxF13, maxF14, maxF15 };
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueF[i] = mins[i];
    maxValueF[i] = maxs[i];
  }
}

void LightValue_BackSensor(
  uint16_t minB0, uint16_t minB1, uint16_t minB2, uint16_t minB3,
  uint16_t minB4, uint16_t minB5, uint16_t minB6, uint16_t minB7,
  uint16_t minB8, uint16_t minB9, uint16_t minB10, uint16_t minB11,
  uint16_t minB12, uint16_t minB13, uint16_t minB14, uint16_t minB15,
  uint16_t maxB0, uint16_t maxB1, uint16_t maxB2, uint16_t maxB3,
  uint16_t maxB4, uint16_t maxB5, uint16_t maxB6, uint16_t maxB7,
  uint16_t maxB8, uint16_t maxB9, uint16_t maxB10, uint16_t maxB11,
  uint16_t maxB12, uint16_t maxB13, uint16_t maxB14, uint16_t maxB15) {
  const uint16_t mins[NUM_SENSORS] = { minB0, minB1, minB2, minB3, minB4, minB5, minB6, minB7,
                                       minB8, minB9, minB10, minB11, minB12, minB13, minB14, minB15 };
  const uint16_t maxs[NUM_SENSORS] = { maxB0, maxB1, maxB2, maxB3, maxB4, maxB5, maxB6, maxB7,
                                       maxB8, maxB9, maxB10, maxB11, maxB12, maxB13, maxB14, maxB15 };
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueB[i] = mins[i];
    maxValueB[i] = maxs[i];
  }
}

void LightValue_CenterSensor(uint16_t minC0, uint16_t minC1, uint16_t maxC0, uint16_t maxC1) {
  minValueC[0] = minC0;
  minValueC[1] = minC1;
  maxValueC[0] = maxC0;
  maxValueC[1] = maxC1;
}


/* ==================================================================================================
 *  5) อ่านค่าที่คาลิเบรตแล้ว (0 - 1000, ค่ามาก = เจอเส้น)
 * ================================================================================================== */

void ReadCalibrateF() {
  ReadF();
  for (int i = 0; i < NUM_SENSORS; i++) {
    F[i] = constrain(F[i], minValueF[i], maxValueF[i]);
    int x;
    if (LineColor == BLACK) x = map(F[i], minValueF[i], maxValueF[i], 1000, 0);
    else x = map(F[i], minValueF[i], maxValueF[i], 0, 1000);
    if (x < FRONT_MIN) x = 0;
    if (x > FRONT_MAX) x = 1000;
    F[i] = x;
  }
}

void ReadCalibrateB() {
  ReadB();
  for (int i = 0; i < NUM_SENSORS; i++) {
    B[i] = constrain(B[i], minValueB[i], maxValueB[i]);
    int x;
    if (LineColor == BLACK) x = map(B[i], minValueB[i], maxValueB[i], 1000, 0);
    else x = map(B[i], minValueB[i], maxValueB[i], 0, 1000);
    if (x < BACK_MIN) x = 0;
    if (x > BACK_MAX) x = 1000;
    B[i] = x;
  }
}

// เซนเซอร์กลางกลับด้านกับหน้า-หลัง
void ReadCalibrateC() {
  ReadC();
  for (int i = 0; i < 2; i++) {
    C[i] = constrain(C[i], minValueC[i], maxValueC[i]);
    int x;
    if (LineColor == BLACK) x = map(C[i], minValueC[i], maxValueC[i], 0, 1000);
    else x = map(C[i], minValueC[i], maxValueC[i], 1000, 0);
    if (x < CENTER_MIN) x = 0;
    if (x > CENTER_MAX) x = 1000;
    C[i] = x;
  }
}

void ReadSensor() {
  ReadCalibrateF();
  ReadCalibrateB();
  ReadCalibrateC();
}


/* ==================================================================================================
 *  6) คาลิเบรตอัตโนมัติ / EEPROM
 * ================================================================================================== */

// เก็บค่า min / max จาก samples ครั้ง (เริ่ม min = 4095 สำหรับ ADC 12 บิต) แล้วบีบช่วงเข้า +60 / -80
void caribrateSensorF(int pauseTime, int samples) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueF[i] = 4095;
    maxValueF[i] = 0;
  }
  for (int s = 0; s <= samples; s++) {
    ReadF();
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (F[i] < minValueF[i]) minValueF[i] = F[i];
      if (F[i] > maxValueF[i]) maxValueF[i] = F[i];
    }
    delay(pauseTime);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueF[i] += 60;
    maxValueF[i] -= 80;
  }
}

void caribrateSensorB(int pauseTime, int samples) {
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueB[i] = 4095;
    maxValueB[i] = 0;
  }
  for (int s = 0; s <= samples; s++) {
    ReadB();
    for (int i = 0; i < NUM_SENSORS; i++) {
      if (B[i] < minValueB[i]) minValueB[i] = B[i];
      if (B[i] > maxValueB[i]) maxValueB[i] = B[i];
    }
    delay(pauseTime);
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    minValueB[i] += 60;
    maxValueB[i] -= 80;
  }
}

void caribrateSensorC(int pauseTime, int samples) {
  for (int i = 0; i < 2; i++) {
    minValueC[i] = 4095;
    maxValueC[i] = 0;
  }
  for (int s = 0; s <= samples; s++) {
    ReadC();
    for (int i = 0; i < 2; i++) {
      if (C[i] < minValueC[i]) minValueC[i] = C[i];
      if (C[i] > maxValueC[i]) maxValueC[i] = C[i];
    }
    delay(pauseTime);
  }
  for (int i = 0; i < 2; i++) {
    minValueC[i] += 60;
    maxValueC[i] -= 80;
  }
}

#define EEPROM_CALIB_ADDR 0
#define EEPROM_CALIB_MAGIC 0xCA11

struct SensorCalibData {
  uint16_t magic;
  int minValueF[NUM_SENSORS];
  int maxValueF[NUM_SENSORS];
  int minValueB[NUM_SENSORS];
  int maxValueB[NUM_SENSORS];
  int minValueC[2];
  int maxValueC[2];
};

void SaveCalibrateToEEPROM() {
  EEPROM.begin(sizeof(SensorCalibData));
  SensorCalibData data;
  data.magic = EEPROM_CALIB_MAGIC;
  memcpy(data.minValueF, minValueF, sizeof(minValueF));
  memcpy(data.maxValueF, maxValueF, sizeof(maxValueF));
  memcpy(data.minValueB, minValueB, sizeof(minValueB));
  memcpy(data.maxValueB, maxValueB, sizeof(maxValueB));
  memcpy(data.minValueC, minValueC, sizeof(minValueC));
  memcpy(data.maxValueC, maxValueC, sizeof(maxValueC));
  EEPROM.put(EEPROM_CALIB_ADDR, data);
  EEPROM.commit();
}

// คืน false ถ้ายังไม่เคยคาลิเบรต (EEPROM ว่าง)
bool LoadCalibrateFromEEPROM() {
  EEPROM.begin(sizeof(SensorCalibData));
  SensorCalibData data;
  EEPROM.get(EEPROM_CALIB_ADDR, data);
  if (data.magic != EEPROM_CALIB_MAGIC) return false;
  memcpy(minValueF, data.minValueF, sizeof(minValueF));
  memcpy(maxValueF, data.maxValueF, sizeof(maxValueF));
  memcpy(minValueB, data.minValueB, sizeof(minValueB));
  memcpy(maxValueB, data.maxValueB, sizeof(maxValueB));
  memcpy(minValueC, data.minValueC, sizeof(minValueC));
  memcpy(maxValueC, data.maxValueC, sizeof(maxValueC));
  return true;
}


/* ==================================================================================================
 *  7) Serial debug — พิมพ์ค่าออก Serial Monitor (115200) ต่อเนื่อง (ไม่ออกจากฟังก์ชัน)
 * ================================================================================================== */

void SerialPrintValues(const int *values, int count) {
  for (int i = 0; i < count; i++) {
    Serial.print(values[i]);
    Serial.print("\t");
  }
}

// ---------- ค่าดิบ ----------
void Serial_FrontSensor() {
  while (1) {
    ReadSensorRaw();
    SerialPrintValues(F, NUM_SENSORS);
    Serial.println();
    delay(50);
  }
}

void Serial_BackSensor() {
  while (1) {
    ReadSensorRaw();
    SerialPrintValues(B, NUM_SENSORS);
    Serial.println();
    delay(50);
  }
}

void Serial_CenterSensor() {
  while (1) {
    ReadSensorRaw();
    SerialPrintValues(C, 2);
    Serial.println();
    delay(50);
  }
}

void SerialAllSensor() {
  while (1) {
    ReadSensorRaw();
    Serial.print("F: ");
    SerialPrintValues(F, NUM_SENSORS);
    Serial.print(" | B: ");
    SerialPrintValues(B, NUM_SENSORS);
    Serial.print(" | C: ");
    SerialPrintValues(C, 2);
    Serial.println();
    delay(100);
  }
}

// ---------- ค่าที่คาลิเบรตแล้ว ----------
void SerialCalibrate_FrontSensor() {
  while (1) {
    ReadSensor();
    SerialPrintValues(F, NUM_SENSORS);
    Serial.println();
    delay(100);
  }
}

void SerialCalibrate_BackSensor() {
  while (1) {
    ReadSensor();
    SerialPrintValues(B, NUM_SENSORS);
    Serial.println();
    delay(100);
  }
}

void SerialCalibrate_CenterSensor() {
  while (1) {
    ReadSensor();
    SerialPrintValues(C, 2);
    Serial.println();
    delay(100);
  }
}

void SerialCalibrate_AllSensor() {
  while (1) {
    ReadSensor();
    Serial.print("F: ");
    SerialPrintValues(F, NUM_SENSORS);
    Serial.print(" | B: ");
    SerialPrintValues(B, NUM_SENSORS);
    Serial.print(" | C: ");
    SerialPrintValues(C, 2);
    Serial.println();
    delay(100);
  }
}

#endif
