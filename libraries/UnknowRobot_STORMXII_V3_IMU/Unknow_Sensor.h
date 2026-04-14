#ifndef UNKNOW_SENSOR_H
#define UNKNOW_SENSOR_H

#include <Arduino.h>

#define NUM_SENSORS 16
#define S0  14
#define S1  16
#define S2  15
#define S3  13
#define S3B 17
#define SIG  A0
#define SIGB A1

#define BLACK 0
#define WHITE 1
#define CCL 0
#define CCR 1

uint8_t F_PIN[NUM_SENSORS] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 };
uint8_t B_PIN[NUM_SENSORS] = { 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0 };

int F[NUM_SENSORS], B[NUM_SENSORS], C[2];
int F_Ref[NUM_SENSORS], B_Ref[NUM_SENSORS], C_Ref[2];
int minValueF[NUM_SENSORS], maxValueF[NUM_SENSORS];
int minValueB[NUM_SENSORS], maxValueB[NUM_SENSORS];
int minValueC[2], maxValueC[2];

int Ref        = 0;
int RefC       = 0;
int LineColor  = 0;
int TrackLineCH = 0;

// ---------- Mux ----------

inline void muxSelectF(uint8_t ch) {
  digitalWrite(S0,  (ch >> 0) & 1);
  digitalWrite(S1,  (ch >> 1) & 1);
  digitalWrite(S2,  (ch >> 2) & 1);
  digitalWrite(S3,  (ch >> 3) & 1);
}

int muxReadF(uint8_t ch) {
  muxSelectF(ch);
  delayMicroseconds(3);
  return analogRead(SIG);
}

inline void muxSelectB(uint8_t ch) {
  digitalWrite(S0,  (ch >> 0) & 1);
  digitalWrite(S1,  (ch >> 1) & 1);
  digitalWrite(S2,  (ch >> 2) & 1);
  digitalWrite(S3B, (ch >> 3) & 1);
}

int muxReadB(uint8_t ch) {
  muxSelectB(ch);
  delayMicroseconds(3);
  return analogRead(SIGB);
}

uint16_t read_sensorA(int sensor) { return muxReadF(sensor); }
uint16_t read_sensorB(int sensor) { return muxReadB(sensor); }

// ---------- Raw Read ----------

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

// ---------- Calibration Values ----------

void LightValue_FrontSensor(
  uint16_t minF0,  uint16_t minF1,  uint16_t minF2,  uint16_t minF3,
  uint16_t minF4,  uint16_t minF5,  uint16_t minF6,  uint16_t minF7,
  uint16_t minF8,  uint16_t minF9,  uint16_t minF10, uint16_t minF11,
  uint16_t minF12, uint16_t minF13, uint16_t minF14, uint16_t minF15,
  uint16_t maxF0,  uint16_t maxF1,  uint16_t maxF2,  uint16_t maxF3,
  uint16_t maxF4,  uint16_t maxF5,  uint16_t maxF6,  uint16_t maxF7,
  uint16_t maxF8,  uint16_t maxF9,  uint16_t maxF10, uint16_t maxF11,
  uint16_t maxF12, uint16_t maxF13, uint16_t maxF14, uint16_t maxF15) {
  minValueF[0]=minF0;   minValueF[1]=minF1;   minValueF[2]=minF2;   minValueF[3]=minF3;
  minValueF[4]=minF4;   minValueF[5]=minF5;   minValueF[6]=minF6;   minValueF[7]=minF7;
  minValueF[8]=minF8;   minValueF[9]=minF9;   minValueF[10]=minF10; minValueF[11]=minF11;
  minValueF[12]=minF12; minValueF[13]=minF13; minValueF[14]=minF14; minValueF[15]=minF15;
  maxValueF[0]=maxF0;   maxValueF[1]=maxF1;   maxValueF[2]=maxF2;   maxValueF[3]=maxF3;
  maxValueF[4]=maxF4;   maxValueF[5]=maxF5;   maxValueF[6]=maxF6;   maxValueF[7]=maxF7;
  maxValueF[8]=maxF8;   maxValueF[9]=maxF9;   maxValueF[10]=maxF10; maxValueF[11]=maxF11;
  maxValueF[12]=maxF12; maxValueF[13]=maxF13; maxValueF[14]=maxF14; maxValueF[15]=maxF15;
}

void LightValue_CenterSensor(uint16_t minC0, uint16_t minC1, uint16_t maxC0, uint16_t maxC1) {
  minValueC[0] = minC0; minValueC[1] = minC1;
  maxValueC[0] = maxC0; maxValueC[1] = maxC1;
}

void LightValue_BackSensor(
  uint16_t minB0,  uint16_t minB1,  uint16_t minB2,  uint16_t minB3,
  uint16_t minB4,  uint16_t minB5,  uint16_t minB6,  uint16_t minB7,
  uint16_t minB8,  uint16_t minB9,  uint16_t minB10, uint16_t minB11,
  uint16_t minB12, uint16_t minB13, uint16_t minB14, uint16_t minB15,
  uint16_t maxB0,  uint16_t maxB1,  uint16_t maxB2,  uint16_t maxB3,
  uint16_t maxB4,  uint16_t maxB5,  uint16_t maxB6,  uint16_t maxB7,
  uint16_t maxB8,  uint16_t maxB9,  uint16_t maxB10, uint16_t maxB11,
  uint16_t maxB12, uint16_t maxB13, uint16_t maxB14, uint16_t maxB15) {
  minValueB[0]=minB0;   minValueB[1]=minB1;   minValueB[2]=minB2;   minValueB[3]=minB3;
  minValueB[4]=minB4;   minValueB[5]=minB5;   minValueB[6]=minB6;   minValueB[7]=minB7;
  minValueB[8]=minB8;   minValueB[9]=minB9;   minValueB[10]=minB10; minValueB[11]=minB11;
  minValueB[12]=minB12; minValueB[13]=minB13; minValueB[14]=minB14; minValueB[15]=minB15;
  maxValueB[0]=maxB0;   maxValueB[1]=maxB1;   maxValueB[2]=maxB2;   maxValueB[3]=maxB3;
  maxValueB[4]=maxB4;   maxValueB[5]=maxB5;   maxValueB[6]=maxB6;   maxValueB[7]=maxB7;
  maxValueB[8]=maxB8;   maxValueB[9]=maxB9;   maxValueB[10]=maxB10; maxValueB[11]=maxB11;
  maxValueB[12]=maxB12; maxValueB[13]=maxB13; maxValueB[14]=maxB14; maxValueB[15]=maxB15;
}

// ---------- Config Setters ----------

void TrackLineColor(int Col)       { LineColor  = Col; }
void SwitchLine()                  { LineColor  = !LineColor; }
void RefLineValue(int x)           { Ref        = x; }
void RefCenterLineValue(int x)     { RefC       = x; }
void SetSensorTrackLine_CH(int x)  { TrackLineCH = x; }

// ---------- Calibrated Read ----------

void ReadCalibrateF() {
  ReadF();
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x;
    if (LineColor == BLACK)
      x = map(F[i], minValueF[i], maxValueF[i], 1000, 0);
    else
      x = map(F[i], minValueF[i], maxValueF[i], 0, 1000);
    if (x < 40)   x = 0;
    if (x > 980)  x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    F[i] = x;
  }
}

void ReadCalibrateC() {
  ReadC();
  for (int i = 0; i < 2; i++) {
    int x;
    if (LineColor == BLACK)
      x = map(C[i], minValueC[i], maxValueC[i], 0, 1000);
    else
      x = map(C[i], minValueC[i], maxValueC[i], 1000, 0);
    if (x < 40)   x = 0;
    if (x > 980)  x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    C[i] = x;
  }
}

void ReadCalibrateB() {
  ReadB();
  for (int i = 0; i < NUM_SENSORS; i++) {
    int x;
    if (LineColor == BLACK)
      x = map(B[i], minValueB[i], maxValueB[i], 1000, 0);
    else
      x = map(B[i], minValueB[i], maxValueB[i], 0, 1000);
    if (x < 40)   x = 0;
    if (x > 980)  x = 1000;
    // if (x < 0)    x = 0;
    // if (x > 1000) x = 1000;
    B[i] = x;
  }
}

void ReadSensor() {
  ReadCalibrateF();
  ReadCalibrateB();
  ReadCalibrateC();
}

void ReadSensorRaw() {
  ReadF();
  ReadB();
  ReadC();
}

// ---------- Auto Calibrate (FIX: init minValue=4095 for 12-bit ADC) ----------

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

// ---------- Serial Debug ----------

void Serial_FrontSensor() {
  while (1) {
    ReadSensorRaw();
    for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(F[i]); Serial.print("\t"); }
    Serial.println();
    delay(50);
  }
}

void Serial_BackSensor() {
  while (1) {
    ReadSensorRaw();
    for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(B[i]); Serial.print("\t"); }
    Serial.println();
    delay(50);
  }
}

void Serial_CenterSensor() {
  while (1) {
    ReadSensorRaw();
    for (int i = 0; i < 2; i++) { Serial.print(C[i]); Serial.print("\t"); }
    Serial.println();
    delay(50);
  }
}

void SerialCalibrate_FrontSensor() {
  while (1) {
    ReadSensor();
    for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(F[i]); Serial.print("\t"); }
    Serial.println();
    delay(100);
  }
}

void SerialCalibrate_BackSensor() {
  while (1) {
    ReadSensor();
    for (int i = 0; i < NUM_SENSORS; i++) { Serial.print(B[i]); Serial.print("\t"); }
    Serial.println();
    delay(100);
  }
}

void SerialCalibrate_CenterSensor() {
  while (1) {
    ReadSensor();
    for (int i = 0; i < 2; i++) { Serial.print(C[i]); Serial.print("\t"); }
    Serial.println();
    delay(100);
  }
}


void SerialCalibrate_AllSensor() {
  while (1) {
    ReadSensor();

    // ---------- FRONT ----------
    Serial.print("F: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(F[i]);
      Serial.print("\t");
    }

    // ---------- BACK ----------
    Serial.print(" | B: ");
    for (int i = 0; i < NUM_SENSORS; i++) {
      Serial.print(B[i]);
      Serial.print("\t");
    }

    // ---------- CENTER ----------
    Serial.print(" | C: ");
    for (int i = 0; i < 2; i++) {
      Serial.print(C[i]);
      Serial.print("\t");
    }

    Serial.println();
    delay(100);
  }
}



#endif
