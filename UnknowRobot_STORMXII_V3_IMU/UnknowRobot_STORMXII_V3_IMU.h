#ifndef UNKNOWROBOT_STORMXII_V3_H
#define UNKNOWROBOT_STORMXII_V3_H

/* ==================================================================================================
 *  UnknowRobot STORM XII V3 — 16CH + IMU (BNO055)
 *
 *   Unknow_Buzzer.h   เสียงบี๊บ
 *   Unknow_Servo.h    เซอร์โว (ที่คีบ)
 *   Unknow_Motor.h    มอเตอร์ / ตารางค่าตามความเร็ว
 *   Unknow_Sensor.h   เซนเซอร์เส้น หน้า / หลัง / กลาง / คาลิเบรต
 *   Unknow_PID.h      วิ่งตามเส้น / เลี้ยว / หมุน ด้วยเซนเซอร์เส้น
 *   Unknow_IMU.h      Gyro (BNO055)
 *
 *  ไฟล์นี้:
 *   1) ปุ่มกด
 *   2) RobotSetup — เริ่มต้นระบบ
 *   3) สวิตช์ (swOK / swSTART)
 *   4) คาลิเบรตเซนเซอร์ (CalibrateSensor)
 * ================================================================================================== */

#include "Unknow_Buzzer.h"
#include "Unknow_Servo.h"
#include "Unknow_Motor.h"
#include "Unknow_Sensor.h"
#include "Unknow_PID.h"
#include "Unknow_IMU.h"


/* ==================================================================================================
 *  1) ปุ่มกด
 * ================================================================================================== */

#define OK_PIN 19
#define START_PIN 20
#define PRESS 0

int OK_PUSH() {
  if (digitalRead(OK_PIN) == PRESS) return PRESS;
  return 1;
}

int START_PUSH() {
  if (digitalRead(START_PIN) == PRESS) return PRESS;
  return 1;
}


/* ==================================================================================================
 *  2) RobotSetup — เริ่มต้นระบบ (เรียกเป็นบรรทัดแรกของ setup())
 * ================================================================================================== */

void RobotSetup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogWriteFreq(20000);
  delay(1000);
  Beep(500);

  // ---------- มอเตอร์ ----------
  pinMode(PH1, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(PH2, OUTPUT);
  pinMode(EN2, OUTPUT);

  // ---------- ปุ่มกด ----------
  pinMode(OK_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);

  // ---------- Mux เซนเซอร์ ----------
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S3B, OUTPUT);

  // ---------- Gyro: ลองหา BNO055 ซ้ำได้ไม่เกิน 5 วินาที ถ้าไม่เจอให้บี๊บเตือน 4 ครั้งแล้วทำงานต่อโดยไม่มี gyro ----------
  bool imuFound = false;
  unsigned long imuStart = millis();
  while (millis() - imuStart < 5000) {
    if (bno.begin()) {
      imuFound = true;
      break;
    }
    delay(100);
  }
  if (!imuFound) {
    Serial.println("BNO055 not detected. Check your wiring or I2C ADDR!");
    for (int i = 0; i < 4; i++) {
      Beep(500);
      delay(200);
    }
  } else {
    Serial.println("BNO055 found!");
    delay(1000);
    bno.setExtCrystalUse(true);
  }

  delay(500);
  Beep2(100, 100);
  delay(50);
  Beep2(200, 100);
}


/* ==================================================================================================
 *  3) สวิตช์
 * ================================================================================================== */

void CalibrateSensor();

// พิมพ์ค่าเซนเซอร์ดิบ F / B / C และมุม gyro 1 บรรทัด
void SerialShowAllSensorOnce() {
  ReadSensorRaw();

  Serial.print("F:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F[i]);
    Serial.print(" ");
  }

  Serial.print("| B:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(B[i]);
    Serial.print(" ");
  }

  Serial.print("| C:");
  for (int i = 0; i < 2; i++) {
    Serial.print(C[i]);
    Serial.print(" ");
  }

  Serial.print("| gyro :");
  Serial.print(angleRead());
  Serial.println();
}

// รอจนกดปุ่ม OK — ระหว่างรอพิมพ์ค่าเซนเซอร์ออก Serial และกด START เพื่อคาลิเบรตเซนเซอร์
void swOK() {
  MotorStop();
  while (1) {
    SerialShowAllSensorOnce();

    if (digitalRead(OK_PIN) == PRESS) break;

    if (digitalRead(START_PIN) == PRESS) {
      delay(30);  // debounce
      if (digitalRead(START_PIN) == PRESS) {
        Beep(200);
        Beep(50);
        Beep(200);

        CalibrateSensor();

        while (digitalRead(START_PIN) == PRESS)  // รอปล่อยปุ่ม
          ;
      }
    }

    delay(50);
  }
  delay(200);
  Beep(200);
}

// รอจนกดปุ่ม START
void swSTART() {
  while (digitalRead(START_PIN) != PRESS)
    ;
  Beep(200);
}


/* ==================================================================================================
 *  4) คาลิเบรตเซนเซอร์ — ถูหุ่นผ่านเส้นขาว-ดำทีละชุด หน้า / หลัง / กลาง แล้วเซฟลง EEPROM
 * ================================================================================================== */

// พิมพ์ค่าคาลิเบรตเป็นคำสั่ง เช่น "LightValue_FrontSensor (min..., max...);" ให้คัดลอกไปวางในโค้ดได้
void PrintLightValue(const char *name, int *minValue, int *maxValue, int count) {
  Serial.print(name);
  Serial.print(" (");
  for (int i = 0; i < count; i++) {
    Serial.print(minValue[i]);
    Serial.print(",");
  }
  for (int i = 0; i < count; i++) {
    Serial.print(maxValue[i]);
    if (i < count - 1) Serial.print(",");
    else Serial.println(");");
  }
}

void CalibrateSensor() {
  Serial.println("Press Cali to start caribrate Front Sensor");
  swSTART();
  Serial.println("Caribrating");
  Beep(100);
  delay(600);
  caribrateSensorF(20, 200);
  Beep(100);
  delay(600);
  Serial.println("Finish\n");

  Serial.println("Press Cali to start caribrate Back Sensor");
  swSTART();
  Serial.println("Caribrating");
  Beep(100);
  delay(600);
  caribrateSensorB(20, 200);
  Beep(100);
  delay(600);
  Serial.println("Finish\n");

  Serial.println("Press Cali to start Caribrate Center Sensor");
  swSTART();
  Serial.println("Caribrating");
  Beep(100);
  delay(600);
  caribrateSensorC(20, 200);
  Beep(100);
  delay(600);
  Serial.println("Finish\n");

  PrintLightValue("LightValue_FrontSensor", minValueF, maxValueF, NUM_SENSORS);
  PrintLightValue("LightValue_BackSensor", minValueB, maxValueB, NUM_SENSORS);
  PrintLightValue("LightValue_CenterSensor", minValueC, maxValueC, 2);

  SaveCalibrateToEEPROM();
  Serial.println("Saved calibration to EEPROM");

  while (digitalRead(START_PIN) == !PRESS)
    ;
}

#endif
