/* =====================================================================
 *  UnknowRobot STORM XII V3 (16CH + IMU BNO055) — ไฟล์หลักของไลบรารี
 *
 *  สารบัญ
 *     1) ขาปุ่มกด
 *     2) ประกาศล่วงหน้า (Prototype)
 *     3) เริ่มต้นหุ่นยนต์ (RobotSetup)
 *     4) แสดงค่าเซนเซอร์
 *     5) ปุ่มกด OK / START
 *     6) คาลิเบรตเซนเซอร์
 * ===================================================================== */

#ifndef UNKNOWROBOT_STORMXII_V3_H
#define UNKNOWROBOT_STORMXII_V3_H

#include "Unknow_Buzzer.h"
#include "Unknow_Servo.h"
#include "Unknow_Motor.h"
#include "Unknow_Sensor.h"
#include "Unknow_PID.h"
#include "Unknow_IMU.h"


// =====================================================================
//  1) ขาปุ่มกด
// =====================================================================

#define OK_PIN 19
#define START_PIN 20
#define PRESS 0


// =====================================================================
//  2) ประกาศล่วงหน้า (Prototype)
// =====================================================================

void CalibrateSensor();


// =====================================================================
//  3) เริ่มต้นหุ่นยนต์ (RobotSetup)
// =====================================================================

void RobotSetup() {
  Serial.begin(115200);
  analogWriteResolution(12);
  analogWriteFreq(20000);
  delay(1000);
  Beep(500);

  pinMode(PH1, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(PH2, OUTPUT);
  pinMode(EN2, OUTPUT);

  pinMode(OK_PIN, INPUT_PULLUP);
  pinMode(START_PIN, INPUT_PULLUP);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S3B, OUTPUT);

  // ลองหา BNO055 ซ้ำได้ไม่เกิน 10 วินาที ถ้าไม่เจอให้บี๊บเตือน 4 ครั้งแล้วทำงานต่อโดยไม่มี gyro
  bool imuFound = false;
  unsigned long imuStart = millis();
  while (millis() - imuStart < 10000) {
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


// =====================================================================
//  4) แสดงค่าเซนเซอร์
// =====================================================================

void SerialShowAllSensorOnce() {
  ReadSensorRaw();

  // Front
  Serial.print("F:");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(F[i]);
    Serial.print(" ");
  }

  Serial.print("| B:");
  // Back
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(B[i]);
    Serial.print(" ");
  }

  Serial.print("| C:");
  // Center
  for (int i = 0; i < 2; i++) {
    Serial.print(C[i]);
    Serial.print(" ");
  }

  Serial.print("| gyro :");
  Serial.print(angleRead());
  Serial.print(" ");

  Serial.println();  // ขึ้นบรรทัดใหม่ครั้งเดียว
}


// =====================================================================
//  5) ปุ่มกด OK / START
// =====================================================================

int OK_PUSH() {
  if (digitalRead(OK_PIN) == PRESS) return PRESS;
  return 1;
}

int START_PUSH() {
  if (digitalRead(START_PIN) == PRESS) return PRESS;
  return 1;
}

void swSTART() {
  while (1) {
    if (digitalRead(START_PIN) == PRESS) {
      break;
    }
  }
  Beep(200);
}

void swOK() {
  MotorStop();
  while (1) {
    // แสดงค่า Sensor แนวนอน
    SerialShowAllSensorOnce();

    // ออกจากโหมด
    if (digitalRead(OK_PIN) == PRESS) {
      break;
    }

    // กด START เพื่อ Calibrate
    if (digitalRead(START_PIN) == PRESS) {
      delay(30);  // debounce
      if (digitalRead(START_PIN) == PRESS) {
        Beep(200);
        Beep(50);
        Beep(200);

        CalibrateSensor();

        // รอปล่อยปุ่ม (กันกดค้าง)
        while (digitalRead(START_PIN) == PRESS)
          ;
      }
    }

    delay(50);  // ลดความถี่ loop / Serial
  }
  delay(200);
  Beep(200);
}


// =====================================================================
//  6) คาลิเบรตเซนเซอร์
// =====================================================================

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

  Serial.print("LightValue_FrontSensor (");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(minValueF[i]);
    Serial.print(",");
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(maxValueF[i]);
    if (i < NUM_SENSORS - 1)
      Serial.print(",");
    else
      Serial.println(");");
  }

  Serial.print("LightValue_BackSensor (");
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(minValueB[i]);
    Serial.print(",");
  }
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(maxValueB[i]);
    if (i < NUM_SENSORS - 1)
      Serial.print(",");
    else
      Serial.println(");");
  }

  Serial.print("LightValue_CenterSensor (");
  for (int i = 0; i < 2; i++) {
    Serial.print(minValueC[i]);
    Serial.print(",");
  }
  for (int i = 0; i < 2; i++) {
    Serial.print(maxValueC[i]);
    if (i < 1)
      Serial.print(",");
    else
      Serial.println(");");
  }

  SaveCalibrateToEEPROM();
  Serial.println("Saved calibration to EEPROM");

  while (digitalRead(START_PIN) == !PRESS)
    ;
}

#endif
