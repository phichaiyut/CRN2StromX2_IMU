#ifndef UNKNOW_IMU_H
#define UNKNOW_IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Unknow_Buzzer.h"
#include "Unknow_Motor.h"
#include "Unknow_Sensor.h"
#include "Unknow_PID.h"

/* ==================================================================================================
 *  Unknow_IMU.h — Gyro (BNO055)
 *
 *   1) ฮาร์ดแวร์ / ตัวแปรสถานะ
 *   2) อ่านมุม / รีเซ็ตมุม
 *   3) ค่าปรับจูน Gyro (เรียกจาก Setting.ino)
 *   4) ค้างมุมอยู่กับที่ (Hold)
 *   5) หมุน / เลี้ยว ตามองศา
 *   6) ตีโค้ง S และตีโค้ง arc
 *   7) วิ่งตรงด้วย Gyro
 *   8) เข้ากลางหุ่น / เข้าเส้น ด้วย Gyro
 *   9) คำสั่งเจอแยก (select) และคำสั่งที่จบด้วย select
 *  10) ทิศสัมบูรณ์ (direction)
 * ================================================================================================== */


/* ==================================================================================================
 *  1) ฮาร์ดแวร์ / ตัวแปรสถานะ
 * ================================================================================================== */

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

float angles_offset[] = { 0, 0, 0 };  // [2] = มุม yaw ดิบตอน reset (ใช้เป็นมุม 0)
float current_degree = 0;             // มุมอ้างอิงที่ RunG / RunGB / SetG พยายามรักษา
int previous_errorG;
int previous_errorGB;
bool useDirectionG = false;  // true = คำสั่ง cm ใช้ทิศที่ตั้งจาก SetDirectionG แทนการจำมุมปัจจุบัน

// บีบมุมให้อยู่ในช่วง -180 ถึง 180
float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}


/* ==================================================================================================
 *  2) อ่านมุม / รีเซ็ตมุม
 * ================================================================================================== */

// มุมปัจจุบันเทียบกับตอน reset (0 ถึง 360)
float angleRead() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float adjusted = euler.x() - angles_offset[2];
  if (adjusted >= 360) adjusted -= 360;
  else if (adjusted < 0) adjusted += 360;
  return adjusted;
}



// รีเซ็ตมุมปัจจุบันของหุ่นยนต์ให้เป็น 0 องศา และใช้เป็นมุมอ้างอิง + ล้างค่า error ของ RunG / RunGB
void setAngleOffset() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angles_offset[2] = euler.x();
  current_degree = angleRead();
  previous_errorG = 0;
  previous_errorGB = 0;
}

void resetAngle() {
  setAngleOffset();
}

void resetAngles() {
  setAngleOffset();
}

// จำมุมปัจจุบันเป็นมุมอ้างอิง (ไม่ reset เป็น 0)
void SetRobotAngle() {
  current_degree = angleRead();
}


/* ==================================================================================================
 *  3) ค่าปรับจูน Gyro (เรียกจาก Setting.ino)
 * ================================================================================================== */

// ---------- หมุน / เลี้ยว (SetGyroSpin / SetGyroTurn) ----------
float gyro_Kp_Spin = 0.75f;
float gyro_Kd_Spin = 0.9f;
int gyro_MaxSpd_Spin = 40;
int gyro_MinSpd_Spin = 8;
float gyro_SmallAngle_Spin = 8.0f;
float gyro_StopThr_Spin = 1.0f;

float gyro_Kp_Turn = 0.75f;
float gyro_Kd_Turn = 0.9f;
int gyro_MaxSpd_Turn = 50;
int gyro_MinSpd_Turn = 10;
float gyro_SmallAngle_Turn = 10.0f;
float gyro_StopThr_Turn = 1.0f;

float gyro_StopThr_Rotate = 1.0f;

void SetGyroTurn(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Turn = kp;
  gyro_Kd_Turn = kd;
  gyro_MaxSpd_Turn = maxSpd;
  gyro_MinSpd_Turn = minSpd;
  gyro_SmallAngle_Turn = smallAngle;
  gyro_StopThr_Turn = stopThr;
}

void SetGyroSpin(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Spin = kp;
  gyro_Kd_Spin = kd;
  gyro_MaxSpd_Spin = maxSpd;
  gyro_MinSpd_Spin = minSpd;
  gyro_SmallAngle_Spin = smallAngle;
  gyro_StopThr_Spin = stopThr;
}

// ---------- วิ่งตรง (SetGyroRun / SetGyroRunB) ----------
float kpG = 1.2;
float kdG = 1.5;
float kpGB = 1.2;
float kdGB = 1.5;

void SetGyroRun(float kp, float kd) {
  kpG = kp;
  kdG = kd;
}

void SetGyroRunB(float kp, float kd) {
  kpGB = kp;
  kdGB = kd;
}

// ---------- โหมดจำกัดกำลังมอเตอร์ของ RunG (เดินหน้า) / RunGB (ถอยหลัง) ----------
// ค่าเริ่มต้น = โหมด 4, min -5 ปรับได้ด้วย ModeSpdGyro() ใน Setting.ino
int MaxSpeedG = 100;
int MinSpeedG = -5;
int ModeGyroStatus = 4;
int ModeGyroBStatus = 4;

// ตั้งโหมดเดียวกันทั้ง RunG และ RunGB
void ModeSpdGyro(int moD, int maX, int miN) {
  ModeGyroStatus = moD;
  ModeGyroBStatus = moD;
  MaxSpeedG = maX;
  MinSpeedG = miN;
}

// ตั้งโหมดแยกเดินหน้า (RunG) / ถอยหลัง (RunGB)
void ModeSpdGyro(int moDF, int moDB, int maX, int miN) {
  ModeGyroStatus = moDF;
  ModeGyroBStatus = moDB;
  MaxSpeedG = maX;
  MinSpeedG = miN;
}

// จำกัดค่า LeftPower / RightPower ของ gyro ตามโหมด
void ClampGyroPower(float &LeftPower, float &RightPower, int SpeedL, int SpeedR, int mode) {
  switch (mode) {
    case 0:  // ..max, ล้อติดลบ → min
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = MinSpeedG;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = MinSpeedG;
      break;
    case 1:  // min..max
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < MinSpeedG) LeftPower = MinSpeedG;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < MinSpeedG) RightPower = MinSpeedG;
      break;
    case 2:  // -Speed..Speed
      if (LeftPower > SpeedL) LeftPower = SpeedL;
      if (LeftPower < -SpeedL) LeftPower = -SpeedL;
      if (RightPower > SpeedR) RightPower = SpeedR;
      if (RightPower < -SpeedR) RightPower = -SpeedR;
      break;
    case 3:  // ..max, ล้อติดลบ → -BaseSpeed
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = -BaseSpeed;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = -BaseSpeed;
      break;
    case 4:  // ..Speed, ล้อติดลบ → min (min = 0 คือไม่ถอยล้อ)
      if (LeftPower > SpeedL) LeftPower = SpeedL;
      if (LeftPower < 0) LeftPower = MinSpeedG;
      if (RightPower > SpeedR) RightPower = SpeedR;
      if (RightPower < 0) RightPower = MinSpeedG;
      break;
    default:
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = 0;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = 0;
  }
}

// ---------- ค้างมุมอยู่กับที่ (HoldAngle / HoldAngleB / HoldAngleF) ----------
float kpHold = 2.5;
float kdHold = 1.5;
float kpFHold = 2.5;
float kdFHold = 1.5;
float kpBHold = 2.5;
float kdBHold = 1.2;


/* ==================================================================================================
 *  4) ค้างมุมอยู่กับที่ (Hold)
 * ================================================================================================== */

float holdAngle = 0;
float prevErrHold = 0;
float prevErrHoldB = 0;
float prevErrHoldF = 0;

// หมุนอยู่กับที่ให้กลับมาที่ current_degree
void HoldAngle() {
  float error = wrap180(current_degree - angleRead());
  float d = error - prevErrHold;
  int power = (error * kpHold) + (d * kdHold);
  power = constrain(power, -50, 50);
  Motor(power, -power);
  prevErrHold = error;
}

// เหมือน HoldAngle แต่กลับทิศแรงหมุน (ใช้ตอนถอยหลัง)
void HoldAngleB() {
  float error = wrap180(current_degree - angleRead());
  float d = error - prevErrHoldB;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);
  Motor(-power, power);
  prevErrHoldB = error;
}

// จำมุมจริงตอนนี้เป็น holdAngle
void SetHoldAngle() {
  holdAngle = angleRead();
  MotorStop();
  prevErrHoldF = 0;
}

// หมุนอยู่กับที่ให้กลับมาที่ holdAngle
void HoldAngleF() {
  float error = wrap180(holdAngle - angleRead());
  float d = error - prevErrHoldF;
  int power = (error * kpFHold) + (d * kdFHold);
  power = constrain(power, -50, 50);
  Motor(power, -power);
  prevErrHoldF = error;
}

// ค้างที่มุมอ้างอิง (current_degree) นาน totalTime ms
void SetG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) HoldAngle();
  BZoff();
}

// เหมือน SetG แต่กลับทิศแรงหมุน
void SetBG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) HoldAngleB();
  BZoff();
}

// จำมุมจริงตอนนี้ แล้วค้างไว้ totalTime ms
void SetFG(int totalTime) {
  BZon();
  SetHoldAngle();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) HoldAngleF();
  BZoff();
}

void setfg(int totalTime) {
  SetFG(totalTime);
}


/* ==================================================================================================
 *  5) หมุน / เลี้ยว ตามองศา   (deg: บวก = ขวา, ลบ = ซ้าย)
 * ================================================================================================== */

// ---------- หมุนอยู่กับที่ (สองล้อสวนทาง) ----------
void spinDegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Spin;
  int max_speed = Speed;
  float kp = gyro_Kp_Spin;
  float kd = gyro_Kd_Spin;
  float small_angle_threshold = gyro_SmallAngle_Spin;
  float stop_threshold = gyro_StopThr_Spin;
  float previous_error = 0;
  float target_degree = wrap180(angleRead() + relative_degree);
  current_degree = target_degree;
  while (1) {
    float error = wrap180(target_degree - angleRead());
    int pd_value = (kp * error) + (kd * (error - previous_error));
    pd_value = constrain(pd_value, -max_speed, max_speed);
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, -min_speed);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, min_speed);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      Motor(pd_value, -pd_value);
    }
    delayMicroseconds(80);  // ลดการสั่น
    previous_error = error;
  }
  Stop(100);
}

void spinDegree(int relative_degree) {
  spinDegree(gyro_MaxSpd_Spin, relative_degree);
}

// ---------- เลี้ยวหน้าล้อเดียว แล้วเบรก ----------
void turnDegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Turn;
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;
  float kd = gyro_Kd_Turn;
  float small_angle_threshold = gyro_SmallAngle_Turn;
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = wrap180(angleRead() + relative_degree);
  current_degree = target_degree;
  while (1) {
    float error = wrap180(target_degree - angleRead());
    int pd_value = (kp * error) + (kd * (error - previous_error));
    pd_value = constrain(pd_value, -max_speed, max_speed);
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, 0);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(0, min_speed);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else if (error <= 0) {
      Motor(-1, -pd_value);
    } else {
      Motor(pd_value, 1);
    }
    previous_error = error;
  }
  SetG(10);
}

void turnDegree(int relative_degree) {
  turnDegree(gyro_MaxSpd_Turn, relative_degree);
}

// ---------- เลี้ยวถอยหลังล้อเดียว แล้วเบรก ----------
void turnDegreeB(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Turn;
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;
  float kd = gyro_Kd_Turn;
  float small_angle_threshold = gyro_SmallAngle_Turn;
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = wrap180(angleRead() + relative_degree);
  current_degree = target_degree;
  while (1) {
    float error = wrap180(target_degree - angleRead());
    int pd_value = (kp * error) + (kd * (error - previous_error));
    pd_value = constrain(pd_value, -max_speed, max_speed);
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(0, -min_speed);
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, 0);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else if (error <= 0) {
      Motor(pd_value, 1);
    } else {
      Motor(-1, -pd_value);
    }
    delayMicroseconds(80);  // ลดการสั่น
    previous_error = error;
  }
  SetG(10);
}

void turnDegreeB(int relative_degree) {
  turnDegreeB(gyro_MaxSpd_Turn, relative_degree);
}

// ---------- เลี้ยวล้อเดียว ไม่เบรก (เข้าโค้งต่อเนื่อง) ----------
// นับองศาต่อจากมุมอ้างอิงเดิม (current_degree) จึงต่อกันหลายคำสั่งได้โดยไม่สะสมความคลาด
// ถ้าหมุนเร็ว มุมจะกระโดดข้ามช่วง ±stop_threshold ได้ จึงถือว่าถึงเป้าเมื่อ error เปลี่ยนเครื่องหมาย
// (หมุนเลยเป้าแล้ว) ด้วย ไม่ให้หมุนกลับไปมา — |error| < 90 กันกรณี error กระโดดจาก +180 เป็น -180 ตอน wrap
bool crossedTarget(float error, float previous_error) {
  return fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
}

void turnDegree_none(int Speed, int relative_degree) {
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = wrap180(current_degree + relative_degree);
  current_degree = target_degree;
  while (1) {
    float error = wrap180(target_degree - angleRead());
    bool crossed = crossedTarget(error, previous_error);
    previous_error = error;

    if ((error >= -stop_threshold && error <= stop_threshold) || crossed) {
      break;
    } else if (error > 0) {
      Motor(Speed, -2);
    } else {
      Motor(-2, Speed);
    }
    delayMicroseconds(80);  // ลดการสั่น
  }
}

void turnDegree_none(int relative_degree) {
  turnDegree_none(gyro_MaxSpd_Turn, relative_degree);
}

void turnDegreeb_none(int Speed, int relative_degree) {
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = wrap180(current_degree + relative_degree);
  current_degree = target_degree;
  while (1) {
    float error = wrap180(target_degree - angleRead());
    bool crossed = crossedTarget(error, previous_error);
    previous_error = error;

    if ((error >= -stop_threshold && error <= stop_threshold) || crossed) {
      break;
    } else if (error > 0) {
      Motor(1, -Speed);
    } else {
      Motor(-Speed, 1);
    }
    delayMicroseconds(80);  // ลดการสั่น
  }
}

void turnDegreeb_none(int relative_degree) {
  turnDegreeb_none(gyro_MaxSpd_Turn, relative_degree);
}

// ---------- ทิศอยู่ในชื่อคำสั่งแล้ว (ใส่องศาเป็นค่าบวก) ----------
void SpinLG(int Angle) {
  spinDegree(-abs(Angle));
}
void SpinLG(int spd, int Angle) {
  spinDegree(spd, -abs(Angle));
}

void SpinRG(int Angle) {
  spinDegree(abs(Angle));
}
void SpinRG(int spd, int Angle) {
  spinDegree(spd, abs(Angle));
}

void TurnLG(int Angle) {
  turnDegree(-abs(Angle));
}
void TurnLG(int spd, int Angle) {
  turnDegree(spd, -abs(Angle));
}

void TurnRG(int Angle) {
  turnDegree(abs(Angle));
}
void TurnRG(int spd, int Angle) {
  turnDegree(spd, abs(Angle));
}

void TurnLBG(int Angle) {
  turnDegreeB(abs(Angle));
}
void TurnLBG(int spd, int Angle) {
  turnDegreeB(spd, abs(Angle));
}

void TurnRBG(int Angle) {
  turnDegreeB(-abs(Angle));
}
void TurnRBG(int spd, int Angle) {
  turnDegreeB(spd, -abs(Angle));
}


/* ==================================================================================================
 *  6) ตีโค้ง S และตีโค้ง arc
 * ================================================================================================== */

// ---------- ตีโค้ง S เดินหน้า: tlrg = ซ้ายก่อนแล้วสวนขวา, trlg = ขวาก่อนแล้วสวนซ้าย ----------
void tlrg(int Angle) {
  turnDegree_none(-abs(Angle));
  turnDegree(abs(Angle));
}
void tlrg(int spd, int Angle) {
  turnDegree_none(spd, -abs(Angle));
  turnDegree(spd, abs(Angle));
}
void tlrg(int spd, int Angle, int Angle2) {
  turnDegree_none(spd, -abs(Angle));
  turnDegree(spd, abs(Angle2));
}

void trlg(int Angle) {
  turnDegree_none(abs(Angle));
  turnDegree(-abs(Angle));
}
void trlg(int spd, int Angle) {
  turnDegree_none(spd, abs(Angle));
  turnDegree(spd, -abs(Angle));
}
void trlg(int spd, int Angle, int Angle2) {
  turnDegree_none(spd, abs(Angle));
  turnDegree(spd, -abs(Angle2));
}

// ---------- ตีโค้ง S ถอยหลัง ----------
void tlrbg(int Angle) {
  turnDegreeb_none(abs(Angle));
  turnDegreeB(-abs(Angle));
}
void tlrbg(int spd, int Angle) {
  turnDegreeb_none(spd, abs(Angle));
  turnDegreeB(spd, -abs(Angle));
}
void tlrbg(int spd, int Angle, int Angle2) {
  turnDegreeb_none(spd, abs(Angle));
  turnDegreeB(spd, -abs(Angle2));
}

void trlbg(int Angle) {
  turnDegreeb_none(-abs(Angle));
  turnDegreeB(abs(Angle));
}
void trlbg(int spd, int Angle) {
  turnDegreeb_none(spd, -abs(Angle));
  turnDegreeB(spd, abs(Angle));
}
void trlbg(int spd, int Angle, int Angle2) {
  turnDegreeb_none(spd, -abs(Angle));
  turnDegreeB(spd, abs(Angle2));
}

// ---------- ชื่อเรียกอื่น ----------
void TurnLRG(int Angle) {
  tlrg(Angle);
}
void TurnLRG(int spd, int Angle) {
  tlrg(spd, Angle);
}
void TurnLRG(int spd, int Angle, int Angle2) {
  tlrg(spd, Angle, Angle2);
}

void TurnRLG(int Angle) {
  trlg(Angle);
}
void TurnRLG(int spd, int Angle) {
  trlg(spd, Angle);
}
void TurnRLG(int spd, int Angle, int Angle2) {
  trlg(spd, Angle, Angle2);
}

void TurnLRBG(int Angle) {
  tlrbg(Angle);
}
void TurnLRBG(int spd, int Angle) {
  tlrbg(spd, Angle);
}
void TurnLRBG(int spd, int Angle, int Angle2) {
  tlrbg(spd, Angle, Angle2);
}

void TurnRLBG(int Angle) {
  trlbg(Angle);
}
void TurnRLBG(int spd, int Angle) {
  trlbg(spd, Angle);
}
void TurnRLBG(int spd, int Angle, int Angle2) {
  trlbg(spd, Angle, Angle2);
}

// ---------- ตีโค้ง arc: กำหนดความเร็วล้อซ้าย/ขวาเอง หมุนจนถึงองศาแล้วเบรก ----------
void rotateDegree(int SpeedL, int SpeedR, int relative_degree, float kp, float kd) {
  float stop_threshold = gyro_StopThr_Rotate;
  float previous_error = 0;
  float target_degree = wrap180(angleRead() + relative_degree);
  current_degree = target_degree;
  while (1) {
    float error = wrap180(target_degree - angleRead());
    if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    }
    float derivative = error - previous_error;
    int pd_value = (error * kp) + (derivative * kd);
    int leftPow = constrain(SpeedL + pd_value, -100, 100);
    int rightPow = constrain(SpeedR - pd_value, -100, 100);
    Motor(leftPow, rightPow);
    previous_error = error;
    delayMicroseconds(80);  // ลดการสั่น
  }
  SetG(max(abs(SpeedL), abs(SpeedR)));
}

void rotateDegree(int SpeedL, int SpeedR, int relative_degree) {
  rotateDegree(SpeedL, SpeedR, relative_degree, 0.9, 0.6);
}


/* ==================================================================================================
 *  7) วิ่งตรงด้วย Gyro (รักษามุม current_degree)
 * ================================================================================================== */

// ---------- 1 รอบควบคุม (เรียกในลูป) ----------
void RunG(int SpeedL, int SpeedR) {
  float error = wrap180(current_degree - angleRead());
  float derivative = error - previous_errorG;
  int pd_value = (error * kpG) + (derivative * kdG);
  float leftPow = SpeedL + pd_value;
  float rightPow = SpeedR - pd_value;
  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroStatus);
  Motor(leftPow, rightPow);
  previous_errorG = error;
  delayMicroseconds(80);  // ลดการสั่น
}

void RunGB(int SpeedL, int SpeedR) {
  float error = wrap180(current_degree - angleRead());
  float derivative = error - previous_errorGB;
  int pd_value = (error * kpGB) + (derivative * kdGB);
  float leftPow = SpeedL - pd_value;
  float rightPow = SpeedR + pd_value;
  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroBStatus);
  Motor(-leftPow, -rightPow);
  previous_errorGB = error;
  delayMicroseconds(80);  // ลดการสั่น
}

// ---------- ตามเวลา ----------
void FFtimerG(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) RunG(LeftBaseSpeed, RightBaseSpeed);
}

void BBtimerG(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
}

// ---------- ตามระยะทาง ความเร็วคงที่ ----------
void FFcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.75;  // ค่าที่คำนวณจากการวัดจริง

  if (!useDirectionG) SetRobotAngle();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;
      if (traveled_distance >= distance) break;
    }
  }
}

void BBcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.5;  // ค่าที่คำนวณจากการวัดจริง

  if (!useDirectionG) SetRobotAngle();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;
      if (traveled_distance >= distance) break;
    }
  }
}

// ---------- ตามระยะทาง มีเร่ง/ผ่อนความเร็ว (ramp) เมื่อระยะ >= 30 cm ----------
void FFcmG(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(LeftBaseSpeed), abs(RightBaseSpeed));
  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  // ---- ค่าที่ปรับได้ ----
  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;
  bool enableRamp = (distance_cm >= 30.0);
  float speed_scale = enableRamp ? 0.99 : 1.75;  // ระยะสั้น (< 30 cm) ใช้ค่า scale แยก

  if (!useDirectionG) SetRobotAngle();
  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;
    if (remaining_cm <= 0.7f) break;

    float target_speed = base_speed;
    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);  // เร่งช่วงแรก
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);  // ชะลอช่วงท้าย
      }
    }
    RunG(target_speed, target_speed);
  }
}

void BBcmG(int Speed, float distance_cm) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (distance_cm <= 0) {
    Motor(0, 0);
    return;
  }

  int base_speed = min(abs(BackLeftBaseSpeed), abs(BackRightBaseSpeed));
  float traveled_distance = 0.0;
  unsigned long last_time = millis();

  // ---- ค่าที่ปรับได้ ----
  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;
  bool enableRamp = (distance_cm >= 30.0);
  float speed_scale = enableRamp ? 0.99 : 1.5;  // ระยะสั้น (< 30 cm) ใช้ค่า scale แยก

  if (!useDirectionG) SetRobotAngle();
  while (true) {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;
    if (remaining_cm <= 0.8f) break;

    float target_speed = base_speed;
    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);  // เร่งช่วงแรก
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);  // ชะลอช่วงท้าย
      }
    }
    RunGB(target_speed, target_speed);
  }
}


/* ==================================================================================================
 *  8) เข้ากลางหุ่น / เข้าเส้น ด้วย Gyro
 * ================================================================================================== */

// เดินหน้าตรงจนเซนเซอร์กลางซ้าย "หรือ" ขวา เจอเส้น
void ToCenterG() {
  RunG(tctL, tctR);
  delay(20);
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(tct_delay_break);
      MotorStop();
      break;
    }
  }
}

// เดินหน้าตรงจนเซนเซอร์กลางซ้าย เจอเส้น
void ToCenterLG() {
  BZon();
  RunG(tctL, tctR);
  delay(20);
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

// เดินหน้าตรงจนเซนเซอร์กลางขวา เจอเส้น
void ToCenterRG() {
  BZon();
  RunG(tctL, tctR);
  delay(20);
  while (1) {
    RunG(tctL, tctR);
    ReadCalibrateC();
    if (C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

// เดินหน้าตรงจนเซนเซอร์หน้า F[3] หรือ F[12] เจอเส้น
void ToFrontG() {
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[3] > Ref || F[12] > Ref) break;
  }
}

// ถอยตรงจนเซนเซอร์กลางซ้าย "หรือ" ขวา เจอเส้น
void BackToCenterG() {
  RunGB(bctL, bctR);
  delay(20);
  while (1) {
    RunGB(bctL, bctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(bct_delay_break);
      MotorStop();
      break;
    }
  }
}

// ถอยตรงจนเซนเซอร์หลัง B[3] หรือ B[12] เจอเส้น
void BackToFrontG() {
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[3] > Ref || B[12] > Ref) break;
  }
}


/* ==================================================================================================
 *  9) คำสั่งเจอแยก (select) และคำสั่งที่จบด้วย select
 * ================================================================================================== */

// ---------- วิ่งตรงด้วย Gyro ไปข้างหน้า ----------
void TrackSelectG(int spd, char select) {
  switch (select) {
    case 'L':  // หมุนซ้ายทันที
      spinDegree(-90);
      break;
    case 'l':  // เข้ากลางหุ่น แล้วหมุนซ้าย
      ToCenterG();
      spinDegree(-90);
      break;
    case 'R':  // หมุนขวาทันที
      spinDegree(90);
      break;
    case 'r':  // เข้ากลางหุ่น แล้วหมุนขวา
      ToCenterG();
      spinDegree(90);
      break;
    case 'Q':  // เลี้ยวซ้ายล้อเดียวทันที
      turnDegree(-90);
      break;
    case 'q':  // ToFront แล้วเลี้ยวซ้ายล้อเดียว
      ToFrontG();
      turnDegree(-90);
      break;
    case 'E':  // เลี้ยวขวาล้อเดียวทันที
      turnDegree(90);
      break;
    case 'e':  // ToFront แล้วเลี้ยวขวาล้อเดียว
      ToFrontG();
      turnDegree(90);
      break;
    case 'p':  // วิ่งตรงผ่านแยก
    case 'P':  // ToFront แล้ววิ่งตรงผ่านแยก
      if (select == 'P') ToFrontG();
      BZon();
      ReadCalibrateF();
      while (1) {
        RunG(spd, spd);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
      FFtimerG(spd, 5);
      while (1) {
        RunG(spd, spd);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
      BZoff();
      break;
    case 's':  // เบรกหยุด
      Motor(-spd, -spd);
      delay(delay_break_f);
      MotorStop();
      break;
    case 'S':  // ToFront แล้วเบรกหยุด
      ToFrontG();
      Motor(-tctL, -tctR);
      delay(tct_delay_break);
      MotorStop();
      break;
    case 'c':  // เข้ากลางหุ่น แล้วหยุด
    case 'C':
      ToCenterG();
      break;
    case 'b':  // วิ่งตรงต่อจนเซนเซอร์หลังเจอเส้น
    case 'B':
      ReadCalibrateB();
      while (1) {
        RunG(spd, spd);
        ReadCalibrateB();
        if (B[3] > Ref && B[12] > Ref) break;
      }
      break;
    case 'g':  // ค้างมุมอยู่กับที่ นาน spd ms
    case 'G':
      SetG(spd);
      break;
    default:
      Stop(100);
  }
}

// ---------- วิ่งตรงด้วย Gyro ถอยหลัง ----------
void TrackSelectGB(int spd, char select) {
  switch (select) {
    case 'L':  // หมุนซ้ายทันที
      spinDegree(-90);
      break;
    case 'l':  // ถอยเข้ากลางหุ่น แล้วหมุนซ้าย
      BackToCenterG();
      spinDegree(-90);
      break;
    case 'R':  // หมุนขวาทันที
      spinDegree(90);
      break;
    case 'r':  // ถอยเข้ากลางหุ่น แล้วหมุนขวา
      BackToCenterG();
      spinDegree(90);
      break;
    case 'q':  // เลี้ยวถอยหลังล้อเดียว
    case 'Q':
      turnDegreeB(90);
      break;
    case 'e':  // เลี้ยวถอยหลังล้อเดียว
    case 'E':
      turnDegreeB(-90);
      break;
    case 'p':  // ถอยตรงผ่านแยก
    case 'P':
      BZon();
      ReadCalibrateB();
      while (1) {
        RunGB(spd, spd);
        ReadCalibrateB();
        if (B[3] < Ref && B[12] < Ref) break;
      }
      BBtimerG(spd, 5);
      while (1) {
        RunGB(spd, spd);
        ReadCalibrateB();
        if (B[3] < Ref && B[12] < Ref) break;
      }
      BZoff();
      break;
    case 's':  // เบรกหยุด
      Motor(spd, spd);
      delay(delay_break_b);
      Motor(1, 1);
      delay(1);
      MotorStop();
      break;
    case 'S':  // ถอยจนเซนเซอร์หลังเจอเส้น แล้วเบรกหยุด
      BackToFrontG();
      Motor(bctL, bctR);
      delay(bct_delay_break);
      Motor(1, 1);
      delay(1);
      MotorStop();
      break;
    case 'c':  // ถอยเข้ากลางหุ่น แล้วหยุด
      BackToCenterG();
      break;
    case 'C':  // ToFront ก่อน แล้วถอยเข้ากลางหุ่น
      ToFrontG();
      BackToCenterG();
      break;
    case 'b':  // ถอยตรงต่อจนเซนเซอร์หลังเจอเส้น
    case 'B':
      ReadCalibrateB();
      while (1) {
        RunGB(spd, spd);
        ReadCalibrateB();
        if (B[3] > Ref && B[12] > Ref) break;
      }
      break;
    case 'g':  // ค้างมุมอยู่กับที่ นาน spd ms
    case 'G':
      SetG(spd);
      break;
    default:
      Stop(100);
  }
}

// ---------- วิ่งตรงจนเซนเซอร์เจอเส้นดำ แล้วทำ select ----------
void FFBG(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[5] > Ref || F[6] > Ref || F[7] > Ref || F[8] > Ref || F[9] > Ref || F[10] > Ref) break;
  }
  TrackSelectG(Speed, select);
}

void BBBG(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[5] > Ref || B[6] > Ref || B[7] > Ref || B[8] > Ref || B[9] > Ref || B[10] > Ref) break;
  }
  TrackSelectGB(Speed, select);
}

// ---------- คำสั่งวิ่งตรง + select ----------
void FFtimerG(int Speed, int totalTime, char select) {
  FFtimerG(Speed, totalTime);
  TrackSelectG(Speed, select);
}

void BBtimerG(int Speed, int totalTime, char select) {
  BBtimerG(Speed, totalTime);
  TrackSelectGB(Speed, select);
}

void FFcmG(int Speed, float distance_cm, char select) {
  FFcmG(Speed, distance_cm);
  TrackSelectG(Speed, select);
}

void BBcmG(int Speed, float distance_cm, char select) {
  BBcmG(Speed, distance_cm);
  TrackSelectGB(Speed, select);
}

void FFcmGS(int Speed, float distance_cm, char select) {
  FFcmGS(Speed, distance_cm);
  TrackSelectG(Speed, select);
}

void BBcmGS(int Speed, float distance_cm, char select) {
  BBcmGS(Speed, distance_cm);
  TrackSelectGB(Speed, select);
}


/* ==================================================================================================
 *  10) ทิศสัมบูรณ์ (direction: 0, 90, 180, 270 ... นับจากตอน reset มุมครั้งล่าสุด)
 * ================================================================================================== */

// คำนวณมุมที่ต้องหมุนจากมุมปัจจุบันไปยังทิศสัมบูรณ์ (ทางที่สั้นที่สุด)
int relativeToDirection(int direction) {
  float relative = wrap180(fmod((float)direction, 360.0f) - angleRead());
  return (int)roundf(relative);
}

// ---------- หมุน / เลี้ยว ไปยังทิศ ----------
void spinDirection(int Speed, int direction) {
  spinDegree(Speed, relativeToDirection(direction));
}
void spinDirection(int direction) {
  spinDegree(relativeToDirection(direction));
}

void turnDirection(int Speed, int direction) {
  turnDegree(Speed, relativeToDirection(direction));
}
void turnDirection(int direction) {
  turnDegree(relativeToDirection(direction));
}

void turnDirectionB(int Speed, int direction) {
  turnDegreeB(Speed, relativeToDirection(direction));
}
void turnDirectionB(int direction) {
  turnDegreeB(relativeToDirection(direction));
}

void turnDirection_none(int Speed, int direction) {
  turnDegree_none(Speed, relativeToDirection(direction));
}
void turnDirection_none(int direction) {
  turnDegree_none(relativeToDirection(direction));
}

void turnDirectionb_none(int Speed, int direction) {
  turnDegreeb_none(Speed, relativeToDirection(direction));
}
void turnDirectionb_none(int direction) {
  turnDegreeb_none(relativeToDirection(direction));
}

// ---------- ตั้งทิศที่ RunG / RunGB วิ่งตรงเข้าหา (dir = ทิศที่หน้าหุ่นหัน ทั้งเดินหน้าและถอยหลัง) ----------
void SetDirectionG(int direction) {
  current_degree = wrap180(fmod((float)direction, 360.0f));
  float error = wrap180(current_degree - angleRead());
  previous_errorG = error;
  previous_errorGB = error;
}

// ---------- วิ่งตรง + ทิศ ----------
void FFtimerG(int Speed, int totalTime, int direction) {
  SetDirectionG(direction);
  FFtimerG(Speed, totalTime);
}
void BBtimerG(int Speed, int totalTime, int direction) {
  SetDirectionG(direction);
  BBtimerG(Speed, totalTime);
}

void FFcmG(int Speed, float distance_cm, int direction) {
  SetDirectionG(direction);
  useDirectionG = true;
  FFcmG(Speed, distance_cm);
  useDirectionG = false;
}
void BBcmG(int Speed, float distance_cm, int direction) {
  SetDirectionG(direction);
  useDirectionG = true;
  BBcmG(Speed, distance_cm);
  useDirectionG = false;
}

void FFcmGS(int Speed, float distance_cm, int direction) {
  SetDirectionG(direction);
  useDirectionG = true;
  FFcmGS(Speed, distance_cm);
  useDirectionG = false;
}
void BBcmGS(int Speed, float distance_cm, int direction) {
  SetDirectionG(direction);
  useDirectionG = true;
  BBcmGS(Speed, distance_cm);
  useDirectionG = false;
}

void FFBG(int Speed, char select, int direction) {
  SetDirectionG(direction);
  FFBG(Speed, select);
}
void BBBG(int Speed, char select, int direction) {
  SetDirectionG(direction);
  BBBG(Speed, select);
}

// ---------- วิ่งตรง + select + ทิศ ----------
void FFtimerG(int Speed, int totalTime, char select, int direction) {
  FFtimerG(Speed, totalTime, direction);
  TrackSelectG(Speed, select);
}
void BBtimerG(int Speed, int totalTime, char select, int direction) {
  BBtimerG(Speed, totalTime, direction);
  TrackSelectGB(Speed, select);
}

void FFcmG(int Speed, float distance_cm, char select, int direction) {
  FFcmG(Speed, distance_cm, direction);
  TrackSelectG(Speed, select);
}
void BBcmG(int Speed, float distance_cm, char select, int direction) {
  BBcmG(Speed, distance_cm, direction);
  TrackSelectGB(Speed, select);
}

void FFcmGS(int Speed, float distance_cm, char select, int direction) {
  FFcmGS(Speed, distance_cm, direction);
  TrackSelectG(Speed, select);
}
void BBcmGS(int Speed, float distance_cm, char select, int direction) {
  BBcmGS(Speed, distance_cm, direction);
  TrackSelectGB(Speed, select);
}

#endif
