/* =====================================================================
 *  Unknow_IMU.h — Gyro (BNO055): อ่านมุม, หมุน/เลี้ยวตามองศา, วิ่งตรงด้วย Gyro
 *
 *  สารบัญ
 *     1) ตัวแปร (Global Variables)
 *     2) อ่านมุมจาก BNO055
 *     3) ตั้ง / รีเซ็ตมุมอ้างอิง
 *     4) ค้างมุมอยู่กับที่ (Hold Angle)
 *     5) ตั้งค่า Gyro (เรียกใน Setting.ino)
 *     6) หมุน / เลี้ยว ตามองศา
 *     7) คำสั่งย่อ หมุน / เลี้ยว
 *     8) หันไปทิศสัมบูรณ์ (0, 90, 180, 270 นับจากตอน reset มุม)
 *     9) วิ่งตรงด้วย Gyro
 *    10) คำสั่งเมื่อเจอแยก (Gyro)
 *    11) วิ่งตรงด้วย Gyro + คำสั่งเจอแยก
 *    12) วิ่งตามระยะทางด้วย Gyro (cm)
 *    13) วิ่งตรงตามทิศสัมบูรณ์
 *    14) วิ่งตามเส้นจนหันถึงทิศ (FFdir / BBdir / FFdirCirCleL / FFdirCirCleR ...)
 * ===================================================================== */

#pragma once

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Unknow_Buzzer.h"
#include "Unknow_Motor.h"
#include "Unknow_Sensor.h"
#include "Unknow_PID.h"


// =====================================================================
//  1) ตัวแปร (Global Variables)
// =====================================================================

// ---------- BNO055 / มุมปัจจุบัน ----------
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);
float angles_offset[] = { 0, 0, 0 };  // [2] = มุม yaw ดิบตอน reset (ใช้เป็นมุม 0)
float lastYaw = 0;                    // มุมล่าสุดที่ angleRead() อ่านได้
float current_degree = 0;
int16_t previous_errorG;  // error มุม ±180
int16_t previous_errorGB;
bool useDirectionG = false;  // true = คำสั่ง cm ใช้ทิศที่ตั้งจาก SetDirectionG แทนการจำมุมปัจจุบัน

// ---------- ค้างมุม (Hold) ----------
float kpHold = 2.5;
float kdHold = 1.5;
float kpFHold = 2.5;
float kdFHold = 1.5;
float kpBHold = 2.5;
float kdBHold = 1.2;
float holdAngle = 0;
float prevErrHold = 0;
float prevErrHoldB = 0;
float prevErrHoldF = 0;
float prevErrHoldBF = 0;

// ---------- หมุน / เลี้ยว — ปรับได้ด้วย SetGyroTurn / SetGyroSpin ใน Setting.ino ----------
float gyro_Kp_Spin = 0.75f;
float gyro_Kd_Spin = 0.9f;
int8_t gyro_MaxSpd_Spin = 40;  // ความเร็ว 0..100
int8_t gyro_MinSpd_Spin = 8;
float gyro_SmallAngle_Spin = 8.0f;
float gyro_StopThr_Spin = 1.0f;
float gyro_Kp_Turn = 0.75f;
float gyro_Kd_Turn = 0.9f;
int8_t gyro_MaxSpd_Turn = 50;
int8_t gyro_MinSpd_Turn = 10;
float gyro_SmallAngle_Turn = 10.0f;
float gyro_StopThr_Turn = 1.0f;
float gyro_Kp_TurnB = 0.75f;  // เลี้ยวถอยหลัง (turnDegreeB) — ตั้งด้วย SetGyroTurnB
float gyro_Kd_TurnB = 0.9f;
int8_t gyro_MaxSpd_TurnB = 50;
int8_t gyro_MinSpd_TurnB = 10;
float gyro_SmallAngle_TurnB = 10.0f;
float gyro_StopThr_TurnB = 1.0f;
float gyro_StopThr_Rotate = 1.0f;

// ---------- วิ่งตรง — ปรับได้ด้วย SetGyroRun / SetGyroRunB / ModeSpdGyro ใน Setting.ino ----------
float kpG = 1.2;
float kdG = 1.5;

float kpGB = 1.2;
float kdGB = 1.5;


// โหมดจำกัดกำลังมอเตอร์ของ RunG (เดินหน้า) / RunGB (ถอยหลัง)
// ค่าเริ่มต้น = โหมด 4, min -5 (เหมือนพฤติกรรมเดิม) ปรับได้ด้วย ModeSpdGyro() ใน Setting.ino
int8_t MaxSpeedG = 100;
int8_t MinSpeedG = -5;
uint8_t ModeGyroStatus = 4;
uint8_t ModeGyroBStatus = 4;
int8_t gyro_limit_up = 20, gyro_limit_down = 20;  // โหมด 2: -(Speed+down)..(Speed+up)
int8_t gyro_limit4_up = 20;                       // โหมด 4: ..(Speed+up)

// ---------- ตัวคูณแปลงความเร็ว → ระยะทาง ของ FFcmG / BBcmG / FFcmGS / BBcmGS ----------
// ปรับได้ด้วย set_speed_scale_ff_g / set_speed_scale_bb_g / set_speed_scale_ff_bb_gs ใน Setting.ino
float speed_scale_ff_g = 1.5;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bb_g = 1.5;
float speed_scale_ff_g_normal = 0.99;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bb_g_normal = 0.99;
float speed_scale_ff_gs = 1.65;  // ค่าตัวคูณความเร็ววิ่งตรง (RunG / RunGB)
float speed_scale_bb_gs = 1.65;

// ---------- เบรกหลังหมุน / เลี้ยวด้วย Gyro ----------
// _g = เวลาล็อกมุม (SetG) หลังถึงมุม | BounceSpd / BounceMs = เบรกสวนทิศสั้น ๆ ทันทีที่ถึงมุม
// ความเร็ว = int8_t | เวลา ms = uint16_t
uint16_t brake_spin_g = 5;
uint16_t brake_turn_g = 5;
uint16_t brake_turn_b_g = 5;
int8_t gyro_BounceSpd_spin = 12;
uint16_t gyro_BounceMs_spin = 5;
int8_t gyro_BounceSpd_turn = 12;
uint16_t gyro_BounceMs_turn = 5;
int8_t gyro_BounceSpd_turn_b = 12;
uint16_t gyro_BounceMs_turn_b = 5;


// =====================================================================
//  2) อ่านมุมจาก BNO055
// =====================================================================

float wrap180(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}

// มุมปัจจุบันเทียบกับตอน reset (0 ถึง 360)
float angleRead() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float adjusted = euler.x() - angles_offset[2];
  if (adjusted >= 360) adjusted -= 360;
  else if (adjusted < 0) adjusted += 360;
  return lastYaw = adjusted;
}

// อ่านมุมใหม่ก่อนตั้งมุมเป้าหมาย — BNO055 อ่านค่าตรงจากเซนเซอร์ทุกครั้งอยู่แล้ว จึงเท่ากับ angleRead()
float angleReadFresh() {
  return angleRead();
}


// =====================================================================
//  3) ตั้ง / รีเซ็ตมุมอ้างอิง
// =====================================================================

// รีเซ็ตมุมปัจจุบันของหุ่นยนต์ให้เป็น 0 องศา
void setAngleOffset() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angles_offset[2] = euler.x();
  current_degree = angleRead();
  previous_errorG = 0;
  previous_errorGB = 0;
}

// รีเซ็ต D-term ของ RunG / RunGB ให้เท่ากับ error ปัจจุบัน (ไม่ใช่ 0) กันกระชากตอนออกตัว
// เรียกจาก InitialSpeed() ด้วย | ใช้ lastYaw (ไม่อ่านเซนเซอร์ใหม่) จึงปลอดภัยแม้ยังไม่เริ่ม IMU
void resetGyroRunError() {
  float error = wrap180(current_degree - lastYaw);
  previous_errorG = error;
  previous_errorGB = -error;  // RunGB ใช้ error กลับด้าน (angleRead() - current_degree)
}

void SetRobotAngle() {
  current_degree = angleRead();
  resetGyroRunError();  // คำสั่ง cm เรียกหลัง InitialSpeed() จึงต้องรีเซ็ต D-term ใหม่ตามมุมที่จำ
}

void resetAngle() {
  setAngleOffset();
}

void resetAngles() {
  setAngleOffset();
}


// =====================================================================
//  4) ค้างมุมอยู่กับที่ (Hold Angle)
// =====================================================================

void HoldAngle() {
  float error = current_degree - angleRead();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHold;
  int power = (error * kpHold) + (d * kdHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(power, -power);               // หมุนอยู่กับที่
  prevErrHold = error;
}

void HoldAngleB() {
  float error = angleRead() - current_degree;
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldB;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(power, -power);               // หมุนอยู่กับที่
  prevErrHoldB = error;
}

void SetHoldAngle() {
  holdAngle = angleRead();  // มุมที่ต้องการให้หุ่น "จำ"
  MotorStop();
  prevErrHoldF = 0;
}

void HoldAngleF() {
  float error = holdAngle - angleRead();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldF;
  int power = (error * kpFHold) + (d * kdFHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(power, -power);               // หมุนอยู่กับที่
  prevErrHoldF = error;
}

void SetFG(int totalTime) {
  BZon();
  SetHoldAngle();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleF();
  }
  BZoff();
}

void setfg(int totalTime) {
  SetFG(totalTime);
}

void HoldAngleBF() {
  float error = angleRead() - holdAngle;
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldBF;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);  // แรงหมุน
  Motor(-power, power);               // หมุนอยู่กับที่
  prevErrHoldBF = error;
}

// ล็อกมุมแบบถอยหลัง: จำมุม ณ ตอนเรียก (เหมือน SetFG)
void SetBG(int totalTime) {
  BZon();
  SetHoldAngle();
  prevErrHoldBF = 0;
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleBF();
  }
  BZoff();
}

void setbg(int totalTime) {
  SetBG(totalTime);
}

void SetG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngle();
  }
  BZoff();
}

// ล็อกมุมแบบถอยหลัง ที่มุมอ้างอิง current_degree (เหมือน SetG)
void SetGB(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleB();
  }
  BZoff();
}

void setg(int time) {
  SetG(time);
}

void setgb(int time) {
  SetGB(time);
}


// =====================================================================
//  5) ตั้งค่า Gyro (เรียกใน Setting.ino)
// =====================================================================

// ---------- PID หมุน / เลี้ยว / วิ่งตรง ----------
void SetGyroTurn(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Turn = kp;
  gyro_Kd_Turn = kd;
  gyro_MaxSpd_Turn = maxSpd;
  gyro_MinSpd_Turn = minSpd;
  gyro_SmallAngle_Turn = smallAngle;
  gyro_StopThr_Turn = stopThr;
}

// เลี้ยวถอยหลังล้อเดียว (turnDegreeB / turnDirectionB) แยกจาก SetGyroTurn
void SetGyroTurnB(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_TurnB = kp;
  gyro_Kd_TurnB = kd;
  gyro_MaxSpd_TurnB = maxSpd;
  gyro_MinSpd_TurnB = minSpd;
  gyro_SmallAngle_TurnB = smallAngle;
  gyro_StopThr_TurnB = stopThr;
}

void SetGyroSpin(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Spin = kp;
  gyro_Kd_Spin = kd;
  gyro_MaxSpd_Spin = maxSpd;
  gyro_MinSpd_Spin = minSpd;
  gyro_SmallAngle_Spin = smallAngle;
  gyro_StopThr_Spin = stopThr;
}

void SetGyroRun(float kp, float kd) {
  kpG = kp;
  kdG = kd;
}

void SetGyroRunB(float kp, float kd) {
  kpGB = kp;
  kdGB = kd;
}

// ตั้งโหมดจำกัดกำลังของ gyro (ใช้ทั้ง RunG และ RunGB)
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

// ช่วงกำลังล้อของโหมด 2 (ใช้ทั้ง RunG และ RunGB)
void set_gyro_mode2_limit(int up, int down) {
  gyro_limit_up = up;
  gyro_limit_down = down;
}

// ช่วงกำลังล้อของโหมด 4: ..(Speed+up) (ใช้ทั้ง RunG และ RunGB)
void set_gyro_mode4_limit(int up) {
  gyro_limit4_up = up;
}

// ---------- ตัวคูณระยะทาง (หุ่นเดินเกินระยะ ให้เพิ่มค่า | เดินไม่ถึงระยะ ให้ลดค่า) ----------
// FFcmG: fw_normal = ระยะ >= 30 cm (มีเร่ง/ชะลอ), fw = ระยะ < 30 cm (ไม่มี ramp)
void set_speed_scale_ff_g(float fw_normal, float fw) {
  speed_scale_ff_g_normal = fw_normal;
  speed_scale_ff_g = fw;
}

// BBcmG: bb_normal = ระยะ >= 30 cm (มีเร่ง/ชะลอ), bb = ระยะ < 30 cm (ไม่มี ramp)
void set_speed_scale_bb_g(float bb_normal, float bb) {
  speed_scale_bb_g_normal = bb_normal;
  speed_scale_bb_g = bb;
}

// FFcmGS / BBcmGS
void set_speed_scale_ff_bb_gs(float ff, float bb) {
  speed_scale_ff_gs = ff;
  speed_scale_bb_gs = bb;
}

// ---------- เบรกหลังหมุน / เลี้ยว ----------
// _setg(ms)  = เวลาล็อกมุม (SetG) หลังหมุน/เลี้ยวเสร็จ
// (spd, ms)  = เบรกสวนทิศสั้น ๆ ทันทีที่ถึงมุม | หมุนเกินมุม ให้เพิ่ม | เด้งกลับ ให้ลด (ใส่ 0 = ไม่เบรก)
void set_brake_spin_gyro_setg(int brake) {
  brake_spin_g = brake;
}

void set_brake_spin_gyro(int spd, int t) {
  gyro_BounceSpd_spin = spd;
  gyro_BounceMs_spin = t;
}

void set_brake_turn_gyro_setg(int brake) {
  brake_turn_g = brake;
}

void set_brake_turn_gyro(int spd, int t) {
  gyro_BounceSpd_turn = spd;
  gyro_BounceMs_turn = t;
}

void set_brake_turn_b_gyro_setg(int brake) {
  brake_turn_b_g = brake;
}

void set_brake_turn_b_gyro(int spd, int t) {
  gyro_BounceSpd_turn_b = spd;
  gyro_BounceMs_turn_b = t;
}


// =====================================================================
//  6) หมุน / เลี้ยว ตามองศา
// =====================================================================

// ---------- หมุนอยู่กับที่ (spin) ----------
void spinDegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Spin;
  int max_speed = Speed;
  float kp = gyro_Kp_Spin;
  float kd = gyro_Kd_Spin;
  float small_angle_threshold = gyro_SmallAngle_Spin;
  float stop_threshold = gyro_StopThr_Spin;
  float previous_error = 0;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
  int spin_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = หมุนขวา, -1 = หมุนซ้าย, 0 = ไม่ได้หมุน
  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
    int pd_value = (kp * error) + (kd * (error - previous_error));
    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, -min_speed);
      spin_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, min_speed);
      spin_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      break;
    } else {
      Motor(pd_value, -pd_value);
      if (pd_value != 0) spin_dir = (pd_value > 0) ? 1 : -1;
    }
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
    previous_error = error;
  }
  // เบรกสวนทิศที่หมุนจริง (ไม่ใช้ relative_degree เพราะหลัง wrap ±180 ทิศอาจกลับด้าน)
  if (spin_dir != 0) {
    Motor(-spin_dir * gyro_BounceSpd_spin, spin_dir * gyro_BounceSpd_spin);
    delay(gyro_BounceMs_spin);
  }
  MotorStop();
  SetG(brake_spin_g);
}

void spinDegree(int relative_degree) {
  spinDegree(gyro_MaxSpd_Spin, relative_degree);
}

// ---------- เลี้ยวล้อเดียว (turn) ----------
void turnDegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Turn;
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;
  float kd = gyro_Kd_Turn;
  float small_angle_threshold = gyro_SmallAngle_Turn;
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
  int turn_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = เลี้ยวขวา (ล้อซ้ายเดิน), -1 = เลี้ยวซ้าย (ล้อขวาเดิน), 0 = ไม่ได้เลี้ยว
  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
    int pd_value = (kp * error) + (kd * (error - previous_error));
    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(min_speed, 0);
      turn_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(0, min_speed);
      turn_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) {
        Motor(-1, -pd_value);
        turn_dir = -1;
      } else {
        Motor(pd_value, 1);
        turn_dir = 1;
      }
    }
    previous_error = error;
  }
  // เบรกล้อที่เดินอยู่ สวนทิศที่เลี้ยวจริง
  if (turn_dir > 0) Motor(-gyro_BounceSpd_turn, 0);
  else if (turn_dir < 0) Motor(0, -gyro_BounceSpd_turn);
  if (turn_dir != 0) delay(gyro_BounceMs_turn);
  MotorStop();
  SetG(brake_turn_g);
}

void turnDegree(int relative_degree) {
  turnDegree(gyro_MaxSpd_Turn, relative_degree);
}

void turnDegreeB(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_TurnB;  // ค่าชุด TurnB แยกจาก turnDegree (ตั้งด้วย SetGyroTurnB)
  int max_speed = Speed;
  float kp = gyro_Kp_TurnB;
  float kd = gyro_Kd_TurnB;
  float small_angle_threshold = gyro_SmallAngle_TurnB;
  float stop_threshold = gyro_StopThr_TurnB;
  float previous_error = 0;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
  int turn_dir = 0;  // ทิศคำสั่งล่าสุด: 1 = ล้อขวาถอย, -1 = ล้อซ้ายถอย, 0 = ไม่ได้เลี้ยว
  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
    int pd_value = (kp * error) + (kd * (error - previous_error));
    if (pd_value > max_speed) pd_value = max_speed;
    else if (pd_value < -max_speed) pd_value = -max_speed;
    if (error > stop_threshold && error < small_angle_threshold) {
      Motor(0, -min_speed);
      turn_dir = 1;
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, 0);
      turn_dir = -1;
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) {
        Motor(pd_value, 1);
        turn_dir = -1;
      } else {
        Motor(-1, -pd_value);
        turn_dir = 1;
      }
    }
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
    previous_error = error;
  }
  // เบรกล้อที่ถอยอยู่ ด้วยการเดินหน้าสวนสั้น ๆ
  if (turn_dir > 0) Motor(0, gyro_BounceSpd_turn_b);
  else if (turn_dir < 0) Motor(gyro_BounceSpd_turn_b, 0);
  if (turn_dir != 0) delay(gyro_BounceMs_turn_b);
  MotorStop();
  SetG(brake_turn_b_g);
}

void turnDegreeB(int relative_degree) {
  turnDegreeB(gyro_MaxSpd_TurnB, relative_degree);
}

// ---------- เลี้ยวล้อเดียวแบบไม่เบรก (_none) ----------
void turnDegree_none(int Speed, int relative_degree) {
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // ถ้าหมุนเร็ว มุมจะกระโดดข้ามช่วง ±stop_threshold ได้
    // จึงถือว่าถึงเป้าเมื่อ error เปลี่ยนเครื่องหมาย (หมุนเลยเป้าแล้ว) ด้วย ไม่ให้หมุนกลับไปมา
    // (|error| < 90 กันกรณี error กระโดดจาก +180 เป็น -180 ตอน wrap)
    bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
    previous_error = error;

    if ((error >= -stop_threshold && error <= stop_threshold) || crossed) {
      break;
    } else if (error > 0) {
      Motor(Speed, -2);
    } else {
      Motor(-2, Speed);
    }
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
  }
}

void turnDegree_none(int relative_degree) {
  turnDegree_none(gyro_MaxSpd_Turn, relative_degree);
}

void turnDegreeb_none(int Speed, int relative_degree) {
  float stop_threshold = gyro_StopThr_TurnB;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    // ถือว่าถึงเป้าเมื่อ error เปลี่ยนเครื่องหมายด้วย (ดู turnDegree_none)
    bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
    previous_error = error;

    if ((error >= -stop_threshold && error <= stop_threshold) || crossed) {
      break;
    } else if (error > 0) {
      Motor(1, -Speed);
    } else {
      Motor(-Speed, 1);
    }
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
  }
}

void turnDegreeb_none(int relative_degree) {
  turnDegreeb_none(gyro_MaxSpd_TurnB, relative_degree);
}

// ---------- โค้งด้วยความเร็วล้อซ้าย / ขวา (rotateDegree) ----------
void rotateDegree(int SpeedL, int SpeedR, int relative_degree, float kp, float kd) {
  float stop_threshold = gyro_StopThr_Rotate;
  float previous_error = 0;
  float target_degree = angleReadFresh() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
  while (1) {
    float error = target_degree - angleRead();
    if (error > 180) error -= 360;
    else if (error < -180) error += 360;
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
    delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นหมุนช้าลงและลดการสั่น
  }
  SetG(max(abs(SpeedL), abs(SpeedR)));
}

void rotateDegree(int SpeedL, int SpeedR, int relative_degree) {
  rotateDegree(SpeedL, SpeedR, relative_degree, 0.9, 0.6);
}


// =====================================================================
//  7) คำสั่งย่อ หมุน / เลี้ยว
// =====================================================================

// ---------- ทางเดียว ----------
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

// ---------- เลี้ยวไป-กลับ (tlrg / trlg) ----------
void tlrg(int Angle) {
  turnDegree_none(-abs(Angle));
  turnDegree(abs(Angle));
}

void trlg(int Angle) {
  turnDegree_none(abs(Angle));
  turnDegree(-abs(Angle));
}

void tlrg(int spd, int Angle) {
  turnDegree_none(spd, -abs(Angle));
  turnDegree(spd, abs(Angle));
}

void trlg(int spd, int Angle) {
  turnDegree_none(spd, abs(Angle));
  turnDegree(spd, -abs(Angle));
}

void tlrg(int spd, int Angle, int Angle2) {
  turnDegree_none(spd, -abs(Angle));
  turnDegree(spd, abs(Angle2));
}

void trlg(int spd, int Angle, int Angle2) {
  turnDegree_none(spd, abs(Angle));
  turnDegree(spd, -abs(Angle2));
}

void tlrbg(int Angle) {
  turnDegreeb_none(abs(Angle));
  turnDegreeB(-abs(Angle));
}

void trlbg(int Angle) {
  turnDegreeb_none(-abs(Angle));
  turnDegreeB(abs(Angle));
}

void tlrbg(int spd, int Angle) {
  turnDegreeb_none(spd, abs(Angle));
  turnDegreeB(spd, -abs(Angle));
}

void trlbg(int spd, int Angle) {
  turnDegreeb_none(spd, -abs(Angle));
  turnDegreeB(spd, abs(Angle));
}

void tlrbg(int spd, int Angle, int Angle2) {
  turnDegreeb_none(spd, abs(Angle));
  turnDegreeB(spd, -abs(Angle2));
}

void trlbg(int spd, int Angle, int Angle2) {
  turnDegreeb_none(spd, -abs(Angle));
  turnDegreeB(spd, abs(Angle2));
}

// ---------- ชื่อเต็มของ tlrg / trlg ----------
void TurnLRG(int Angle) {
  tlrg(Angle);
}

void TurnRLG(int Angle) {
  trlg(Angle);
}

void TurnLRG(int spd, int Angle) {
  tlrg(spd, Angle);
}

void TurnRLG(int spd, int Angle) {
  trlg(spd, Angle);
}

void TurnLRG(int spd, int Angle, int Angle2) {
  tlrg(spd, Angle, Angle2);
}

void TurnRLG(int spd, int Angle, int Angle2) {
  trlg(spd, Angle, Angle2);
}

void TurnLRBG(int Angle) {
  tlrbg(Angle);
}

void TurnRLBG(int Angle) {
  trlbg(Angle);
}

void TurnLRBG(int spd, int Angle) {
  tlrbg(spd, Angle);
}

void TurnRLBG(int spd, int Angle) {
  trlbg(spd, Angle);
}

void TurnLRBG(int spd, int Angle, int Angle2) {
  tlrbg(spd, Angle, Angle2);
}

void TurnRLBG(int spd, int Angle, int Angle2) {
  trlbg(spd, Angle, Angle2);
}


// =====================================================================
//  8) หันไปทิศสัมบูรณ์ (0, 90, 180, 270 นับจากตอน reset มุม)
// =====================================================================


// คำนวณมุมที่ต้องหมุนจากมุมปัจจุบันไปยังทิศทางสัมบูรณ์ (ทางที่สั้นที่สุด)
int relativeToDirection(int direction) {
  float relative = fmod((float)direction, 360.0f) - angleReadFresh();
  while (relative > 180.0f) relative -= 360.0f;
  while (relative < -180.0f) relative += 360.0f;
  return (int)roundf(relative);
}

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
  turnDirectionB(gyro_MaxSpd_TurnB, direction);
}

void turnDirection_none(int Speed, int direction) {
  turnDegree_none(Speed, relativeToDirection(direction));
}

void turnDirectionb_none(int Speed, int direction) {
  turnDegreeb_none(Speed, relativeToDirection(direction));
}

void turnDirection_none(int direction) {
  turnDegree_none(relativeToDirection(direction));
}

void turnDirectionb_none(int direction) {
  turnDirectionb_none(gyro_MaxSpd_TurnB, direction);
}

// ตั้งทิศทางสัมบูรณ์ให้ RunG / RunGB วิ่งตรงตาม
void SetDirectionG(int direction) {
  float target = fmod((float)direction, 360.0f);
  if (target > 180) target -= 360;
  else if (target < -180) target += 360;
  current_degree = target;
  resetGyroRunError();
}


// =====================================================================
//  9) วิ่งตรงด้วย Gyro
// =====================================================================

// ---------- PD คุมทิศ ----------

// จำกัดค่า LeftPower/RightPower ของ gyro ตามโหมด
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
    case 2:  // -(Speed+down)..Speed+up
      if (LeftPower > SpeedL + gyro_limit_up) LeftPower = SpeedL + gyro_limit_up;
      if (LeftPower < -SpeedL - gyro_limit_down) LeftPower = -SpeedL - gyro_limit_down;
      if (RightPower > SpeedR + gyro_limit_up) RightPower = SpeedR + gyro_limit_up;
      if (RightPower < -SpeedR - gyro_limit_down) RightPower = -SpeedR - gyro_limit_down;
      break;
    case 3:  // ..max, ล้อติดลบ → -BaseSpeed
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = -BaseSpeed;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = -BaseSpeed;
      break;
    case 4:  // ..Speed+up, ล้อติดลบ → min (min = 0 คือไม่ถอยล้อ)
      if (LeftPower > SpeedL + gyro_limit4_up) LeftPower = SpeedL + gyro_limit4_up;
      if (LeftPower < 0) LeftPower = MinSpeedG;
      if (RightPower > SpeedR + gyro_limit4_up) RightPower = SpeedR + gyro_limit4_up;
      if (RightPower < 0) RightPower = MinSpeedG;
      break;
    default:
      if (LeftPower > MaxSpeedG) LeftPower = MaxSpeedG;
      if (LeftPower < 0) LeftPower = 0;
      if (RightPower > MaxSpeedG) RightPower = MaxSpeedG;
      if (RightPower < 0) RightPower = 0;
  }
}

void RunG(int SpeedL, int SpeedR) {
  float error = current_degree - angleRead();
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorG;
  int pd_value = (error * kpG) + (derivative * kdG);
  float leftPow = SpeedL + pd_value;
  float rightPow = SpeedR - pd_value;
  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroStatus);
  Motor(leftPow, rightPow);
  previous_errorG = error;
  delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นวิ่งช้าลงและลดการสั่น
}

void RunGB(int SpeedL, int SpeedR) {
  float error = angleRead() - current_degree;  // ถอยหลัง: กลับด้าน error เทียบกับ RunG
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorGB;
  int pd_value = (error * kpGB) + (derivative * kdGB);
  float leftPow = SpeedL + pd_value;
  float rightPow = SpeedR - pd_value;
  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroBStatus);
  Motor(-leftPow, -rightPow);
  previous_errorGB = error;
  delayMicroseconds(80);  // เพิ่ม delay เพื่อให้หุ่นวิ่งช้าลงและลดการสั่น
}

// ---------- จับเวลา ----------
void FFtimerG(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
  }
}

void BBtimerG(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
  }
}

// ---------- เข้ากลางหุ่น / เข้าหาเส้น ----------
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

void ToFrontG() {
  while (1) {
    RunG(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[3] > Ref || F[12] > Ref) break;
  }
}

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

void BackToFrontG() {
  while (1) {
    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[3] > Ref || B[12] > Ref) break;
  }
}


// =====================================================================
//  10) คำสั่งเมื่อเจอแยก (Gyro)
// =====================================================================

void TrackSelectG(int spd, char select) {
  if (select == 'L') {
    spinDegree(-90);
  } else if (select == 'l') {
    ToCenterG();
    spinDegree(-90);
  } else if (select == 'R') {
    spinDegree(90);
  } else if (select == 'r') {
    ToCenterG();
    spinDegree(90);
  } else if (select == 'Q') {
    turnDegree(-90);
  } else if (select == 'q') {
    ToFrontG();
    turnDegree(-90);
  } else if (select == 'E') {
    turnDegree(90);
  } else if (select == 'e') {
    ToFrontG();
    turnDegree(90);
  } else if (select == 'p') {
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
  } else if (select == 'P') {
    ToFrontG();
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
  } else if (select == 's') {
    Motor(-spd, -spd);
    delay(delay_break_f);
    MotorStop();
  } else if (select == 'S') {
    ToFrontG();
    Motor(-tctL, -tctR);
    delay(tct_delay_break);
    MotorStop();
  } else if (select == 'c' || select == 'C') {
    ToCenterG();
  } else if (select == 'b' || select == 'B') {
    ReadCalibrateB();
    while (1) {
      RunG(spd, spd);
      ReadCalibrateB();
      if (B[3] > Ref && B[12] > Ref) break;
    }
  } else if (select == 'g' || select == 'G') {
    SetG(spd);
  } else {
    Stop(100);
  }
}

void TrackSelectGB(int spd, char select) {
  if (select == 'L') {
    spinDegree(-90);
  } else if (select == 'l') {
    BackToCenterG();
    spinDegree(-90);
  } else if (select == 'R') {
    spinDegree(90);
  } else if (select == 'r') {
    BackToCenterG();
    spinDegree(90);
  } else if (select == 'q' || select == 'Q') {
    turnDegreeB(90);
  } else if (select == 'e' || select == 'E') {
    turnDegreeB(-90);
  } else if (select == 'p') {
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
  } else if (select == 'P') {
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
  } else if (select == 's') {
    Motor(spd, spd);
    delay(delay_break_b);
    Motor(1, 1);
    delay(1);
    MotorStop();
  } else if (select == 'S') {
    BackToFrontG();
    Motor(bctL, bctR);
    delay(bct_delay_break);
    Motor(1, 1);
    delay(1);
    MotorStop();
  } else if (select == 'c') {
    BackToCenterG();
  } else if (select == 'C') {
    ToFrontG();
    BackToCenterG();
  } else if (select == 'b' || select == 'B') {
    ReadCalibrateB();
    while (1) {
      RunGB(spd, spd);
      ReadCalibrateB();
      if (B[3] > Ref && B[12] > Ref) break;
    }
  } else if (select == 'g' || select == 'G') {
    SetG(spd);
  } else {
    Stop(100);
  }
}


// =====================================================================
//  11) วิ่งตรงด้วย Gyro + คำสั่งเจอแยก
// =====================================================================

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

void FFtimerG(int Speed, int totalTime, char select) {
  FFtimerG(Speed, totalTime);
  TrackSelectG(Speed, select);
}

void BBtimerG(int Speed, int totalTime, char select) {
  BBtimerG(Speed, totalTime);
  TrackSelectGB(Speed, select);
}


// =====================================================================
//  12) วิ่งตามระยะทางด้วย Gyro (cm)
// =====================================================================

void FFcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();


  if (!useDirectionG) SetRobotAngle();  // เซ็ตค่าปัจจุบัน
  unsigned long prevT = millis();
  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    RunG(LeftBaseSpeed, RightBaseSpeed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale_ff_gs) * delta_time;
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


  if (!useDirectionG) SetRobotAngle();  // เซ็ตค่าปัจจุบัน
  unsigned long prevT = millis();

  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    RunGB(BackLeftBaseSpeed, BackRightBaseSpeed);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale_bb_gs) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
  }
}

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

  // ====================== ค่าที่สามารถปรับได้ ======================
  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  // ค่า speed_scale ที่คุณต้องการปรับได้ (ค่าดีฟอลต์ = 0.99)
  float speed_scale = speed_scale_ff_g_normal;  // ปรับได้ด้วย set_speed_scale_ff_g()

  // ตัดสินใจว่าใช้ Ramp หรือไม่
  bool enableRamp = (distance_cm >= 30.0);

  // ถ้าระยะสั้นมาก (< 30) ให้ปรับ speed_scale ได้ง่ายขึ้น
  if (!enableRamp) {
    speed_scale = speed_scale_ff_g;
  }
  if (!useDirectionG) SetRobotAngle();  // เซ็ตค่าปัจจุบัน

  while (true) {
    // คำนวณระยะทาง
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;

    if (remaining_cm <= 0.7f) break;

    // ====================== คำนวณ target_speed ======================
    float target_speed = base_speed;

    if (enableRamp) {
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        // เร่งช่วงแรก
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        // ชะลอช่วงสุดท้าย
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    // ถ้า enableRamp = false → ใช้ความเร็วคงที่ตลอดทาง
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

  float speed_scale = speed_scale_bb_g_normal;  // ปรับได้ด้วย set_speed_scale_bb_g()

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  // ตัดสินใจว่าใช้ Ramp หรือไม่
  bool enableRamp = (distance_cm >= 30.0);

  // ถ้าระยะสั้นมาก (< 30) → ไม่ใช้ Ramp + ปรับ speed_scale
  if (!enableRamp) {
    speed_scale = speed_scale_bb_g;
  }
  if (!useDirectionG) SetRobotAngle();  // เซ็ตค่าปัจจุบัน

  while (true) {
    // คำนวณระยะทาง
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0;
    traveled_distance += (base_speed * speed_scale) * delta_time;
    last_time = current_time;

    float remaining_cm = distance_cm - traveled_distance;

    if (remaining_cm <= 0.8f) break;

    // ====================== คำนวณความเร็ว ======================
    float target_speed = base_speed;

    if (enableRamp) {  // ใช้ Ramp เฉพาะระยะยาว (>=30 cm)
      if (traveled_distance < ACCEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
      } else if (remaining_cm < DECEL_DISTANCE_CM) {
        target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
      }
    }
    // ถ้า !enableRamp → ใช้ความเร็วคงที่ตลอดทาง (base_speed)

    RunGB(target_speed, target_speed);
  }
}

void FFcmGS(int Speed, float distance_cm, char select) {
  FFcmGS(Speed, distance_cm);
  TrackSelectG(Speed, select);
}

void BBcmGS(int Speed, float distance_cm, char select) {
  BBcmGS(Speed, distance_cm);
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


// =====================================================================
//  13) วิ่งตรงตามทิศสัมบูรณ์
// =====================================================================

// ---------- ไม่มีคำสั่งเจอแยก ----------
void FFtimerG(int Speed, int totalTime, int direction) {
  SetDirectionG(direction);
  FFtimerG(Speed, totalTime);
}

void BBtimerG(int Speed, int totalTime, int direction) {
  SetDirectionG(direction);
  BBtimerG(Speed, totalTime);
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

// ---------- มีคำสั่งเจอแยก ----------
void FFBG(int Speed, char select, int direction) {
  SetDirectionG(direction);
  FFBG(Speed, select);
}

void BBBG(int Speed, char select, int direction) {
  SetDirectionG(direction);
  BBBG(Speed, select);
}

void FFtimerG(int Speed, int totalTime, char select, int direction) {
  FFtimerG(Speed, totalTime, direction);
  TrackSelectG(Speed, select);
}

void BBtimerG(int Speed, int totalTime, char select, int direction) {
  BBtimerG(Speed, totalTime, direction);
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

void FFcmG(int Speed, float distance_cm, char select, int direction) {
  FFcmG(Speed, distance_cm, direction);
  TrackSelectG(Speed, select);
}

void BBcmG(int Speed, float distance_cm, char select, int direction) {
  BBcmG(Speed, distance_cm, direction);
  TrackSelectGB(Speed, select);
}

// =====================================================================
//  14) วิ่งตามเส้นจนหันถึงทิศ (FFdir / BBdir)
// =====================================================================
// วิ่งตามเส้น (PIDF / PIDB) ไปเรื่อย ๆ จนมุมไจโรถึงทิศสัมบูรณ์ (0 / 90 / 180 / 270 อ้างอิงตอน resetAngles)
// ใช้กับเส้นโค้ง เช่น FFdir(50, 270); = วิ่งตามเส้นโค้งไปจนหุ่นหันไปทิศ 270
// ไม่มี select: จบแล้วมอเตอร์ยังหมุนอยู่ ต่อคำสั่งถัดไปได้ทันที

float line_dir_threshold = 5.0f;  // ถือว่าถึงทิศเมื่อ |error| <= ค่านี้ (องศา)

void set_line_dir_threshold(float thr) {
  line_dir_threshold = thr;
}

// true = มุมปัจจุบันถึงทิศ direction แล้ว (อยู่ในช่วง threshold หรือหมุนเลยเป้าไปแล้ว)
static bool reachedDirection(int direction, float &previous_error) {
  float error = wrap180(fmodf((float)direction, 360.0f) - angleRead());
  // มุมอาจกระโดดข้ามช่วง threshold ตอนเลี้ยวเร็ว จึงถือว่าถึงเมื่อ error เปลี่ยนเครื่องหมายด้วย
  // (|error| < 90 กันกรณี error กระโดดจาก +180 เป็น -180 ตอน wrap)
  bool crossed = fabsf(error) < 90.0f && ((error > 0 && previous_error < 0) || (error < 0 && previous_error > 0));
  previous_error = error;
  return fabsf(error) <= line_dir_threshold || crossed;
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Front / PID_KD_Front)
static void FFdir_pid(int Speed, int direction, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  float previous_error = wrap180(fmodf((float)direction, 360.0f) - angleReadFresh());
  while (!reachedDirection(direction, previous_error)) PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
}

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Back / PID_KD_Back)
static void BBdir_pid(int Speed, int direction, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  float previous_error = wrap180(fmodf((float)direction, 360.0f) - angleReadFresh());
  while (!reachedDirection(direction, previous_error)) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
}

void FFdir(int Speed, int direction) {
  FFdir_pid(Speed, direction, -1, -1);
}

void BBdir(int Speed, int direction) {
  BBdir_pid(Speed, direction, -1, -1);
}

void FFdir(int Speed, int direction, char select) {
  FFdir(Speed, direction);
  TrackSelectF(Speed, select);
}

void BBdir(int Speed, int direction, char select) {
  BBdir(Speed, direction);
  TrackSelectB(Speed, select);
}

// --- กำหนด Kp / Kd เอง: (speed, kp, [kd,] ทิศ [, select]) ---
void FFdir(int Speed, float Kp, int direction) {
  FFdir_pid(Speed, direction, Kp, -1);
}

void BBdir(int Speed, float Kp, int direction) {
  BBdir_pid(Speed, direction, Kp, -1);
}

void FFdir(int Speed, float Kp, int direction, char select) {
  FFdir_pid(Speed, direction, Kp, -1);
  TrackSelectF(Speed, select);
}

void BBdir(int Speed, float Kp, int direction, char select) {
  BBdir_pid(Speed, direction, Kp, -1);
  TrackSelectB(Speed, select);
}

void FFdir(int Speed, float Kp, float Kd, int direction) {
  FFdir_pid(Speed, direction, Kp, Kd);
}

void BBdir(int Speed, float Kp, float Kd, int direction) {
  BBdir_pid(Speed, direction, Kp, Kd);
}

void FFdir(int Speed, float Kp, float Kd, int direction, char select) {
  FFdir_pid(Speed, direction, Kp, Kd);
  TrackSelectF(Speed, select);
}

void BBdir(int Speed, float Kp, float Kd, int direction, char select) {
  BBdir_pid(Speed, direction, Kp, Kd);
  TrackSelectB(Speed, select);
}

// --- วิ่งโค้ง CirCleL = เบี่ยงซ้าย (set_position_l), CirCleR = เบี่ยงขวา (set_position_r) ---

void FFdirCirCleL(int Speed, int direction) {
  int temp = set_position;
  set_position = set_position_l;
  FFdir(Speed, direction);
  set_position = temp;
}

void FFdirCirCleR(int Speed, int direction) {
  int temp = set_position;
  set_position = set_position_r;
  FFdir(Speed, direction);
  set_position = temp;
}

void BBdirCirCleL(int Speed, int direction) {
  int temp = set_position;
  set_position = set_position_l;
  BBdir(Speed, direction);
  set_position = temp;
}

void BBdirCirCleR(int Speed, int direction) {
  int temp = set_position;
  set_position = set_position_r;
  BBdir(Speed, direction);
  set_position = temp;
}

void FFdirCirCleL(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_position_l;
  FFdir(Speed, direction, select);
  set_position = temp;
}

void FFdirCirCleR(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_position_r;
  FFdir(Speed, direction, select);
  set_position = temp;
}

void BBdirCirCleL(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_position_l;
  BBdir(Speed, direction, select);
  set_position = temp;
}

void BBdirCirCleR(int Speed, int direction, char select) {
  int temp = set_position;
  set_position = set_position_r;
  BBdir(Speed, direction, select);
  set_position = temp;
}

// --- *dirCirCleL/R แบบกำหนด Kp / Kd เอง: (speed, kp, [kd,] ทิศ [, select]) ---

void FFdirCirCleL(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_position_l;
  FFdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void FFdirCirCleR(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_position_r;
  FFdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void BBdirCirCleL(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_position_l;
  BBdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void BBdirCirCleR(int Speed, float Kp, float Kd, int direction) {
  int temp = set_position;
  set_position = set_position_r;
  BBdir(Speed, Kp, Kd, direction);
  set_position = temp;
}

void FFdirCirCleL(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_position_l;
  FFdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void FFdirCirCleR(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_position_r;
  FFdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void BBdirCirCleL(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_position_l;
  BBdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void BBdirCirCleR(int Speed, float Kp, float Kd, int direction, char select) {
  int temp = set_position;
  set_position = set_position_r;
  BBdir(Speed, Kp, Kd, direction, select);
  set_position = temp;
}

void FFdirCirCleL(int Speed, float Kp, int direction) {
  FFdirCirCleL(Speed, Kp, -1.0f, direction);
}

void FFdirCirCleR(int Speed, float Kp, int direction) {
  FFdirCirCleR(Speed, Kp, -1.0f, direction);
}

void BBdirCirCleL(int Speed, float Kp, int direction) {
  BBdirCirCleL(Speed, Kp, -1.0f, direction);
}

void BBdirCirCleR(int Speed, float Kp, int direction) {
  BBdirCirCleR(Speed, Kp, -1.0f, direction);
}

void FFdirCirCleL(int Speed, float Kp, int direction, char select) {
  FFdirCirCleL(Speed, Kp, -1.0f, direction, select);
}

void FFdirCirCleR(int Speed, float Kp, int direction, char select) {
  FFdirCirCleR(Speed, Kp, -1.0f, direction, select);
}

void BBdirCirCleL(int Speed, float Kp, int direction, char select) {
  BBdirCirCleL(Speed, Kp, -1.0f, direction, select);
}

void BBdirCirCleR(int Speed, float Kp, int direction, char select) {
  BBdirCirCleR(Speed, Kp, -1.0f, direction, select);
}
