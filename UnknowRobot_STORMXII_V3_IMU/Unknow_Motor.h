/* =====================================================================
 *  Unknow_Motor.h — สั่งมอเตอร์ และตั้งค่าความเร็ว / PID / เวลาเบรก ตามช่วงความเร็ว
 *
 *  สารบัญ
 *     1) ขามอเตอร์
 *     2) ตัวแปร (Global Variables)
 *     3) ประกาศล่วงหน้า (Prototype)
 *     4) index ช่วงความเร็ว (SPD_10 .. SPD_100)
 *     5) สั่งมอเตอร์
 *     6) ตั้งค่าตามช่วงความเร็ว
 * ===================================================================== */

#ifndef UNKNOW_MOTOR_H
#define UNKNOW_MOTOR_H

#include <Arduino.h>


// =====================================================================
//  1) ขามอเตอร์
// =====================================================================

#define PH1 22
#define EN1 21
#define PH2 25
#define EN2 24


// =====================================================================
//  2) ตัวแปร (Global Variables)
// =====================================================================

// ---------- ชดเชยมอเตอร์ / ความเร็วฐาน ----------

// ชนิดตัวแปร: ความเร็ว -100..100 = int8_t | ค่าที่บวก/ลบกันแล้วอาจเกิน ±127 = int16_t | เวลา ms = uint16_t
int8_t BalanceMotorLeft, BalanceMotorRight;
int8_t BaseSpeed;
int16_t LeftBaseSpeed, RightBaseSpeed, BackLeftBaseSpeed, BackRightBaseSpeed;  // BaseSpeed - ค่า balance

// ---------- PID ตามช่วงความเร็ว ----------
float PID_KP_Front, PID_KD_Front;
float PID_KP_Back, PID_KD_Back;
float KP[10], KD[10];
float KP_Back[10], KD_Back[10];

// ---------- error ก่อนหน้าของ PID (รีเซ็ตใน InitialSpeed) ----------
int16_t LastError_F, LastError_B;            // ค่า error ก่อนหน้าของ PIDF / PIDB (รีเซ็ตใน InitialSpeed) ช่วง ±15000
int16_t LastError_F_none, LastError_B_none;  // ค่า error ก่อนหน้าของ PID แบบ _none

// ---------- ตาราง balance / เวลาเบรก ----------
int8_t L[10], R[10];
int8_t BL[10], BR[10];
uint16_t delay_break_f = 30;
uint16_t delay_break_b = 30;
uint16_t delay_break_f_table[10] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };
uint16_t delay_break_b_table[10] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };


// =====================================================================
//  3) ประกาศล่วงหน้า (Prototype)
// =====================================================================

void resetGyroRunError();  // อยู่ใน Unknow_IMU.h: รีเซ็ต previous_errorG / previous_errorGB


// =====================================================================
//  4) index ช่วงความเร็ว (SPD_10 .. SPD_100)
// =====================================================================

//  3) index ช่วงความเร็ว (SPD_10 .. SPD_100)

// กำหนด index ความเร็ว
#define SPD_10 0
#define SPD_20 1
#define SPD_30 2
#define SPD_40 3
#define SPD_50 4
#define SPD_60 5
#define SPD_70 6
#define SPD_80 7
#define SPD_90 8
#define SPD_100 9


// =====================================================================
//  5) สั่งมอเตอร์
// =====================================================================

//  4) สั่งมอเตอร์
void Motor1(int Pow) {
  analogWriteResolution(12);
  analogWriteFreq(20000);
  bool dir = (Pow >= 0);
  Pow = map(abs(Pow), 0, 100, 0, 4095);
  digitalWrite(PH1, dir);
  analogWrite(EN1, Pow);
}

void Motor2(int Pow) {
  analogWriteResolution(12);
  analogWriteFreq(20000);
  bool dir = (Pow >= 0);
  Pow = map(abs(Pow), 0, 100, 0, 4095);
  digitalWrite(PH2, dir);
  analogWrite(EN2, Pow);
}

void Motor(int leftspeed, int rightspeed) {
  Motor1(leftspeed + BalanceMotorLeft);
  Motor2(rightspeed + BalanceMotorRight);
}

void Move(int leftspeed, int rightspeed, int Movedelay) {
  Motor(leftspeed, rightspeed);
  delay(Movedelay);
}

void MotorStop() {
  digitalWrite(PH1, 1);
  analogWrite(EN1, 0);
  digitalWrite(PH2, 1);
  analogWrite(EN2, 0);
  delay(5);
}

void MotorStop(int StopDelay) {
  MotorStop();
  delay(StopDelay);
}

void Stop(int StopDelay) {
  MotorStop();
  delay(StopDelay);
}

void TestMotor() {
  Move(20, 0, 300);  //สั่งมอเตอร์ซ้าย
  MotorStop();       // หยุดการทำงานมอเตอร์
  delay(500);        // หน่วงเวลา 0.5 วินาที
  Move(0, 20, 300);  //สั่งมอเตอร์ขวา
  MotorStop();       // หยุดการทำงานมอเตอร์
}


// =====================================================================
//  6) ตั้งค่าตามช่วงความเร็ว
// =====================================================================

//  5) ตั้งค่าตามช่วงความเร็ว
void setBalanceSpeed(int ch, int spdL, int spdR) {
  L[ch] = spdL;
  R[ch] = spdR;
}

void setBalanceBackSpeed(int ch, int spdL, int spdR) {
  BL[ch] = spdL;
  BR[ch] = spdR;
}

void Set_KP_KD(int ch, float kp, float kd) {
  KP[ch] = kp;
  KD[ch] = kd;
}

void Set_KP_KD_Back(int ch, float kp, float kd) {
  KP_Back[ch] = kp;
  KD_Back[ch] = kd;
}

void SetDelayBreak(int ch, int delay_f, int delay_b) {
  delay_break_f_table[ch] = delay_f;
  delay_break_b_table[ch] = delay_b;
}

// เลือกชุดค่าความเร็ว/PID ตามช่วงของ BaseSpeed (ปัดขึ้นเป็นสิบ เช่น 35 ใช้ชุดของ 40)
// เดิมเป็น if/else 10 ชุดที่โครงสร้างเหมือนกันทุกอัน ต่างแค่ index ตาราง จึงรวมเป็นสูตรเดียว:
// ดัชนี 0-9 คำนวณจาก (BaseSpeed - 1) / 10 แล้วจำกัดไม่ให้เกิน SPD_100 (ผลลัพธ์เหมือนเดิมทุกกรณี)
void InitialSpeed() {
  int idx = constrain((BaseSpeed - 1) / 10, SPD_10, SPD_100);

  LeftBaseSpeed = BaseSpeed - L[idx];
  RightBaseSpeed = BaseSpeed - R[idx];
  BackLeftBaseSpeed = BaseSpeed - BL[idx];
  BackRightBaseSpeed = BaseSpeed - BR[idx];
  PID_KP_Front = KP[idx];  // forward PID
  PID_KD_Front = KD[idx];
  PID_KP_Back = KP_Back[idx];  // backward PID
  PID_KD_Back = KD_Back[idx];
  delay_break_f = delay_break_f_table[idx];
  delay_break_b = delay_break_b_table[idx];
  LastError_F = 0;  // รีเซ็ต D-term ไม่ให้ค้างจากคำสั่งก่อนหน้า
  LastError_B = 0;
  LastError_F_none = 0;
  LastError_B_none = 0;
  resetGyroRunError();  // previous_errorG / previous_errorGB = error ปัจจุบัน
}

#endif
