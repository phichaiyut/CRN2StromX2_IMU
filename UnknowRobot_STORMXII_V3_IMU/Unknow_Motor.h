#ifndef UNKNOW_MOTOR_H
#define UNKNOW_MOTOR_H

#include <Arduino.h>

/* ==================================================================================================
 *  Unknow_Motor.h — มอเตอร์ และตารางค่าตามความเร็ว
 *
 *   1) ขามอเตอร์ / สั่งมอเตอร์
 *   2) ตารางค่าตามความเร็ว (SPD_10 ... SPD_100)
 *   3) InitialSpeed — เลือกค่าจากตารางตาม BaseSpeed
 * ================================================================================================== */


/* ==================================================================================================
 *  1) ขามอเตอร์ / สั่งมอเตอร์   (ความเร็ว -100 ถึง 100)
 * ================================================================================================== */

#define PH1 22
#define EN1 21
#define PH2 25
#define EN2 24

int BalanceMotorLeft, BalanceMotorRight;  // ชดเชยความเร็วล้อซ้าย / ขวา ทุกคำสั่ง

// มอเตอร์ซ้าย
void Motor1(int Pow) {
  analogWriteResolution(12);
  analogWriteFreq(20000);
  bool dir = (Pow >= 0);
  Pow = map(abs(Pow), 0, 100, 0, 4095);
  digitalWrite(PH1, dir);
  analogWrite(EN1, Pow);
}

// มอเตอร์ขวา
void Motor2(int Pow) {
  analogWriteResolution(12);
  analogWriteFreq(20000);
  bool dir = (Pow >= 0);
  Pow = map(abs(Pow), 0, 100, 0, 4095);
  digitalWrite(PH2, dir);
  analogWrite(EN2, Pow);
}

// สั่งมอเตอร์ทันที ไม่หน่วงเวลา
void Motor(int leftspeed, int rightspeed) {
  Motor1(leftspeed + BalanceMotorLeft);
  Motor2(rightspeed + BalanceMotorRight);
}

// สั่งมอเตอร์ แล้วหน่วงเวลา Movedelay ms
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
  MotorStop(StopDelay);
}

// ทดสอบ: ล้อซ้ายหมุนไปหน้า แล้วตามด้วยล้อขวา
void TestMotor() {
  Move(20, 0, 300);
  MotorStop();
  delay(500);
  Move(0, 20, 300);
  MotorStop();
}


/* ==================================================================================================
 *  2) ตารางค่าตามความเร็ว (ตั้งใน Setting.ino)
 * ================================================================================================== */

// index ของตาราง (ความเร็ว 1-10 → SPD_10, 11-20 → SPD_20, ... , 91 ขึ้นไป → SPD_100)
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

int L[10], R[10];          // ชดเชยล้อ เดินหน้า
int BL[10], BR[10];        // ชดเชยล้อ ถอยหลัง
float KP[10], KD[10];      // PID เดินหน้า
float KP_Back[10], KD_Back[10];  // PID ถอยหลัง
int delay_break_f_table[10] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };  // เวลาเบรก เดินหน้า (ms)
int delay_break_b_table[10] = { 30, 30, 30, 30, 30, 30, 30, 30, 30, 30 };  // เวลาเบรก ถอยหลัง (ms)

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


/* ==================================================================================================
 *  3) InitialSpeed — ตั้ง BaseSpeed ก่อน แล้วเรียกเพื่อเลือกค่าจากตาราง
 * ================================================================================================== */

int BaseSpeed, LeftBaseSpeed, RightBaseSpeed, BackLeftBaseSpeed, BackRightBaseSpeed;
float PID_KP_Front, PID_KD_Front;
float PID_KP_Back, PID_KD_Back;
int delay_break_f = 30;
int delay_break_b = 30;

void InitialSpeed() {
  int idx = constrain((BaseSpeed - 1) / 10, SPD_10, SPD_100);  // ปัดขึ้นเป็นสิบ เช่น 35 ใช้ชุดของ 40

  LeftBaseSpeed = BaseSpeed - L[idx];
  RightBaseSpeed = BaseSpeed - R[idx];
  BackLeftBaseSpeed = BaseSpeed - BL[idx];
  BackRightBaseSpeed = BaseSpeed - BR[idx];

  PID_KP_Front = KP[idx];
  PID_KD_Front = KD[idx];
  PID_KP_Back = KP_Back[idx];
  PID_KD_Back = KD_Back[idx];

  delay_break_f = delay_break_f_table[idx];
  delay_break_b = delay_break_b_table[idx];
}

#endif
