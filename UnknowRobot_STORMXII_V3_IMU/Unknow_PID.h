#ifndef UNKNOW_PID_H
#define UNKNOW_PID_H

#include "Unknow_Sensor.h"
#include "Unknow_Motor.h"
#include "Unknow_Buzzer.h"

/* ==================================================================================================
 *  Unknow_PID.h — วิ่งตามเส้นด้วย PID / เลี้ยว / หมุน ด้วยเซนเซอร์เส้น
 *
 *   1) ตัวแปร / ค่าตั้ง (Config)
 *   2) อ่านตำแหน่งเส้น
 *   3) PID หลัก
 *   4) วิ่งตามเวลา / ระยะทาง
 *   5) เข้ากลางหุ่น / เข้าเส้น
 *   6) เลี้ยว / หมุน ด้วยเซนเซอร์เส้น
 *   7) คำสั่งเจอแยก (TrackSelectF / TrackSelectB)
 *   8) วิ่งจนเจอแยก (FFC / FFL / FFR ... และฝั่ง BB)
 *   9) วิ่งกินโค้ง (CirCle)
 *  10) ออกตัว / เข้าเส้นชัย
 *  11) ปรับสมดุลหุ่นยนต์ (Balance)
 *  12) Serial debug
 * ================================================================================================== */

// อยู่ใน Unknow_IMU.h ซึ่งถูก include หลังไฟล์นี้ (TrackSelectF / TrackSelectB เรียกใช้)
void SetFG(int totalTime);
void SetBG(int totalTime);


/* ==================================================================================================
 *  1) ตัวแปร / ค่าตั้ง (Config)
 * ================================================================================================== */

float PID_KP, PID_KD;
int LastError_F, LastError_B;
int Error_F = 0, Error_B = 0;

// ---------- ความเร็วเข้ากลางหุ่น / เลี้ยว ----------
int tct, bct, tspd;
int tctL, tctR, bctL, bctR;
int LTurnSpdL, LTurnSpdR, TurnDelayL;
int RTurnSpdL, RTurnSpdR, TurnDelayR;
int LChTurn = 5, RChTurn = 10;
int LChTurnB = 5, RChTurnB = 10;
int tct_delay_break = 0;
int bct_delay_break = 0;
float slow_kp_f = 0.014, slow_kd_f = 0.14;
float slow_kp_b = 0.014, slow_kd_b = 0.14;

// ---------- ตำแหน่งเส้น / ช่วงเซนเซอร์ ----------
int set_position = 7500;
int set_position_l = 1500;
int set_position_r = 13500;
int setsensortracklineL = 0, setsensortracklineR = 15;
int line_centor = 0;

// ---------- จำกัดกำลังล้อ (ModeSpdPID) ----------
int MaxSpeed = 100;
int MinSpeed = -5;
int ModePidStatus = 0;

void SetRobotPID(float Kp, float Kd) {
  PID_KP = Kp;
  PID_KD = Kd;
}

// ความเร็วเดินเข้ากลางหุ่น (ToCenter / BackCenter / ToFront / ToBack)
void SetToCenterSpeed(int tctv) {
  tct = tctv;
  bct = tctv;
  BaseSpeed = tctv;
  InitialSpeed();
  tctL = LeftBaseSpeed;
  tctR = RightBaseSpeed;
  bctL = BackLeftBaseSpeed;
  bctR = BackRightBaseSpeed;

  slow_kp_f = PID_KP_Front;
  slow_kd_f = PID_KD_Front;
  slow_kp_b = PID_KP_Back;
  slow_kd_b = PID_KD_Back;

  tct_delay_break = delay_break_f;
  bct_delay_break = delay_break_b;
}

// เซนเซอร์ที่หยุดเลี้ยว TurnLeft (F[0..l]) / TurnRight (F[15..r])
void SetSensorTurnLeftRight(int l, int r) {
  LChTurn = l;
  RChTurn = r;
}

// เซนเซอร์ที่หยุดเลี้ยว TurnLeft_B (B[0..l]) / TurnRight_B (B[15..r])
void SetSensorTurnLeftRight_B(int l, int r) {
  LChTurnB = l;
  RChTurnB = r;
}

// ความเร็วหมุน SpinL / SpinR เมื่อไม่ระบุความเร็ว
void SetTurnSpeed(int tspdv) {
  tspd = tspdv;
}

// ความเร็วล้อตอนเลี้ยวล้อเดียว + เวลาหน่วงก่อนเลี้ยว
void TurnSpeedLeft(int l, int r, int de) {
  LTurnSpdL = l;
  LTurnSpdR = r;
  TurnDelayL = de;
}

void TurnSpeedRight(int l, int r, int de) {
  RTurnSpdL = l;
  RTurnSpdR = r;
  TurnDelayR = de;
}

void ModeSpdPID(int moD, int maX, int miN) {
  ModePidStatus = moD;
  MaxSpeed = maX;
  MinSpeed = miN;
}

// ตำแหน่งเส้นตรงกลาง / ตอนกินโค้งซ้าย / ขวา (0-15000)
void set_position_line(int _pos) {
  set_position = constrain(_pos, 0, 15000);
}

void set_position_line_l(int _pos) {
  set_position_l = constrain(_pos, 0, 15000);
}

void set_position_line_r(int _pos) {
  set_position_r = constrain(_pos, 0, 15000);
}

// ช่วงเซนเซอร์ที่ใช้วิ่งตามเส้น เช่น (0, 15) / (2, 14) / (3, 13) / (4, 12)
void set_sensor_track_line(int L, int R) {
  setsensortracklineL = L;
  setsensortracklineR = R;
}

// วิธีเดินเข้ากลางหุ่น 0 = เดินตรงธรรมดา, 1 = เดินตามเส้น
void set_line_center(int x) {
  line_centor = x;
}


/* ==================================================================================================
 *  2) อ่านตำแหน่งเส้น (0 - 15000, กลาง = 7500)
 * ================================================================================================== */

int readPositionF(int Track, int noise) {
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateF();
  for (int i = setsensortracklineL; i <= setsensortracklineR; i++) {
    int values = F[i];
    if (values > Track) online = true;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2) return setsensortracklineL * 1000;
    else return setsensortracklineR * 1000;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

int readPositionB(int Track, int noise) {
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateB();
  for (int i = setsensortracklineL; i <= setsensortracklineR; i++) {
    int values = B[i];
    if (values > Track) online = true;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2) return setsensortracklineL * 1000;
    else return setsensortracklineR * 1000;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

// แบบ _none: หลุดเส้นแล้วคืนค่ากลาง (7500) — ใช้ตอนเดินเข้ากลางหุ่น
int readPositionF_none(int Track, int noise) {
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  ReadCalibrateF();
  for (int i = setsensortracklineL; i < setsensortracklineR; i++) {
    int values = F[i];
    if (values > Track) online = true;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) return 7500;
  return avg / sum;
}

int readPositionB_none(int Track, int noise) {
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  ReadCalibrateB();
  for (int i = setsensortracklineL; i < setsensortracklineR; i++) {
    int values = B[i];
    if (values > Track) online = true;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) return 7500;
  return avg / sum;
}


/* ==================================================================================================
 *  3) PID หลัก
 * ================================================================================================== */

// จำกัดกำลังล้อตามโหมด ModeSpdPID
void ClampPower(int &LeftPower, int &RightPower, int SpeedL, int SpeedR) {
  switch (ModePidStatus) {
    case 0:  // ..max, ล้อติดลบ → min
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = MinSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = MinSpeed;
      break;
    case 1:  // min..max
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < MinSpeed) LeftPower = MinSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < MinSpeed) RightPower = MinSpeed;
      break;
    case 2:  // -Speed..Speed
      if (LeftPower > SpeedL) LeftPower = SpeedL;
      if (LeftPower < -SpeedL) LeftPower = -SpeedL;
      if (RightPower > SpeedR) RightPower = SpeedR;
      if (RightPower < -SpeedR) RightPower = -SpeedR;
      break;
    case 3:  // ..max, ล้อติดลบ → -BaseSpeed
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = -BaseSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = -BaseSpeed;
      break;
    default:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = 0;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = 0;
  }
}

// ---------- 1 รอบควบคุม (เรียกในลูป) ----------
void PIDF(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos = readPositionF(250, 50);
  int Error = Pos - set_position;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_F));
  LastError_F = Error;
  int LeftPower = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  ClampPower(LeftPower, RightPower, SpeedL, SpeedR);
  Motor(LeftPower, RightPower);
  delayMicroseconds(80);
}

void PIDB(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos = readPositionB(250, 50);
  int Error = Pos - set_position;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_B));
  LastError_B = Error;
  int LeftPower = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  ClampPower(LeftPower, RightPower, SpeedL, SpeedR);
  Motor(-LeftPower, -RightPower);
  delayMicroseconds(80);
}

// ---------- แบบ _none: ใช้ตอนเดินเข้ากลางหุ่น (จำกัดกำลัง -100..100) ----------
void PIDF_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos;
  ReadCalibrateF();
  if (F[7] > Ref && F[8] > Ref) Pos = 7500;
  else Pos = readPositionF_none(250, 50);
  int Error = Pos - set_position;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_F));
  LastError_F = Error;
  int LeftPower = constrain(SpeedL + PID_Value, -100, 100);
  int RightPower = constrain(SpeedR - PID_Value, -100, 100);
  Motor(LeftPower, RightPower);
  delayMicroseconds(80);
}

void PIDB_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos;
  ReadCalibrateB();
  if (B[7] > Ref && B[8] > Ref) Pos = 7500;
  else Pos = readPositionB_none(250, 50);
  int Error = Pos - set_position;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_B));
  LastError_B = Error;
  int LeftPower = constrain(SpeedL + PID_Value, -100, 100);
  int RightPower = constrain(SpeedR - PID_Value, -100, 100);
  Motor(-LeftPower, -RightPower);
  delayMicroseconds(80);
}


/* ==================================================================================================
 *  4) วิ่งตามเวลา / ระยะทาง (ไม่จับแยกระหว่างทาง)
 * ================================================================================================== */

void FFtimer(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
}

void BBtimer(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
}

void FFcm(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.5;  // ค่าที่คำนวณจากการวัดจริง

  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;
      if (traveled_distance >= distance) break;
    }
  }
}

void BBcm(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();
  float speed_scale = 1.5;  // ค่าที่คำนวณจากการวัดจริง

  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;
      if (traveled_distance >= distance) break;
    }
  }
}


/* ==================================================================================================
 *  5) เข้ากลางหุ่น / เข้าเส้น   (ความเร็วจาก SetToCenterSpeed, วิธีเดินจาก set_line_center)
 * ================================================================================================== */

// ---------- เดินหน้า 1 รอบควบคุม: ตรงธรรมดา หรือ ตามเส้น ----------
void ToCenterStepF() {
  if (line_centor == 0) Motor(tctL, tctR);
  else PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
}

void ToCenterStartF() {
  if (line_centor == 0) {
    Motor(tctL, tctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
  }
}

// เดินหน้าจนเซนเซอร์กลางซ้าย "หรือ" ขวา เจอเส้น
void ToCenter() {
  BZon();
  ToCenterStartF();
  while (1) {
    ToCenterStepF();
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) break;
  }
  Motor(-tctL, -tctR);
  delay(5);
  MotorStop();
  BZoff();
}

// เดินหน้าจนเซนเซอร์กลางซ้าย เจอเส้น
void ToCenterL() {
  BZon();
  ToCenterStartF();
  while (1) {
    ToCenterStepF();
    ReadCalibrateC();
    if (C[CCL] >= RefC) break;
  }
  Motor(-tctL, -tctR);
  delay(5);
  MotorStop();
  BZoff();
}

// เดินหน้าจนเซนเซอร์กลางขวา เจอเส้น
void ToCenterR() {
  BZon();
  ToCenterStartF();
  while (1) {
    ToCenterStepF();
    ReadCalibrateC();
    if (C[CCR] >= RefC) break;
  }
  Motor(-tctL, -tctR);
  delay(5);
  MotorStop();
  BZoff();
}

// ถอยหลังจนเซนเซอร์กลางซ้าย "หรือ" ขวา เจอเส้น
void BackCenter() {
  BZon();
  if (line_centor == 0) {
    Motor(-bctL, -bctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
  }
  while (1) {
    if (line_centor == 0) Motor(-bctL, -bctR);
    else PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) break;
  }
  Motor(bctL, bctR);
  delay(5);
  MotorStop();
  BZoff();
}

// เดินหน้าตามเส้นจนเซนเซอร์หน้า F[3] หรือ F[12] เจอเส้นขวาง
void ToFront() {
  while (1) {
    PIDF(tctL, tctR, slow_kp_f, slow_kd_f);
    ReadCalibrateF();
    if (F[3] > Ref || F[12] > Ref) break;
  }
}

// ถอยหลังตามเส้นจนเซนเซอร์หลัง B[3] หรือ B[12] เจอเส้นขวาง (คู่กับ ToFront)
void ToBack() {
  while (1) {
    PIDB(bctL, bctR, slow_kp_b, slow_kd_b);
    ReadCalibrateB();
    if (B[3] > Ref || B[12] > Ref) break;
  }
}

void BackToFront() {
  ToBack();
}


/* ==================================================================================================
 *  6) เลี้ยว / หมุน ด้วยเซนเซอร์เส้น   (ไม่ระบุ Speed = ใช้ค่าจาก SetTurnSpeed)
 * ================================================================================================== */

// ---------- จัดหุ่นให้ตรงเส้นอยู่กับที่ด้วย PID (ความเร็วฐาน 0) เป็นเวลา totalTime ms ----------
void LF(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(0, 0, 0.015, 0.10);
  MotorStop();
}

void LB(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(0, 0, 0.015, 0.10);
  MotorStop();
}

// ---------- เลี้ยวล้อเดียว: หมุนจนเซนเซอร์เจอเส้นไล่ทีละตัว ----------
// TurnLeft: F[0..LChTurn]   TurnRight: F[15..RChTurn]
void TurnLeft() {
  for (int i = 0; i <= LChTurn; i++) {
    do {
      Motor(-LTurnSpdL, LTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
  MotorStop();
}

void TurnRight() {
  for (int i = 15; i >= RChTurn; i--) {
    do {
      Motor(RTurnSpdL, -RTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
  MotorStop();
}

// TurnLeft_B: B[0..LChTurnB]   TurnRight_B: B[15..RChTurnB]
void TurnLeft_B() {
  for (int i = 0; i <= LChTurnB; i++) {
    do {
      Motor(-LTurnSpdL, LTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  MotorStop();
}

void TurnRight_B() {
  for (int i = 15; i >= RChTurnB; i--) {
    do {
      Motor(RTurnSpdL, -RTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  MotorStop();
}

// ---------- หมุนอยู่กับที่ ----------
// เลือกเซนเซอร์หยุดของ SpinL ตามความเร็ว: ยิ่งหมุนเร็ว ยิ่งหยุดที่เซนเซอร์ขอบนอก (เผื่อระยะเหวี่ยง)
// SpinR ใช้ฝั่งตรงข้าม คือ 15 - ค่านี้
int SpinStopSensor(int Speed) {
  if (Speed >= 80) return 2;
  else if (Speed <= 60) return 6;
  else if (Speed <= 70) return 4;
  else return 2;
}

// หมุนจนเซนเซอร์ขอบนอกพ้นเส้นเดิม แล้วไล่เซนเซอร์ทีละตัวจากขอบนอกเข้าหาเซนเซอร์หยุดจนเจอเส้น
// (ทำซ้ำ times รอบ = ข้ามเส้น) จากนั้นเบรกและจัดตรงเส้นด้วย LF() / LB() เป็นเวลา Speed ms
void SpinLTimes(int Speed, int times) {
  MotorStop();
  int stopIdx = SpinStopSensor(Speed);
  while (1) {
    ReadCalibrateF();
    Motor(-Speed, Speed);
    if (F[0] <= Ref) break;
  }
  for (int n = 0; n < times; n++) {
    for (int i = 0; i <= stopIdx; i++) {
      do {
        Motor(-Speed, Speed);
        delayMicroseconds(80);
        ReadCalibrateF();
      } while (F[i] <= Ref);
    }
  }
  Motor(Speed, -Speed);
  delay(5);
  LF(Speed);
  MotorStop();
}

void SpinRTimes(int Speed, int times) {
  MotorStop();
  int stopIdx = 15 - SpinStopSensor(Speed);
  while (1) {
    ReadCalibrateF();
    Motor(Speed, -Speed);
    if (F[15] <= Ref) break;
  }
  for (int n = 0; n < times; n++) {
    for (int i = 15; i >= stopIdx; i--) {
      do {
        Motor(Speed, -Speed);
        delayMicroseconds(80);
        ReadCalibrateF();
      } while (F[i] <= Ref);
    }
  }
  Motor(-Speed, Speed);
  delay(5);
  LF(Speed);
  MotorStop();
}

void SpinL_BTimes(int Speed, int times) {
  MotorStop();
  int stopIdx = SpinStopSensor(Speed);
  while (1) {
    ReadCalibrateB();
    Motor(-Speed, Speed);
    if (B[0] <= Ref) break;
  }
  for (int n = 0; n < times; n++) {
    for (int i = 0; i <= stopIdx; i++) {
      do {
        Motor(-Speed, Speed);
        delayMicroseconds(80);
        ReadCalibrateB();
      } while (B[i] <= Ref);
    }
  }
  Motor(Speed, -Speed);
  delay(5);
  LB(Speed);
  MotorStop();
}

void SpinR_BTimes(int Speed, int times) {
  MotorStop();
  int stopIdx = 15 - SpinStopSensor(Speed);
  while (1) {
    ReadCalibrateB();
    Motor(Speed, -Speed);
    if (B[15] <= Ref) break;
  }
  for (int n = 0; n < times; n++) {
    for (int i = 15; i >= stopIdx; i--) {
      do {
        Motor(Speed, -Speed);
        delayMicroseconds(80);
        ReadCalibrateB();
      } while (B[i] <= Ref);
    }
  }
  Motor(-Speed, Speed);
  delay(5);
  LB(Speed);
  MotorStop();
}

// ---------- เซนเซอร์หน้า ----------
void SpinL(int Speed) {
  SpinLTimes(Speed, 1);
}
void SpinL() {
  SpinL(tspd);
}

void SpinR(int Speed) {
  SpinRTimes(Speed, 1);
}
void SpinR() {
  SpinR(tspd);
}

// หมุนข้ามเส้นแรกไปหยุดเส้นที่ 2 (แยกถี่ติดกัน)
void SpinL2(int Speed) {
  SpinLTimes(Speed, 2);
}
void SpinL2() {
  SpinL2(tspd);
}

void SpinR2(int Speed) {
  SpinRTimes(Speed, 2);
}
void SpinR2() {
  SpinR2(tspd);
}

// ---------- เซนเซอร์หลัง ----------
void SpinL_B(int Speed) {
  SpinL_BTimes(Speed, 1);
}
void SpinL_B() {
  SpinL_B(tspd);
}

void SpinR_B(int Speed) {
  SpinR_BTimes(Speed, 1);
}
void SpinR_B() {
  SpinR_B(tspd);
}

void SpinL2_B(int Speed) {
  SpinL_BTimes(Speed, 2);
}
void SpinL2_B() {
  SpinL2_B(tspd);
}

void SpinR2_B(int Speed) {
  SpinR_BTimes(Speed, 2);
}
void SpinR2_B() {
  SpinR2_B(tspd);
}


/* ==================================================================================================
 *  7) คำสั่งเจอแยก (select)   — ตัวพิมพ์ใหญ่ส่วนใหญ่ = ToFront() / ToBack() ก่อน แล้วทำเหมือนตัวพิมพ์เล็ก
 * ================================================================================================== */

// วิ่งตรง (ไม่ตามเส้น) จนเซนเซอร์ F[3] และ F[12] พ้นเส้นขวาง
void CrossLineF(int spdL, int spdR) {
  while (1) {
    Motor(spdL, spdR);
    delayMicroseconds(80);
    ReadCalibrateF();
    if (F[3] < Ref && F[12] < Ref) break;
  }
}

// ถอยตรง (ไม่ตามเส้น) จนเซนเซอร์ B[3] และ B[12] พ้นเส้นขวาง — ใส่ค่ามอเตอร์ติดลบ
void CrossLineB(int spdL, int spdR) {
  while (1) {
    Motor(spdL, spdR);
    delayMicroseconds(80);
    ReadCalibrateB();
    if (B[3] < Ref && B[12] < Ref) break;
  }
}

// ---------- วิ่งตามเส้นไปข้างหน้า ----------
void TrackSelectF(int spd, char x) {
  switch (x) {
    case 's':  // เบรกหยุด
      Motor(-spd, -spd);
      delay(delay_break_f);
      MotorStop();
      break;
    case 'S':  // ToFront แล้วเบรกหยุด
      ToFront();
      Motor(-tctL, -tctR);
      delay(tct_delay_break);
      MotorStop();
      break;

    case 'p':  // วิ่งตรงผ่านแยก
      BZon();
      CrossLineF(spd, spd);
      delay(5);
      CrossLineF(spd, spd);
      BZoff();
      break;
    case 'P':  // ToFront แล้ววิ่งตรงผ่านแยก
      ToFront();
      BZon();
      for (int n = 0; n < 2; n++) {
        CrossLineF(spd, spd);
        delay(5);
      }
      BZoff();
      break;

    case 'l':  // เข้ากลางหุ่น แล้วหมุนซ้าย
    case 'L':
      if (x == 'L') ToFront();
      ToCenter();
      SpinL();
      FFtimer(0, 8);
      break;
    case 'r':  // เข้ากลางหุ่น แล้วหมุนขวา
    case 'R':
      if (x == 'R') ToFront();
      ToCenter();
      SpinR();
      FFtimer(0, 8);
      break;

    case 'q':  // เลี้ยวซ้ายล้อเดียว
    case 'Q':
      if (x == 'Q') ToFront();
      BZon();
      for (int n = 0; n < 2; n++) CrossLineF(tctL, tctR);
      delay(TurnDelayL);
      BZoff();
      TurnLeft();
      FFtimer(0, 5);
      break;
    case 'e':  // เลี้ยวขวาล้อเดียว (รอเฉพาะ F[12] พ้นเส้น)
      BZon();
      for (int n = 0; n < 2; n++) {
        while (1) {
          Motor(tctL, tctR);
          delayMicroseconds(80);
          ReadCalibrateF();
          if (F[12] < Ref) break;
        }
      }
      delay(TurnDelayR);
      BZoff();
      TurnRight();
      FFtimer(0, 5);
      break;
    case 'E':  // ToFront แล้วเลี้ยวขวาล้อเดียว
      ToFront();
      BZon();
      for (int n = 0; n < 2; n++) CrossLineF(tctL, tctR);
      delay(TurnDelayR);
      BZoff();
      TurnRight();
      FFtimer(0, 5);
      break;

    case 'c':  // เข้ากลางหุ่น แล้วหยุด
    case 'C':
      if (x == 'C') ToFront();
      ToCenter();
      break;
    case 'a':  // เข้ากลางหุ่น แล้วหมุนซ้ายด้วยเซนเซอร์หลัง
    case 'A':
      if (x == 'A') ToFront();
      ToCenter();
      SpinL_B();
      BBtimer(0, 8);
      break;
    case 'd':  // เข้ากลางหุ่น แล้วหมุนขวาด้วยเซนเซอร์หลัง
    case 'D':
      if (x == 'D') ToFront();
      ToCenter();
      SpinR_B();
      BBtimer(0, 8);
      break;

    case 'g':  // ค้างมุมอยู่กับที่ นาน spd ms
    case 'G':
      if (x == 'G') ToFront();
      SetFG(spd);
      break;

    default:
      MotorStop();
  }
}

// ---------- วิ่งตามเส้นถอยหลัง ----------
void TrackSelectB(int spd, char x) {
  switch (x) {
    case 's':  // เบรกหยุด
    case 'S':
      if (x == 'S') ToBack();
      Motor(spd, spd);
      delay(delay_break_b);
      MotorStop(2);
      break;

    case 'p':  // ถอยผ่านแยก
    case 'P':
      for (int n = 0; n < 2; n++) CrossLineB(-spd, -spd);
      break;

    case 'l':  // ถอยเข้ากลางหุ่น แล้วหมุนซ้าย
    case 'L':
      if (x == 'L') ToBack();
      BackCenter();
      SpinL();
      FFtimer(0, 2);
      break;
    case 'r':  // ถอยเข้ากลางหุ่น แล้วหมุนขวา
    case 'R':
      if (x == 'R') ToBack();
      BackCenter();
      SpinR();
      FFtimer(0, 2);
      break;

    case 'c':  // ถอยเข้ากลางหุ่น แล้วหยุด
    case 'C':
      if (x == 'C') ToBack();
      BackCenter();
      break;
    case 'a':  // ถอยเข้ากลางหุ่น แล้วหมุนซ้ายด้วยเซนเซอร์หลัง
    case 'A':
      if (x == 'A') ToBack();
      BackCenter();
      SpinL_B();
      BBtimer(0, 8);
      break;
    case 'd':  // ถอยเข้ากลางหุ่น แล้วหมุนขวาด้วยเซนเซอร์หลัง
    case 'D':
      if (x == 'D') ToBack();
      BackCenter();
      SpinR_B();
      BBtimer(0, 8);
      break;

    case 'e':  // เลี้ยวถอยหลังล้อเดียว TurnLeft_B
    case 'E':
      if (x == 'E') ToBack();
      for (int n = 0; n < 2; n++) CrossLineB(-bctL, -bctR);
      delay(TurnDelayL);
      TurnLeft_B();
      BBtimer(0, 5);
      break;
    case 'q':  // เลี้ยวถอยหลังล้อเดียว TurnRight_B
    case 'Q':
      if (x == 'Q') ToBack();
      for (int n = 0; n < 2; n++) CrossLineB(-bctL, -bctR);
      delay(TurnDelayR);
      TurnRight_B();
      BBtimer(0, 5);
      break;

    case 'g':  // ค้างมุมอยู่กับที่ นาน spd ms
    case 'G':
      if (x == 'G') ToBack();
      SetBG(spd);
      break;

    default:
      MotorStop();
  }
}

// ---------- คำสั่งวิ่งตามเวลา / ระยะทาง + select ----------
void FFtimer(int Speed, int totalTime, char select) {
  FFtimer(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void BBtimer(int Speed, int totalTime, char select) {
  BBtimer(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void FFtimer(int Speed, float kp, float kd, int totalTime, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
  TrackSelectF(Speed, select);
}

void BBtimer(int Speed, float kp, float kd, int totalTime, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
  TrackSelectB(Speed, select);
}

void FFcm(int Speed, float distance, char select) {
  FFcm(Speed, distance);
  TrackSelectF(Speed, select);
}

void BBcm(int Speed, float distance, char select) {
  BBcm(Speed, distance);
  TrackSelectB(Speed, select);
}


/* ==================================================================================================
 *  8) วิ่งจนเจอแยก
 *     แบบ (Speed, select)         ใช้ค่า KP / KD จากตาราง Set_KP_KD ตามความเร็ว
 *     แบบ (Speed, kp, kd, select) กำหนด KP / KD เอง
 * ================================================================================================== */

// ---------- เซนเซอร์คู่กลาง (แยกแคบ-กลาง) ----------
void FFC(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if ((F[5] >= Ref && F[10] >= Ref) || (F[4] >= Ref && F[11] >= Ref) ||
        (F[3] >= Ref && F[12] >= Ref) || (F[5] >= Ref && F[6] >= Ref && F[9] >= Ref && F[10] >= Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBC(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if ((B[5] >= Ref && B[10] >= Ref) || (B[4] >= Ref && B[11] >= Ref) ||
        (B[3] >= Ref && B[12] >= Ref) || (B[5] >= Ref && B[6] >= Ref && B[9] >= Ref && B[10] >= Ref)) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- เซนเซอร์ริมสุด F[0] และ F[15] พร้อมกัน (แยกกว้างเต็มบอร์ด) ----------
void FFC2(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if (F[0] >= Ref && F[15] >= Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBC2(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[0] >= Ref && B[15] >= Ref) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- เซนเซอร์คู่ซ้าย ----------
void FFL(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if ((F[1] > Ref && F[5] > Ref) || (F[2] > Ref && F[6] > Ref) || (F[3] > Ref && F[7] > Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBL(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if ((B[1] > Ref && B[5] > Ref) || (B[2] > Ref && B[6] > Ref) || (B[3] > Ref && B[7] > Ref)) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- เซนเซอร์คู่ขวา ----------
void FFR(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if ((F[14] > Ref && F[10] > Ref) || (F[13] > Ref && F[9] > Ref) || (F[12] > Ref && F[8] > Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBR(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if ((B[14] > Ref && B[10] > Ref) || (B[13] > Ref && B[9] > Ref) || (B[12] > Ref && B[8] > Ref)) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- เซนเซอร์ริมซ้ายสุด (ตัวที่ 0) ----------
void FFL0(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBL0(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- เซนเซอร์ริมขวาสุด (ตัวที่ 15) ----------
void FFR15(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if (F[15] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBR15(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[15] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- เลือกตำแหน่งเซนเซอร์เอง ----------
void FFNUM(int Speed, float kp, float kd, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if (F[numm] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBNUM(int Speed, float kp, float kd, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[numm] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- วิ่งจนเซนเซอร์ F[3..12] / B[3..12] เป็นขาวทั้งหมด ----------
void FFWhite(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if (F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref &&
        F[8] < Ref && F[9] < Ref && F[10] < Ref && F[11] < Ref && F[12] < Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBWhite(int Speed, float kp, float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref &&
        B[8] < Ref && B[9] < Ref && B[10] < Ref && B[11] < Ref && B[12] < Ref) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- แบบใช้ค่า KP / KD จากตาราง (Set_KP_KD / Set_KP_KD_Back) ----------
void FFC(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFC(Speed, PID_KP_Front, PID_KD_Front, select);
}
void BBC(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBC(Speed, PID_KP_Back, PID_KD_Back, select);
}

void FFC2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFC2(Speed, PID_KP_Front, PID_KD_Front, select);
}
void BBC2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBC2(Speed, PID_KP_Back, PID_KD_Back, select);
}

void FFL(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFL(Speed, PID_KP_Front, PID_KD_Front, select);
}
void BBL(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBL(Speed, PID_KP_Back, PID_KD_Back, select);
}

void FFR(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFR(Speed, PID_KP_Front, PID_KD_Front, select);
}
void BBR(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBR(Speed, PID_KP_Back, PID_KD_Back, select);
}

void FFL0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFL0(Speed, PID_KP_Front, PID_KD_Front, select);
}
void BBL0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBL0(Speed, PID_KP_Back, PID_KD_Back, select);
}

void FFR15(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFR15(Speed, PID_KP_Front, PID_KD_Front, select);
}
void BBR15(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBR15(Speed, PID_KP_Back, PID_KD_Back, select);
}

void FFNUM(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFNUM(Speed, PID_KP_Front, PID_KD_Front, select, numm);
}
void BBNUM(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBNUM(Speed, PID_KP_Back, PID_KD_Back, select, numm);
}

void FFWhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFWhite(Speed, PID_KP_Front, PID_KD_Front, select);
}
void BBWhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBWhite(Speed, PID_KP_Back, PID_KD_Back, select);
}

// ---------- เดินตรง (ไม่ตามเส้น) จนเซนเซอร์ F[5..10] / B[5..10] เจอเส้นดำ ----------
void FFBlack(int SpeedL, int SpeedR, char select) {
  Move(SpeedL, SpeedR, 50);
  while (1) {
    Motor(SpeedL, SpeedR);
    ReadCalibrateF();
    if (F[5] > Ref || F[6] > Ref || F[7] > Ref || F[8] > Ref || F[9] > Ref || F[10] > Ref) break;
  }
  TrackSelectF(SpeedL, select);
}

void FFBlack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  FFBlack(LeftBaseSpeed, RightBaseSpeed, select);
}

void BBBlack(int SpeedL, int SpeedR, char select) {
  Move(-SpeedL, -SpeedR, 50);
  while (1) {
    Motor(-SpeedL, -SpeedR);
    ReadCalibrateB();
    if (B[5] > Ref || B[6] > Ref || B[7] > Ref || B[8] > Ref || B[9] > Ref || B[10] > Ref) break;
  }
  TrackSelectB(SpeedL, select);
}

void BBBlack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  BBBlack(BackLeftBaseSpeed, BackRightBaseSpeed, select);
}


/* ==================================================================================================
 *  9) วิ่งกินโค้ง (CirCle) — ใช้ตำแหน่งเส้น set_position_line_l / _r แทนเส้นกลางชั่วคราว
 * ================================================================================================== */

// ---------- จนเจอแยก (เซนเซอร์ริม F[0] / F[15], B[0] / B[15]) ----------
void FFCirCleL(int Speed, char select) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFL0(Speed, select);
  set_position = temp_position;
}

void FFCirCleR(int Speed, char select) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFR15(Speed, select);
  set_position = temp_position;
}

void BBCirCleL(int Speed, char select) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBL0(Speed, select);
  set_position = temp_position;
}

void BBCirCleR(int Speed, char select) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBR15(Speed, select);
  set_position = temp_position;
}

// ---------- ตามเวลา ----------
void FFtimerCirCleL(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFtimer(Speed, totalTime);
  set_position = temp_position;
}

void FFtimerCirCleR(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFtimer(Speed, totalTime);
  set_position = temp_position;
}

void BBtimerCirCleL(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBtimer(Speed, totalTime);
  set_position = temp_position;
}

void BBtimerCirCleR(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBtimer(Speed, totalTime);
  set_position = temp_position;
}

void FFtimerCirCleL(int Speed, int totalTime, char select) {
  FFtimerCirCleL(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void FFtimerCirCleR(int Speed, int totalTime, char select) {
  FFtimerCirCleR(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void BBtimerCirCleL(int Speed, int totalTime, char select) {
  BBtimerCirCleL(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void BBtimerCirCleR(int Speed, int totalTime, char select) {
  BBtimerCirCleR(Speed, totalTime);
  TrackSelectB(Speed, select);
}

// ---------- ตามระยะทาง ----------
void FFcmCirCleL(int Speed, int distance) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFcm(Speed, distance);
  set_position = temp_position;
}

void FFcmCirCleR(int Speed, int distance) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFcm(Speed, distance);
  set_position = temp_position;
}

void BBcmCirCleL(int Speed, int distance) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBcm(Speed, distance);
  set_position = temp_position;
}

void BBcmCirCleR(int Speed, int distance) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBcm(Speed, distance);
  set_position = temp_position;
}

void FFcmCirCleL(int Speed, int distance, char select) {
  FFcmCirCleL(Speed, distance);
  TrackSelectF(Speed, select);
}

void FFcmCirCleR(int Speed, int distance, char select) {
  FFcmCirCleR(Speed, distance);
  TrackSelectF(Speed, select);
}

void BBcmCirCleL(int Speed, int distance, char select) {
  BBcmCirCleL(Speed, distance);
  TrackSelectB(Speed, select);
}

void BBcmCirCleR(int Speed, int distance, char select) {
  BBcmCirCleR(Speed, distance);
  TrackSelectB(Speed, select);
}


/* ==================================================================================================
 *  10) ออกตัว / เข้าเส้นชัย
 * ================================================================================================== */

// เคลื่อนที่จนพ้นจุด Start (เซนเซอร์ริม F[0] และ F[15] พ้นเส้น)
void GoStart(int LeftSpeed, int RightSpeed) {
  Move(LeftSpeed, RightSpeed, 100);
  while (1) {
    ReadCalibrateF();
    Motor(LeftSpeed, RightSpeed);
    if (F[0] < 500 && F[15] < 500) break;
  }
}

void GoStart(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  GoStart(LeftBaseSpeed, RightBaseSpeed);
}

// เคลื่อนที่จนถึงจุด Finish (เซนเซอร์กลางทั้งคู่เจอเส้น)
void GoEnd(int LeftSpeed, int RightSpeed) {
  while (1) {
    ReadCalibrateC();
    Motor(LeftSpeed, RightSpeed);
    if (C[0] > 500 && C[1] > 500) {
      Move(LeftSpeed, RightSpeed, 100);
      break;
    }
  }
  MotorStop();
}

void GoEnd(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  GoEnd(LeftBaseSpeed, RightBaseSpeed);
}


/* ==================================================================================================
 *  11) ปรับสมดุลหุ่นยนต์ (Balance) — จัดหุ่นให้ตรงเส้นขวาง ทำซ้ำ Counter + 1 ครั้ง
 * ================================================================================================== */

// ใช้เซนเซอร์หน้า F[0] / F[15] (ขยับเดินหน้า)
void BalanceF(int Counter) {
  Move(-10, -10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(-10, -10, 80);
    while (1) {
      Motor(10, 10);
      ReadCalibrateF();
      if (F[0] > Ref) {
        while (1) {
          Motor(0, 5);
          ReadCalibrateF();
          if (F[15] > Ref) {
            MotorStop();
            break;
          }
        }
      }
      if (F[15] > Ref) {
        while (1) {
          Motor(5, 0);
          ReadCalibrateF();
          if (F[0] > Ref) {
            MotorStop();
            break;
          }
        }
      }
      if (F[0] > Ref && F[15] > Ref) {
        MotorStop();
        break;
      }
    }
    MotorStop();
    delay(50);
  }
}

// ใช้เซนเซอร์หลัง B[0] / B[15] (ขยับถอยหลัง)
void BalanceB(int Counter) {
  Move(10, 10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(10, 10, 80);
    while (1) {
      Motor(-12, -12);
      ReadCalibrateB();
      if (B[0] > Ref) {
        while (1) {
          Motor(0, -5);
          ReadCalibrateB();
          if (B[15] > Ref) {
            MotorStop();
            break;
          }
        }
      }
      if (B[15] > Ref) {
        while (1) {
          Motor(-5, 0);
          ReadCalibrateB();
          if (B[0] > Ref) {
            MotorStop();
            break;
          }
        }
      }
      if (B[0] > Ref && B[15] > Ref) {
        MotorStop();
        break;
      }
    }
    MotorStop();
    delay(50);
  }
}

// ใช้เซนเซอร์คู่กลาง C[0] / C[1] (ขยับเดินหน้า)
void BalanceFC(int Counter) {
  Move(-10, -10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(-10, -10, 80);
    while (1) {
      Motor(10, 10);
      ReadCalibrateC();
      if (C[0] > RefC) {
        while (1) {
          Motor(0, 5);
          ReadCalibrateC();
          if (C[1] > RefC) {
            MotorStop();
            break;
          }
        }
      }
      if (C[1] > RefC) {
        while (1) {
          Motor(5, 0);
          ReadCalibrateC();
          if (C[0] > RefC) {
            MotorStop();
            break;
          }
        }
      }
      if (C[0] > RefC && C[1] > RefC) {
        MotorStop();
        break;
      }
    }
    MotorStop();
    delay(50);
  }
}

// ใช้เซนเซอร์คู่กลาง C[0] / C[1] (ขยับถอยหลัง)
void BalanceBC(int Counter) {
  Move(10, 10, 50);
  for (int i = 0; i <= Counter; i++) {
    Move(10, 10, 80);
    while (1) {
      Motor(-12, -12);
      ReadCalibrateC();
      if (C[0] > RefC) {
        while (1) {
          Motor(0, -5);
          ReadCalibrateC();
          if (C[1] > RefC) {
            MotorStop();
            break;
          }
        }
      }
      if (C[1] > RefC) {
        while (1) {
          Motor(-5, 0);
          ReadCalibrateC();
          if (C[0] > RefC) {
            MotorStop();
            break;
          }
        }
      }
      if (C[0] > RefC && C[1] > RefC) {
        MotorStop();
        break;
      }
    }
    MotorStop();
    delay(50);
  }
}


/* ==================================================================================================
 *  12) Serial debug — พิมพ์ตำแหน่งเส้นออก Serial Monitor ต่อเนื่อง (ไม่ออกจากฟังก์ชัน)
 * ================================================================================================== */

void SerialPositionF() {
  while (1) {
    int pos = readPositionF(200, 50);
    Serial.print("Position F : ");
    Serial.println(pos);
    delay(100);
  }
}

void SerialPositionB() {
  while (1) {
    int pos = readPositionB(200, 50);
    Serial.print("Position B : ");
    Serial.println(pos);
    delay(100);
  }
}

void SerialPositionFB() {
  while (1) {
    int posF = readPositionF(200, 50);
    int posB = readPositionB(200, 50);
    Serial.print("Position F : ");
    Serial.print(posF);
    Serial.print("  |  Position B : ");
    Serial.println(posB);
    delay(100);
  }
}

#endif
