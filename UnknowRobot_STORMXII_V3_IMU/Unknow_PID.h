/* =====================================================================
 *  Unknow_PID.h — วิ่งตามเส้นด้วย PID, เลี้ยว/หมุนด้วยเซนเซอร์เส้น, คำสั่งเจอแยก
 *
 *  สารบัญ
 *     1) ตัวแปร (Global Variables)
 *     2) ตั้งค่า (Config)
 *     3) อ่านตำแหน่งเส้น (Line Position)
 *     4) PID พื้นฐาน
 *     5) เดินตามเส้น จับเวลา / ระยะทาง (พื้นฐาน)
 *     6) เข้ากลางหุ่น / เข้าหาเส้น
 *     7) เลี้ยว / หมุน ด้วยเซนเซอร์หน้า
 *     8) เลี้ยว / หมุน ด้วยเซนเซอร์หลัง
 *     9) คำสั่งเมื่อเจอแยก (Track Select)
 *    10) เดินตามเส้น จับเวลา / ระยะทาง + kp / kd / คำสั่งเจอแยก
 *    11) เดินตามเส้นจนเจอแยก
 *    12) วิ่งเกาะขอบเส้น (CirCle)
 *    13) ออกตัว / เข้าเส้นชัย
 *    14) ปรับสมดุลหุ่นยนต์ (Balance)
 *    15) Serial Debug
 * ===================================================================== */

#ifndef UNKNOW_PID_H
#define UNKNOW_PID_H

#include "Unknow_Sensor.h"
#include "Unknow_Motor.h"
#include "Unknow_Buzzer.h"

// Defined in Unknow_IMU.h, which is included after this file by
// UnknowRobot_STORMXII_V3_IMU.h. Forward-declared here because
// TrackSelectF()/TrackSelectB() below call them.
void SetFG(int totalTime);
void SetBG(int totalTime);


// =====================================================================
//  1) ตัวแปร (Global Variables)
// =====================================================================

// ---------- ค่า PID หลัก ----------
float PID_KP, PID_KD;

// ---------- ความเร็ว / PID ตอนเข้ากลางหุ่น และเลี้ยว ----------
int8_t tspd;
int16_t tctL, tctR, bctL, bctR;  // มาจาก LeftBaseSpeed ฯลฯ (รวมค่า balance แล้ว)
int8_t LTurnSpdL, LTurnSpdR;
int8_t RTurnSpdL, RTurnSpdR;
uint16_t TurnDelayL, TurnDelayR;    // ms
uint8_t LChTurn = 5, RChTurn = 10;  // index เซนเซอร์ 0..15
uint8_t LChTurnB = 5, RChTurnB = 10;
uint16_t tct_delay_break = 0;
uint16_t bct_delay_break = 0;
float slow_kp_f = 0.014, slow_kd_f = 0.14;
float slow_kp_b = 0.014, slow_kd_b = 0.14;
float lf_kp = 0.015, lf_kd = 0.10;  // PID อยู่กับที่ LF()
float lb_kp = 0.015, lb_kd = 0.10;  // PID อยู่กับที่ LB()
uint8_t line_centor = 0;            // 0 / 1

// ---------- ตำแหน่งเส้น / ช่วงเซนเซอร์ที่ใช้วิ่ง ----------
int16_t set_position = 7500;  // 0..15000
int16_t set_position_l = 1500;
int16_t set_position_r = 13500;
uint8_t setsensortracklineL = 0, setsensortracklineR = 15;

// ---------- จำกัดกำลังมอเตอร์ (ModeSpdPID) ----------
int8_t MaxSpeed = 100;
int8_t MinSpeed = -5;
uint8_t ModePidStatus = 0;
int8_t pid_mode2_up = 20, pid_mode2_down = 20;  // ModePidStatus 2: ช่วงกำลังล้อ = -(Speed+down)..(Speed+up)

// ---------- ตัวคูณแปลงความเร็ว → ระยะทาง ของ FFcm / BBcm (set_speed_scale) ----------
float speed_scale_ff = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง
float speed_scale_bb = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง


// =====================================================================
//  2) ตั้งค่า (Config)
// =====================================================================

void SetRobotPID(float Kp, float Kd) {
  PID_KP = Kp;
  PID_KD = Kd;
}

void SetToCenterSpeed(int tctv) {
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

void SetSensorTurnLeftRight(int l, int r) {
  LChTurn = l;
  RChTurn = r;
}

void SetSensorTurnLeftRightB(int l, int r) {
  LChTurnB = l;
  RChTurnB = r;
}

void SetTurnSpeed(int tspdv) {
  tspd = tspdv;
}

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

// ตัวคูณแปลงความเร็ว → ระยะทาง ของ FFcm / BBcm (เดินหน้า, ถอยหลัง)
// หุ่นเดินเกินระยะ ให้เพิ่มค่า | หุ่นเดินไม่ถึงระยะ ให้ลดค่า
void set_speed_scale(float fw, float bw) {
  speed_scale_ff = fw;
  speed_scale_bb = bw;
}

void set_pid_lf(float Kp, float Kd) {
  lf_kp = Kp;
  lf_kd = Kd;
}

void set_pid_lb(float Kp, float Kd) {
  lb_kp = Kp;
  lb_kd = Kd;
}

void set_pid_mode2_limit(int up, int down) {
  pid_mode2_up = up;
  pid_mode2_down = down;
}

void ModeSpdPID(int moD, int maX, int miN) {
  ModePidStatus = moD;
  MaxSpeed = maX;
  MinSpeed = miN;
}

void set_position_line(int _pos) {
  if (_pos < 0) {
    set_position = 0;
  } else if (_pos > 15000) {
    set_position = 15000;
  } else {
    set_position = _pos;
  }
}

void set_position_line_l(int _pos) {
  if (_pos < 0) {
    set_position_l = 0;
  } else if (_pos > 15000) {
    set_position_l = 15000;
  } else {
    set_position_l = _pos;
  }
}

void set_position_line_r(int _pos) {
  if (_pos < 0) {
    set_position_r = 0;
  } else if (_pos > 15000) {
    set_position_r = 15000;
  } else {
    set_position_r = _pos;
  }
}

void set_sensor_track_line(int L, int R) {
  setsensortracklineL = L;
  setsensortracklineR = R;
}

void set_line_center(int x) {
  line_centor = x;
}


// =====================================================================
//  3) อ่านตำแหน่งเส้น (Line Position)
// =====================================================================


int readPositionF(int Track, int noise) {
  int i;
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int16_t last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateF();
  for (i = setsensortracklineL; i <= setsensortracklineR; i++) {
    int values = F[i];
    if (values > Track) online = true;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2)
      return setsensortracklineL * 1000;
    else
      return setsensortracklineR * 1000;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

int readPositionB(int Track, int noise) {
  int i;
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int16_t last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateB();
  for (i = setsensortracklineL; i <= setsensortracklineR; i++) {
    int values = B[i];
    if (values > Track) online = true;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2)
      return setsensortracklineL * 1000;
    else
      return setsensortracklineR * 1000;
  }
  if (sum == 0) return last_value;
  last_value = avg / sum;
  return last_value;
}

int readPositionF_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int16_t last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateF();
  for (i = setsensortracklineL; i < setsensortracklineR; i++) {
    int values = F[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (NUM_SENSORS - 1) * 1000 / 2)
      return 7500;
    else
      return 7500;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionB_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int16_t last_value = (NUM_SENSORS - 1) * 1000 / 2;
  ReadCalibrateB();
  for (i = setsensortracklineL; i < setsensortracklineR; i++) {
    int values = B[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (TrackLineCH - 1) * 1000 / 2)
      return 7500;
    else
      return 7500;
  }
  last_value = avg / sum;
  return last_value;
}


// =====================================================================
//  4) PID พื้นฐาน
// =====================================================================

void ClampPower(int &LeftPower, int &RightPower, int SpeedL, int SpeedR) {
  switch (ModePidStatus) {
    case 0:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < 0) LeftPower = MinSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < 0) RightPower = MinSpeed;
      break;
    case 1:
      if (LeftPower > MaxSpeed) LeftPower = MaxSpeed;
      if (LeftPower < MinSpeed) LeftPower = MinSpeed;
      if (RightPower > MaxSpeed) RightPower = MaxSpeed;
      if (RightPower < MinSpeed) RightPower = MinSpeed;
      break;
    case 2:
      if (LeftPower > SpeedL + pid_mode2_up) LeftPower = SpeedL + pid_mode2_up;
      if (LeftPower < -SpeedL - pid_mode2_down) LeftPower = -SpeedL - pid_mode2_down;
      if (RightPower > SpeedR + pid_mode2_up) RightPower = SpeedR + pid_mode2_up;
      if (RightPower < -SpeedR - pid_mode2_down) RightPower = -SpeedR - pid_mode2_down;
      break;
    case 3:
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

void PIDF_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos;
  ReadCalibrateF();
  if (F[7] > Ref && F[8] > Ref) {
    Pos = 7500;
  } else {
    Pos = readPositionF_none(250, 50);
  }
  int Error = Pos - set_position;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_F_none));
  LastError_F_none = Error;
  int LeftPower = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  if (LeftPower > 100) LeftPower = 100;
  if (LeftPower < -100) LeftPower = -100;
  if (RightPower > 100) RightPower = 100;
  if (RightPower < -100) RightPower = -100;
  Motor(LeftPower, RightPower);
  delayMicroseconds(80);
}

void PIDB_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos;
  ReadCalibrateB();
  if (B[7] > Ref && B[8] > Ref) {
    Pos = 7500;
  } else {
    Pos = readPositionB_none(250, 50);
  }
  int Error = Pos - set_position;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_B_none));
  LastError_B_none = Error;
  int LeftPower = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  if (LeftPower > 100) LeftPower = 100;
  if (LeftPower < -100) LeftPower = -100;
  if (RightPower > 100) RightPower = 100;
  if (RightPower < -100) RightPower = -100;
  Motor(-LeftPower, -RightPower);
  delayMicroseconds(80);
}


// =====================================================================
//  5) เดินตามเส้น จับเวลา / ระยะทาง (พื้นฐาน)
// =====================================================================

//  5) เดินตามเส้น จับเวลา / ระยะทาง

// Kp / Kd ติดลบ = ใช้ค่าจากตารางความเร็ว (PID_KP_Front/Back, PID_KD_Front/Back)
static void FFtimer_pid(int baseSpeed, int totalTime, float Kp, float Kd) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
}

void FFtimer(int baseSpeed, int totalTime) {
  FFtimer_pid(baseSpeed, totalTime, -1, -1);
}

static void BBtimer_pid(int baseSpeed, int totalTime, float Kp, float Kd) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
}

void BBtimer(int baseSpeed, int totalTime) {
  BBtimer_pid(baseSpeed, totalTime, -1, -1);
}

static void FFcm_pid(int Speed, float distance, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();


  unsigned long prevT = millis();
  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale_ff) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
  }
}

void FFcm(int Speed, float distance) {
  FFcm_pid(Speed, distance, -1, -1);
}

static void BBcm_pid(int Speed, float distance, float Kp, float Kd) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();


  unsigned long prevT = millis();
  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);

    if (distance > 0) {
      unsigned long current_time = millis();
      float delta_time = (current_time - last_time) / 1000.0;
      traveled_distance += (target_speed * speed_scale_bb) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
  }
}

void BBcm(int Speed, float distance) {
  BBcm_pid(Speed, distance, -1, -1);
}


// =====================================================================
//  6) เข้ากลางหุ่น / เข้าหาเส้น
// =====================================================================

void ToCenter() {
  BZon();
  if (line_centor == 0) {
    Motor(tctL, tctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
  }
  while (1) {
    if (line_centor == 0) {
      Motor(tctL, tctR);
    } else {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToCenterL() {
  BZon();
  if (line_centor == 0) {
    Motor(tctL, tctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
  }
  while (1) {
    if (line_centor == 0) {
      Motor(tctL, tctR);
    } else {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
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

void ToCenterR() {
  BZon();
  if (line_centor == 0) {
    Motor(tctL, tctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
  }
  while (1) {
    if (line_centor == 0) {
      Motor(tctL, tctR);
    } else {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
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

void ToFront() {
  while (1) {
    PIDF(tctL, tctR, slow_kp_f, slow_kd_f);
    ReadCalibrateF();
    if (F[3] > Ref || F[12] > Ref) break;
  }
}

// ถอยหลังตามเส้นจนเซนเซอร์หลังเจอเส้นขวาง (คู่กับ ToFront)
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

void BackCenter() {
  BZon();
  if (line_centor == 0) {
    Motor(-bctL, -bctR);
    delay(20);
  } else {
    for (int i = 0; i <= 20; i++) {
      PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
    }
  }
  while (1) {
    if (line_centor == 0) {
      Motor(-bctL, -bctR);
    } else {
      PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
    }
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}


// =====================================================================
//  7) เลี้ยว / หมุน ด้วยเซนเซอร์หน้า
// =====================================================================

// ---------- จัดตรงเส้นอยู่กับที่ (LF / LB) ----------

// จัดหุ่นให้ตรงเส้นอยู่กับที่ด้วย PID (ความเร็วฐาน 0) เป็นเวลา totalTime ms
void LF(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(0, 0, lf_kp, lf_kd);
  MotorStop();
}

void LB(int totalTime) {
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(0, 0, lb_kp, lb_kd);
  MotorStop();
}

// ---------- เลี้ยวล้อเดียว ----------
// เลี้ยวจน F[0..3] / F[15..12] พ้นเส้นเดิม แล้วหยุดเมื่อเจอเส้นไล่ทีละตัว F[0..LChTurn] (ซ้าย) / F[15..RChTurn] (ขวา)
void TurnLeft() {
  for (int i = 0; i <= 3; i++) {
    do {
      Motor(-LTurnSpdL, LTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] >= Ref);
  }
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
  for (int i = 15; i >= 12; i--) {
    do {
      Motor(RTurnSpdL, -RTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] >= Ref);
  }
  for (int i = 15; i >= RChTurn; i--) {
    do {
      Motor(RTurnSpdL, -RTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
  MotorStop();
}

// ---------- หมุนอยู่กับที่ ----------
// หมุนจน F[0..3] / F[15..12] พ้นเส้นเดิม แล้วไล่เซนเซอร์จนเจอเส้นใหม่ที่ F[sensorIdx]
// (เลือกตามความเร็ว) จากนั้นเบรก และจัดตรงเส้นด้วย LF()
void SpinL(int Speed) {
  MotorStop();
  int sensorIdx;
  if (Speed <= 50) sensorIdx = 6;
  else if (Speed <= 60) sensorIdx = 4;
  else sensorIdx = 2;
  for (int i = 0; i <= 3; i++) {
    do {
      Motor(-Speed, Speed);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] >= Ref);
  }
  for (int i = 0; i <= sensorIdx; i++) {
    do {
      Motor(-Speed, Speed);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
  Motor(Speed, -Speed);
  delay(5);
  LF(Speed);
  MotorStop();
}

void SpinL() {
  SpinL(tspd);
}

void SpinL2(int Speed) {
  MotorStop();
  int sensorIdx;
  if (Speed <= 50) sensorIdx = 6;
  else if (Speed <= 60) sensorIdx = 4;
  else sensorIdx = 2;
  for (int i = 0; i <= 3; i++) {
    do {
      Motor(-Speed, Speed);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] >= Ref);
  }
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 0; i <= sensorIdx; i++) {
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

void SpinL2() {
  SpinL2(tspd);
}

void SpinR(int Speed) {
  MotorStop();
  int sensorIdx;
  if (Speed <= 50) sensorIdx = 9;
  else if (Speed <= 60) sensorIdx = 11;
  else sensorIdx = 13;
  for (int i = 15; i >= 12; i--) {
    do {
      Motor(Speed, -Speed);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] >= Ref);
  }
  for (int i = 15; i >= sensorIdx; i--) {
    do {
      Motor(Speed, -Speed);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] <= Ref);
  }
  Motor(-Speed, Speed);
  delay(5);
  LF(Speed);
  MotorStop();
}

void SpinR() {
  SpinR(tspd);
}

void SpinR2(int Speed) {
  MotorStop();
  int sensorIdx;
  if (Speed <= 50) sensorIdx = 9;
  else if (Speed <= 60) sensorIdx = 11;
  else sensorIdx = 13;
  for (int i = 15; i >= 12; i--) {
    do {
      Motor(Speed, -Speed);
      delayMicroseconds(80);
      ReadCalibrateF();
    } while (F[i] >= Ref);
  }
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 15; i >= sensorIdx; i--) {
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

void SpinR2() {
  SpinR2(tspd);
}


// =====================================================================
//  8) เลี้ยว / หมุน ด้วยเซนเซอร์หลัง
// =====================================================================

// ---------- เลี้ยวล้อเดียว ----------

// เลี้ยวล้อเดียว: หมุนจนเซนเซอร์หลังเจอเส้นไล่ทีละตัว B[0..LChTurnB] (ซ้าย) / B[15..RChTurnB] (ขวา)
void TurnLeftB() {
  for (int i = 0; i <= 3; i++) {
    do {
      Motor(-LTurnSpdL, LTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] >= Ref);
  }
  for (int i = 0; i <= LChTurnB; i++) {
    do {
      Motor(-LTurnSpdL, LTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  MotorStop();
}

void TurnRightB() {
  for (int i = 15; i >= 12; i--) {
    do {
      Motor(RTurnSpdL, -RTurnSpdR);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] >= Ref);
  }
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
void SpinLB(int Speed) {
  MotorStop();
  int sensorIdx;
  if (Speed <= 50) sensorIdx = 6;
  else if (Speed <= 60) sensorIdx = 4;
  else sensorIdx = 2;
  for (int i = 0; i <= 3; i++) {
    do {
      Motor(-Speed, Speed);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] >= Ref);
  }
  for (int i = 0; i <= sensorIdx; i++) {
    do {
      Motor(-Speed, Speed);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  Motor(Speed, -Speed);
  delay(5);
  LB(Speed);
  MotorStop();
}

void SpinLB() {
  SpinLB(tspd);
}

void SpinL2B(int Speed) {
  MotorStop();
  int sensorIdx;
  if (Speed <= 50) sensorIdx = 6;
  else if (Speed <= 60) sensorIdx = 4;
  else sensorIdx = 2;
  for (int i = 0; i <= 3; i++) {
    do {
      Motor(-Speed, Speed);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] >= Ref);
  }
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 0; i <= sensorIdx; i++) {
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

void SpinL2B() {
  SpinL2B(tspd);
}

void SpinRB(int Speed) {
  MotorStop();
  int sensorIdx;
  if (Speed <= 50) sensorIdx = 9;
  else if (Speed <= 60) sensorIdx = 11;
  else sensorIdx = 13;
  for (int i = 15; i >= 12; i--) {
    do {
      Motor(Speed, -Speed);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] >= Ref);
  }
  for (int i = 15; i >= sensorIdx; i--) {
    do {
      Motor(Speed, -Speed);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] <= Ref);
  }
  Motor(-Speed, Speed);
  delay(5);
  LB(Speed);
  MotorStop();
}

void SpinRB() {
  SpinRB(tspd);
}

void SpinR2B(int Speed) {
  MotorStop();
  int sensorIdx;
  if (Speed <= 50) sensorIdx = 9;
  else if (Speed <= 60) sensorIdx = 11;
  else sensorIdx = 13;
  for (int i = 15; i >= 12; i--) {
    do {
      Motor(Speed, -Speed);
      delayMicroseconds(80);
      ReadCalibrateB();
    } while (B[i] >= Ref);
  }
  for (int pass = 0; pass < 2; pass++) {
    for (int i = 15; i >= sensorIdx; i--) {
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

void SpinR2B() {
  SpinR2B(tspd);
}


// =====================================================================
//  9) คำสั่งเมื่อเจอแยก (Track Select)
// =====================================================================

void TrackSelectF(int spd, char x) {
  if (x == 's') {
    Motor(-spd, -spd);
    delay(delay_break_f);
    MotorStop();
  } else if (x == 'S') {
    ToFront();
    Motor(-tctL, -tctR);
    delay(tct_delay_break);
    MotorStop();
  } else if (x == 'p') {
    BZon();
    ReadCalibrateF();
    while (1) {
      Motor(spd, spd);
      delayMicroseconds(80);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    while (1) {
      Motor(spd, spd);
      delayMicroseconds(80);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    BZoff();
  } else if (x == 'P') {
    ToFront();
    BZon();
    for (int n = 0; n < 2; n++) {
      ReadCalibrateF();
      while (1) {
        Motor(spd, spd);
        delayMicroseconds(80);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
    }
    BZoff();
  } else if (x == 'l') {
    ToCenter();
    SpinL();
    FFtimer(0, 8);
  } else if (x == 'L') {
    ToFront();
    ToCenter();
    SpinL();
    FFtimer(0, 8);
  } else if (x == 'r') {
    ToCenter();
    SpinR();
    FFtimer(0, 8);
  } else if (x == 'R') {
    ToFront();
    ToCenter();
    SpinR();
    FFtimer(0, 8);
  } else if (x == 'q') {
    BZon();
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(tctL, tctR);
        delayMicroseconds(80);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
    }
    delay(TurnDelayL);
    BZoff();
    TurnLeft();
    FFtimer(0, 5);
  } else if (x == 'Q') {
    ToFront();
    BZon();
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(tctL, tctR);
        delayMicroseconds(80);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
    }
    delay(TurnDelayL);
    BZoff();
    TurnLeft();
    FFtimer(0, 5);
  } else if (x == 'e') {
    BZon();
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(tctL, tctR);
        delayMicroseconds(80);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
    }
    delay(TurnDelayR);
    BZoff();
    TurnRight();
    FFtimer(0, 5);
  } else if (x == 'E') {
    ToFront();
    BZon();
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(tctL, tctR);
        delayMicroseconds(80);
        ReadCalibrateF();
        if (F[3] < Ref && F[12] < Ref) break;
      }
    }
    delay(TurnDelayR);
    BZoff();
    TurnRight();
    FFtimer(0, 5);
  } else if (x == 'c') {
    ToCenter();
  } else if (x == 'C') {
    ToFront();
    ToCenter();
  } else if (x == 'd') {
    ToCenter();
    SpinRB();
    BBtimer(0, 8);
  } else if (x == 'D') {
    ToFront();
    ToCenter();
    SpinRB();
    BBtimer(0, 8);
  } else if (x == 'a') {
    ToCenter();
    SpinLB();
    BBtimer(0, 8);
  } else if (x == 'A') {
    ToFront();
    ToCenter();
    SpinLB();
    BBtimer(0, 8);
  } else if (x == 'g') {
    SetFG(spd);
  } else if (x == 'G') {
    ToFront();
    SetFG(spd);
  } else {
    MotorStop();
  }
}

void TrackSelectB(int spd, char x) {
  if (x == 's') {
    Motor(spd, spd);
    delay(delay_break_b);
    MotorStop(2);
  } else if (x == 'S') {
    ToBack();
    Motor(spd, spd);
    delay(delay_break_b);
    MotorStop(2);
  } else if (x == 'p' || x == 'P') {
    ReadCalibrateB();
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(-spd, -spd);
        delayMicroseconds(80);
        ReadCalibrateB();
        if (B[3] < Ref && B[12] < Ref) break;
      }
    }
  } else if (x == 'l') {
    BackCenter();
    SpinL();
    FFtimer(0, 2);
  } else if (x == 'L') {
    ToBack();
    BackCenter();
    SpinL();
    FFtimer(0, 2);
  } else if (x == 'r') {
    BackCenter();
    SpinR();
    FFtimer(0, 2);
  } else if (x == 'R') {
    ToBack();
    BackCenter();
    SpinR();
    FFtimer(0, 2);
  } else if (x == 'c') {
    BackCenter();
  } else if (x == 'C') {
    ToBack();
    BackCenter();
  } else if (x == 'd') {
    BackCenter();
    SpinRB();
    BBtimer(0, 8);
  } else if (x == 'D') {
    ToBack();
    BackCenter();
    SpinRB();
    BBtimer(0, 8);
  } else if (x == 'a') {
    BackCenter();
    SpinLB();
    BBtimer(0, 8);
  } else if (x == 'A') {
    ToBack();
    BackCenter();
    SpinLB();
    BBtimer(0, 8);
  } else if (x == 'e') {
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(-bctL, -bctR);
        delayMicroseconds(80);
        ReadCalibrateB();
        if (B[3] < Ref && B[12] < Ref) break;
      }
    }
    delay(TurnDelayL);
    TurnLeftB();
    BBtimer(0, 5);
  } else if (x == 'E') {
    ToBack();
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(-bctL, -bctR);
        delayMicroseconds(80);
        ReadCalibrateB();
        if (B[3] < Ref && B[12] < Ref) break;
      }
    }
    delay(TurnDelayL);
    TurnLeftB();
    BBtimer(0, 5);
  } else if (x == 'q') {
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(-bctL, -bctR);
        delayMicroseconds(80);
        ReadCalibrateB();
        if (B[12] < Ref && B[3] < Ref) break;
      }
    }
    delay(TurnDelayR);
    TurnRightB();
    BBtimer(0, 5);
  } else if (x == 'Q') {
    ToBack();
    for (int n = 0; n < 2; n++) {
      while (1) {
        Motor(-bctL, -bctR);
        delayMicroseconds(80);
        ReadCalibrateB();
        if (B[12] < Ref && B[3] < Ref) break;
      }
    }
    delay(TurnDelayR);
    TurnRightB();
    BBtimer(0, 5);
  } else if (x == 'g') {
    SetFG(spd);
  } else if (x == 'G') {
    ToBack();
    SetFG(spd);
  } else {
    MotorStop();
  }
}


// =====================================================================
//  10) เดินตามเส้น จับเวลา / ระยะทาง + kp / kd / คำสั่งเจอแยก
// =====================================================================

// ---------- จับเวลา: (speed, [kp, [kd,]] ms [, select])  ใส่ค่าติดลบ = ใช้ค่าจากตาราง ----------
//  10) เดินตามเส้น + คำสั่งเจอแยก
void FFtimer(int Speed, int totalTime, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
  TrackSelectF(Speed, select);
}

//  11) เดินตามเส้น + กำหนด kp kd เอง

// (speed, kp, [kd,] เวลา/ระยะ [, select])  ใส่ค่าติดลบ = ใช้ค่าจากตาราง
// เวลา/ระยะ เป็น int เพื่อไม่ให้ชนกับ FFtimer(speed, ms, select) / FFcm(speed, cm, select)
void FFtimer(int Speed, float Kp, int totalTime) {
  FFtimer_pid(Speed, totalTime, Kp, -1);
}

void FFtimer(int Speed, float Kp, int totalTime, char select) {
  FFtimer_pid(Speed, totalTime, Kp, -1);
  TrackSelectF(Speed, select);
}

void FFtimer(int Speed, float Kp, float Kd, int totalTime) {
  FFtimer_pid(Speed, totalTime, Kp, Kd);
}

void FFtimer(int Speed, float Kp, float Kd, int totalTime, char select) {
  FFtimer_pid(Speed, totalTime, Kp, Kd);
  TrackSelectF(Speed, select);
}

void BBtimer(int Speed, int totalTime, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
  TrackSelectB(Speed, select);
}

void BBtimer(int Speed, float Kp, int totalTime) {
  BBtimer_pid(Speed, totalTime, Kp, -1);
}

void BBtimer(int Speed, float Kp, int totalTime, char select) {
  BBtimer_pid(Speed, totalTime, Kp, -1);
  TrackSelectB(Speed, select);
}

void BBtimer(int Speed, float Kp, float Kd, int totalTime) {
  BBtimer_pid(Speed, totalTime, Kp, Kd);
}

void BBtimer(int Speed, float Kp, float Kd, int totalTime, char select) {
  BBtimer_pid(Speed, totalTime, Kp, Kd);
  TrackSelectB(Speed, select);
}

// ---------- ระยะทาง: (speed, [kp, [kd,]] cm [, select])  cm เป็น int เมื่อใส่ kp เพื่อไม่ชนกับ (speed, cm, select) ----------
void FFcm(int Speed, float distance, char select) {
  FFcm(Speed, distance);
  TrackSelectF(Speed, select);
}

void FFcm(int Speed, float Kp, int distance) {
  FFcm_pid(Speed, distance, Kp, -1);
}

void FFcm(int Speed, float Kp, int distance, char select) {
  FFcm_pid(Speed, distance, Kp, -1);
  TrackSelectF(Speed, select);
}

void FFcm(int Speed, float Kp, float Kd, int distance) {
  FFcm_pid(Speed, distance, Kp, Kd);
}

void FFcm(int Speed, float Kp, float Kd, int distance, char select) {
  FFcm_pid(Speed, distance, Kp, Kd);
  TrackSelectF(Speed, select);
}

void BBcm(int Speed, float distance, char select) {
  BBcm(Speed, distance);
  TrackSelectB(Speed, select);
}

void BBcm(int Speed, float Kp, int distance) {
  BBcm_pid(Speed, distance, Kp, -1);
}

void BBcm(int Speed, float Kp, int distance, char select) {
  BBcm_pid(Speed, distance, Kp, -1);
  TrackSelectB(Speed, select);
}

void BBcm(int Speed, float Kp, float Kd, int distance) {
  BBcm_pid(Speed, distance, Kp, Kd);
}

void BBcm(int Speed, float Kp, float Kd, int distance, char select) {
  BBcm_pid(Speed, distance, Kp, Kd);
  TrackSelectB(Speed, select);
}


// =====================================================================
//  11) เดินตามเส้นจนเจอแยก
// =====================================================================

// ---------- จับแยก: (speed, [kp, [kd,]] select) ----------
// ทุกคำสั่งในหมวดนี้ใส่ Kp / Kd เองได้ต่อจาก speed: (speed, select) / (speed, Kp, select) / (speed, Kp, Kd, select)
// ใส่ค่าติดลบ หรือไม่ใส่ = ใช้ค่าจากตารางความเร็ว (PID_KP_Front/Back, PID_KD_Front/Back)
void FFC(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if ((F[5] >= Ref && F[10] >= Ref) || (F[4] >= Ref && F[11] >= Ref) || (F[3] >= Ref && F[12] >= Ref) || (F[5] >= Ref && F[6] >= Ref && F[9] >= Ref && F[10] >= Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFC(int Speed, float Kp, char select) {
  FFC(Speed, Kp, -1, select);
}

void FFC(int Speed, char select) {
  FFC(Speed, -1, -1, select);
}

void BBC(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if ((B[5] >= Ref && B[10] >= Ref) || (B[4] >= Ref && B[11] >= Ref) || (B[3] >= Ref && B[12] >= Ref) || (B[5] >= Ref && B[6] >= Ref && B[9] >= Ref && B[10] >= Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBC(int Speed, float Kp, char select) {
  BBC(Speed, Kp, -1, select);
}

void BBC(int Speed, char select) {
  BBC(Speed, -1, -1, select);
}

void FFC2(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[0] >= Ref && F[15] >= Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFC2(int Speed, float Kp, char select) {
  FFC2(Speed, Kp, -1, select);
}

void FFC2(int Speed, char select) {
  FFC2(Speed, -1, -1, select);
}

void BBC2(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[0] >= Ref && B[15] >= Ref) break;
  }
  TrackSelectB(Speed, select);
}

void BBC2(int Speed, float Kp, char select) {
  BBC2(Speed, Kp, -1, select);
}

void BBC2(int Speed, char select) {
  BBC2(Speed, -1, -1, select);
}

void FFL(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if ((F[1] > Ref && F[5] > Ref) || (F[2] > Ref && F[6] > Ref) || (F[3] > Ref && F[7] > Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFL(int Speed, float Kp, char select) {
  FFL(Speed, Kp, -1, select);
}

void FFL(int Speed, char select) {
  FFL(Speed, -1, -1, select);
}

void BBL(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if ((B[1] > Ref && B[5] > Ref) || (B[2] > Ref && B[6] > Ref) || (B[3] > Ref && B[7] > Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBL(int Speed, float Kp, char select) {
  BBL(Speed, Kp, -1, select);
}

void BBL(int Speed, char select) {
  BBL(Speed, -1, -1, select);
}

void FFR(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if ((F[14] > Ref && F[10] > Ref) || (F[13] > Ref && F[9] > Ref) || (F[12] > Ref && F[8] > Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFR(int Speed, float Kp, char select) {
  FFR(Speed, Kp, -1, select);
}

void FFR(int Speed, char select) {
  FFR(Speed, -1, -1, select);
}

void BBR(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if ((B[14] > Ref && B[10] > Ref) || (B[13] > Ref && B[9] > Ref) || (B[12] > Ref && B[8] > Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBR(int Speed, float Kp, char select) {
  BBR(Speed, Kp, -1, select);
}

void BBR(int Speed, char select) {
  BBR(Speed, -1, -1, select);
}

void FFL0(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFL0(int Speed, float Kp, char select) {
  FFL0(Speed, Kp, -1, select);
}

void FFL0(int Speed, char select) {
  FFL0(Speed, -1, -1, select);
}

void BBL0(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void BBL0(int Speed, float Kp, char select) {
  BBL0(Speed, Kp, -1, select);
}

void BBL0(int Speed, char select) {
  BBL0(Speed, -1, -1, select);
}

void FFR15(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[15] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFR15(int Speed, float Kp, char select) {
  FFR15(Speed, Kp, -1, select);
}

void FFR15(int Speed, char select) {
  FFR15(Speed, -1, -1, select);
}

void BBR15(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[15] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void BBR15(int Speed, float Kp, char select) {
  BBR15(Speed, Kp, -1, select);
}

void BBR15(int Speed, char select) {
  BBR15(Speed, -1, -1, select);
}

void FFNUM(int Speed, float Kp, float Kd, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[numm] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFNUM(int Speed, float Kp, char select, int numm) {
  FFNUM(Speed, Kp, -1, select, numm);
}

void FFNUM(int Speed, char select, int numm) {
  FFNUM(Speed, -1, -1, select, numm);
}

void BBNUM(int Speed, float Kp, float Kd, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[numm] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void BBNUM(int Speed, float Kp, char select, int numm) {
  BBNUM(Speed, Kp, -1, select, numm);
}

void BBNUM(int Speed, char select, int numm) {
  BBNUM(Speed, -1, -1, select, numm);
}

// ---------- ผลรวมเซนเซอร์ (FFSUM / BBSUM) ----------
// วิ่งตามเส้นจนผลรวมเซนเซอร์หน้า F[0]..F[15] (แต่ละตัว 0-1000) >= sum_target (0-16000)
void FFSUM(int Speed, float Kp, float Kd, char select, int sum_target) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    long sum = 0;
    for (int i = 0; i < NUM_SENSORS; i++) sum += F[i];
    if (sum >= sum_target) break;
  }
  TrackSelectF(Speed, select);
}

void FFSUM(int Speed, float Kp, char select, int sum_target) {
  FFSUM(Speed, Kp, -1, select, sum_target);
}

void FFSUM(int Speed, char select, int sum_target) {
  FFSUM(Speed, -1, -1, select, sum_target);
}

// ถอยตามเส้นจนผลรวมเซนเซอร์หลัง B[0]..B[15] (แต่ละตัว 0-1000) >= sum_target (0-16000)
void BBSUM(int Speed, float Kp, float Kd, char select, int sum_target) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    long sum = 0;
    for (int i = 0; i < NUM_SENSORS; i++) sum += B[i];
    if (sum >= sum_target) break;
  }
  TrackSelectB(Speed, select);
}

void BBSUM(int Speed, float Kp, char select, int sum_target) {
  BBSUM(Speed, Kp, -1, select, sum_target);
}

void BBSUM(int Speed, char select, int sum_target) {
  BBSUM(Speed, -1, -1, select, sum_target);
}

// ---------- พื้นดำ / พื้นขาว ----------
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

void FFWhite(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref && F[8] < Ref && F[9] < Ref && F[10] < Ref && F[11] < Ref && F[12] < Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFWhite(int Speed, float Kp, char select) {
  FFWhite(Speed, Kp, -1, select);
}

void FFWhite(int Speed, char select) {
  FFWhite(Speed, -1, -1, select);
}

void BBWhite(int Speed, float Kp, float Kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref && B[8] < Ref && B[9] < Ref && B[10] < Ref && B[11] < Ref && B[12] < Ref) break;
  }
  TrackSelectB(Speed, select);
}

void BBWhite(int Speed, float Kp, char select) {
  BBWhite(Speed, Kp, -1, select);
}

void BBWhite(int Speed, char select) {
  BBWhite(Speed, -1, -1, select);
}


// =====================================================================
//  12) วิ่งเกาะขอบเส้น (CirCle)
// =====================================================================

// ---------- วิ่งจนเจอแยก ----------
void FFCirCleL(int Speed, float Kp, float Kd, char select) {
  int temp_position = set_position;
  set_position = set_position_l;
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp_position;
}

void FFCirCleL(int Speed, float Kp, char select) {
  FFCirCleL(Speed, Kp, -1, select);
}

void FFCirCleL(int Speed, char select) {
  FFCirCleL(Speed, -1, -1, select);
}

void FFCirCleR(int Speed, float Kp, float Kd, char select) {
  int temp_position = set_position;
  set_position = set_position_r;
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Front;
  if (Kd < 0) Kd = PID_KD_Front;
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, Kp, Kd);
    ReadCalibrateF();
    if (F[15] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp_position;
}

void FFCirCleR(int Speed, float Kp, char select) {
  FFCirCleR(Speed, Kp, -1, select);
}

void FFCirCleR(int Speed, char select) {
  FFCirCleR(Speed, -1, -1, select);
}

void BBCirCleL(int Speed, float Kp, float Kd, char select) {
  int temp_position = set_position;
  set_position = set_position_l;
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp_position;
}

void BBCirCleL(int Speed, float Kp, char select) {
  BBCirCleL(Speed, Kp, -1, select);
}

void BBCirCleL(int Speed, char select) {
  BBCirCleL(Speed, -1, -1, select);
}

void BBCirCleR(int Speed, float Kp, float Kd, char select) {
  int temp_position = set_position;
  set_position = set_position_r;
  BaseSpeed = Speed;
  InitialSpeed();
  if (Kp < 0) Kp = PID_KP_Back;
  if (Kd < 0) Kd = PID_KD_Back;
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, Kp, Kd);
    ReadCalibrateB();
    if (B[15] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp_position;
}

void BBCirCleR(int Speed, float Kp, char select) {
  BBCirCleR(Speed, Kp, -1, select);
}

void BBCirCleR(int Speed, char select) {
  BBCirCleR(Speed, -1, -1, select);
}

// ---------- จับเวลา: (speed, [kp, [kd,]] ms [, select]) ----------
void FFtimerCirCleL(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFtimer(Speed, totalTime);
  set_position = temp_position;
}

void FFtimerCirCleL(int Speed, int totalTime, char select) {
  FFtimerCirCleL(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void FFtimerCirCleL(int Speed, float Kp, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFtimer(Speed, Kp, totalTime);
  set_position = temp_position;
}

void FFtimerCirCleL(int Speed, float Kp, int totalTime, char select) {
  FFtimerCirCleL(Speed, Kp, totalTime);
  TrackSelectF(Speed, select);
}

void FFtimerCirCleL(int Speed, float Kp, float Kd, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFtimer(Speed, Kp, Kd, totalTime);
  set_position = temp_position;
}

void FFtimerCirCleL(int Speed, float Kp, float Kd, int totalTime, char select) {
  FFtimerCirCleL(Speed, Kp, Kd, totalTime);
  TrackSelectF(Speed, select);
}

void FFtimerCirCleR(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFtimer(Speed, totalTime);
  set_position = temp_position;
}

void FFtimerCirCleR(int Speed, int totalTime, char select) {
  FFtimerCirCleR(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void FFtimerCirCleR(int Speed, float Kp, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFtimer(Speed, Kp, totalTime);
  set_position = temp_position;
}

void FFtimerCirCleR(int Speed, float Kp, int totalTime, char select) {
  FFtimerCirCleR(Speed, Kp, totalTime);
  TrackSelectF(Speed, select);
}

void FFtimerCirCleR(int Speed, float Kp, float Kd, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFtimer(Speed, Kp, Kd, totalTime);
  set_position = temp_position;
}

void FFtimerCirCleR(int Speed, float Kp, float Kd, int totalTime, char select) {
  FFtimerCirCleR(Speed, Kp, Kd, totalTime);
  TrackSelectF(Speed, select);
}

void BBtimerCirCleL(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBtimer(Speed, totalTime);
  set_position = temp_position;
}

void BBtimerCirCleL(int Speed, int totalTime, char select) {
  BBtimerCirCleL(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void BBtimerCirCleL(int Speed, float Kp, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBtimer(Speed, Kp, totalTime);
  set_position = temp_position;
}

void BBtimerCirCleL(int Speed, float Kp, int totalTime, char select) {
  BBtimerCirCleL(Speed, Kp, totalTime);
  TrackSelectB(Speed, select);
}

void BBtimerCirCleL(int Speed, float Kp, float Kd, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBtimer(Speed, Kp, Kd, totalTime);
  set_position = temp_position;
}

void BBtimerCirCleL(int Speed, float Kp, float Kd, int totalTime, char select) {
  BBtimerCirCleL(Speed, Kp, Kd, totalTime);
  TrackSelectB(Speed, select);
}

void BBtimerCirCleR(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBtimer(Speed, totalTime);
  set_position = temp_position;
}

void BBtimerCirCleR(int Speed, int totalTime, char select) {
  BBtimerCirCleR(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void BBtimerCirCleR(int Speed, float Kp, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBtimer(Speed, Kp, totalTime);
  set_position = temp_position;
}

void BBtimerCirCleR(int Speed, float Kp, int totalTime, char select) {
  BBtimerCirCleR(Speed, Kp, totalTime);
  TrackSelectB(Speed, select);
}

void BBtimerCirCleR(int Speed, float Kp, float Kd, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBtimer(Speed, Kp, Kd, totalTime);
  set_position = temp_position;
}

void BBtimerCirCleR(int Speed, float Kp, float Kd, int totalTime, char select) {
  BBtimerCirCleR(Speed, Kp, Kd, totalTime);
  TrackSelectB(Speed, select);
}

// ---------- ระยะทาง: (speed, [kp, [kd,]] cm [, select]) ----------
void FFcmCirCleL(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFcm(Speed, totalTime);
  set_position = temp_position;
}

void FFcmCirCleL(int Speed, int totalTime, char select) {
  FFcmCirCleL(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void FFcmCirCleL(int Speed, float Kp, int distance) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFcm(Speed, Kp, distance);
  set_position = temp_position;
}

void FFcmCirCleL(int Speed, float Kp, int distance, char select) {
  FFcmCirCleL(Speed, Kp, distance);
  TrackSelectF(Speed, select);
}

void FFcmCirCleL(int Speed, float Kp, float Kd, int distance) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFcm(Speed, Kp, Kd, distance);
  set_position = temp_position;
}

void FFcmCirCleL(int Speed, float Kp, float Kd, int distance, char select) {
  FFcmCirCleL(Speed, Kp, Kd, distance);
  TrackSelectF(Speed, select);
}

void FFcmCirCleR(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFcm(Speed, totalTime);
  set_position = temp_position;
}

void FFcmCirCleR(int Speed, int totalTime, char select) {
  FFcmCirCleR(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void FFcmCirCleR(int Speed, float Kp, int distance) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFcm(Speed, Kp, distance);
  set_position = temp_position;
}

void FFcmCirCleR(int Speed, float Kp, int distance, char select) {
  FFcmCirCleR(Speed, Kp, distance);
  TrackSelectF(Speed, select);
}

void FFcmCirCleR(int Speed, float Kp, float Kd, int distance) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFcm(Speed, Kp, Kd, distance);
  set_position = temp_position;
}

void FFcmCirCleR(int Speed, float Kp, float Kd, int distance, char select) {
  FFcmCirCleR(Speed, Kp, Kd, distance);
  TrackSelectF(Speed, select);
}

void BBcmCirCleL(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBcm(Speed, totalTime);
  set_position = temp_position;
}

void BBcmCirCleL(int Speed, int totalTime, char select) {
  BBcmCirCleL(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void BBcmCirCleL(int Speed, float Kp, int distance) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBcm(Speed, Kp, distance);
  set_position = temp_position;
}

void BBcmCirCleL(int Speed, float Kp, int distance, char select) {
  BBcmCirCleL(Speed, Kp, distance);
  TrackSelectB(Speed, select);
}

void BBcmCirCleL(int Speed, float Kp, float Kd, int distance) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBcm(Speed, Kp, Kd, distance);
  set_position = temp_position;
}

void BBcmCirCleL(int Speed, float Kp, float Kd, int distance, char select) {
  BBcmCirCleL(Speed, Kp, Kd, distance);
  TrackSelectB(Speed, select);
}

void BBcmCirCleR(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBcm(Speed, totalTime);
  set_position = temp_position;
}

void BBcmCirCleR(int Speed, int totalTime, char select) {
  BBcmCirCleR(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void BBcmCirCleR(int Speed, float Kp, int distance) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBcm(Speed, Kp, distance);
  set_position = temp_position;
}

void BBcmCirCleR(int Speed, float Kp, int distance, char select) {
  BBcmCirCleR(Speed, Kp, distance);
  TrackSelectB(Speed, select);
}

void BBcmCirCleR(int Speed, float Kp, float Kd, int distance) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBcm(Speed, Kp, Kd, distance);
  set_position = temp_position;
}

void BBcmCirCleR(int Speed, float Kp, float Kd, int distance, char select) {
  BBcmCirCleR(Speed, Kp, Kd, distance);
  TrackSelectB(Speed, select);
}


// =====================================================================
//  13) ออกตัว / เข้าเส้นชัย
// =====================================================================

void GoStart(int LeftSpeed, int RightSpeed) {
  Move(LeftSpeed, RightSpeed, 100);
  while (1) {
    ReadCalibrateF();
    Motor(LeftSpeed, RightSpeed);
    if (F[0] < 500 && F[15] < 500) {
      break;
    }
  }
}

void GoStart(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  GoStart(LeftBaseSpeed, RightBaseSpeed);
}

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


// =====================================================================
//  14) ปรับสมดุลหุ่นยนต์ (Balance)
// =====================================================================

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


// =====================================================================
//  15) Serial Debug
// =====================================================================

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
    Serial.print("Position  B : ");
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
