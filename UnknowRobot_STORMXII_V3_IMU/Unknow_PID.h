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

float PID_KP, PID_KD;
int LastError_F, LastError_B;
int Error_F = 0, Error_B = 0;
int tct, bct, tspd;
int tctL, tctR, bctL, bctR;
int LTurnSpdL, LTurnSpdR, TurnDelayL;
int RTurnSpdL, RTurnSpdR, TurnDelayR;
int set_position = 7500;
int set_position_l = 1500;
int set_position_r = 13500;
float slow_kp = 0.014, slow_kd = 0.14;
float slow_kp_f = 0.014, slow_kd_f = 0.14;
float slow_kp_b = 0.014, slow_kd_b = 0.14;
int line_centor = 0;

int MaxSpeed = 100;
int MinSpeed = -5;
int ModePidStatus = 0;
int setsensortracklineL = 0, setsensortracklineR = 15;

// ---------- Config ----------

void SetRobotPID(float Kp, float Kd) {
  PID_KP = Kp;
  PID_KD = Kd;
}

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
}

void SetSlowKpKd(float sl_kp, float sl_kd) {
  slow_kp = sl_kp;
  slow_kd = sl_kd;
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
// ---------- Position Reading ----------

// int readPositionF(int Track, int noise) {
//   unsigned char i, online = 0;
//   unsigned long avg = 0;
//   unsigned long  sum = 0;
//   static int last_value = (TrackLineCH - 1) * 1000 / 2;
//   ReadCalibrateF();
//   for (i = 0; i < TrackLineCH; i++) {
//     int values;
//     if      (TrackLineCH == 6)  values = F[i + 5];
//     else if (TrackLineCH == 8)  values = F[i + 4];
//     else if (TrackLineCH == 10) values = F[i + 3];
//     else if (TrackLineCH == 12) values = F[i + 2];
//     else if (TrackLineCH == 14) values = F[i + 1];
//     else                        values = F[i];
//     if (values > Track) online = 1;
//     if (values > noise) {
//       avg += (long)(values) * (i * 1000);
//       sum += values;
//     }
//   }
//   if (!online) {
//     if (last_value < (TrackLineCH - 1) * 1000 / 2) return 0;
//     else return (TrackLineCH - 1) * 1000;
//   }
//   last_value = avg / sum;
//   return last_value;
// }

// int readPositionB(int Track, int noise) {
//   unsigned char i, online = 0;
//   unsigned long avg = 0;
//   unsigned long  sum = 0;
//   static int last_value = (TrackLineCH - 1) * 1000 / 2;
//   ReadCalibrateB();
//   for (i = 0; i < TrackLineCH; i++) {
//     int values;
//     if      (TrackLineCH == 6)  values = B[i + 5];
//     else if (TrackLineCH == 8)  values = B[i + 4];
//     else if (TrackLineCH == 10) values = B[i + 3];
//     else if (TrackLineCH == 12) values = B[i + 2];
//     else if (TrackLineCH == 14) values = B[i + 1];
//     else                        values = B[i];
//     if (values > Track) online = 1;
//     if (values > noise) {
//       avg += (long)(values) * (i * 1000);
//       sum += values;
//     }
//   }
//   if (!online) {
//     if (last_value < (TrackLineCH - 1) * 1000 / 2) return 0;
//     else return (TrackLineCH - 1) * 1000;
//   }
//   last_value = avg / sum;
//   return last_value;
// }

int readPositionF(int Track, int noise) {
  int i;
  bool online = false;
  unsigned long avg = 0;
  unsigned long sum = 0;
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
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
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
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
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
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
  static int last_value = (NUM_SENSORS - 1) * 1000 / 2;
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

// ---------- PID ----------

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
      if (LeftPower > SpeedL) LeftPower = SpeedL;
      if (LeftPower < -SpeedL) LeftPower = -SpeedL;
      if (RightPower > SpeedR) RightPower = SpeedR;
      if (RightPower < -SpeedR) RightPower = -SpeedR;
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
}

void PIDF_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos;
  ReadCalibrateF();
  if (F[7] > Ref && F[8] > Ref) {
    Pos = 7500;
  } else {
    Pos = readPositionF_none(250, 50);
  }
  // int Pos      = readPositionF_none(250, 50);
  int Error = Pos - set_position;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_F));
  LastError_F = Error;
  int LeftPower = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  if (LeftPower > 100) LeftPower = 100;
  if (LeftPower < -100) LeftPower = -100;
  if (RightPower > 100) RightPower = 100;
  if (RightPower < -100) RightPower = -100;
  Motor(LeftPower, RightPower);
}

void PIDB_none(int SpeedL, int SpeedR, float Kp, float Kd) {
  int Pos;
  ReadCalibrateB();
  if (B[7] > Ref && B[8] > Ref) {
    Pos = 7500;
  } else {
    Pos = readPositionB_none(250, 50);
  }
  // int Pos      = readPositionB_none(250, 50);
  int Error = Pos - set_position;
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_B));
  LastError_B = Error;
  int LeftPower = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  if (LeftPower > 100) LeftPower = 100;
  if (LeftPower < -100) LeftPower = -100;
  if (RightPower > 100) RightPower = 100;
  if (RightPower < -100) RightPower = -100;
  Motor(-LeftPower, -RightPower);
}

// ---------- Timed Motion ----------

void FFtimer(int baseSpeed, int totalTime) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
}

void BBtimer(int baseSpeed, int totalTime) {
  BaseSpeed = baseSpeed;
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

  float speed_scale = 1.5;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

  unsigned long prevT = millis();
  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

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

  float speed_scale = 1.5;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

  unsigned long prevT = millis();
  while (1) {
    unsigned long now = millis();
    float dt = (now - prevT) / 1000.0;
    if (dt <= 0) dt = 0.001;
    prevT = now;

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

// ---------- ToCenter / BackCenter ----------

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
  // Motor(tctL, tctR);
  // delay(20);
  while (1) {
    if (line_centor == 0) {
      Motor(tctL, tctR);
    } else {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
    // Motor(tctL, tctR);
    ReadSensor();
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
  // Motor(tctL, tctR);
  // delay(20);
  while (1) {
    // Motor(tctL, tctR);
    if (line_centor == 0) {
      Motor(tctL, tctR);
    } else {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
    ReadSensor();
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
  // Motor(tctL, tctR);
  // delay(20);
  while (1) {
    // Motor(tctL, tctR);
    if (line_centor == 0) {
      Motor(tctL, tctR);
    } else {
      PIDF_none(tctL, tctR, slow_kp_f, slow_kd_f);
    }
    ReadSensor();
    if (C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

void ToFront(){
  while(1){
    PIDF(tctL,tctR,slow_kp_f,slow_kd_f);
    ReadCalibrateF();
      if (F[3] > Ref || F[12] > Ref) break;
  }
}

void BackToFront(){
  while(1){
    PIDB(bctL,bctR,slow_kp_b,slow_kd_b);
    ReadCalibrateB();
      if (B[3] > Ref || B[12] > Ref) break;
  }
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
  // Motor(-bctL, -bctR);
  // delay(20);
  while (1) {
    // Motor(-bctL, -bctR);
    if (line_centor == 0) {
      Motor(-bctL, -bctR);
    } else {
      PIDB_none(bctL, bctR, slow_kp_b, slow_kd_b);
    }
    ReadSensor();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(5);
      MotorStop();
      BZoff();
      break;
    }
  }
}

// ---------- Turns / Spins ----------

void TurnLeft() {
  Motor(-LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
  while (1) {
    Motor(-LTurnSpdL, LTurnSpdR);
    ReadCalibrateF();
    if (F[5] >= Ref) {
      MotorStop();
      break;
    }
  }
}

void TurnRight() {
  Motor(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    Motor(RTurnSpdL, -RTurnSpdR);
    ReadCalibrateF();
    if (F[10] >= Ref) {
      MotorStop();
      break;
    }
  }
}

void SpinL(int Speed) {
  MotorStop();
  delay(10);
  Motor(-Speed, Speed);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(-Speed, Speed);
    if (F[6] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateF();
    Motor(-Speed, Speed);
    if (F[6] >= Ref) {
      Motor(Speed, -Speed);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinL() {
SpinL(tspd);
}

void SpinL2(int Speed) {
  MotorStop();
  delay(10);
  Motor(-Speed, Speed);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(-Speed, Speed);
    if (F[6] >= Ref) break;
  }
  Motor(-Speed, Speed);
  delay(30);
  while (1) {
    ReadCalibrateF();
    Motor(-Speed, Speed);
    if (F[6] >= Ref) {
      Motor(Speed, -Speed);
      delay(5);
      MotorStop();
      break;
    }
  }
}
void SpinL2(){
  SpinL2(tspd);
}

void SpinR(int Speed) {
  MotorStop();
  delay(10);
  Motor(Speed, -Speed);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(Speed, -Speed);
    if (F[9] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateF();
    Motor(Speed, -Speed);
    if (F[9] >= Ref) {
      Motor(-Speed, Speed);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinR(){
  SpinR(tspd);
}
void SpinR2(int Speed) {
  MotorStop();
  delay(10);
  Motor(Speed, -Speed);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(Speed, -Speed);
    if (F[9] >= Ref) break;
  }
  Motor(Speed, -Speed);
  delay(30);
  while (1) {
    ReadCalibrateF();
    Motor(Speed, -Speed);
    if (F[9] >= Ref) {
      Motor(-Speed, Speed);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinR2(){
  SpinR2(tspd);
}

// back sensor
void TurnLeft_B() {
  Motor(-LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
  while (1) {
    Motor(-LTurnSpdL, LTurnSpdR);
    ReadCalibrateB();
    if (B[5] >= Ref) {
      MotorStop();
      break;
    }
  }
}

void TurnRight_B() {
  Motor(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    Motor(RTurnSpdL, -RTurnSpdR);
    ReadCalibrateB();
    if (B[10] >= Ref) {
      MotorStop();
      break;
    }
  }
}

void SpinL_B(int Speed) {
  MotorStop();
  delay(10);
  Motor(-Speed, Speed);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(-Speed, Speed);
    if (B[6] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateB();
    Motor(-Speed, Speed);
    if (B[6] >= Ref) {
      Motor(Speed, -Speed);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinL_B(){
  SpinL_B(tspd);
}

void SpinL2_B(int Speed) {
  MotorStop();
  delay(10);
  Motor(-Speed, Speed);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(-Speed, Speed);
    if (B[6] >= Ref) break;
  }
  Motor(-Speed, Speed);
  delay(30);
  while (1) {
    ReadCalibrateB();
    Motor(-Speed, Speed);
    if (B[6] >= Ref) {
      Motor(Speed, -Speed);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinL2_B(){
  SpinL2_B(tspd);
}

void SpinR_B(int Speed) {
  MotorStop();
  delay(10);
  Motor(Speed, -Speed);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(Speed, -Speed);
    if (B[9] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateB();
    Motor(Speed, -Speed);
    if (B[9] >= Ref) {
      Motor(-Speed, Speed);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinR_B(){
  SpinR_B(tspd);
}

void SpinR2_B(int Speed) {
  MotorStop();
  delay(10);
  Motor(Speed, -Speed);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(Speed, -Speed);
    if (B[9] >= Ref) break;
  }
  Motor(Speed, -Speed);
  delay(30);
  while (1) {
    ReadCalibrateB();
    Motor(Speed, -Speed);
    if (B[9] >= Ref) {
      Motor(-Speed, Speed);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinR2_B(){
  SpinR2_B(tspd);
}

// ---------- Track Select ----------

void TrackSelectF(int spd, char x) {
  if (x == 's') {
    MotorStop();
  } else if (x == 'S') {
    ToFront();
    MotorStop();
  } else if (x == 'p') {
    ReadCalibrateF();
    while (1) {
      Motor(spd, spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    delay(5);
    while (1) {
      Motor(spd, spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
  } else if (x == 'P') {
    ToFront();
    ReadCalibrateF();
    while (1) {
      Motor(spd, spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    delay(5);
    while (1) {
      Motor(spd, spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
  } else if (x == 'l' || x == 'L') {
    ToCenter();
    SpinL();
    FFtimer(0, 8);
  } else if (x == 'r' || x == 'R') {
    ToCenter();
    SpinR();
    FFtimer(0, 8);
  } else if (x == 'q') {
    while (1) {
      Motor(tctL / 2, tctR / 2);
      ReadCalibrateF();
      if (F[3] < Ref) break;
    }
    TurnLeft();
    FFtimer(0, 5);
  } else if (x == 'Q') {
    ToFront();
    while (1) {
      Motor(tctL / 2, tctR / 2);
      ReadCalibrateF();
      if (F[3] < Ref) break;
    }
    TurnLeft();
    FFtimer(0, 5);
  } else if (x == 'e') {
    while (1) {
      Motor(tctL / 2, tctR / 2);
      ReadCalibrateF();
      if (F[12] < Ref) break;
    }
    TurnRight();
    FFtimer(0, 5);
  } else if (x == 'E') {
    ToFront();
    while (1) {
      Motor(tctL / 2, tctR / 2);
      ReadCalibrateF();
      if (F[12] < Ref) break;
    }
    TurnRight();
    FFtimer(0, 5);
  } else if (x == 'c' || x == 'C') {
    ToCenter();
  } else if (x == 'd' || x == 'D') {
    ToCenter();
    SpinR_B();
    BBtimer(0, 8);
  } else if (x == 'a' || x == 'A') {
    ToCenter();
    SpinL_B();
    BBtimer(0, 8);
  } else if (x == 'g' || x == 'G') {
    SetFG(100);
  } else {
    MotorStop();
  }
  // Beep(20);
}

void TrackSelectB(int spd, char x) {
  if (x == 's' || x == 'S') {
    MotorStop();
  } else if (x == 'p' || x == 'P') {
    ReadCalibrateB();
    while (1) {
      Motor(-spd, -spd);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    delay(5);
    while (1) {
      Motor(-spd, -spd);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
  } else if (x == 'l' || x == 'L') {
    BackCenter();
    SpinL();
    FFtimer(0, 2);
  } else if (x == 'r' || x == 'R') {
    BackCenter();
    SpinR();
    FFtimer(0, 2);
  } else if (x == 'c' || x == 'C') {
    BackCenter();
  } else if (x == 'd' || x == 'D') {
    BackCenter();
    SpinR_B();
    BBtimer(0, 8);
  } else if (x == 'a' || x == 'A') {
    BackCenter();
    SpinL_B();
    BBtimer(0, 8);
  } else if (x == 'e' || x == 'E') {
    while (1) {
      Motor(-spd / 2, -spd / 2);
      ReadCalibrateB();
      if (B[3] < Ref) break;
    }
    TurnLeft_B();
    BBtimer(0, 5);
  } else if (x == 'q' || x == 'Q') {
    while (1) {
      Motor(-spd / 2, -spd / 2);
      ReadCalibrateB();
      if (B[12] < Ref) break;
    }
    TurnRight_B();
    BBtimer(0, 5);
  } else if (x == 'g' || x == 'G') {
    SetBG(100);
  } else {
    MotorStop();
  }
  Beep(20);
}

void FFtimer(int Speed, int totalTime, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
  TrackSelectF(Speed, select);
}

void BBtimer(int Speed, int totalTime, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
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
// ---------- FF / BB Patterns ----------

void FFC(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[5] >= Ref && F[10] >= Ref) || (F[4] >= Ref && F[11] >= Ref) ||
        (F[3] >= Ref && F[12] >= Ref) || (F[5] >= Ref && F[6] >= Ref && F[9] >= Ref && F[10] >= Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFC2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] >= Ref && F[15] >= Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBC(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[5] >= Ref && B[10] >= Ref) || (B[4] >= Ref && B[11] >= Ref) ||
        (B[3] >= Ref && B[12] >= Ref) || (B[5] >= Ref && B[6] >= Ref && B[9] >= Ref && B[10] >= Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBC2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] >= Ref && B[15] >= Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFL(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[1] > Ref && F[5] > Ref) || (F[2] > Ref && F[6] > Ref) || (F[3] > Ref && F[7] > Ref) || (F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref &&
        F[8] < Ref && F[9] < Ref && F[10] < Ref && F[11] < Ref && F[12] < Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBL(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[1] > Ref && B[5] > Ref) || (B[2] > Ref && B[6] > Ref) || (B[3] > Ref && B[7] > Ref) || ( B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref &&
        B[8] < Ref && B[9] < Ref && B[10] < Ref && B[11] < Ref && B[12] < Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void FFR(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[14] > Ref && F[10] > Ref) || (F[13] > Ref && F[9] > Ref) || (F[12] > Ref && F[8] > Ref)||( F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref &&
        F[8] < Ref && F[9] < Ref && F[10] < Ref && F[11] < Ref && F[12] < Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBR(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[14] > Ref && B[10] > Ref) || (B[13] > Ref && B[9] > Ref) || (B[12] > Ref && B[8] > Ref) ||( B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref &&
        B[8] < Ref && B[9] < Ref && B[10] < Ref && B[11] < Ref && B[12] < Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBL0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref || (B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref &&
        B[8] < Ref && B[9] < Ref && B[10] < Ref && B[11] < Ref && B[12] < Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBR15(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[15] > Ref || (B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref &&
        B[8] < Ref && B[9] < Ref && B[10] < Ref && B[11] < Ref && B[12] < Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void FFL0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref || ( F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref &&
        F[8] < Ref && F[9] < Ref && F[10] < Ref && F[11] < Ref && F[12] < Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFR15(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[15] > Ref || (F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref &&
        F[8] < Ref && F[9] < Ref && F[10] < Ref && F[11] < Ref && F[12] < Ref)) break;
  }
  TrackSelectF(Speed, select);
}

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

void FFWhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[3] < Ref && F[4] < Ref && F[5] < Ref && F[6] < Ref && F[7] < Ref &&
        F[8] < Ref && F[9] < Ref && F[10] < Ref && F[11] < Ref && F[12] < Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBWhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[3] < Ref && B[4] < Ref && B[5] < Ref && B[6] < Ref && B[7] < Ref &&
        B[8] < Ref && B[9] < Ref && B[10] < Ref && B[11] < Ref && B[12] < Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFNUM(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[numm] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBNUM(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[numm] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

// ---------- GoStart / GoEnd ----------

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

// ---------- Balance ----------

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
          if (C[1] > RefC) {
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

//ฉบับใส่ kp kd เอง

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
// ---------- FF / BB Patterns ----------

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

void FFCirCleL(int Speed, char select) {
  int temp_position = set_position;
  set_position = set_position_l;
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp_position;
}

void FFCirCleR(int Speed, char select) {
  int temp_position = set_position;
  set_position = set_position_r;
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed, RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[15] > Ref) break;
  }
  TrackSelectF(Speed, select);
  set_position = temp_position;
}

void BBCirCleL(int Speed, char select) {
  int temp_position = set_position;
  set_position = set_position_l;
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp_position;
}

void BBCirCleR(int Speed, char select) {
  int temp_position = set_position;
  set_position = set_position_r;
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed, BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[15] > Ref) break;
  }
  TrackSelectB(Speed, select);
  set_position = temp_position;
}

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

void FFtimerCirCleL(int Speed, int totalTime, char select) {
  FFtimerCirCleL(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void FFtimerCirCleR(int Speed, int totalTime, char select) {
  FFtimerCirCleR(Speed, totalTime);
  TrackSelectF(Speed, select);
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

void BBtimerCirCleL(int Speed, int totalTime, char select) {
  BBtimerCirCleL(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void BBtimerCirCleR(int Speed, int totalTime, char select) {
  BBtimerCirCleR(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void FFcmCirCleL(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  FFcm(Speed, totalTime);
  set_position = temp_position;
}

void FFcmCirCleR(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  FFcm(Speed, totalTime);
  set_position = temp_position;
}

void FFcmCirCleL(int Speed, int totalTime, char select) {
  FFcmCirCleL(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void FFcmCirCleR(int Speed, int totalTime, char select) {
  FFcmCirCleR(Speed, totalTime);
  TrackSelectF(Speed, select);
}

void BBcmCirCleL(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_l;
  BBcm(Speed, totalTime);
  set_position = temp_position;
}

void BBcmCirCleR(int Speed, int totalTime) {
  int temp_position = set_position;
  set_position = set_position_r;
  BBcm(Speed, totalTime);
  set_position = temp_position;
}

void BBcmCirCleL(int Speed, int totalTime, char select) {
  BBcmCirCleL(Speed, totalTime);
  TrackSelectB(Speed, select);
}

void BBcmCirCleR(int Speed, int totalTime, char select) {
  BBcmCirCleR(Speed, totalTime);
  TrackSelectB(Speed, select);
}

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
