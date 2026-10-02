#pragma once
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Unknow_Motor.h"

/* ===========================
 *         IMU CONFIG
 * =========================== */

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

float angles_offset[] = { 0, 0, 0 };
float current_degree = 0;
float power_factor = 1.0;
int previous_errorG;
int previous_errorGB;
bool useDirectionG = false;  // true = คำสั่ง cm ใช้ทิศที่ตั้งจาก SetDirectionG แทนการจำมุมปัจจุบัน

/* ---------- angle read ---------- */

void setAngleOffset() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angles_offset[2] = euler.x();
}

float angleRead() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float adjusted = euler.x() - angles_offset[2];
  if (adjusted >= 360)
    adjusted -= 360;
  else if (adjusted < 0)
    adjusted += 360;
  return adjusted;
}

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

void HoldAngle() {
  float error = current_degree - angleRead();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHold;
  int power = (error * kpHold) + (d * kdHold);
  power = constrain(power, -50, 50);   // แรงหมุน
  Motor(power, -power);   // หมุนอยู่กับที่
  prevErrHold = error;
}

void HoldAngleB() {
  float error = current_degree - angleRead();
  // wrap -180 ถึง 180
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float d = error - prevErrHoldB;
  int power = (error * kpBHold) + (d * kdBHold);
  power = constrain(power, -50, 50);   // แรงหมุน
  Motor(-power, power);   // หมุนอยู่กับที่
  prevErrHoldB = error;
}

void SetHoldAngle() {
  holdAngle = angleRead();   // มุมที่ต้องการให้หุ่น "จำ"
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
  power = constrain(power, -50, 50);   // แรงหมุน
  Motor(power, -power);   // หมุนอยู่กับที่
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

void SetG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngle();
  }
  BZoff();
}

void SetBG(int totalTime) {
  BZon();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    HoldAngleB();
  }
  BZoff();
}

void SetRobotAngle() {
  current_degree = angleRead();
}

// ===== ตัวแปรปรับค่าไจโร (ปรับได้ด้วย SetGyroTurn / SetGyroSpin ใน Setting.ino) =====
float gyro_Kp_Spin         = 0.75f;
float gyro_Kd_Spin         = 0.9f;
int   gyro_MaxSpd_Spin     = 40;
int   gyro_MinSpd_Spin     = 8;
float gyro_SmallAngle_Spin = 8.0f;
float gyro_StopThr_Spin    = 1.0f;
float gyro_Kp_Turn         = 0.75f;
float gyro_Kd_Turn         = 0.9f;
int   gyro_MaxSpd_Turn     = 50;
int   gyro_MinSpd_Turn     = 10;
float gyro_SmallAngle_Turn = 10.0f;
float gyro_StopThr_Turn    = 1.0f;
float gyro_StopThr_Rotate  = 1.0f;

void SetGyroTurn(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Turn         = kp;
  gyro_Kd_Turn         = kd;
  gyro_MaxSpd_Turn     = maxSpd;
  gyro_MinSpd_Turn     = minSpd;
  gyro_SmallAngle_Turn = smallAngle;
  gyro_StopThr_Turn    = stopThr;
}

void SetGyroSpin(float kp, float kd, int maxSpd, int minSpd, float smallAngle, float stopThr) {
  gyro_Kp_Spin         = kp;
  gyro_Kd_Spin         = kd;
  gyro_MaxSpd_Spin     = maxSpd;
  gyro_MinSpd_Spin     = minSpd;
  gyro_SmallAngle_Spin = smallAngle;
  gyro_StopThr_Spin    = stopThr;
}

/* ---------- spin / turn ---------- */

void spinDegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Spin;
  int max_speed = Speed;
  float kp = gyro_Kp_Spin;
  float kd = gyro_Kd_Spin;
  float small_angle_threshold = gyro_SmallAngle_Spin;
  float stop_threshold = gyro_StopThr_Spin;
  float previous_error = 0;
  // float target_degree = current_degree + relative_degree;
  float target_degree = angleRead() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
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
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, min_speed);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      Motor(pd_value, -pd_value);
    }
    previous_error = error;
  }
  Stop(100);
}


void spinDegree(int relative_degree){
  spinDegree(gyro_MaxSpd_Spin, relative_degree);
}
void turnDegree(int Speed, int relative_degree) {
  int min_speed = gyro_MinSpd_Turn;
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;
  float kd = gyro_Kd_Turn;
  float small_angle_threshold = gyro_SmallAngle_Turn;
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  // float target_degree = current_degree + relative_degree;
   float target_degree = angleRead() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
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
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(0, min_speed);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) Motor(-1, -pd_value);
      else if (error > 0) Motor(pd_value, 1);
    }
    previous_error = error;
  }
  SetG(10);
}

void turnDegreeB(int Speed ,int relative_degree) {
  int min_speed = gyro_MinSpd_Turn;
  int max_speed = Speed;
  float kp = gyro_Kp_Turn;
  float kd = gyro_Kd_Turn;
  float small_angle_threshold = gyro_SmallAngle_Turn;
  float stop_threshold = gyro_StopThr_Turn;
  float previous_error = 0;
  // float target_degree = current_degree + relative_degree;
   float target_degree = angleRead() + relative_degree;
  if (target_degree > 180) target_degree -= 360;
  if (target_degree < -180) target_degree += 360;
  current_degree = target_degree;
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
    } else if (error < -stop_threshold && error > -small_angle_threshold) {
      Motor(-min_speed, 0);
    } else if (error >= -stop_threshold && error <= stop_threshold) {
      MotorStop();
      break;
    } else {
      if (error <= 0) Motor(pd_value, 1);
      else if (error > 0) Motor(-1, -pd_value);
    }
    previous_error = error;
  }
  SetG(10);
}

void turnDegree_none(int Speed, int relative_degree) {
  float stop_threshold = 1.0;
  float target_degree = current_degree + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    if (error >= -stop_threshold && error <= stop_threshold) {
      //MotorStop();
      break;
    } else if (error > 0) {
      Motor(Speed, -2);
    } else {
      Motor(-2, Speed);
    }
  }
}

void turnDegreeb_none(int Speed, int relative_degree) {
  float stop_threshold = 1.0;
  float target_degree = current_degree + relative_degree;
  if (target_degree > 180.0f) target_degree -= 360.0f;
  if (target_degree < -180.0f) target_degree += 360.0f;
  current_degree = target_degree;

  while (1) {
    float current_angle = angleRead();
    float error = target_degree - current_angle;

    if (error > 180.0f) error -= 360.0f;
    else if (error < -180.0f) error += 360.0f;

    if (error >= -stop_threshold && error <= stop_threshold) {
      //MotorStop();
      break;
    } else if (error > 0) {
      Motor(1, -Speed);
    } else {
      Motor(-Speed, 1);
    }
  }
}

void turnDegree_none(int relative_degree) {
  turnDegree_none(gyro_MaxSpd_Turn, relative_degree);
}

void turnDegreeb_none(int relative_degree) {
  turnDegreeb_none(gyro_MaxSpd_Turn, relative_degree);
}
void turnDegree(int relative_degree){
  turnDegree(gyro_MaxSpd_Turn, relative_degree);
}

void turnDegreeB(int relative_degree){
  turnDegreeB(gyro_MaxSpd_Turn, relative_degree);
}

void SpinLG(int Angle) {
  spinDegree(-abs(Angle));
}
void SpinRG(int Angle) {
  spinDegree(abs(Angle));
}
void TurnLG(int Angle) {
  turnDegree(-abs(Angle));
}
void TurnRG(int Angle) {
  turnDegree(abs(Angle));
}
void TurnLBG(int Angle) {
  turnDegreeB(abs(Angle));
}
void TurnRBG(int Angle) {
  turnDegreeB(-abs(Angle));
}


void tlrg(int Angle) {turnDegree_none(-abs(Angle)); turnDegree(abs(Angle)); }
void trlg(int Angle) {turnDegree_none(abs(Angle)); turnDegree(-abs(Angle));}

void tlrg(int spd, int Angle) {turnDegree_none(spd, -abs(Angle)); turnDegree(spd, abs(Angle)); }
void trlg(int spd, int Angle) {turnDegree_none(spd, abs(Angle)); turnDegree(spd, -abs(Angle)); }

void tlrg(int spd, int Angle, int Angle2) {turnDegree_none(spd, -abs(Angle)); turnDegree(spd, abs(Angle2)); /*SetG(spd);*/}
void trlg(int spd, int Angle, int Angle2) {turnDegree_none(spd, abs(Angle)); turnDegree(spd, -abs(Angle2)); /*SetG(spd);*/}

void tlrbg(int Angle) {turnDegreeb_none(abs(Angle)); turnDegreeB(-abs(Angle)); /*SetGB(50);*/}
void trlbg(int Angle) {turnDegreeb_none(-abs(Angle)); turnDegreeB(abs(Angle)); /*SetGB(50);*/}

void tlrbg(int spd, int Angle) {turnDegreeb_none(spd, abs(Angle)); turnDegreeB(spd, -abs(Angle)); /*SetGB(spd);*/}
void trlbg(int spd, int Angle) {turnDegreeb_none(spd, -abs(Angle)); turnDegreeB(spd, abs(Angle)); /*SetGB(spd);*/}

void tlrbg(int spd, int Angle, int Angle2) {turnDegreeb_none(spd, abs(Angle)); turnDegreeB(spd, -abs(Angle2)); /*SetG(spd);*/}
void trlbg(int spd, int Angle, int Angle2) {turnDegreeb_none(spd, -abs(Angle)); turnDegreeB(spd, abs(Angle2)); /*SetG(spd);*/}




void TurnLRG(int Angle){
  tlrg(Angle);
}

void TurnRLG(int Angle){ trlg(Angle); }

void TurnLRG(int spd, int Angle){
  tlrg(spd,Angle);
}

void TurnRLG(int spd,int Angle){
  trlg(spd, Angle);
}

void TurnLRG(int spd, int Angle,int Angle2){
  tlrg(spd,Angle,Angle2);
}

void TurnRLG(int spd,int Angle,int Angle2){
  trlg(spd, Angle,Angle2);
}

void TurnLRBG(int Angle){
  tlrbg(Angle);
}

void TurnRLBG(int Angle){
  trlbg(Angle);
}

void TurnLRBG(int spd, int Angle){
  tlrbg(spd,Angle);
}

void TurnRLBG(int spd,int Angle){
  trlbg(spd, Angle);
}

void TurnLRBG(int spd, int Angle,int Angle2){
  tlrbg(spd,Angle,Angle2);
}

void TurnRLBG(int spd,int Angle,int Angle2){
  trlbg(spd, Angle,Angle2);
}

/* ---------- gyro-guided straight move ---------- */
float kpG = 1.2;
float kdG = 1.5;
// float kdG = 10;
float kpGB = 1.2;
float kdGB = 1.5;
// float kdGB = 10;

void SetGyroRun(float kp, float kd) {
  kpG = kp;
  kdG = kd;
}

void SetGyroRunB(float kp, float kd) {
  kpGB = kp;
  kdGB = kd;
}

// โหมดจำกัดกำลังมอเตอร์ของ RunG (เดินหน้า) / RunGB (ถอยหลัง)
// ค่าเริ่มต้น = โหมด 4, min -5 (เหมือนพฤติกรรมเดิม) ปรับได้ด้วย ModeSpdGyro() ใน Setting.ino
int MaxSpeedG = 100;
int MinSpeedG = -5;
int ModeGyroStatus = 4;
int ModeGyroBStatus = 4;

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

void RunG(int SpeedL,int SpeedR) {
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
}

void RunGB(int SpeedL,int SpeedR) {
  float error = current_degree - angleRead();
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorGB;
  int pd_value = (error * kpGB  ) + (derivative * kdGB);
  float leftPow = SpeedL - pd_value;
  float rightPow = SpeedR + pd_value;
  ClampGyroPower(leftPow, rightPow, SpeedL, SpeedR, ModeGyroBStatus);
  Motor(-leftPow, -rightPow);
  previous_errorGB = error;
}



void FFtimerG(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(LeftBaseSpeed,RightBaseSpeed);
  }
}


void BBtimerG(int Speed, int totalTime) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunGB(BackLeftBaseSpeed,BackRightBaseSpeed);
  }
}

void ToCenterG(){
 RunG(tctL,tctR);
  delay(20);
  while (1) {
    RunG(tctL,tctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-tctL, -tctR);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void ToFrontG(){
 while (1) {
      RunG(LeftBaseSpeed,RightBaseSpeed);
      ReadCalibrateF();
      if (F[3] > Ref || F[12] > Ref) break;
    }
}

void BackToCenterG(){
 RunGB(bctL,bctR);
  delay(20);
  while (1) {
    RunGB(bctL,bctR);
    ReadCalibrateC();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(bctL, bctR);
      delay(5);
      MotorStop();
      break;
    }
  }

}

void BackToFrontG(){
 while (1) {
      RunGB(BackLeftBaseSpeed,BackRightBaseSpeed);
      ReadCalibrateB();
      if (B[3] > Ref || B[12] > Ref) break;
    }
}
/* ---------- track select (gyro) ---------- */

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
  }else if (select == 'P') {
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
  }
   else if (select == 'c') {
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

void FFBG(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunG(LeftBaseSpeed,RightBaseSpeed);
    ReadCalibrateF();
    if (F[5] > Ref || F[6] > Ref || F[7] > Ref || F[8] > Ref || F[9] > Ref || F[10] > Ref) break;
  }
  TrackSelectG(Speed,select);
}

void BBBG(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    RunGB(BackLeftBaseSpeed,BackRightBaseSpeed);
    ReadCalibrateB();
    if (B[5] > Ref || B[6] > Ref || B[7] > Ref || B[8] > Ref || B[9] > Ref || B[10] > Ref) break;
  }
  TrackSelectGB(Speed,select);
}

void FFtimerG(int Speed, int totalTime, char select) {
  FFtimerG(Speed,totalTime);
  TrackSelectG(Speed,select);
}


void BBtimerG(int Speed, int totalTime, char select) {
  BBtimerG(Speed,totalTime);
  TrackSelectGB(Speed,select);
}



void SpinLG(int spd, int Angle) {
  spinDegree(spd, -abs(Angle));
}
void SpinRG(int spd, int Angle) {
  spinDegree(spd, abs(Angle));
}



void TurnLG(int spd, int Angle) {
  turnDegree(spd, -abs(Angle));
}
void TurnRG(int spd ,int Angle) {
  turnDegree(spd, abs(Angle));
}


void TurnLBG(int spd, int Angle) {
  turnDegreeB(spd, abs(Angle));
}
void TurnRBG(int spd, int Angle) {
  turnDegreeB(spd, -abs(Angle));
}


void FFcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(LeftBaseSpeed, RightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();

  float speed_scale = 1.75;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

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
      traveled_distance += (target_speed * speed_scale) * delta_time;
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
  float speed_scale = 0.99;  // ← คุณสามารถปรับตรงนี้ได้

  // ตัดสินใจว่าใช้ Ramp หรือไม่
  bool enableRamp = (distance_cm >= 30.0);

  // ถ้าระยะสั้นมาก (< 30) ให้ปรับ speed_scale ได้ง่ายขึ้น
  if (!enableRamp) {
    speed_scale = 1.75;  // คุณสามารถเปลี่ยนเป็น 0.95, 0.98, 1.0 ได้ตามต้องการ
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

void BBcmGS(int Speed, float distance) {
  BaseSpeed = Speed;
  InitialSpeed();
  int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed);
  float traveled_distance = 0;
  unsigned long last_time = millis();

  float speed_scale = 1.5;  // ใช้ค่าที่คาลิเบรตจาก fw()

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
      traveled_distance += (target_speed * speed_scale) * delta_time;
      last_time = current_time;

      if (traveled_distance >= distance) break;
    }
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

  float speed_scale = 0.99;  // ค่าเริ่มต้นสำหรับถอยหลัง

  const float ACCEL_DISTANCE_CM = 20.0;
  const float DECEL_DISTANCE_CM = 25.0;
  const float MIN_SPEED = 10.0;

  // ตัดสินใจว่าใช้ Ramp หรือไม่
  bool enableRamp = (distance_cm >= 30.0);

  // ถ้าระยะสั้นมาก (< 30) → ไม่ใช้ Ramp + ปรับ speed_scale
  if (!enableRamp) {
    speed_scale = 1.5;  // คุณสามารถปรับตรงนี้ได้ (แนะนำ 0.92 - 0.97)
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

/* ---------- sensor ---------- */

void resetAngles() {
  setAngleOffset();
  current_degree = 0;
  previous_errorG = 0;
  previous_errorGB = 0;
}

void resetAngle() {
  resetAngles();
}


/* ---------- rotate degree (arc: independent left/right cruise speed) ---------- */

void rotateDegree(int SpeedL, int SpeedR, int relative_degree, float kp, float kd) {
  float stop_threshold = gyro_StopThr_Rotate;
  float previous_error = 0;
  // float target_degree = current_degree + relative_degree;
   float target_degree = angleRead() + relative_degree;
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
  }
  SetG(max(abs(SpeedL), abs(SpeedR)));
}

void rotateDegree(int SpeedL, int SpeedR, int relative_degree) {
  rotateDegree(SpeedL, SpeedR, relative_degree, 0.9, 0.6);
}

/* ---------- turn to absolute direction (อ้างอิงจากตอน resetAngles) เช่น 0, 90, 180, 270, 360 ---------- */

// คำนวณมุมที่ต้องหมุนจากทิศปัจจุบันไปยังทิศทางสัมบูรณ์ (ทางที่สั้นที่สุด)
int relativeToDirection(int direction) {
  float relative = fmod((float)direction, 360.0f) - current_degree;
  while (relative > 180.0f) relative -= 360.0f;
  while (relative < -180.0f) relative += 360.0f;
  return (int)roundf(relative);
}

void spinDirection(int Speed, int direction) { spinDegree(Speed, relativeToDirection(direction)); }
void spinDirection(int direction) { spinDegree(relativeToDirection(direction)); }
void turnDirection(int Speed, int direction) { turnDegree(Speed, relativeToDirection(direction)); }
void turnDirection(int direction) { turnDegree(relativeToDirection(direction)); }
void turnDirectionB(int Speed, int direction) { turnDegreeB(Speed, relativeToDirection(direction)); }
void turnDirectionB(int direction) { turnDegreeB(relativeToDirection(direction)); }
void turnDirection_none(int Speed, int direction) { turnDegree_none(Speed, relativeToDirection(direction)); }
void turnDirectionb_none(int Speed, int direction) { turnDegreeb_none(Speed, relativeToDirection(direction)); }
void turnDirection_none(int direction) { turnDegree_none(relativeToDirection(direction)); }
void turnDirectionb_none(int direction) { turnDegreeb_none(relativeToDirection(direction)); }

// ตั้งทิศทางสัมบูรณ์ให้ RunG / RunGB วิ่งตรงตาม
void SetDirectionG(int direction) {
  float target = fmod((float)direction, 360.0f);
  if (target > 180) target -= 360;
  else if (target < -180) target += 360;
  current_degree = target;
  float error = current_degree - angleRead();
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  previous_errorG = error;
  previous_errorGB = error;
}

/* ---------- gyro straight with absolute direction ---------- */

void FFtimerG(int Speed, int totalTime, int direction) { SetDirectionG(direction); FFtimerG(Speed, totalTime); }
void BBtimerG(int Speed, int totalTime, int direction) { SetDirectionG(direction); BBtimerG(Speed, totalTime); }

void FFcmGS(int Speed, float distance_cm, int direction) { SetDirectionG(direction); useDirectionG = true; FFcmGS(Speed, distance_cm); useDirectionG = false; }
void BBcmGS(int Speed, float distance_cm, int direction) { SetDirectionG(direction); useDirectionG = true; BBcmGS(Speed, distance_cm); useDirectionG = false; }

void FFcmG(int Speed, float distance_cm, int direction) { SetDirectionG(direction); useDirectionG = true; FFcmG(Speed, distance_cm); useDirectionG = false; }
void BBcmG(int Speed, float distance_cm, int direction) { SetDirectionG(direction); useDirectionG = true; BBcmG(Speed, distance_cm); useDirectionG = false; }

void FFBG(int Speed, char select, int direction) { SetDirectionG(direction); FFBG(Speed, select); }
void BBBG(int Speed, char select, int direction) { SetDirectionG(direction); BBBG(Speed, select); }

/* ---------- with select + absolute direction ---------- */

void FFtimerG(int Speed, int totalTime, char select, int direction) { FFtimerG(Speed, totalTime, direction); TrackSelectG(Speed, select); }
void BBtimerG(int Speed, int totalTime, char select, int direction) { BBtimerG(Speed, totalTime, direction); TrackSelectGB(Speed, select); }

void FFcmGS(int Speed, float distance_cm, char select, int direction) { FFcmGS(Speed, distance_cm, direction); TrackSelectG(Speed, select); }
void BBcmGS(int Speed, float distance_cm, char select, int direction) { BBcmGS(Speed, distance_cm, direction); TrackSelectGB(Speed, select); }

void FFcmG(int Speed, float distance_cm, char select, int direction) { FFcmG(Speed, distance_cm, direction); TrackSelectG(Speed, select); }
void BBcmG(int Speed, float distance_cm, char select, int direction) { BBcmG(Speed, distance_cm, direction); TrackSelectGB(Speed, select); }

/* ---------- to center (gyro) เช็คเซนเซอร์กลางข้างเดียว ---------- */

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
