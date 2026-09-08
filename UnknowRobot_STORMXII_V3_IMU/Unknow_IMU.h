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

/* ---------- spin / turn ---------- */

void spinDegree(int Speed, int relative_degree) {
  int min_speed = 8;
  int max_speed = Speed;
  float kp = 0.75;
  float kd = 0.9;
  float small_angle_threshold = 8;
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
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
  Stop(200);
}


void spinDegree(int relative_degree){
  spinDegree(40, relative_degree);
}
void turnDegree(int Speed, int relative_degree) {
  int min_speed = 10;
  int max_speed = Speed;
  float kp = 0.75;
  float kd = 0.9;
  float small_angle_threshold = 10;
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
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
  int min_speed = 10;
  int max_speed = Speed;
  float kp = 0.75;
  float kd = 0.9;
  float small_angle_threshold = 10;
  float stop_threshold = 1.0;
  float previous_error = 0;
  float target_degree = current_degree + relative_degree;
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
  turnDegree_none(50, relative_degree);
}

void turnDegreeb_none(int relative_degree) {
  turnDegreeb_none(50, relative_degree);
}
void turnDegree(int relative_degree){
  turnDegree(50, relative_degree);
}

void turnDegreeB(int relative_degree){
  turnDegreeB(50, relative_degree);
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


void RunG(int SpeedL,int SpeedR) {
  float error = current_degree - angleRead();
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorG;
  int pd_value = (error * kpG) + (derivative * kdG);
  int leftPow = SpeedL + pd_value;
  int rightPow = SpeedR - pd_value;
  if (leftPow > SpeedL) leftPow = SpeedL;
  if (leftPow < 0) leftPow = -5;
  if (rightPow > SpeedR) rightPow = SpeedR;
  if (rightPow < 0) rightPow = -5;
  Motor(leftPow, rightPow);
  previous_errorG = error;
}

void RunGB(int SpeedL,int SpeedR) {
  float error = current_degree - angleRead();
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorGB;
  int pd_value = (error * kpGB  ) + (derivative * kdGB);
  int leftPow = SpeedL - pd_value;
  int rightPow = SpeedR + pd_value;
  if (leftPow > SpeedL) leftPow = SpeedL;
  if (leftPow < 0) leftPow = -5;
  if (rightPow > SpeedR) rightPow = SpeedR;
  if (rightPow < 0) rightPow = -5;
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
  } else if (select == 'P') {
    ToFrontG();
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
    SetG(100);
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
  } else if (select == 'p' || select == 'P') {
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
    SetG(100);
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

  SetRobotAngle();  // เซ็ตค่าปัจจุบัน
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
    speed_scale = 1.7;  // คุณสามารถเปลี่ยนเป็น 0.95, 0.98, 1.0 ได้ตามต้องการ
  }
  SetRobotAngle();  // เซ็ตค่าปัจจุบัน

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

  SetRobotAngle();  // เซ็ตค่าปัจจุบัน
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
  SetRobotAngle();  // เซ็ตค่าปัจจุบัน

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
