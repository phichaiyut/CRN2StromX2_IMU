#ifndef UNKNOW_PID_H
#define UNKNOW_PID_H

#include "Unknow_Sensor.h"
#include "Unknow_Motor.h"
#include "Unknow_Buzzer.h"

float PID_KP, PID_KD;
int LastError_F, LastError_B;
int Error_F = 0, Error_B = 0;
int tct, bct, tspd;
int tctL,tctR, bctL,bctR;
int LTurnSpdL, LTurnSpdR, TurnDelayL;
int RTurnSpdL, RTurnSpdR, TurnDelayR;
int set_position = 7500;
float slow_kp = 0.005,slow_kd = 0.05;


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
  InitialSpeed() ;
  tctL = LeftBaseSpeed ;
  tctR = RightBaseSpeed;
  bctL = BackLeftBaseSpeed;
  bctR = BackRightBaseSpeed;

}

void SetSlowKpKd(float sl_kp,float sl_kd){
  slow_kp = sl_kp;
  slow_kd = sl_kd;
}

void SetTurnSpeed(int tspdv) {
  tspd = tspdv;
}

void TurnSpeedLeft(int l, int r, int de) {
  LTurnSpdL  = l;
  LTurnSpdR  = r;
  TurnDelayL = de;
}

void TurnSpeedRight(int l, int r, int de) {
  RTurnSpdL  = l;
  RTurnSpdR  = r;
  TurnDelayR = de;
}

void ModeSpdPID(int moD, int maX, int miN){
  ModePidStatus = moD;
  MaxSpeed = maX;
  MinSpeed = miN;
}

void set_position_line(int _pos){
 if (_pos < 0) {
    set_position = 0;
  } 
  else if (_pos > 15000) {
    set_position = 15000;
  } 
  else {
    set_position = _pos;
  }
}

void set_sensor_track_line(int L ,int R){
  setsensortracklineL = L;
  setsensortracklineR = R;
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
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = (TrackLineCH - 1) * 1000 / 2;
  ReadCalibrateF();
  for (i = setsensortracklineL; i <= setsensortracklineR; i++) {
    int values = F[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (TrackLineCH - 1) * 1000 / 2) return setsensortracklineL * 1000;
    else return setsensortracklineR * 1000;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionB(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = (TrackLineCH - 1) * 1000 / 2;
  ReadCalibrateB();
  for (i = setsensortracklineL; i <= setsensortracklineR; i++)  {
    int values = B[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
     if (last_value < (TrackLineCH - 1) * 1000 / 2) return setsensortracklineL * 1000;
    else return setsensortracklineR * 1000;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionF_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = (TrackLineCH - 1) * 1000 / 2;
  ReadCalibrateF();
  for (i = 0; i < TrackLineCH; i++) {
    int values;
    if      (TrackLineCH == 6)  values = F[i + 5];
    else if (TrackLineCH == 8)  values = F[i + 4];
    else if (TrackLineCH == 10) values = F[i + 3];
    else if (TrackLineCH == 12) values = F[i + 2];
    else if (TrackLineCH == 14) values = F[i + 1];
    else                        values = F[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (TrackLineCH - 1) * 1000 / 2) return 7500;
    else return 7500;
  }
  last_value = avg / sum;
  return last_value;
}

int readPositionB_none(int Track, int noise) {
  unsigned char i, online = 0;
  unsigned long avg = 0;
  unsigned long  sum = 0;
  static int last_value = (TrackLineCH - 1) * 1000 / 2;
  ReadCalibrateB();
  for (i = 0; i < TrackLineCH; i++) {
    int values;
    if      (TrackLineCH == 6)  values = B[i + 5];
    else if (TrackLineCH == 8)  values = B[i + 4];
    else if (TrackLineCH == 10) values = B[i + 3];
    else if (TrackLineCH == 12) values = B[i + 2];
    else if (TrackLineCH == 14) values = B[i + 1];
    else                        values = B[i];
    if (values > Track) online = 1;
    if (values > noise) {
      avg += (long)(values) * (i * 1000);
      sum += values;
    }
  }
  if (!online) {
    if (last_value < (TrackLineCH - 1) * 1000 / 2) return 7500;
    else return 7500;
  }
  last_value = avg / sum;
  return last_value;
}

// ---------- PID ----------

void PIDF(int SpeedL,int SpeedR, float Kp, float Kd) {
  int Pos      = readPositionF(250, 50);
  int Error    = Pos - ((TrackLineCH - 1) * 1000 / 2);
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_F));
  LastError_F  = Error;
  int LeftPower  = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  // if (leftPow  > 100) leftPow  = 100;
  // if (leftPow  < 0)   leftPow  = -5;
  // if (rightPow > 100) rightPow = 100;
  // if (rightPow < 0)   rightPow = -5;
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

  Motor(LeftPower, RightPower);
}

void PIDB(int SpeedL,int SpeedR, float Kp, float Kd) {
  int Pos      = readPositionB(250, 50);
  int Error    = Pos - ((TrackLineCH - 1) * 1000 / 2);
  int PID_Value = (Kp * Error) + (Kd * (Error - LastError_B));
  LastError_B  = Error;
 int LeftPower  = SpeedL + PID_Value;
  int RightPower = SpeedR - PID_Value;
  // if (leftPow  > 100) leftPow  = 100;
  // if (leftPow  < 0)   leftPow  = -5;
  // if (rightPow > 100) rightPow = 100;
  // if (rightPow < 0)   rightPow = -5;
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
  Motor(-LeftPower, -RightPower);
}

// ---------- Timed Motion ----------

void FFtimer(int baseSpeed, int totalTime) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed,RightBaseSpeed, PID_KP_Front, PID_KD_Front);
}

void BBtimer(int baseSpeed, int totalTime) {
  BaseSpeed = baseSpeed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(LeftBaseSpeed,RightBaseSpeed, PID_KP_Back, PID_KD_Back);
}


// ---------- ToCenter / BackCenter ----------

void ToCenter() {
  BZon();
  Motor(tctL, tctR);
  delay(20);
  while (1) {
    Motor(tctL, tctR);
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
  Motor(tctL, tctR);
  delay(20);
  while (1) {
    Motor(tctL, tctR);
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
  Motor(tctL, tctR);
  delay(20);
  while (1) {
    Motor(tctL, tctR);
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

void BackCenter() {
  BZon();
  Motor(-bctL, -bctR);
  delay(20);
  while (1) {
    Motor(-bctL, -bctR);
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
    if (F[5] >= Ref) { MotorStop(); break; }
  }
}

void TurnRight() {
  Motor(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    Motor(RTurnSpdL, -RTurnSpdR);
    ReadCalibrateF();
    if (F[10] >= Ref) { MotorStop(); break; }
  }
}

void SpinL() {
  MotorStop();
  delay(10);
  Motor(-tspd, tspd);
  delay(60); while (1) {
    ReadCalibrateF();
    Motor(-tspd, tspd);
    if (F[6] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateF();
    Motor(-tspd, tspd);
    if (F[6] >= Ref) {
      Motor(tspd, -tspd);
      delay(5);
      MotorStop();
      break;
    }
  }
}



void SpinL2() {
  MotorStop();
  delay(10);
  Motor(-tspd, tspd);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(-tspd, tspd);
    if (F[6] >= Ref) break;
  }
  Motor(-tspd, tspd);
  delay(30);
  while (1) {
    ReadCalibrateF();
    Motor(-tspd, tspd);
    if (F[6] >= Ref) {
      Motor(tspd, -tspd);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinR() {
  MotorStop();
  delay(10);
  Motor(tspd, -tspd);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(tspd, -tspd);
    if (F[9] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateF();
    Motor(tspd, -tspd);
    if (F[9] >= Ref) {
      Motor(-tspd, tspd);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinR2() {
  MotorStop();
  delay(10);
  Motor(tspd, -tspd);
  delay(60);
  while (1) {
    ReadCalibrateF();
    Motor(tspd, -tspd);
    if (F[9] >= Ref) break;
  }
  Motor(tspd, -tspd);
  delay(30);
  while (1) {
    ReadCalibrateF();
    Motor(tspd, -tspd);
    if (F[9] >= Ref) {
      Motor(-tspd, tspd);
      delay(5);
      MotorStop();
      break;
    }
  }
}


//back sensor
void TurnLeft_B() {
  Motor(-LTurnSpdL, LTurnSpdR);
  delay(TurnDelayL);
  while (1) {
    Motor(-LTurnSpdL, LTurnSpdR);
    ReadCalibrateB();
    if (B[5] >= Ref) { MotorStop(); break; }
  }
}

void TurnRight_B() {
  Motor(RTurnSpdL, -RTurnSpdR);
  delay(TurnDelayR);
  while (1) {
    Motor(RTurnSpdL, -RTurnSpdR);
    ReadCalibrateB();
    if (F[10] >= Ref) { MotorStop(); break; }
  }
}

void SpinL_B() {
  MotorStop();
  delay(10);
  Motor(-tspd, tspd);
  delay(60); while (1) {
    ReadCalibrateB();
    Motor(-tspd, tspd);
    if (B[6] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateF();
    Motor(-tspd, tspd);
    if (F[6] >= Ref) {
      Motor(tspd, -tspd);
      delay(5);
      MotorStop();
      break;
    }
  }
}



void SpinL2_B() {
  MotorStop();
  delay(10);
  Motor(-tspd, tspd);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(-tspd, tspd);
    if (B[6] >= Ref) break;
  }
  Motor(-tspd, tspd);
  delay(30);
  while (1) {
    ReadCalibrateB();
    Motor(-tspd, tspd);
    if (B[6] >= Ref) {
      Motor(tspd, -tspd);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinR_B() {
  MotorStop();
  delay(10);
  Motor(tspd, -tspd);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(tspd, -tspd);
    if (B[9] <= Ref) {
      break;
    }
  }
  while (1) {
    ReadCalibrateB();
    Motor(tspd, -tspd);
    if (B[9] >= Ref) {
      Motor(-tspd, tspd);
      delay(5);
      MotorStop();
      break;
    }
  }
}

void SpinR2_B() {
  MotorStop();
  delay(10);
  Motor(tspd, -tspd);
  delay(60);
  while (1) {
    ReadCalibrateB();
    Motor(tspd, -tspd);
    if (B[9] >= Ref) break;
  }
  Motor(tspd, -tspd);
  delay(30);
  while (1) {
    ReadCalibrateB();
    Motor(tspd, -tspd);
    if (B[9] >= Ref) {
      Motor(-tspd, tspd);
      delay(5);
      MotorStop();
      break;
    }
  }
}

// ---------- Track Select ----------

void TrackSelectF(int spd, char x) {
  if (x == 's' || x == 'S') {
    MotorStop();
  } else if (x == 'p' || x == 'P') {
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
  } else if (x == 'q' || x == 'Q') {
    while (1) {
      Motor(spd / 2, spd / 2);
      ReadCalibrateF();
      if (F[3] < Ref) break;
    }
    TurnLeft();
    FFtimer(0, 5);
  } else if (x == 'e' || x == 'E') {
    while (1) {
      Motor(spd / 2, spd / 2);
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
  }
  else if (x == 'a' || x == 'A') {
    ToCenter();
    SpinL_B();
    BBtimer(0, 8);
  }else {
    MotorStop();
  }
  Beep(20);
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
  } 
  else if (x == 'd' || x == 'D') {
    BackCenter();
    SpinR_B();
    BBtimer(0, 8);
  }
  else if (x == 'a' || x == 'A') {
    BackCenter();
    SpinL_B();
    BBtimer(0, 8);
  }else if (x == 'e' || x == 'E'){
    while (1) {
      Motor(-spd / 2, -spd / 2);
      ReadCalibrateB();
      if (B[3] < Ref) break;
    }
    TurnLeft_B();
    BBtimer(0, 5);
  } else if (x == 'q' || x == 'Q')  {
    while (1) {
      Motor(-spd / 2, -spd / 2);
      ReadCalibrateB();
      if (B[12] < Ref) break;
    }
  }
  else {
    MotorStop();
  }
  Beep(20);
}


void FFtimer(int Speed, int totalTime,char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed,RightBaseSpeed, PID_KP_Front , PID_KD_Front );
  TrackSelectF(Speed, select);
}

void BBtimer(int Speed, int totalTime,char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
  TrackSelectB(Speed, select);
}
// ---------- FF / BB Patterns ----------

void FFC(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[5]>=Ref && F[10]>=Ref) || (F[4]>=Ref && F[11]>=Ref) ||
        (F[3]>=Ref && F[12]>=Ref) || (F[5]>=Ref && F[6]>=Ref && F[9]>=Ref && F[10]>=Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFC2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed, PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[0]>=Ref && F[15]>=Ref ) break;
  }
  TrackSelectF(Speed, select);
}

void BBC(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[5]>=Ref && B[10]>=Ref) || (B[4]>=Ref && B[11]>=Ref) ||
        (B[3]>=Ref && B[12]>=Ref) || (B[5]>=Ref && B[6]>=Ref && B[9]>=Ref && B[10]>=Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBC2(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0]>=Ref && B[15]>=Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFL(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed,  PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[1]>Ref&&F[5]>Ref)||(F[2]>Ref&&F[6]>Ref)||(F[3]>Ref&&F[7]>Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBL(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[1]>Ref&&B[5]>Ref)||(B[2]>Ref&&B[6]>Ref)||(B[3]>Ref&&B[7]>Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void FFR(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed,  PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if ((F[14]>Ref&&F[10]>Ref)||(F[13]>Ref&&F[9]>Ref)||(F[12]>Ref&&F[8]>Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBR(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if ((B[14]>Ref&&B[10]>Ref)||(B[13]>Ref&&B[9]>Ref)||(B[12]>Ref&&B[8]>Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBL0(int Speed, char select) {
BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void BBR15(int Speed, char select) {
BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB();
    if (B[15] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFL0(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDF(LeftBaseSpeed,RightBaseSpeed,  PID_KP_Front, PID_KD_Back);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFR15(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDF(LeftBaseSpeed,RightBaseSpeed,  PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[15] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFBlack(int SpeedL, int SpeedR, char select) {
  Move(SpeedL, SpeedR, 50);
  while (1) {
    Motor(SpeedL, SpeedR);
    ReadCalibrateF();
    if (F[5]>Ref||F[6]>Ref||F[7]>Ref||F[8]>Ref||F[9]>Ref||F[10]>Ref) break;
  }
  TrackSelectF(SpeedL, select);
}

void FFBlack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(LeftBaseSpeed ,RightBaseSpeed ,50);
  while (1) {
    Motor(LeftBaseSpeed, RightBaseSpeed);
    ReadCalibrateF();
    if (F[5]>Ref||F[6]>Ref||F[7]>Ref||F[8]>Ref||F[9]>Ref||F[10]>Ref) break;
  }
  TrackSelectF(Speed, select);
}


void BBBlack(int SpeedL, int SpeedR, char select) {
  Move(-SpeedL, -SpeedR, 50);
  while (1) {
    Motor(-SpeedL, -SpeedR);
    ReadCalibrateB();
    if (B[5]>Ref||B[6]>Ref||B[7]>Ref||B[8]>Ref||B[9]>Ref||B[10]>Ref) break;
  }
  TrackSelectB(SpeedL, select);
}
void BBBlack(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(-BackLeftBaseSpeed ,-RightBaseSpeed ,50);
  while (1) {
    Motor(-BackLeftBaseSpeed, -RightBaseSpeed);
    ReadCalibrateB();
    if (B[5]>Ref||B[6]>Ref||B[7]>Ref||B[8]>Ref||B[9]>Ref||B[10]>Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFWhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed,  PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[3]<Ref&&F[4]<Ref&&F[5]<Ref&&F[6]<Ref&&F[7]<Ref&&
        F[8]<Ref&&F[9]<Ref&&F[10]<Ref&&F[11]<Ref&&F[12]<Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBWhite(int Speed, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
    ReadCalibrateB(); 
    if (B[3]<Ref&&B[4]<Ref&&B[5]<Ref&&B[6]<Ref&&B[7]<Ref&&
        B[8]<Ref&&B[9]<Ref&&B[10]<Ref&&B[11]<Ref&&B[12]<Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFNUM(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed,  PID_KP_Front, PID_KD_Front);
    ReadCalibrateF();
    if (F[numm] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBNUM(int Speed, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, PID_KP_Back, PID_KD_Back);
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
      Move(LeftSpeed, RightSpeed, 100);
      break;
    }
  }
}

void GoStart(int Speed) {
  BaseSpeed = Speed;
  InitialSpeed();
  Move(LeftBaseSpeed ,RightBaseSpeed, 100);
  while (1) {
    ReadCalibrateF();
    Motor(LeftBaseSpeed, RightBaseSpeed);
    if (F[0] < 500 && F[15] < 500) {
      Move(LeftBaseSpeed, RightBaseSpeed, 100);
      break;
    }
  }
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
  while (1) {
    ReadCalibrateC();
    Motor(LeftBaseSpeed, RightBaseSpeed);
    if (C[0] > 500 && C[1] > 500) {
      Move(LeftBaseSpeed, RightBaseSpeed, 100);
      break;
    }
  }
  MotorStop();
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
          if (F[15] > Ref) { MotorStop(); break; }
        }
      }
      if (F[15] > Ref) {
        while (1) {
          Motor(5, 0);
          ReadCalibrateF();
          if (F[0] > Ref) { MotorStop(); break; }
        }
      }
      if (F[0] > Ref && F[15] > Ref) { MotorStop(); break; }
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
          if (B[15] > Ref) { MotorStop(); break; }
        }
      }
      if (B[15] > Ref) {
        while (1) {
          Motor(-5, 0);
          ReadCalibrateB();
          if (B[0] > Ref) { MotorStop(); break; }
        }
      }
      if (B[0] > Ref && B[15] > Ref) { MotorStop(); break; }
    }
    MotorStop();
    delay(50);
  }
}


//ฉบับใส่ kp kd เอง

void FFtimer(int Speed,float kp,float kd, int totalTime,char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDF(LeftBaseSpeed,RightBaseSpeed, kp, kd );
  TrackSelectF(Speed, select);
}

void BBtimer(int Speed,float kp,float kd, int totalTime,char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
  TrackSelectB(Speed, select);
}
// ---------- FF / BB Patterns ----------

void FFC(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if ((F[5]>=Ref && F[10]>=Ref) || (F[4]>=Ref && F[11]>=Ref) ||
        (F[3]>=Ref && F[12]>=Ref) || (F[5]>=Ref && F[6]>=Ref && F[9]>=Ref && F[10]>=Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void FFC2(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed, kp, kd);
    ReadCalibrateF();
    if (F[0]>=Ref && F[15]>=Ref ) break;
  }
  TrackSelectF(Speed, select);
}

void BBC(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if ((B[5]>=Ref && B[10]>=Ref) || (B[4]>=Ref && B[11]>=Ref) ||
        (B[3]>=Ref && B[12]>=Ref) || (B[5]>=Ref && B[6]>=Ref && B[9]>=Ref && B[10]>=Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBC2(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[0]>=Ref && B[15]>=Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFL(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed,  kp, kd);
    ReadCalibrateF();
    if ((F[1]>Ref&&F[5]>Ref)||(F[2]>Ref&&F[6]>Ref)||(F[3]>Ref&&F[7]>Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBL(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if ((B[1]>Ref&&B[5]>Ref)||(B[2]>Ref&&B[6]>Ref)||(B[3]>Ref&&B[7]>Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void FFR(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed,  kp, kd);
    ReadCalibrateF();
    if ((F[14]>Ref&&F[10]>Ref)||(F[13]>Ref&&F[9]>Ref)||(F[12]>Ref&&F[8]>Ref)) break;
  }
  TrackSelectF(Speed, select);
}

void BBR(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if ((B[14]>Ref&&B[10]>Ref)||(B[13]>Ref&&B[9]>Ref)||(B[12]>Ref&&B[8]>Ref)) break;
  }
  TrackSelectB(Speed, select);
}

void BBL0(int Speed,float kp,float kd, char select) {
BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[0] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void BBR15(int Speed,float kp,float kd, char select) {
BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[15] > Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFL0(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDF(LeftBaseSpeed,RightBaseSpeed,  kp, kd);
    ReadCalibrateF();
    if (F[0] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void FFR15(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
     PIDF(LeftBaseSpeed,RightBaseSpeed,  kp, kd);
    ReadCalibrateF();
    if (F[15] > Ref) break;
  }
  TrackSelectF(Speed, select);
}





void FFWhite(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed,  kp, kd);
    ReadCalibrateF();
    if (F[3]<Ref&&F[4]<Ref&&F[5]<Ref&&F[6]<Ref&&F[7]<Ref&&
        F[8]<Ref&&F[9]<Ref&&F[10]<Ref&&F[11]<Ref&&F[12]<Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBWhite(int Speed,float kp,float kd, char select) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
    ReadCalibrateB(); 
    if (B[3]<Ref&&B[4]<Ref&&B[5]<Ref&&B[6]<Ref&&B[7]<Ref&&
        B[8]<Ref&&B[9]<Ref&&B[10]<Ref&&B[11]<Ref&&B[12]<Ref) break;
  }
  TrackSelectB(Speed, select);
}

void FFNUM(int Speed,float kp,float kd, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDF(LeftBaseSpeed,RightBaseSpeed,  kp, kd);
    ReadCalibrateF();
    if (F[numm] > Ref) break;
  }
  TrackSelectF(Speed, select);
}

void BBNUM(int Speed,float kp,float kd, char select, int numm) {
  BaseSpeed = Speed;
  InitialSpeed();
  while (1) {
    PIDB(BackLeftBaseSpeed,BackRightBaseSpeed, kp, kd);
    ReadCalibrateB();
    if (B[numm] > Ref) break;
  }
  TrackSelectB(Speed, select);
}



#endif
