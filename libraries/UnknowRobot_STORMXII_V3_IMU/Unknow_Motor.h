#ifndef UNKNOW_MOTOR_H
#define UNKNOW_MOTOR_H

#include <Arduino.h>

#define PH1 22
#define EN1 21
#define PH2 25
#define EN2 24

int BalanceMotorLeft, BalanceMotorRight;



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
  Motor1(leftspeed+BalanceMotorLeft);
  Motor2(rightspeed+BalanceMotorRight);
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

int BaseSpeed, LeftBaseSpeed, RightBaseSpeed, BackLeftBaseSpeed, BackRightBaseSpeed;
float PID_KP_Front, PID_KD_Front;
float PID_KP_Back, PID_KD_Back;
// int LastError_F, LastError_B;
int L[10], R[10];
int BL[10], BR[10];
float KP[10], KD[10];
float KP_Back[10], KD_Back[10];

// ��˹� index ���ӧ���
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

void setBalanceSpeed(int ch, int spdL, int spdR) {
  L[ch] = spdL;
  R[ch] = spdR;
}

void setBalanceBackSpeed(int ch, int spdL, int spdR) {
  BL[ch] = spdL;
  BR[ch] = spdR;
}

void Set_KP_KD(int ch, float kp,float kd){
  KP[ch] = kp;
  KD[ch] = kd;
}

void Set_KP_KD_Back(int ch, float kp,float kd){
  KP_Back[ch] = kp;
  KD_Back[ch] = kd;
}


void InitialSpeed() {
//  LastError_F   = 3500, LastError_B = 3500;
if (BaseSpeed <= 10) {
    LeftBaseSpeed = BaseSpeed - L[SPD_10];
    RightBaseSpeed = BaseSpeed - R[SPD_10];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_10];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_10];
    PID_KP_Front = KP[SPD_10];  //forward PID
    PID_KD_Front = KD[SPD_10];

    PID_KP_Back = KP_Back[SPD_10];  //backward PID
    PID_KD_Back = KD_Back[SPD_10];

  }
  else if (BaseSpeed <= 20) {
    LeftBaseSpeed = BaseSpeed - L[SPD_20];
    RightBaseSpeed = BaseSpeed - R[SPD_20];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_20];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_20];
    PID_KP_Front = KP[SPD_20];  //forward PID
    PID_KD_Front = KD[SPD_20];

    PID_KP_Back = KP_Back[SPD_20];  //backward PID
    PID_KD_Back = KD_Back[SPD_20];

  }

  else if (BaseSpeed <= 30) {
    LeftBaseSpeed = BaseSpeed - L[SPD_30];
    RightBaseSpeed = BaseSpeed - R[SPD_30];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_30];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_30];
    PID_KP_Front = KP[SPD_30];  //forward PID
    PID_KD_Front = KD[SPD_30];

    PID_KP_Back = KP_Back[SPD_30];  //backward PID
    PID_KD_Back = KD_Back[SPD_30];

  } else if (BaseSpeed <= 40) {
    LeftBaseSpeed = BaseSpeed - L[SPD_40];
    RightBaseSpeed = BaseSpeed - R[SPD_40];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_40];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_40];
    PID_KP_Front = KP[SPD_40];  //forward PID
    PID_KD_Front = KD[SPD_40];  //11

    PID_KP_Back = KP_Back[SPD_50];  //backward PID
    PID_KD_Back = KD_Back[SPD_50];

  } else if (BaseSpeed <= 50) {
    LeftBaseSpeed = BaseSpeed - L[SPD_50];
    RightBaseSpeed = BaseSpeed - R[SPD_50];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_50];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_50];
    PID_KP_Front = KP[SPD_50];  //forward PID
    PID_KD_Front = KD[SPD_50]; //14

    PID_KP_Back = KP_Back[SPD_50];  //backward PID
    PID_KD_Back = KD_Back[SPD_50];

  } else if (BaseSpeed <= 60) {
    LeftBaseSpeed = BaseSpeed - L[SPD_60];
    RightBaseSpeed = BaseSpeed - R[SPD_60];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_60];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_60];
    PID_KP_Front = KP[SPD_60];  //forward PID
    PID_KD_Front = KD[SPD_60];//17

    PID_KP_Back = KP_Back[SPD_60];  //backward PID
    PID_KD_Back = KD_Back[SPD_60];

  } else if (BaseSpeed <= 70) {
    LeftBaseSpeed = BaseSpeed - L[SPD_70];
    RightBaseSpeed = BaseSpeed - R[SPD_70];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_70];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_70];
    PID_KP_Front = KP[SPD_70];  //forward PID
    PID_KD_Front = KD[SPD_70];//20

    PID_KP_Back = KP_Back[SPD_70];  //backward PID
    PID_KD_Back = KD_Back[SPD_70];

  } else if (BaseSpeed <= 80) {
    LeftBaseSpeed = BaseSpeed - L[SPD_80];
    RightBaseSpeed = BaseSpeed - R[SPD_80];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_80];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_80];
    PID_KP_Front = KP[SPD_80];  //forward PID
    PID_KD_Front = KD[SPD_80]; //20

    PID_KP_Back = KP_Back[SPD_80];  //backward PID
    PID_KD_Back = KD_Back[SPD_80];


  } else if (BaseSpeed <= 90) {
    LeftBaseSpeed = BaseSpeed - L[SPD_90];
    RightBaseSpeed = BaseSpeed - R[SPD_90];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_90];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_90];
    PID_KP_Front = KP[SPD_90];  //forward PID
    PID_KD_Front = KD[SPD_90];//22

    PID_KP_Back = KP_Back[SPD_90];  //backward PID
    PID_KD_Back = KD_Back[SPD_90];


  } else {
    LeftBaseSpeed = BaseSpeed - L[SPD_100];
    RightBaseSpeed = BaseSpeed - R[SPD_100];
    BackLeftBaseSpeed = BaseSpeed - BL[SPD_100];
    BackRightBaseSpeed = BaseSpeed - BL[SPD_100];
    PID_KP_Front = KP[SPD_100];  //forward PID
    PID_KD_Front = KD[SPD_100];//25

    PID_KP_Back = KP_Back[SPD_100];  //backward PID
    PID_KD_Back = KD_Back[SPD_100];
  }
}


#endif
