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

// void SetRobotAngle() {
//   current_degree = angleRead();
// }

void SetRobotAngle() {
  float sum = 0;

  for (int i = 0; i < 10; i++) {
    sum += angleRead();
    delay(6);
  }

  current_degree = sum / 20.0;
}

/* ---------- spin / turn ---------- */

void spinDegree(int relative_degree) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  int min_speed = 5;
  int max_speed = 30;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
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

void turnDegree(int relative_degree) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  int min_speed = 10;
  int max_speed = 30;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
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
      if (error <= 0) Motor(5, -pd_value);
      else if (error > 0) Motor(pd_value, 5);
    }
    previous_error = error;
  }
  Stop(200);
}

void turnDegreeB(int relative_degree) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  int min_speed = 10;
  int max_speed = 30;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
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
      if (error <= 0) Motor(pd_value, -5);
      else if (error > 0) Motor(-5, -pd_value);
    }
    previous_error = error;
  }
  Stop(200);
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

/* ---------- gyro-guided straight move ---------- */
float kpG = 1.2;
float kdG = 1.5;
float kpGB = 1.2;
float kdGB = 1.5;


void RunG(int Speed) {
  
  float error = current_degree - angleRead();
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorG;
  int pd_value = (error * kpG) + (derivative * kdG);
  int leftPow = Speed + pd_value;
  int rightPow = Speed - pd_value;
  if (leftPow > Speed) leftPow = Speed;
  if (leftPow < 0) leftPow = -5;
  if (rightPow > Speed) rightPow = Speed;
  if (rightPow < 0) rightPow = -5;
  Motor(leftPow, rightPow);
  previous_errorG = error;
}

void RunGB(int Speed) {
  float error = current_degree - angleRead();
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;
  float derivative = error - previous_errorG;
  int pd_value = (error * kpGB  ) + (derivative * kdGB);
  int leftPow = Speed - pd_value;
  int rightPow = Speed + pd_value;
  if (leftPow > Speed) leftPow = Speed;
  if (leftPow < 0) leftPow = -5;
  if (rightPow > Speed) rightPow = Speed;
  if (rightPow < 0) rightPow = -5;
  Motor(-leftPow, -rightPow);
  previous_errorGB = error;
}


void FFtimerG(int Speed, int totalTime) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(Speed);
  }
}


void BBtimerG(int Speed, int totalTime) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunGB(Speed);
  }
}

/* ---------- track select (gyro) ---------- */

void TrackSelectG(int spd, char select) {
  if (select == 'L' || select == 'l') {
    spinDegree(-90);
  } else if (select == 'R' || select == 'r') {
    spinDegree(90);
  } else if (select == 'q' || select == 'Q') {
    turnDegree(-90);
  } else if (select == 'e' || select == 'E') {
    turnDegree(90);
  } else if (select == 'p' || select == 'P') {
    ReadCalibrateF();
    while (1) {
      RunG(spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
    FFtimerG(spd, 5);
    while (1) {
      RunG(spd);
      ReadCalibrateF();
      if (F[3] < Ref && F[12] < Ref) break;
    }
  } 
  else if (select == 'c' || select == 'C') {
    RunG(spd);
  delay(20);
  while (1) {
    RunG(spd);
    ReadSensor();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(-spd, -spd);
      delay(5);
      MotorStop();
      break;
    }
  }
  }else {
    Stop(100);
  }
}

void TrackSelectGB(int spd,char select) {
  if (select == 'L' || select == 'l') {
    spinDegree(-90);
  } else if (select == 'R' || select == 'r') {
    spinDegree(90);
  } else if (select == 'q' || select == 'Q') {
    turnDegreeB(90);
  } else if (select == 'e' || select == 'E') {
    turnDegreeB(-90);
  } else if (select == 'p' || select == 'P') {
    ReadCalibrateB();
    while (1) {
      RunGB(spd);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
    BBtimerG(spd, 5);
    while (1) {
      RunGB(spd);
      ReadCalibrateB();
      if (B[3] < Ref && B[12] < Ref) break;
    }
  } else if (select == 'c' || select == 'C') {
    RunGB(spd);
  delay(20);
  while (1) {
    RunGB(spd);
    ReadSensor();
    if (C[CCL] >= RefC || C[CCR] >= RefC) {
      Motor(spd, spd);
      delay(5);
      MotorStop();
      break;
    }
  }

  } 
  else {
    Stop(100);
  }
}

void FFBG(int Speed, char select) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  while (1) {
    RunG(Speed);
    ReadCalibrateF();
    if (F[5] > Ref || F[6] > Ref || F[7] > Ref || F[8] > Ref || F[9] > Ref || F[10] > Ref) break;
  }
  TrackSelectG(Speed,select);
}

void BBBG(int Speed, char select) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  while (1) {
    RunGB(Speed);
    ReadCalibrateB();
    if (B[5] > Ref || B[6] > Ref || B[7] > Ref || B[8] > Ref || B[9] > Ref || B[10] > Ref) break;
  }
  TrackSelectGB(Speed,select);
}



/************* SPIN FAST → FINE *************/
void spinDegree(int speed, int relative_degree) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  int min_speed = 5;
  int max_speed = speed;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
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
  Stop(60);
}


void SpinLG(int spd, int Angle) {
  spinDegree(spd, -abs(Angle));
}
void SpinRG(int spd, int Angle) {
  spinDegree(spd, abs(Angle));
}



void turnDegree(int speed,int relative_degree) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  int min_speed = 10;
  int max_speed = speed;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
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
      if (error <= 0) Motor(5, -pd_value);
      else if (error > 0) Motor(pd_value, 5);
    }
    previous_error = error;
  }
  Stop(60);
}


void TurnLG(int spd, int Angle) {
  turnDegree(spd, -abs(Angle));
}
void TurnRG(int spd ,int Angle) {
  turnDegree(spd, abs(Angle));
}


void turnDegreeB(int speed,int relative_degree) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  int min_speed = 10;
  int max_speed = speed;
  float kp = 0.9;
  float kd = 0.15;
  float small_angle_threshold = 25;
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
      if (error <= 0) Motor(pd_value, 0);
      else if (error > 0) Motor(0, -pd_value);
    }
    previous_error = error;
  }
  Stop(60);
}


void TurnLBG(int spd, int Angle) {
  turnDegreeB(spd, abs(Angle));
}
void TurnRBG(int spd, int Angle) {
  turnDegreeB(spd, -abs(Angle));
}
