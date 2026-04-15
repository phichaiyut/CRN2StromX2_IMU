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

  current_degree = sum / 10.0;
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
// float kdG = 10;
float kpGB = 1.2;
float kdGB = 1.5;
// float kdGB = 10;


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
  float derivative = error - previous_errorGB;
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

void FFtimerG(int Speed, int totalTime, char select) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunG(Speed);
  }
  TrackSelectG(Speed,select);
}


void BBtimerG(int Speed, int totalTime, char select) {
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน
  unsigned long endTime = millis() + totalTime;
  while (millis() <= endTime) {
    RunGB(Speed);
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



void fw_gyro(int spd, float kp,  float distance, int offset) 
{     BaseSpeed = spd;
  InitialSpeed();
    int target_speed = min(LeftBaseSpeed, RightBaseSpeed); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.5;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

   SetRobotAngle(); // เซ็ตค่าปัจจุบัน
 unsigned long prevT = millis(); 
    while (1) 
    {
      
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

       RunG(spd);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        //delayMicroseconds(80);
    }

    // soft stop
    if(offset >0)
      {
        Motor(-15, -15); delay(offset);
        Motor(-1, -1);   delay(10);
      }
    else{Motor(0, 0);delay(5);}
    
}

void bw_gyro(int spd, float kp,  float distance, int offset) 
 {     
    BaseSpeed = spd;
  InitialSpeed();
    int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.5;  // ใช้ค่าที่คาลิเบรตจาก fw()

    SetRobotAngle(); // เซ็ตค่าปัจจุบัน


   
    unsigned long prevT = millis();   


    while (1) 
    {
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

      RunGB(spd);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        //delayMicroseconds(80);
    }



    if(offset >0)
      {
        Motor(15, 15); delay(offset);
        Motor(-1, -1);   delay(10);
      }
    else{Motor(0, 0);delay(5);}
  }



  void fw_gyro(int spd, float kp,  float distance,char select, int offset) 
{     BaseSpeed = spd;
  InitialSpeed();
    int target_speed = min(LeftBaseSpeed, RightBaseSpeed); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.5;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

   SetRobotAngle(); // เซ็ตค่าปัจจุบัน
 unsigned long prevT = millis(); 
    while (1) 
    {
      
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

       RunG(spd);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        // delayMicroseconds(80);
    }

    // soft stop
    // if(offset >0)
    //   {
    //     Motor(-15, -15); delay(offset);
    //     Motor(-1, -1);   delay(10);
    //   }
    // else{Motor(0, 0);delay(5);}
    TrackSelectG(spd,select);
}

void bw_gyro(int spd, float kp,  float distance,char select,  int offset) 
 {     
    BaseSpeed = spd;
  InitialSpeed();
    int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.5;  // ใช้ค่าที่คาลิเบรตจาก fw()

    SetRobotAngle(); // เซ็ตค่าปัจจุบัน


    
    unsigned long prevT = millis();   


    while (1) 
    {
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

      RunGB(spd);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        // delayMicroseconds(80);
    }



    // if(offset >0)
    //   {
    //     Motor(-15, -15); delay(offset);
    //     Motor(1, 1);   delay(10);
    //   }
    // else{Motor(0, 0);delay(5);}
    TrackSelectGB(spd,select);
  }




void FFcmGS(int Speed,  float distance) 
{     BaseSpeed = Speed;
  InitialSpeed();
    int target_speed = min(LeftBaseSpeed, RightBaseSpeed); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.5;  // <-- ใช้ค่าที่คำนวณจากการวัดจริง

   SetRobotAngle(); // เซ็ตค่าปัจจุบัน
 unsigned long prevT = millis(); 
    while (1) 
    {
      
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

       RunG(Speed);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        //delayMicroseconds(80);
    }

    // soft stop
    // if(offset >0)
    //   {
    //     Motor(-15, -15); delay(offset);
    //     Motor(-1, -1);   delay(10);
    //   }
    // else{Motor(0, 0);delay(5);}
    
}


void FFcmG(int Speed, float distance_cm){
  BaseSpeed = Speed;
  InitialSpeed();

  if (distance_cm <= 0) {
        Motor(0, 0);
        return;
    }

    int base_speed = min(abs(LeftBaseSpeed), abs(RightBaseSpeed));
   // bool is_forward = (LeftBaseSpeed >= 0 && RightBaseSpeed >= 0);


    float traveled_distance = 0.0;
    unsigned long last_time = millis();

    // ====================== ค่าที่สามารถปรับได้ ======================
    const float ACCEL_DISTANCE_CM = 20.0;
    const float DECEL_DISTANCE_CM = 25.0;
    const float MIN_SPEED = 10.0;

    // ค่า speed_scale ที่คุณต้องการปรับได้ (ค่าดีฟอลต์ = 0.99)
    float speed_scale = 0.99;        // ← คุณสามารถปรับตรงนี้ได้

    // ตัดสินใจว่าใช้ Ramp หรือไม่
    bool enableRamp = (distance_cm >= 30.0);

    // ถ้าระยะสั้นมาก (< 30) ให้ปรับ speed_scale ได้ง่ายขึ้น
    if (!enableRamp) {
        speed_scale = 1.7;   // คุณสามารถเปลี่ยนเป็น 0.95, 0.98, 1.0 ได้ตามต้องการ
    }
 SetRobotAngle(); // เซ็ตค่าปัจจุบัน
    

    while (true)
    {
        // คำนวณระยะทาง
        unsigned long current_time = millis();
        float delta_time = (current_time - last_time) / 1000.0;
        traveled_distance += (base_speed * speed_scale) * delta_time;
        last_time = current_time;

        float remaining_cm = distance_cm - traveled_distance;

        if (remaining_cm <= 0.7f) break;

        // ====================== คำนวณ target_speed ======================
        float target_speed = base_speed;

        if (enableRamp)
        {
            if (traveled_distance < ACCEL_DISTANCE_CM) {
                // เร่งช่วงแรก
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
            }
            else if (remaining_cm < DECEL_DISTANCE_CM) {
                // ชะลอช่วงสุดท้าย
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
            }
        }
        // ถ้า enableRamp = false → ใช้ความเร็วคงที่ตลอดทาง
          RunG(base_speed);
        // ====================== คำนวณความเร็วซ้าย-ขวา ======================
        
    }

    // ====================== Soft Stop ======================
    // if (offset > 0) {
    //     if (is_forward) {
    //         Motor(-3, -2); delay(offset);
    //     } else {
    //         Motor(2, 3); delay(offset);
    //     }
    //     Motor(-1, -1); delay(10);
    // } 
    // else {
    //     Motor(0, 0);
    // }
    
    // Motor(1, 1);      // Pulse เล็กน้อยเพื่อหยุดตรงขึ้น
    // delay(5);
}


void BBcmGS(int Speed,  float distance) 
 {     
    BaseSpeed = Speed;
  InitialSpeed();
    int target_speed = min(BackLeftBaseSpeed, BackRightBaseSpeed); 
    float traveled_distance = 0;
    unsigned long last_time = millis();
    
    float speed_scale = 1.5;  // ใช้ค่าที่คาลิเบรตจาก fw()

    SetRobotAngle(); // เซ็ตค่าปัจจุบัน
    unsigned long prevT = millis();   


    while (1) 
    {
        unsigned long now = millis();
        float dt = (now - prevT) / 1000.0;
        if (dt <= 0) dt = 0.001; 
        prevT = now;

      RunGB(Speed);

        if (distance > 0) 
        {
            unsigned long current_time = millis();
            float delta_time = (current_time - last_time) / 1000.0;
            traveled_distance += (target_speed * speed_scale) * delta_time;
            last_time = current_time;

            if (traveled_distance >= distance) break;
        }

        //delayMicroseconds(80);
    }



    // if(offset >0)
    //   {
    //     Motor(15, 15); delay(offset);
    //     Motor(-1, -1);   delay(10);
    //   }
    // else{Motor(0, 0);delay(5);}
  }

 void BBcmG(int Speed, float distance_cm)
{
    BaseSpeed = Speed;
    InitialSpeed();
    if (distance_cm <= 0) {
        Motor(0, 0);
        return;
    }

    int base_speed = min(abs(BackLeftBaseSpeed), abs(BackRightBaseSpeed));
   

    float traveled_distance = 0.0;
    unsigned long last_time = millis();

    float speed_scale = 0.99;                    // ค่าเริ่มต้นสำหรับถอยหลัง

    const float ACCEL_DISTANCE_CM = 20.0;
    const float DECEL_DISTANCE_CM = 25.0;
    const float MIN_SPEED = 10.0;

    // ตัดสินใจว่าใช้ Ramp หรือไม่
    bool enableRamp = (distance_cm >= 30.0);

    // ถ้าระยะสั้นมาก (< 30) → ไม่ใช้ Ramp + ปรับ speed_scale
    if (!enableRamp) {
        speed_scale = 1.5;     // คุณสามารถปรับตรงนี้ได้ (แนะนำ 0.92 - 0.97)
    }
  SetRobotAngle(); // เซ็ตค่าปัจจุบัน

    while (true)
    {
        // คำนวณระยะทาง
        unsigned long current_time = millis();
        float delta_time = (current_time - last_time) / 1000.0;
        traveled_distance += (base_speed * speed_scale) * delta_time;
        last_time = current_time;

        float remaining_cm = distance_cm - traveled_distance;

        if (remaining_cm <= 0.8f) break;

        // ====================== คำนวณความเร็ว ======================
        float target_speed = base_speed;

        if (enableRamp)   // ใช้ Ramp เฉพาะระยะยาว (>=30 cm)
        {
            if (traveled_distance < ACCEL_DISTANCE_CM) {
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (traveled_distance / ACCEL_DISTANCE_CM);
            }
            else if (remaining_cm < DECEL_DISTANCE_CM) {
                target_speed = MIN_SPEED + (base_speed - MIN_SPEED) * (remaining_cm / DECEL_DISTANCE_CM);
            }
        }
        // ถ้า !enableRamp → ใช้ความเร็วคงที่ตลอดทาง (base_speed)

        RunGB(base_speed);
    }

    // // ====================== Soft Stop ======================
    // if (offset > 0) {
    //     if (is_backward) {
    //         // ถอยหลัง → เบรกด้วยการเดินหน้าเล็กน้อย
    //         Motor(4, 4); delay(offset);
    //     } else {
    //         // เดินหน้า → เบรกด้วยการถอยหลัง
    //         Motor(-4, -4); delay(offset);
    //     }
    //     Motor(0, 0); delay(10);
    // } 
    // else {
    //     Motor(0, 0);
    // }

    // Motor(1, 1);
    // delay(5);
}

  void FFcmGS(int Speed,  float distance_cm, char select) 
{   
    FFcmGS(Speed, distance_cm);
    TrackSelectG(Speed,select);
}

void BBcmGS(int Speed,  float distance_cm, char select) 
{     
    BBcmGS(Speed, distance_cm);
    TrackSelectGB(Speed,select);
}
  void FFcmG(int Speed,  float distance_cm, char select) 
{   
    FFcmG(Speed, distance_cm);
    TrackSelectG(Speed,select);
}

void BBcmG(int Speed,  float distance_cm, char select) 
{     
    BBcmG(Speed, distance_cm);
    TrackSelectGB(Speed,select);
}
