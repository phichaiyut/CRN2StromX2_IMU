#ifndef UNKNOW_SERVO_H
#define UNKNOW_SERVO_H

#include <Servo.h>

/* ==================================================================================================
 *  Unknow_Servo.h — เซอร์โว (ที่คีบ)
 *   คำสั่งสั่งงานที่คีบ (ServoOpen / ServoClose / ServoUp ...) อยู่ใน Servo.ino ของตัวอย่าง
 *
 *   1) ขา / ตัวแปร
 *   2) สั่งเซอร์โว
 *   3) ตั้งความเร็ว
 *   4) ตั้งองศา
 *   5) ปรับองศาผ่าน Serial Monitor
 * ================================================================================================== */


/* ==================================================================================================
 *  1) ขา / ตัวแปร
 * ================================================================================================== */

#define servo39 39
#define servo38 38
#define servo37 37
#define servo36 36
#define servo35 35
#define servo34 34

// ความเร็ว (หน่วงต่อ 1 องศา, ไมโครวินาที)
#define SERVO_SLOW 6000
#define SERVO_MEDIUM 1500
#define SERVO_FAST 500

Servo servo_39, servo_38, servo_37, servo_36, servo_35, servo_34;

// องศาที่ตั้งด้วย setOpen / setClose / setUpDown ...
int OpenL, OpenR, CloseL, CloseR, CloseLSmall, CloseRSmall;
int Up, Up45, Down, OpenL2, OpenR2, OpenRMax, OpenLMax, OpenRMax2, OpenLMax2;

// ความเร็วที่ตั้งด้วย setServoSpeed_xxx
int SPD_OPEN, SPD_OPENMAX, SPD_CLOSE, SPD_CLOSESMALL;
int SPD_UP, SPD_UP45, SPD_DOWN;

int servoPos[3] = { 90, 90, 90 };  // องศาล่าสุดของเซอร์โว 0-2 (ใช้กับการขยับแบบคุมความเร็ว)


/* ==================================================================================================
 *  2) สั่งเซอร์โว
 * ================================================================================================== */

// สั่งองศาทันที — servo = 39 38 37 36 35 34 (หรือ 2 1 0 แทน 36 35 34)
void ServoWrite(int servo, int angle) {
  if (servo == 39) {
    servo_39.attach(servo39, 500, 2500);
    servo_39.write(angle);
  } else if (servo == 38) {
    servo_38.attach(servo38, 500, 2500);
    servo_38.write(angle);
  } else if (servo == 37) {
    servo_37.attach(servo37, 500, 2500);
    servo_37.write(angle);
  } else if (servo == 2 || servo == 36) {
    servo_36.attach(servo36, 500, 2500);
    servo_36.write(angle);
  } else if (servo == 1 || servo == 35) {
    servo_35.attach(servo35, 500, 2500);
    servo_35.write(angle);
  } else if (servo == 0 || servo == 34) {
    servo_34.attach(servo34, 500, 2500);
    servo_34.write(angle);
  }
}

// ขยับทีละองศาแบบคุมความเร็ว (เซอร์โว 0-2 เท่านั้น)
void ServoMoveSpeed(int servo, int targetAngle, int speedDelay) {
  if (servo < 0 || servo > 2) return;
  int current = servoPos[servo];
  int step = (targetAngle > current) ? 1 : -1;
  while (current != targetAngle) {
    current += step;
    ServoWrite(servo, current);
    delayMicroseconds(speedDelay);
  }
  servoPos[servo] = targetAngle;
}

// ขยับสองตัวพร้อมกันแบบคุมความเร็ว (เซอร์โว 0-2 เท่านั้น)
void ServoMove2Sync(int sL, int targetL, int sR, int targetR, int speedDelay) {
  if (sL < 0 || sL > 2 || sR < 0 || sR > 2) return;
  int curL = servoPos[sL];
  int curR = servoPos[sR];
  while (curL != targetL || curR != targetR) {
    if (curL != targetL) {
      curL += (targetL > curL) ? 1 : -1;
      ServoWrite(sL, curL);
    }
    if (curR != targetR) {
      curR += (targetR > curR) ? 1 : -1;
      ServoWrite(sR, curR);
    }
    delayMicroseconds(speedDelay);
  }
  servoPos[sL] = targetL;
  servoPos[sR] = targetR;
}


/* ==================================================================================================
 *  3) ตั้งความเร็ว (SERVO_SLOW / SERVO_MEDIUM / SERVO_FAST)
 * ================================================================================================== */

void setServoSpeed_Up(int SpeedUp) {
  SPD_UP = SpeedUp;
}

void setServoSpeed_Up45(int SpeedUp45) {
  SPD_UP45 = SpeedUp45;
}

void setServoSpeed_Down(int SpeedDown) {
  SPD_DOWN = SpeedDown;
}

void setServoSpeed_Open(int SpeedOpen) {
  SPD_OPEN = SpeedOpen;
  SPD_OPENMAX = SpeedOpen;
}

void setServoSpeed_Close(int SpeedClose) {
  SPD_CLOSE = SpeedClose;
  SPD_CLOSESMALL = SpeedClose;
}


/* ==================================================================================================
 *  4) ตั้งองศา (ซ้าย, ขวา)
 * ================================================================================================== */

void setOpen(int degree1, int degree2) {
  OpenL = degree1;
  OpenR = degree2;
}

void setOpen2(int degree1, int degree2) {
  OpenL2 = degree1;
  OpenR2 = degree2;
}

void setOpenMax(int degree1, int degree2) {
  OpenLMax = degree1;
  OpenRMax = degree2;
}

void setOpenMax2(int degree1, int degree2) {
  OpenLMax2 = degree1;
  OpenRMax2 = degree2;
}

void setClose(int degree1, int degree2) {
  CloseL = degree1;
  CloseR = degree2;
}

void setCloseSmall(int degree1, int degree2) {
  CloseLSmall = degree1;
  CloseRSmall = degree2;
}

// ยก / วาง
void setUpDown(int degree1, int degree2) {
  Up = degree1;
  Down = degree2;
}

// ยกครึ่งทาง / วาง
void setUpDown45(int degree1, int degree2) {
  Up45 = degree1;
  Down = degree2;
}


/* ==================================================================================================
 *  5) ปรับองศาผ่าน Serial Monitor — พิมพ์ "เลขเซอร์โว องศา" เช่น 39 90 แล้ว Enter, กดปุ่ม OK เพื่อออก
 * ================================================================================================== */

int currentServo = -1;
int currentAngle = 90;

void SerialServoControl() {
  Serial.println("Serial Servo Control Mode");
  Serial.println("Type: servo angle  (ex: 39 90)");

  while (1) {
    if (digitalRead(19) == 0) {  // ปุ่ม OK
      Serial.println("Exit Servo Control");
      break;
    }

    if (Serial.available()) {
      int s = Serial.parseInt();
      int a = Serial.parseInt();

      if (a >= 0 && a <= 180) {
        currentServo = s;
        currentAngle = a;
        Serial.print("Set Servo ");
        Serial.print(currentServo);
        Serial.print(" -> ");
        Serial.print(currentAngle);
        Serial.println(" deg");
      } else {
        Serial.println("Angle must be 0-180");
      }

      while (Serial.available()) Serial.read();
    }

    if (currentServo != -1) ServoWrite(currentServo, currentAngle);  // เขียนค่าเดิมซ้ำเรื่อย ๆ

    delay(20);  // ~50Hz
  }
}

#endif
