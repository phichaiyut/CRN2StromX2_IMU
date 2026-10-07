void setup_robot() {
  set_sensor_track_line(0, 15);  //ตั้งค่าจำนวนเซนเซอร์ที่ใช้วิ่ง 0 , 15 || 2 , 14 || 3 , 13 || 4 , 12
  RobotSetupSpeed();
  clampSensorValueF(100, 800);  //สำหรับกรองค่า  calibrate
  clampSensorValueB(100, 800);  //สำหรับกรองค่า  calibrate
  clampSensorValueC(0, 1000);   //สำหรับกรองค่า  calibrate

  set_position_line(7500);     //ตั้งค่าเส้น
  set_position_line_l(1500);   //ตั้งค่าเส้น
  set_position_line_l(13500);  //ตั้งค่าเส้น

  // set_line_center(0);  // เดินธรรมดา เข้ากลางหุ่น
  set_line_center(1);  // เดินตามเส้น เข้ากลางหุ่น

  SetSlowKpKd(0.014, 0.14);

  /******************** GYRO SPEED MODE ********************/
  // เลือกเปิดใช้ทีละบรรทัด (mode, max, min)
  // ModeSpdGyro(0, 100, -10);  // 0 = ..max         ล้อติดลบ → min
  // ModeSpdGyro(1, 100, -100); // 1 = min..max
  // ModeSpdGyro(2, 100, -10);  // 2 = -Speed..Speed ถอยล้อได้เต็มที่
  // ModeSpdGyro(3, 100, -10);  // 3 = ..max         ล้อติดลบ → -Speed
  ModeSpdGyro(4, 100, -5);  // 4 = ..Speed       ล้อติดลบ → min (ใส่ min = 0 คือไม่ถอยล้อ)
  // ModeSpdGyro(2, 4, 100, -5);  // แยกโหมด (เดินหน้า, ถอยหลัง, max, min)

  /******************** GYRO PID CONFIG ********************/
  // (kp, kd, maxSpd, minSpd, smallAngle, stopThr)
  SetGyroTurn(0.75, 0.9, 50, 10, 25.0, 1.0);  // เลี้ยวล้อเดียวด้วยไจโร (turnDegree / turnDegreeB)
  SetGyroSpin(0.75, 0.9, 40, 8, 25.0, 1.0);   // หมุนตัวอยู่กับที่ด้วยไจโร (spinDegree)
  // (kp, kd)
  SetGyroRun(1.2, 1.5);   // เดินหน้าตรงด้วยไจโร (RunG)
  SetGyroRunB(1.2, 1.5);  // ถอยหลังตรงด้วยไจโร (RunGB)
  //SerialCalibrate_AllSensor(); //โชว์ค่าคาลิเบท เซนเซอร์ทั้งหมด
  //SerialPositionFB();
}

void RobotSetupSpeed() {
  SetBalanceSpeedForward();   //ฟังก์ชั่นตั้งค่าความสมดุลมอเตอร์ในแต่ละความเร็ว
  SetBalanceSpeedBackward();  //ฟังก์ชั่นตั้งค่าความสมดุลมอเตอร์ในแต่ละความเร็ว
  SetKpKd();                  //ฟังก์ชั่นตั้งค่า KP KD ในแต่ละความเร็ว
  SetKpKdBack();              //ฟังก์ชั่นตั้งค่า KP KD ในแต่ละความเร็ว
  SetDelayBreakSpeed();       //ฟังก์ชั่นตั้งค่าเวลาเบรกในแต่ละความเร็ว
}

void SetDelayBreakSpeed() {  //เวลาเบรก (ms) ตอนหยุดที่เส้น
  // หุ่นไถลเลยเส้น ให้เพิ่มค่า | หุ่นถอยกลับเลยเส้น ให้ลดค่า
  //______________________________SetDelayBreak(SPD_10, เดินหน้า, ถอยหลัง);__________________________________
  SetDelayBreak(SPD_10, 30, 30);   //ความเร็ว 10
  SetDelayBreak(SPD_20, 30, 30);   //ความเร็ว 20
  SetDelayBreak(SPD_30, 30, 30);   //ความเร็ว 30
  SetDelayBreak(SPD_40, 30, 30);   //ความเร็ว 40
  SetDelayBreak(SPD_50, 30, 30);   //ความเร็ว 50
  SetDelayBreak(SPD_60, 30, 30);   //ความเร็ว 60
  SetDelayBreak(SPD_70, 30, 30);   //ความเร็ว 70
  SetDelayBreak(SPD_80, 30, 30);   //ความเร็ว 80
  SetDelayBreak(SPD_90, 30, 30);   //ความเร็ว 90
  SetDelayBreak(SPD_100, 30, 30);  //ความเร็ว 100
}

void SetKpKd() {                    //เดินหน้า
  Set_KP_KD(SPD_10, 0.006, 0.10);   //ความเร็ว 10
  Set_KP_KD(SPD_20, 0.007, 0.14);   //ความเร็ว 20
  Set_KP_KD(SPD_30, 0.008, 0.18);   //ความเร็ว 30
  Set_KP_KD(SPD_40, 0.009, 0.22);   //ความเร็ว 40
  Set_KP_KD(SPD_50, 0.010, 0.26);   //ความเร็ว 50
  Set_KP_KD(SPD_60, 0.010, 0.30);   //ความเร็ว 60
  Set_KP_KD(SPD_70, 0.011, 0.34);   //ความเร็ว 70
  Set_KP_KD(SPD_80, 0.011, 0.38);   //ความเร็ว 80
  Set_KP_KD(SPD_90, 0.012, 0.42);   //ความเร็ว 90
  Set_KP_KD(SPD_100, 0.012, 0.46);  //ความเร็ว 100
}

void SetKpKdBack() {                     //ถอยหลัง
  Set_KP_KD_Back(SPD_10, 0.006, 0.10);   //ความเร็ว 10
  Set_KP_KD_Back(SPD_20, 0.007, 0.14);   //ความเร็ว 20
  Set_KP_KD_Back(SPD_30, 0.008, 0.18);   //ความเร็ว 30
  Set_KP_KD_Back(SPD_40, 0.009, 0.22);   //ความเร็ว 40
  Set_KP_KD_Back(SPD_50, 0.010, 0.26);   //ความเร็ว 50
  Set_KP_KD_Back(SPD_60, 0.010, 0.30);   //ความเร็ว 60
  Set_KP_KD_Back(SPD_70, 0.011, 0.34);   //ความเร็ว 70
  Set_KP_KD_Back(SPD_80, 0.011, 0.38);   //ความเร็ว 80
  Set_KP_KD_Back(SPD_90, 0.012, 0.42);   //ความเร็ว 90
  Set_KP_KD_Back(SPD_100, 0.012, 0.46);  //ความเร็ว 100
}

void SetBalanceSpeedForward() {  //เดินหน้า
  //ข้างไหนแรงกว่าไปข้างเพิ่มข้างนั้น
  //______________________________setBalanceSpeed(SPD_10,ข้างซ้าย, ข้างขวา);__________________________________
  setBalanceSpeed(SPD_10, 0, 0);   //ความเร็ว 10
  setBalanceSpeed(SPD_20, 0, 0);   //ความเร็ว 20
  setBalanceSpeed(SPD_30, 0, 0);   //ความเร็ว 30
  setBalanceSpeed(SPD_40, 0, 0);   //ความเร็ว 40
  setBalanceSpeed(SPD_50, 0, 0);   //ความเร็ว 50
  setBalanceSpeed(SPD_60, 0, 0);   //ความเร็ว 60
  setBalanceSpeed(SPD_70, 0, 0);   //ความเร็ว 70
  setBalanceSpeed(SPD_80, 0, 0);   //ความเร็ว 80
  setBalanceSpeed(SPD_90, 0, 0);   //ความเร็ว 90
  setBalanceSpeed(SPD_100, 0, 0);  //ความเร็ว 100
}

void SetBalanceSpeedBackward() {  //ถอยหลัง
  //ข้างไหนแรงกว่าไปข้างเพิ่มข้างนั้น
  //______________________setBalanceBackSpeed(SPD_10, ข้างซ้าย, ข้างขวา);____________________________________________
  setBalanceBackSpeed(SPD_10, 0, 0);   //ความเร็ว 10
  setBalanceBackSpeed(SPD_20, 0, 0);   //ความเร็ว 20
  setBalanceBackSpeed(SPD_30, 0, 0);   //ความเร็ว 30
  setBalanceBackSpeed(SPD_40, 0, 0);   //ความเร็ว 40
  setBalanceBackSpeed(SPD_50, 0, 0);   //ความเร็ว 50
  setBalanceBackSpeed(SPD_60, 0, 0);   //ความเร็ว 60
  setBalanceBackSpeed(SPD_70, 0, 0);   //ความเร็ว 70
  setBalanceBackSpeed(SPD_80, 0, 0);   //ความเร็ว 80
  setBalanceBackSpeed(SPD_90, 0, 0);   //ความเร็ว 90
  setBalanceBackSpeed(SPD_100, 0, 0);  //ความเร็ว 100
}
