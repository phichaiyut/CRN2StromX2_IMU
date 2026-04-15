void setup_robot() {

  
  set_sensor_track_line(0, 15);  //ตั้งค่าจำนวนเซนเซอร์ที่ใช้วิ่ง 0 , 15 || 2 , 14 || 3 , 13 || 4 , 12

  set_position_line(7500);  //ตั้งค่าเส้น

 // set_line_center(0);  // เดินธรรมดา เข้ากลางหุ่น
  set_line_center(1);  // เดินตามเส้น เข้ากลางหุ่น
  
  SetSlowKpKd(0.014, 0.14);
  // SerialCalibrate_AllSensor(); //โชว์ค่าคาลิเบท เซนเซอร์ทั้งหมด



}

void RobotSetupSpeed() {
  SetBalanceSpeedForward();   //ฟังก์ชั่นตั้งค่าความสมดุลมอเตอร์ในแต่ละความเร็ว
  SetBalanceSpeedBackward();  //ฟังก์ชั่นตั้งค่าความสมดุลมอเตอร์ในแต่ละความเร็ว
  SetKpKd();                  //ฟังก์ชั่นตั้งค่า KP KD ในแต่ละความเร็ว
  SetKpKdBack();              //ฟังก์ชั่นตั้งค่า KP KD ในแต่ละความเร็ว
}

void SetKpKd() {                    //เดินหน้า
  Set_KP_KD(SPD_10, 0.004, 0.08);   //ความเร็ว 10
  Set_KP_KD(SPD_20, 0.005, 0.10);   //ความเร็ว 20
  Set_KP_KD(SPD_30, 0.006, 0.12);   //ความเร็ว 30
  Set_KP_KD(SPD_40, 0.007, 0.14);   //ความเร็ว 40
  Set_KP_KD(SPD_50, 0.008, 0.16);   //ความเร็ว 50
  Set_KP_KD(SPD_60, 0.009, 0.18);   //ความเร็ว 60
  Set_KP_KD(SPD_70, 0.010, 0.20);   //ความเร็ว 70
  Set_KP_KD(SPD_80, 0.011, 0.22);   //ความเร็ว 80
  Set_KP_KD(SPD_90, 0.012, 0.24);   //ความเร็ว 90
  Set_KP_KD(SPD_100, 0.013, 0.26);  //ความเร็ว 100
}

void SetKpKdBack() {                     //ถอยหลัง
  Set_KP_KD_Back(SPD_10, 0.004, 0.08);   //ความเร็ว 10
  Set_KP_KD_Back(SPD_20, 0.005, 0.10);   //ความเร็ว 20
  Set_KP_KD_Back(SPD_30, 0.006, 0.12);   //ความเร็ว 30
  Set_KP_KD_Back(SPD_40, 0.007, 0.14);   //ความเร็ว 40
  Set_KP_KD_Back(SPD_50, 0.008, 0.16);   //ความเร็ว 50
  Set_KP_KD_Back(SPD_60, 0.009, 0.18);   //ความเร็ว 60
  Set_KP_KD_Back(SPD_70, 0.010, 0.20);   //ความเร็ว 70
  Set_KP_KD_Back(SPD_80, 0.011, 0.22);   //ความเร็ว 80
  Set_KP_KD_Back(SPD_90, 0.012, 0.24);   //ความเร็ว 90
  Set_KP_KD_Back(SPD_100, 0.013, 0.26);  //ความเร็ว 100

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
