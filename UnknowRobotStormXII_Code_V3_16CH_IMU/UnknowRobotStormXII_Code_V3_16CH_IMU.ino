#include <UnknowRobot_STORMXII_V3_IMU.h>

void setup() {
  RobotSetup();

  //////////// CALIBRATE ROBOT SENSOR /////
  // CalibrateSensor();
  /////////////////////////////////////////
  LightValue_FrontSensor(168, 155, 172, 164, 193, 166, 166, 152, 159, 164, 173, 176, 156, 150, 153, 160, 873, 873, 874, 872, 877, 873, 874, 867, 871, 876, 879, 877, 870, 862, 871, 876);
  LightValue_BackSensor(172, 138, 177, 162, 172, 168, 169, 167, 163, 163, 160, 142, 154, 150, 150, 159, 878, 856, 879, 875, 876, 871, 872, 877, 877, 877, 871, 831, 863, 874, 873, 879);
  LightValue_CenterSensor(127, 121, 866, 807);
  /////////////////////////////////////////

  setup_robot();
  RobotSetupSpeed();  //ตั้งค่าความเร็ว ตั้งค่าสมดุล ซ้ายขวา ตั้งค่า kp kd

  //////////// SET LIGHT VALUE ////////////
  RefLineValue(500);          // ค่าในการจับเส้น เซนเซอร์หน้า หลัง  //ถ้าหุ่นยนต์ไม่นับแยก ลดค่าลง 100
  RefCenterLineValue(500);    // ค่าในการจับเส้น เซนเซอร์คู่กลาง    //ถ้าหุ่นยนต์ไม่นับแยก ลดค่าลง 100
  TrackLineColor(BLACK);      // สีที่หุ่นยนต์วิ่ง BLACK / WHITE
  SetRobotPID(0.009, 0.12);   // ค่า PID  >> KP, KD //////////////////////////////////////////////////
  SetSensorTrackLine_CH(16);  // จำนวนเซนเซอร์ที่ใช้ตรวจจับเส้น 6 , 8 , 10 , 12 , 14 , 16 CH
  /////////////////////////////////////////

  //////////// SET TURN SPEED /////////////
  TurnSpeedLeft(25, 80, 60);   // ความเร็วเลี้ยวซ้าย
  TurnSpeedRight(80, 25, 60);  // ความเร็วเลี้ยวขวา
  SetToCenterSpeed(30);        // ความเร็วเข้ากลางหุ่น
  SetTurnSpeed(45);            // ความเร็วเลี้ยวเข้าแยก
  /////////////////////////////////////////

  //////////// SET BALANCE MOTOR //////////
  BalanceMotorLeft = 0;
  BalanceMotorRight = 0;
  /////////////////////////////////////////

  //////////// SET SERVO ANGLE ////////////
  setOpen(150, 30);
  setOpen2(60, 120);
  setOpenMax(165, 255);

  setClose(40, 110);
  setCloseSmall(15, 165);
  setUpDown(150, 50);
  /////////////////////////////////////////

  //////////// SET SERVO SPEED ////////////
  /// SERVO_SLOW   << SERVO RUN IN SLOW SPEED
  /// SERVO_MEDIUM << SERVO RUN IN MEDIUM SPEED
  /// SERVO_FAST   << SERVO RUN IN FAST SPEED
  setServoSpeed_Up(SERVO_FAST);
  setServoSpeed_Down(SERVO_FAST);
  setServoSpeed_Open(SERVO_FAST);
  setServoSpeed_Close(SERVO_MEDIUM);
  /////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////
  //////////////////////////////เช็คค่าเซนเซอร์//////////////////////////
  // Serial_FrontSensor();            // Serial Monitor เซนเซอร์หน้า
  // Serial_BackSensor();             // Serial Monitor เซนเซอร์หลัง
  // Serial_CenterSensor();           // Serial Monitor เซนเซอร์กลาง
  // SerialCalibrate_FrontSensor();  // Serial Monitor เซนเซอร์หน้า คาริเบทแล้ว
  // SerialCalibrate_BackSensor();    // Serial Monitor เซนเซอร์หลัง คาริเบทแล้ว
  // SerialCalibrate_CenterSensor();  // Serial Monitor เซนเซอร์กลาง คาริเบทแล้ว

  // SerialCalibrate_AllSensor(); //โชว์ค่าคาลิเบท เซนเซอร์ทั้งหมด
  ////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////



  ServoOpen();
  // ServoCloseSmall();
  ServoUp();

  swOK();
  delay(200);
  setAngleOffset();  // SETUP GYRO ANGLE
  //TestMotor(); //ทดสอบมอเตอร์ ล้อซ้ายจะหมุนไปข้างหน้าก่อน ล้อขวาหมุนไปข้างหน้า
  //SerialServoControl();  //ตั้งค่าเซอร์โวผ่าน serial monitor. 39 38 37  36  35 34 2 1 0


  Mission();


  MotorStop();




}


void loop() {

  MotorStop();
}
