void ServoOpen() {
  MotorStop();
  ServoMove2Sync(1, OpenL, 2, OpenR, SPD_OPEN);
  // delay(200);
}

void ServoOpen2() {
  MotorStop();
  ServoMove2Sync(1, OpenL2, 2, OpenR2, SPD_OPEN);
  // delay(200);
}

void ServoOpenMax() {
  MotorStop();
  ServoMove2Sync(1, OpenLMax, 2, OpenRMax, SPD_OPEN);
  delay(200);
}

void ServoOpenMax2() {
  MotorStop();
  ServoMove2Sync(1, OpenLMax + 40, 2, OpenRMax - 40, SPD_OPEN);
  delay(200);
}

void ServoClose() {
  ServoMove2Sync(1, CloseL, 2, CloseR, SPD_CLOSE);
  delay(40);
}

void ServoGrip() {
  MotorStop();
  ServoMove2Sync(1, CloseL + 15, 2, CloseR - 15, SPD_CLOSE);
  delay(100);
}

void ServoCloseSmall() {
  MotorStop();
  ServoMove2Sync(1, CloseLSmall, 2, CloseRSmall, SPD_CLOSE);
  delay(100);
}

void ServoUp() {
  MotorStop();
  ServoMoveSpeed(0, Up, SPD_UP);
  delay(100);
}

void ServoUp45() {
  MotorStop();
  ServoMoveSpeed(0, Up - 40, SPD_UP);
  delay(100);
}

void ServoDown() {
  MotorStop();
  ServoMoveSpeed(0, Down, SPD_DOWN);
  delay(200);
}

void ServoLClose() {
  MotorStop();
  ServoMoveSpeed(1, CloseL, SPD_OPEN);
  delay(100);
}
void ServoRClose() {
  MotorStop();
  ServoMoveSpeed(2, CloseR, SPD_OPEN);
  delay(100);
}

void ServoLCloseSmall() {
  MotorStop();
  ServoMoveSpeed(1, CloseLSmall, SPD_OPEN);
  delay(100);
}
void ServoRCloseSmall() {
  MotorStop();
  ServoMoveSpeed(2, CloseRSmall, SPD_OPEN);
  delay(100);
}

void ServoLOpen() {
  MotorStop();
  ServoMoveSpeed(1, OpenL, SPD_OPEN);
  delay(100);
}
void ServoROpen() {
  MotorStop();
  ServoMoveSpeed(2, OpenR, SPD_OPEN);
  delay(100);
}

void ServoLOpenMax() {
  MotorStop();
  ServoMoveSpeed(1, OpenLMax, SPD_OPEN);
  delay(100);
}
void ServoROpenMax() {
  MotorStop();
  ServoMoveSpeed(2, OpenRMax, SPD_OPEN);
  delay(100);
}

void ServoLOpenMax2() {
  MotorStop();
  ServoMoveSpeed(1, OpenLMax + 40, SPD_OPEN);
  delay(100);
}
void ServoROpenMax2() {
  MotorStop();
  ServoMoveSpeed(2, OpenRMax - 40, SPD_OPEN);
  delay(100);
}
