#ifndef UNKNOW_BUZZER_H
#define UNKNOW_BUZZER_H

#include <Arduino.h>

/* ==================================================================================================
 *  Unknow_Buzzer.h — เสียงบี๊บ
 * ================================================================================================== */

#define BUZZER_PIN 33

// บี๊บนาน delayb ms
void Beep(int delayb) {
  pinMode(BUZZER_PIN, OUTPUT);
  analogWriteFreq(2700);
  analogWriteRange(255);
  analogWrite(BUZZER_PIN, 200);
  delay(delayb);
  analogWrite(BUZZER_PIN, 0);
}

// บี๊บนาน delayb ms กำหนดความดัง freq (0-255)
void Beep2(int freq, int delayb) {
  pinMode(BUZZER_PIN, OUTPUT);
  analogWriteFreq(2700);
  analogWriteRange(255);
  analogWrite(BUZZER_PIN, freq);
  delay(delayb);
  analogWrite(BUZZER_PIN, 0);
}

// เปิดเสียงค้างไว้ (ปิดด้วย BZoff)
void BZon() {
  pinMode(BUZZER_PIN, OUTPUT);
  analogWriteFreq(2700);
  analogWriteRange(255);
  analogWrite(BUZZER_PIN, 200);
}

void BZoff() {
  analogWrite(BUZZER_PIN, 0);
}

#endif
