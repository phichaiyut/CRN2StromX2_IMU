#ifndef UNKNOW_BUZZER_H
#define UNKNOW_BUZZER_H

#include <Arduino.h>

#define BUZZER_PIN 33

void Beep(int delayb) {
  pinMode(BUZZER_PIN, OUTPUT);
  analogWriteFreq(2700);
  analogWriteRange(255);
  analogWrite(BUZZER_PIN, 200);
  delay(delayb);
  analogWrite(BUZZER_PIN, 0);
}

void Beep2(int freq, int delayb) {
  pinMode(BUZZER_PIN, OUTPUT);
  analogWriteFreq(2700);
  analogWriteRange(255);
  analogWrite(BUZZER_PIN, freq);
  delay(delayb);
  analogWrite(BUZZER_PIN, 0);
}

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
