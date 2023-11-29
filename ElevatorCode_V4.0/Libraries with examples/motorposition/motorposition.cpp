//unfinished
//  motorposition.cpp - Library for rotary encoder.
//  Created by Andrew Nitchman 11/29/2023
#include "Arduino.h"
#include "motorposition.h"    

  attachInterrupt(digitalPinToInterrupt(2), doEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(3), doEncoderB, RISING);

 motorposition::motorposition(float encoderPos)
 {
   _encoderPos = encoderPos;
 }

 void motorposition::begin()
 {
    pinMode(2, INPUT_PULLUP);
    pinMode(3, INPUT_PULLUP);

 }

 void motorposition::doEncoderA() {
  _encoderPos += (digitalRead(2)==digitalRead(3))?1:0;
}

 void motorposition::doEncoderB(){  
  _encoderPos += (digitalRead(2)==digitalRead(3))?-1:0;
}