//unfinished
//  motorposition.h - Library for rotary encoder.
//  Created by Andrew Nitchman 11/29/2023
#ifndef motorposition_h
#define motorposition_h

#include "Arduino.h"

class motorposition
{
  public:
    motorposition(float encoderPos);
    void begin();
    void doEncoderA();
    void doEncoderB();
  private:
    float _encoderPos; 
};

#endif