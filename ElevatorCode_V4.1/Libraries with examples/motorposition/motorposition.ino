#include "motorposition.h"

float encoderPos = 0;

void setup()
{
  // pinMode(2, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(2), doEncoderA, RISING);
  
  // pinMode(3, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(3), doEncoderB, RISING);


  
  Serial.begin(9600);
}

void loop()
{
  delay(1000);
  Serial.println(encoderPos);

}

// // functions
// void doEncoderA() {
//   encoderPos += (digitalRead(2)==digitalRead(3))?1:0;
// }
// void doEncoderB(){  
//   encoderPos += (digitalRead(2)==digitalRead(3))?-1:0;
// }