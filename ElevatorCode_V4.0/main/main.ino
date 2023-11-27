#include <AccelStepper.h>
#include <NewPing.h>
#include "step.h"
#include "ObjectDetection.h"

int state = 1;

void setup() {
Serial.begin(9600); // Opens serial port and sets the data rate to 9600 bits-per-second
}

void loop() {
  switch(state){
    case 1:
      Serial.println("State 1");
      // idle goes here
      if(objectDetect() == 0){
        break;
      };
     if(objectDetect() == 1){
       actuateTop();
       //idle again
      };
      state = 3;
      break;
    case 2:
    
     Serial.println("State 2");
     //idle goes here
     if(objectDetect() == 0){
        break;
      };
      if(objectDetect() == 2){
        //actuate bottom
        actuateBottom();
        //idle again
      };
      state = 4;
      break;
    case 3:
      Serial.println("State 3");
      //move down code goes here
      state = 2;
      break;
    case 4:
      Serial.println("State 4");
      //move up code goes here
      state = 1;
      break;

  }
}
