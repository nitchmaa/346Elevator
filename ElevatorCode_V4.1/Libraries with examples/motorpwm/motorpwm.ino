#include "motorpwm.h"
 
void setup() {

}

void loop() {
  runMotorUp();  //Sequence: turning on low speed, stop, turning again in high speed and stop
  delay(2000);
 
  //TurnOFFA();

  delay(2000);
}

