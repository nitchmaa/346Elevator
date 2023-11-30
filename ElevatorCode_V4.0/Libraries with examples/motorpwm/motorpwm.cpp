#include "motorpwm.h"           // local library hence the "quotes"


//DEFINE VARIABLES CALLED INTO FUNCTIONS, PROTOTYPES, AND CLASSES////////////////////////////////////////////////////
#define MP1 10
#define MP2 12
#define MPEN 11
int pwm=255;
// pinMode(MP1, OUTPUT); //Declaring the pin modes, obviously they're outputs
// pinMode(MP2, OUTPUT);
// pinMode(MPEN, OUTPUT);

//DEFINE YOUR CLASSES


//PUT YOUR FUNCTION DEFINITIONS HERE/////////////////////////////////////////////////////////////////////////////////
void runMotorUp(){

  pinMode(MP1, OUTPUT); //Declaring the pin modes, obviously they're outputs
  pinMode(MP2, OUTPUT);
  pinMode(MPEN, OUTPUT);
  digitalWrite(MP1, LOW);
  digitalWrite(MP2, HIGH);
  analogWrite(MPEN,pwm);
}
 
void TurnOFFA(){
  pinMode(MP1, OUTPUT); //Declaring the pin modes, obviously they're outputs
  pinMode(MP2, OUTPUT);
  pinMode(MPEN, OUTPUT); 
  digitalWrite(MP1, LOW);
  digitalWrite(MP2, LOW);
  analogWrite(MPEN,0);
}