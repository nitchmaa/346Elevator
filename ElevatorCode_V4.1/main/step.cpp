#include "step.h"           // local library hence the "quotes"
#include <AccelStepper.h>   // not local so use arrows


//DEFINE VARIABLES CALLED INTO FUNCTIONS, PROTOTYPES, AND CLASSES////////////////////////////////////////////////////
// Motor pin definitions:
#define Bot_motorPin1  28     // IN1 on the ULN2003 driver
#define Bot_motorPin2  29     // IN2 on the ULN2003 driver
#define Bot_motorPin3  30     // IN3 on the ULN2003 driver
#define Bot_motorPin4  31     // IN4 on the ULN2003 driver
#define Top_motorPin1  22     // IN1 on the ULN2003 driver
#define Top_motorPin2  23     // IN2 on the ULN2003 driver
#define Top_motorPin3  24     // IN3 on the ULN2003 driver
#define Top_motorPin4  25     // IN4 on the ULN2003 driver
#define MotorInterfaceType 8  
#define Swing 1200

//DEFINE YOUR CLASSES
AccelStepper Bottom = AccelStepper(MotorInterfaceType, Bot_motorPin1, Bot_motorPin3, Bot_motorPin2, Bot_motorPin4);
AccelStepper Top = AccelStepper(MotorInterfaceType, Top_motorPin1, Top_motorPin3, Top_motorPin2, Top_motorPin4);

//PUT YOUR FUNCTION DEFINITIONS HERE/////////////////////////////////////////////////////////////////////////////////
void actuateBottom(){
  // Bottom Motor first. Set position to 0.  Run forward to complete swing. Run backward to 0.
  Bottom.setMaxSpeed(1000);
  Bottom.setCurrentPosition(0);
    while (Bottom.currentPosition() != Swing) {
    Bottom.setSpeed(999);
    Bottom.runSpeed();
  }
  while (Bottom.currentPosition() != 0) {
    Bottom.setSpeed(-999);
    Bottom.runSpeed();
  }
}

void actuateTop(){
    Top.setMaxSpeed(1000);
    Top.setCurrentPosition(0);
    while (Top.currentPosition() != Swing) {
    Top.setSpeed(999);
    Top.runSpeed();
  }
  while (Top.currentPosition() != 0) {
    Top.setSpeed(-999);
    Top.runSpeed();
  }
}