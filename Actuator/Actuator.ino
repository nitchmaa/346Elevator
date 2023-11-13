#include <AccelStepper.h> //Stepper motor library

//Function Prototypes
void topActLoad (void); //run top actuator
void botActLoad (void); //run bottom actuator


//DEFINE STEPPER MOTOR VARIABLES
#define TS1  22 // IN1 on the ULN2003, top
#define TS2  23 // IN2 on the ULN2003, top
#define TS3  24 // IN3 on the ULN2003, top
#define TS4  25 // IN4 on the ULN2003, top
#define BS1  22 // IN1 on the ULN2003, bot
#define BS2  23 // IN2 on the ULN2003, bot
#define BS3  24 // IN3 on the ULN2003, bot
#define BS4  25 // IN4 on the ULN2003, bot
#define MotorInterfaceType 8 // Define the interface type as 8 = 4 wires * step factor (2 for half step)
AccelStepper stepTop = AccelStepper(MotorInterfaceType, TS1, TS3, TS2, TS4);//Define the pin sequence (IN1-IN3-IN2-IN4)
AccelStepper stepBot = AccelStepper(MotorInterfaceType, BS1, BS3, BS2, BS4);//Define the pin sequence (IN1-IN3-IN2-IN4)
const int SPR = 2048;//Steps per revolution

void setup() {
 Serial.begin(9600); 
   //STEPPER MOTORS
  pinMode(TS1,OUTPUT);
  pinMode(TS2,OUTPUT);
  pinMode(TS3,OUTPUT);
  pinMode(TS4,OUTPUT);
  stepTop.setMaxSpeed(1000);//Set the maximum motor speed in steps per second
  stepTop.setAcceleration(200);//Set the maximum acceleration in steps per second^2
  pinMode(BS1,OUTPUT);
  pinMode(BS2,OUTPUT);
  pinMode(BS3,OUTPUT);
  pinMode(BS4,OUTPUT);
  stepBot.setMaxSpeed(1000);//Set the maximum motor speed in steps per second
  stepBot.setAcceleration(200);//Set the maximum acceleration in steps per second^2
}

void loop(){
Serial.println("  ");
Serial.println("Step 1");
topActLoad();
delay(2000);
botActLoad();
delay(2000);
}

void topActLoad (void){ //run top actuator
  Serial.println("Running top stepper motor.");
  stepTop.moveTo(3/8*SPR); //Set the target motor position (i.e. turn motor for 3/8 full revolutions)
  stepTop.runToPosition(); // Run the motor to the target position
  stepTop.moveTo(-3/8*SPR); //Set the target motor position (back to start)
  stepTop.runToPosition(); // Run the motor to the target position (start)
}
 
void botActLoad (void){ //run bottom actuator
  Serial.println("Running bottom stepper motor.");
  stepBot.moveTo(3/8*SPR); //Set the target motor position (i.e. turn motor for 3/8 full revolutions)
  stepBot.runToPosition(); // Run the motor to the target position
  stepBot.moveTo(-3/8*SPR); //Set the target motor position (back to start)
  stepBot.runToPosition(); // Run the motor to the target position (start)
}