#include "definePins.h"
#include "sonar_actuators.h"
#include <NewPing.h>
#include <AccelStepper.h>

#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).
NewPing sonarTop(trigTop, echoTop, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarBot(trigBot, echoBot, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

#define MotorInterfaceType 8 // Define the interface type as 8 = 4 wires * step factor (2 for half step)
AccelStepper stepTop = AccelStepper(MotorInterfaceType, TS1, TS3, TS2, TS4);//Define the pin sequence (IN1-IN3-IN2-IN4)
AccelStepper stepBot = AccelStepper(MotorInterfaceType, BS1, BS3, BS2, BS4);//Define the pin sequence (IN1-IN3-IN2-IN4)
const int SPR = 2048;//Steps per revolution


int objectDetect(void){ //detect object at platforms; returns 0 if no object, 1 if top, 2 if bottom
  int objectPosition = 0; //stores position of object; value to be returned

  int distanceTop = sonarTop.ping_cm(); //Send ping for top, get distance in cm and print result (0 = outside set distance range)
  Serial.print("Top distance: ");
  Serial.print(distanceTop);
  Serial.println("cm");
  
  int distanceBot = sonarBot.ping_cm(); //Send ping for bot, get distance in cm and print result (0 = outside set distance range)
  Serial.print("Bottom distance: ");
  Serial.print(distanceBot);
  Serial.println("cm");
  
  if(distanceTop <= 5) objectPosition = 1; //top sensor triggered
  else if(distanceBot <= 5) objectPosition = 2; //bottom sensor triggered
  else objectPosition = 0; //neither sensor triggered, no object present
  
  return objectPosition;
}


void topActLoad (double fraction){ //run top actuator
  stepTop.setMaxSpeed(1000);//Set the maximum motor speed in steps per second
  stepTop.setAcceleration(200);//Set the maximum acceleration in steps per second^2
  
  Serial.println("Running top stepper motor.");
  stepTop.moveTo(fraction*SPR); //Set the target motor position (i.e. turn motor for 3/8 full revolutions)
  stepTop.runToPosition(); // Run the motor to the target position
  delay(500);
  stepTop.moveTo(-fraction*SPR); //Set the target motor position (back to start)
  stepTop.runToPosition(); // Run the motor to the target position (start)
    
  delay(1000);
}


void botActLoad (double fraction){ //run bottom actuator
  stepBot.setMaxSpeed(1000);//Set the maximum motor speed in steps per second
  stepBot.setAcceleration(200);//Set the maximum acceleration in steps per second^2

  Serial.println("Running bottom stepper motor.");
  stepBot.moveTo(fraction*SPR); //Set the target motor position (i.e. turn motor for 3/8 full revolutions)
  stepBot.runToPosition(); // Run the motor to the target position
  delay(500);
  stepBot.moveTo(-fraction*SPR); //Set the target motor position (back to start)
  stepBot.runToPosition(); // Run the motor to the target position (start)
    
  delay(1000);
}
