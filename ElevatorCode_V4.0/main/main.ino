#include <AccelStepper.h>
#include <NewPing.h>
#include "step.h"
#include "ObjectDetection.h"
#include <Encoder.h>
#include "motorpwm.h"
#include "PIDController.h"

Encoder encoder(2,3);
long position = -999;
int state = 1;

PIDController pid; // Create an instance of the PID controller class, called "pid"
// Pins
int outputPin   = 11;    // The pin the digital output PMW is connected to
int sensorPin   = A0;   // The pin the analog sensor is connected to
    int pastError = 0;
    int finished = 0;

void setup() {
  delay(5000);
Serial.begin(9600); // Opens serial port and sets the data rate to 9600 bits-per-second
pinMode(outputPin, OUTPUT);
  pinMode(sensorPin, INPUT);
  
  pid.begin();          // initialize the PID instance
  pid.setpoint(240);    // The "goal" the PID controller tries to "reach"
  pid.tune(5, 3.5, 0);    // Tune the PID, arguments: kP, kI, kD
  pid.limit(0, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral wind
  while(finished == 0){
    int sensorValue = encoder.read();  // Read the value from the sensor
    Serial.print("Error: ");
    Serial.print(240-sensorValue);
    int error = 300-sensorValue;
    // int sensorValue = analogRead(sensorPin);  // Read the value from the sensor
    int output = pid.compute(sensorValue);    // Let the PID compute the value, returns the optimal output
    Serial.print("    output: ");
    Serial.println(output);
    runMotorUp();
    if(error == 1){
      finished = 1;}
    if(error == pastError && error < 200){
      finished = 1;
     // analogWrite(outputPin, 3);
    }
    else {
      pastError = error;
      //analogWrite(outputPin, 3);
      analogWrite(outputPin, output); 
     }
    delay(1);   
  }                             // Delay for 30 ms
}

void loop() {
  switch(state){
    case 1:
      Serial.println("State 1");
      // runMotorUp();
      // Serial.println(encoder.read());
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
