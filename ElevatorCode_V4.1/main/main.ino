#include <AccelStepper.h>
#include <NewPing.h>
#include "step.h"
#include "ObjectDetection.h"
#include <Encoder.h>
#include "motorpwm.h"
#include "PIDController.h"

Encoder encoder(2,3);
long position = -999;
// int state = 1;

// Objects
PIDController pid; // Create an instance of the PID controller class, called "pid"

// Pins
int outputPin   = 11;    // The pin the digital output PMW is connected to
int sensorPin   = A0;   // The pin the analog sensor is connected to
//int goal

void setup () {
  Serial.begin(9600);   // Some methods require the Serial.begin() method to be called first
  pinMode(outputPin, OUTPUT);
  pinMode(sensorPin, INPUT);
  
  pid.begin();          // initialize the PID instance
  pid.setpoint(300);    // The "goal" the PID controller tries to "reach"
  pid.tune(12, .035, 0);    // Tune the PID, arguments: kP, kI, kD
  pid.limit(0, 255);    // Limit the PID output between 0 and 255, this is important to get rid of integral windup!
}

void loop () {

  delay(5000);


  /*
  long prevTime;
  double rad, radDot;
  double degree, degreeDot;
  
  long currTime = millis();
  long prevTime = currTime;
  
  double deltaTime;
  double deltaTimeSec;
  double deltaRad = 0;
  double prevRad = 0;
   */
  
  while(1){
    
  int sensorValue = encoder.read();  // Read the value from the sensor

  /*
  currTime = millis();
  prevTime = currTime;

  deltaTime = (currTime - prevTime);
  deltaTimeSec = deltaTime/1000;

  if(deltaTime > 10){ //greater than 10 ms
    prevTime = currTime;
    
    sensorValue=myEnc.read();

    //-440 encoder count for one rev
    rad = (double)sensorValue/-440.0*(2.0 * 3.14159); //radians
      
    deltaRad = rad - prevRad;
    prevRad = rad;
      
    radDot = deltaRad/deltaTimeSec; //rad/s
    degree = rad*180/3.14159;
    degreeDot = radDot*180/3.14159;
    x=((double)(rad * 1.358)); //cm
    xDot=((double)(radDot * 1.358)); //cm/s
  }
   */
  
  Serial.print("Error: ");
  Serial.print(300-sensorValue);
  // int sensorValue = analogRead(sensorPin);  // Read the value from the sensor
  int output = pid.compute(sensorValue);    // Let the PID compute the value, returns the optimal output
  Serial.print("    output: ");
  Serial.println(output);
  runMotorUp();
  analogWrite(outputPin, output);           // Write the output to the output pin
  delay(1);   
  }                             // Delay for 30 ms
}
