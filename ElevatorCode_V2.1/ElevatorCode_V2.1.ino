// ELEVATOR CODE
// VERSION 2.1
// LAST REVISED BY: MB
// REVISION DATE: 11/13/23

/* TO DO:
 * - Complete functions
 * - Find motor deadband limits
 * - Adjust constants as needed
 * - Confirm DC and stepper motor direction functions
 */
 

/* STATES: 1 - at top
 *         2 - at bottom
 *         3 - moving down
 *         4 - moving up
 * SUBSTATES: A - no load in car
 *            B - loaded at top
 *            C - loaded at bottom
 */

// Include the Arduino libraries:
#include <AccelStepper.h> //Stepper motor library
#include <NewPing.h> //Sonar library

//FUNCTION PROTOTYPES
void setUp(void); //configures pins; setup function
int objectDetect (void); //returns 0 if no object, 1 if top, 2 if bottom
void topActLoad (void); //run top actuator
void botActLoad (void); //run bottom actuator
void motorDown (void); //set direction down
void motorUp (void); //set direction up
void stopMotor (void); //stops elevator
void motorSpeed(int ein); //set motor speed
int constantSpeed(); //adjust voltage for constant speed; return new voltage
int proportionSpeed(int deltaX); //find volt to reduce speed proportional to distance from floor; receive dx, return new voltage
           //combine constantSpeed and proportionSpeed, use deltaX to decide on how to adjust?
void readThetaAndX(void); //read theta and thetaDot

void moveDown (void); //feedback loop down
void moveUp (void); //feedback loop up

void rollUp(void);

void updateEncoder(void); //Increment Value for Each pulse from Encoder 

/* FOR MOTOR DEADBAND
#define c_kinetic_pos 72 // static friction 
#define c_kinetic_neg 87 // make the value positive 
#define c_static_pos 99 // kinetic friction 
#define c_static_neg 96 // make the value positive
#define c_max 255
#define c_min 255 //make the value positive
*/

//DEFINE DC MOTOR VARIABLES
#define MP1 10 //motor pin 1
#define MP2 12 //motor pin 2
#define MPEN 11 //motor pin enable


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


//DEFINE SONAR VARIABLES
#define trigTop 48
#define echoTop 49
#define trigBot 52
#define echoBot 53
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).
NewPing sonarTop(trigTop, echoTop, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarBot(trigBot, echoBot, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


//DEFINE ROTARY ENCODER VARIABLES
#define COUNT_REV 115 //Rodery encoder output pulse per rotation (Need to change depemnding on encoder)
#define ENCODER_IN 3 //Encoder output to Aurdino Interrupt pin
#define ENCODER_IN2 2

 volatile long encoderValue = 0; //Pulse count from encoder 
 int interval = 100; //1 second interval measuments 
 long previousMillis = 0;
 long currentMillis = 0;
 float rpm = 0;
 float theta =0;
 float thetaDot = 0;
 float x = 0;
 float xDot = 0;
 int encoderOverallValue;
 int b;


//DEFINE GENERAL USE VARIABLES
int state = 1;
char substate = 'A';

int objectPlat = 0; //stores response from function for whether object has been detected

int topX = 70; //x of top floor
int botX = 0; //x of bottom floor
float errorMargin = 0.25; //margin of error
int xClose = 8; //to determine whether wanted - actual difference is small or large

int dir = 1; //1 up, 2 down
float changeTheta;

volatile long voltage; //store current voltage being sent to motor

void setup() {
  Serial.begin(9600); // Opens serial port and sets the data rate to 9600 bits-per-second
  
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  setUp(); //function to set pins
  rollUp();
}


void loop() {
  while(1){
    switch (state) {
      case 1:
        if(substate == 'A'){
          Serial.println("State 1A");
          objectPlat = objectDetect(); //is an object detected? if no (return of 0), do nothing
          if(objectPlat == 1){ //if on top, load top actuator, set to state 3-B
            topActLoad();
            state = 3;
            substate = 'B';
          }
          else if(objectPlat == 2){ //if on bot, set to state 3
            state = 3;
          }
        }
      
        else if(substate == 'B'){
          Serial.println("State 1B");
          stopMotor(); //stop motor. Path B has finished.        
        }

        else if(substate == 'C'){
          Serial.println("State 1C");
          delay(1000); //pause 1 sec, then set to state 3
          state = 3;
        }
        break;
      
      case 2:
        if(substate == 'A'){
          Serial.println("State 2A");
          objectPlat = objectDetect(); //is an object detected? if no, do nothing
          if(objectPlat == 2){ //if on bottom, load bottom actuator, set to state 4-C (should never be on top floor here)
            botActLoad();
            state = 4;
            substate = 'C';
          }
        }
      
        else if(substate == 'B'){
          Serial.println("State 2B");   
          delay(1000); //pause 1 sec, then set to state 4
          state = 4;
        }

        else if(substate == 'C'){
          stopMotor(); //stop motor. Path C has finished. 
        }
        break;
      
      case 3:
        moveDown(); //enter feedback loop, moving down, then set to state 2
        state = 2;
        break;
      
      case 4:
        moveUp(); //enter feedback loop, moving up, then set to state 1
        state = 1;
        break;
      
      default:
        Serial.println("Error in switch case. Stopping motor.");
        digitalWrite(LED_BUILTIN, HIGH);
        stopMotor();
        break;
    }
  }
}




//FUNCTION DEFINITONS

void setUp(void){ //setup function
  //DC MOTOR
  pinMode(MP1, OUTPUT);
  pinMode(MP2, OUTPUT);
  pinMode(MPEN, OUTPUT);
  digitalWrite(MP1, LOW);
  digitalWrite(MP2, LOW);
  analogWrite(MPEN, 0);

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

  //SONAR
  pinMode(trigTop,OUTPUT);
  pinMode(echoTop,INPUT);
  pinMode(trigBot,OUTPUT);
  pinMode(echoBot,INPUT);

  //ROTARY ENCODER
  pinMode (ENCODER_IN, INPUT_PULLUP);
  pinMode (ENCODER_IN2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN), updateEncoder, RISING); // Interupt to find X and X dot
  previousMillis = millis();
}

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

void topActLoad (void){ //run top actuator
  Serial.println("Running top stepper motor.");
  stepTop.moveTo(5/16*SPR); //Set the target motor position (i.e. turn motor for 3/8 full revolutions)
  stepTop.runToPosition(); // Run the motor to the target position
  stepTop.moveTo(-5/16*SPR); //Set the target motor position (back to start)
  stepTop.runToPosition(); // Run the motor to the target position (start)
}

void botActLoad (void){ //run bottom actuator
  Serial.println("Running bottom stepper motor.");
  stepBot.moveTo(5/16*SPR); //Set the target motor position (i.e. turn motor for 3/8 full revolutions)
  stepBot.runToPosition(); // Run the motor to the target position
  stepBot.moveTo(-5/16*SPR); //Set the target motor position (back to start)
  stepBot.runToPosition(); // Run the motor to the target position (start)
}

void motorDown (void){ //set direction down
  digitalWrite(MP1, LOW);
  digitalWrite(MP2, HIGH);
  Serial.println("Direction: DOWN");
  dir = 2;
}

void motorUp (void){ //set direction up
  digitalWrite(MP1, HIGH);
  digitalWrite(MP2, LOW);
  Serial.println("Direction: UP");
  dir = 1;
}

void stopMotor (void){ //stops elevator
  digitalWrite(MP1, LOW);
  digitalWrite(MP2, LOW);
  Serial.println("Motor stopped.");
  dir = 0;
}

void motorSpeed(int ein){ //set motor speed; receives input voltage
  analogWrite(MPEN, ein);
  Serial.print("Changing input voltage to: ");
  Serial.println(ein);
}

int constantSpeed(void){ //adjust voltage for constant speed; return new voltage
  int voltAdjust; //value to be returned
  double Kc = 0.1;
  int speedSignal;
  int speedDiff;
    
//  int largeChange = 20;
//  int smallChange = 5;

  speedDiff = xDot - 50; //find speed difference
  speedSignal = Kc*speedDiff; //find proportion for speed change
  voltAdjust = voltage + speedSignal; //calculate new voltage

/*
  if(xDot > 60){ //current velocity greater than 60 cm/s = 0.6 m/s, large input voltage decrease
    voltAdjust = voltAdjust - largeChange;
  }
  else if(xDot < 40){ //current velocity less than 40 cm/s = 0.4 m/s, large input voltage increase
    voltAdjust = voltAdjust + largeChange;
  }

  else if(xDot > 50){ //current velocity greater than 50 cm/s, less than 60 cm/s, small input voltage decrease
    voltAdjust = voltAdjust - smallChange;
  }
  else if(xDot < 50){ //current velocity less than 50 cm/s, greater than 40 cm/s, small input voltage increase
    voltAdjust = voltAdjust + smallChange;
  }
  
  else{} //if current velocity is 50 cm/s = 0.5 m/s, do nothing
  */

  voltage = voltAdjust; //update global voltage

  return voltAdjust;
}

int proportionSpeed(int deltaX){ //find volt to reduce speed proportional to distance from floor; receive dx, return new voltage
  int voltAdjust; //value to be returned
  int minUnload = 50; //minimum voltage to move motor when unloaded
  int minLoad = 75; //minimum voltage (approx) to move motor when loaded
  
  double Kp = 0.1;
  int speedSig = Kp*deltaX;
  voltAdjust = voltage - speedSig; //calculate new voltage

//  voltAdjust = Kp*voltAdjust; //adjust voltage to percrentage
  if(substate == 'A' && voltAdjust < minUnload){ //unloaded, minimum voltage needed to move
    voltAdjust = minUnload;
  }
  if(voltAdjust < minLoad){ //loaded, minimum voltage needed to move
    voltAdjust = minLoad;
  }
  
  voltage = voltAdjust; //update global voltage
  
  return voltAdjust;
}

void readThetaAndX(void){ //read theta and thetaDot, translate to x and xDot
  currentMillis = millis();
  
  if (currentMillis - previousMillis > interval){
    int timefortheta = currentMillis - previousMillis;
    previousMillis = currentMillis;
  

   rpm = (float)(encoderValue * 600.0 / COUNT_REV);
   //theta = (float)(theta + ((rpm / 60.0) * 2.0 * 3.14159)* (timefortheta/1000)); // Radians 
   theta = (((float)encoderOverallValue / COUNT_REV ) * 2 * 3.14159);
   thetaDot = (float)((rpm /60.0) * 2.0 * 3.14159); //Radians Per Second
   x = ((float)(theta * 1.358)); //However big the radius of the pully is goes here, x starts at 70 
   xDot = ((float)thetaDot * 1.358); //Radius of the pully goes in the second term 

  if(rpm != 0){
  
    Serial.print("Pulses: ");
    Serial.print(encoderValue);
    Serial.print('\t');
    Serial.print("Speed: ");
    Serial.print(rpm);
    Serial.println(" RPM");
    Serial.print('\t');
    Serial.print("Theta: ");
    Serial.print(theta);
    Serial.println(" Rad");
    Serial.print('\t');
    Serial.print("Theta Dot: ");
    Serial.print(thetaDot);
    Serial.println(" Rad/sec");
    Serial.print('\t');
    Serial.print("X ");
    Serial.print(x);
    Serial.println(" cm");
    Serial.print('\t');
    Serial.print("XDot ");
    Serial.print(xDot);
    Serial.println(" cm/s");
  }
  
  encoderValue = 0;

  }
  
}

void moveDown (void){ //feedback loop down
  int initialVolt = 30;
  int counter = 0;
  int finished = 0;
  int volt;
  float dx;

  Serial.println("Feedback loop, down.");

  voltage = initialVolt; //update global voltage

  motorDown(); //set initial direction to down
  motorSpeed(initialVolt); //set initial motor speed

  while(finished != 1){ //loop until elevator stablized at destination
    readThetaAndX(); //get current position and speed

    if(xDot < 5){
      voltage = voltage + 10;
    }

    if(x < botX && x > (botX - errorMargin)){ //x at bottom floor within margin of error
      counter++; //increment counter
      if(counter == 200){ //elevator has stablized, end while loop
        finished = 1;
      }
    }
    else if (x > botX){ //x above floor
      counter = 0; //if not at bottom floor, reset counter
      motorDown(); //set direction down
      dx = x - botX; //find difference between actual x and wanted x
      if(dx <= xClose){ //small difference, set speed proportional to difference
        voltage = 20;
        motorSpeed(voltage);
      }
      else{ //large difference, set constant speed
        voltage = initialVolt;
        motorSpeed(voltage);
      }
    }
    else if(x < (botX - errorMargin)){ //x below floor (overshot)
      counter = 0; //if not at bottom floor, reset counter
      motorUp(); //set direction up
      voltage = 20;
      motorSpeed(voltage);
    }/*
    else if (x > botX){ //x above floor
      counter = 0; //if not at bottom floor, reset counter
      motorDown(); //set direction down
      dx = x - botX; //find difference between actual x and wanted x
      if(dx <= xClose){ //small difference, set speed proportional to difference
        volt = proportionSpeed(dx);
        motorSpeed(volt);
      }
      else{ //large difference, set constant speed
        volt = constantSpeed();
        motorSpeed(volt);
      }
    }
    else if(x < (botX - errorMargin)){ //x below floor (overshot)
      counter = 0; //if not at bottom floor, reset counter
      motorUp(); //set direction up
      dx = botX - x; //find difference between wanted x and actual x
      volt = proportionSpeed(dx); //assume small difference, set speed proportional to difference
      motorSpeed(volt);
    }*/
    Serial.print("Current motor voltage: ");
    Serial.println(voltage);
  }
}


void moveUp (void){ //feedback loop up
  int initialVolt = 30;
  int counter = 0;
  int finished = 0;
  int volt;
  float dx;

  Serial.println("Feedback loop, up.");

  voltage = initialVolt; //update global voltage

  motorUp(); //set initial direction to up
  motorSpeed(initialVolt); //set initial motor speed

  while(finished != 1){ //loop until elevator stablized at destination
    readThetaAndX(); //get current position and speed

    if(xDot < 5){
      voltage = voltage + 10;
    }

    if(x < topX && x > (topX - errorMargin)){ //x at top floor within margin of error
      counter++; //increment counter
      if(counter == 200){ //elevator has stablized, end while loop
        Serial.print("Counter: ");
        Serial.println(counter);
        finished = 1;
      }
    }
    else if (x > topX){ //x above floor (overshoot)
      counter = 0; //if not at top floor, reset counter
      motorDown(); //set direction down
      voltage = 20;
      motorSpeed(voltage);
    }
    else if(x < (topX - errorMargin)){ //x below floor
      counter = 0; //if not at top floor, reset counter
      motorUp(); //set direction up
      dx = topX - x; //find difference between wanted x and actual x
      if(dx <= xClose){ //small difference, set speed proportional to difference
        voltage = 20;
        motorSpeed(voltage);
      }
      else{ //large difference, set constant speed
        voltage = initialVolt;
        motorSpeed(voltage);
      }
    }
    /*
    else if (x > topX){ //x above floor (overshoot)
      counter = 0; //if not at top floor, reset counter
      motorDown(); //set direction down
      dx = x - topX; //find difference between actual x and wanted x
      volt = proportionSpeed(dx); //assume small difference, set speed proportional to difference
      motorSpeed(volt);
    }
    else if(x < (botX - errorMargin)){ //x below floor
      counter = 0; //if not at top floor, reset counter
      motorUp(); //set direction up
      dx = topX - x; //find difference between wanted x and actual x
      if(dx <= xClose){ //small difference, set speed proportional to difference
        volt = proportionSpeed(dx);
        motorSpeed(volt);
      }
      else{ //large difference, set constant speed
        volt = constantSpeed();
        motorSpeed(volt);
      }
    }*/
    Serial.print("Current motor voltage: ");
    Serial.println(voltage);
    Serial.println(encoderValue);
  }
}


void updateEncoder(void){
 b = digitalRead(ENCODER_IN2);
  if (b == 0){
      encoderValue --;
      encoderOverallValue --;
  }else {
      encoderValue ++;
      encoderOverallValue ++;
  }
}

void rollUp(void){
  moveUp();
}
