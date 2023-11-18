// ELEVATOR CODE
// VERSION 3.1
// LAST REVISED BY: MB
// REVISION DATE: 11/17/23

/* STATES: 1 - at top
 *         2 - at bottom
 *         3 - moving down
 *         4 - moving up
 * SUBSTATES: A - no load in car
 *            B - loaded at top
 *            C - loaded at bottom
 */

#include "definePins.h"
#include "sonar_actuators.h"
#include "dcMotor_encoder.h"


//FUNCTION PROTOTYPES
void setUp(void); //configures pins; setup function
int constantSpeed(); //adjust voltage for constant speed; return new voltage
int proportionSpeed(int deltaX); //find volt to reduce speed proportional to distance from floor; receive dx, return new voltage
void moveDown (void); //feedback loop down
void moveUp (void); //feedback loop up
void idle (void);

//DEFINE GENERAL USE VARIABLES
int state = 1;
char substate = 'A';

int objectPlat = 0; //stores response from function for whether object has been detected
int topX = 70; //x of top floor
int botX = 0; //x of bottom floor
float errorMargin = 0.25; //margin of error
int xClose = 8; //to determine whether wanted - actual difference is small or large
volatile long voltage; //store current voltage being sent to motor




//POSITION VALUES FOR CONTROL
 float theta =0;
 float thetaDot = 0;
 float x = 0;
 float xDot = 0;
  long previousMillis = 0;


 //NEW FUNCTION CALL:
 //readThetaAndX(&theta, &thetaDot, &x, &xDot, &previousMillis);
 



void setup() {
  Serial.begin(9600); // Opens serial port and sets the data rate to 9600 bits-per-second
  
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  
  setUp(); //function to set pins
  topActLoad(0.1); //pre-loop run of actuators
  botActLoad(0.1);
  
  //moveUp();
}


void loop() {
  while(1){
    while(1){
      readThetaAndX(&theta, &thetaDot, &x, &xDot, &previousMillis);
      Serial.println("MAIN X: ");
      Serial.print(x);
    }
    switch (state) {
      case 1:
        if(substate == 'A'){
          idle();
          Serial.println("State 1A");
          objectPlat = objectDetect(); //is an object detected? if no (return of 0), do nothing
          idle();
          if(objectPlat == 1){ //if on top, load top actuator, set to state 3-B
            topActLoad(0.3125);
            Serial.println("actuating");
            delay(5000);
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
            botActLoad(0.3125);
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
         substate = 'A';
        break;
      
      case 4:
        moveUp(); //enter feedback loop, moving up, then set to state 1
        state = 1;
        substate = 'B';
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
  pinMode(BS1,OUTPUT);
  pinMode(BS2,OUTPUT);
  pinMode(BS3,OUTPUT);
  pinMode(BS4,OUTPUT);
  
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

int constantSpeed(void){ //adjust voltage for constant speed; return new voltage
  /*
  int voltAdjust; //value to be returned
  double Kc = 0.1;
  int speedSignal;
  int speedDiff;

  speedDiff = xDot - 50; //find speed difference
  speedSignal = Kc*speedDiff; //find proportion for speed change
  voltAdjust = voltage + speedSignal; //calculate new voltage

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
  */
}



void moveDown (void){ //feedback loop down
/*
 int initialVolt = 10;
  int counter = 0;
  int finished = 0;
  int volt;
  float dx;
  float kp = 3;

  Serial.println("Feedback loop, up.");

  voltage = initialVolt; //update global voltage

  while(finished != 1){ //loop until elevator stablized at destination
    readThetaAndX(&theta, &thetaDot, &x, &xDot, &previousMillis); //get current position and speed

    if(xDot > -1){
      voltage = voltage - 1;
       Serial.println("running the rundown 1");
    }
    if(x > 1){
      voltage = voltage-1;
       Serial.println("running the rundown2");
    }
    if(voltage <1){
      voltage = 1;
       Serial.println("running the rundown3");
    }
    if(x < 1){
     finished = 1;
     Serial.println("running the rundown 4 done");
     delay(5000);
    }
  
      //motorUp(); //set direction up
      motorSpeed(voltage);
    }
    */
}



void moveUp (void){ //feedback loop up
/*
  int initialVolt = 80;
  int counter = 0;
  int finished = 0;
  int volt;
  float dx;
  float kp = 3;

  Serial.println("Feedback loop, up.");

  voltage = initialVolt; //update global voltage

  while(finished != 1){ //loop until elevator stablized at destination
    readThetaAndX(&theta, &thetaDot, &x, &xDot, &previousMillis); //get current position and speed

    if(x > (69)){ //x at top floor within margin of error
        voltage = voltage/3.0;
        finished = 1;
        Serial.print("moveup function");
    }
    else if(x < 69){ //x below floor
      if(xDot < 1){
        voltage = voltage + 1;
        if(voltage > 255){
          voltage = 255;
        }
      }
      
      motorUp(); //set direction up
      motorSpeed(voltage);
    }
  }
  */
}


void idle (void){ //idle
/*
  int initialVolt = 30;
  int counter = 0;
  int finished = 0;
  int volt;
  float dx;
  float kp = 10;

  Serial.println("Idling.");

  voltage = initialVolt; //update global voltage

  readThetaAndX(&theta, &thetaDot, &x, &xDot, &previousMillis); //get current position and speed

    if(x > (69)){ //x at top floor within margin of error
        volt = voltage/3;
        motorSpeed(volt);
        Serial.println("1");
    }
    else if(x <= 69){ //x below floor
      Serial.println("2");
      motorUp(); //set direction up
      //if(xDot < 70){
        voltage = initialVolt + kp*(topX - x);
        if(xDot < 1){
          initialVolt++;
          Serial.println("3");
        }
        if(voltage > 255){
          voltage = 255;
          Serial.println("4");
        }
        motorSpeed(voltage);
  }
  */
}
