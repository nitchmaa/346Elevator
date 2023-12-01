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
#include <PID_v1.h>
#include <Encoder.h>


//FUNCTION PROTOTYPES
void setUp(void); //configures pins; setup function
void speedControl(double desiredSpeed); //PID control for speed
void positionControl(double desiredPos); //PID control for close distances
void moveDown (void); //feedback loop down
void moveUp (void); //feedback loop up
void readEncoder(void);

//DEFINE GENERAL USE VARIABLES
int state = 1;
char substate = 'A';

int objectPlat = 0; //stores response from function for whether object has been detected

//POSITION VALUES FOR CONTROL
double theta =0;
double thetaDot = 0;
double x = 0;
double xDot = 0;
long previousMillis = 0;

int maxVolt = 190;
int minVolt = 50;


//PID
double kp, ki, kd;
volatile long voltage = 100;
double totalError = 0, error = 0, eSignal = 0;

Encoder myEnc(2,3);
long pos = -999; //stores encoder position value

long currTime = millis();
long prevTime = currTime;
double deltaTimeSec = 0;

double deltaRad = 0;
double prevRad = 0;

double deltaError = 0;
double prevError = 0;
double errorDot = 0;



void setup() {
  Serial.begin(9600); // Opens serial port and sets the data rate to 9600 bits-per-second
  
  setUp(); //function to set pins
  
  myEnc.write(0); //reset encoder position
  currTime = millis();
  prevTime = currTime;
  deltaTime = currTime - prevTime;
  
  //topActLoad(0.1); //pre-loop run of actuators
  //botActLoad(0.1);
  
  //moveUp(); //move car to top for start of loop
}


void loop() {
    switch (state) {
      case 1:
        if(substate == 'A'){
          Serial.println("State 1A");

          //positionControl(70); //hold at top
          
          objectPlat = objectDetect(); //is an object detected? if no (return of 0), do nothing
          if(objectPlat == 1){ //if on top, load top actuator, set to state 3-B
            topActLoad(0.3125);
            Serial.println("Top actuating");
            state = 3;
            substate = 'B';
          }
          else if(objectPlat == 2){ //if on bot, set to state 3
            state = 3;
          }
        }
        else if(substate == 'B'){
          Serial.println("State 1B");
          //positionControl(70); //hold at top. Path B has finished.        
        }
        else if(substate == 'C'){
          Serial.println("State 1C");

          //positionControl(70); //hold at top
          
          delay(1000); //pause 1 sec, then set to state 3
          state = 3;
        }
        break;
      
      case 2:
        if(substate == 'A'){
          Serial.println("State 2A");

          //positionControl(0); //hold at bottom
          
          objectPlat = objectDetect(); //is an object detected? if no, do nothing
          if(objectPlat == 2){ //if on bottom, load bottom actuator, set to state 4-C (object should never be on top floor here)
            botActLoad(0.3125);
            state = 4;
            substate = 'C';
          }
        }
        else if(substate == 'B'){
          Serial.println("State 2B");

          //positionControl(0); //hold at bottom
          
          delay(1000); //pause 1 sec, then set to state 4
          state = 4;
        }
        else if(substate == 'C'){
          //positionControl(0); //hold at bottom. Path C has finished. 
        }
        break;
      
      case 3:
        moveDown(); //enter feedback loop, moving down, then set to state 2
        state = 2;
        //substate = 'A';
        break;
      
      case 4:
        moveUp(); //enter feedback loop, moving up, then set to state 1
        state = 1;
        //substate = 'B';
        break;
      
      default:
        Serial.println("Error in switch case. Stopping motor.");
        stopMotor();
        break;
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
  //pinMode (ENCODER_IN, INPUT_PULLUP);
  //pinMode (ENCODER_IN2, INPUT_PULLUP);
  //attachInterrupt(digitalPinToInterrupt(ENCODER_IN), updateEncoder, RISING); // Interupt to find X and X dot
  previousMillis = millis();
}

void speedControl(double desiredSpeed){ //PID control for speed
  kp = 2.5;
  ki = 0.25;
  kd = 2;
  
  readEncoder();

  error = desiredSpeed - xDot; //find Kp error
        //Serial.print("Error: ");
        //Serial.print(error);
        //Serial.println(" cm/s");

  deltaError = error - prevError;
  prevError = error;
  totalError = totalError + error*deltaTime; //find Ki error
        //Serial.print("Total Error: ");
        //Serial.println(totalError);

  errorDot = deltaError / deltaTime;
        //Serial.print("ErrorDot: ");
        //Serial.println(errorDot);
        
  eSignal = kp*error + ki*totalError + kd*errorDot;
  
        Serial.print("eSignal: ");
        Serial.println(eSignal);

  voltage = voltage + eSignal;
  if(voltage > maxVolt) voltage = maxVolt;
  else if (voltage < minVolt) voltage = minVolt;
        //Serial.print("Voltage: ");
        //Serial.print(voltage);
        //Serial.println(" V");
        
  //Serial.println("------------------------------------");
}


void positionControl(double desiredPos){ //PID control for close distances
  kp = 2.5;
  ki = 0.25;
  kd = 2;
  
  readEncoder();

  error = desiredPos - x;
        //Serial.print("Error: ");
        //Serial.print(error);
        //Serial.println(" cm/s");

  deltaError = error - prevError;
  prevError = error;
  totalError = totalError + error*deltaError;
        //Serial.print("Total Error: ");
        //Serial.println(totalError);

  errorDot = deltaError / deltaTime;
        //Serial.print("ErrorDot: ");
        //Serial.println(errorDot);
        
  eSignal = kp*error + ki*totalError + kd*errorDot;
  
        Serial.print("eSignal: ");
        Serial.println(eSignal);

  voltage = voltage + eSignal;
  if(voltage > maxVolt) voltage = maxVolt;
  else if (voltage < minVolt) voltage = minVolt;
        //Serial.print("Voltage: ");
        //Serial.print(voltage);
        //Serial.println(" V");
        
  //Serial.println("------------------------------------");
}


void moveDown (void){ //feedback loop down
//copy moveUp here when finished
}



void moveUp (void){ //feedback loop up
  int finished = 0;

  Serial.println("Feedback loop, up.");

  motorUp();
  
  while(finished != 1){ //loop in function until elevator stablized at destination
    if(x > 69.5){ //x at top floor within margin of error, exit moveUp function
      finished = 1;
    }
    else if(x > 63){ //position control above this point
      positionControl(70); //position control, want 70 cm
      motorSpeed(voltage);
    }
    else{
      speedControl(50); //constant speed control, want 50 cm/s
      motorSpeed(voltage);
    }
  }
}


void readEncoder(void){
  long newPos;
  double rad, radDot;
  double degree, degreeDot;

  currTime = millis();
  deltaTime = (currTime - prevTime);
  deltaTimeSec = deltaTime/1000;

  if(deltaTime > 10){
    prevTime = currTime;
    
    newPos=myEnc.read();
    
    pos=newPos;

    //-440 encoder count for one rev
    rad = (double)pos/-440.0*(2.0 * 3.14159); //radians
      
    deltaRad = rad - prevRad;
    prevRad = rad;
      
    radDot = deltaRad/deltaTimeSec; //rad/s
    degree = rad*180/3.14159;
    degreeDot = radDot*180/3.14159;
    x=((double)(rad * 1.358)); //cm
    xDot=((double)(radDot * 1.358)); //cm
    /*
    Serial.print('\t');
    Serial.print("Theta, R: ");
    Serial.print(rad);
    Serial.println(" Rad");
      
    Serial.print('\t');
    Serial.print("Theta, D: ");
    Serial.print(degree);
    Serial.println(" degrees");
      
    Serial.print('\t');
    Serial.print("Theta Dot: ");
    Serial.print(degreeDot);
    Serial.println(" Deg/sec");
      
    Serial.print('\t');
    Serial.print("X ");
    Serial.print(x);
    Serial.println(" cm");
      
    Serial.print('\t');
    Serial.print("XDot ");
    Serial.print(xDot);
    Serial.println(" cm/s");
    */
  }
}
