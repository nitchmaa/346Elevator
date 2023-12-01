#include "definePins.h"
#include "dcMotor_encoder.h"

 volatile long encoderValue = 0; //Pulse count from encoder 
 int interval = 100; //1 second interval measuments 
 //long previousMillis = 0;
 long currentMillis = 0;
 /*
 double theta =0;
 double thetaDot = 0;
 double x = 0;
 double xDot = 0;
 */
 int encoderOverallValue;
 int b;
 


void motorDown (void){ //set direction down
  digitalWrite(MP1, LOW);
  digitalWrite(MP2, HIGH);
  Serial.println("Direction: DOWN");
}

void motorUp (void){ //set direction up
  digitalWrite(MP1, HIGH);
  digitalWrite(MP2, LOW);
  Serial.println("Direction: UP");
}

void stopMotor (void){ //stops elevator
  digitalWrite(MP1, LOW);
  digitalWrite(MP2, LOW);
  Serial.println("Motor stopped.");
}

void motorSpeed(int ein){ //set motor speed; receives input voltage
  analogWrite(MPEN, ein);
  Serial.print("Changing input voltage to: ");
  Serial.println(ein);
}


void readThetaAndX(double *thetaPTR, double *thetaDotPTR, double *xPTR, double *xDotPTR, long *previousMilllisPTR){ //read theta and thetaDot, translate to x and xDot
  double rpm = 0;
  currentMillis = millis();
  
  if (currentMillis - *previousMilllisPTR > interval){
    int timefortheta = currentMillis - *previousMilllisPTR;
    *previousMilllisPTR = currentMillis;
  
   rpm = (double)(encoderValue * 600.0 / COUNT_REV);
   //theta = (double)(theta + ((rpm / 60.0) * 2.0 * 3.14159)* (timefortheta/1000)); // Radians 
   *thetaPTR = (((double)encoderOverallValue / COUNT_REV ) * 2 * 3.14159);
   *thetaDotPTR = (double)((rpm /60.0) * 2.0 * 3.14159); //Radians Per Second
   *xPTR = ((double)(*thetaPTR * 1.358)); //However big the radius of the pully is goes here, x starts at 70 
   *xDotPTR = ((double)*thetaDotPTR * 1.358); //Radius of the pully goes in the second term 

  //if(rpm != 0){
    /*
    Serial.print("Pulses: ");
    Serial.print(encoderValue);
    Serial.print('\t');
    Serial.print("Speed: ");
    Serial.print(rpm);
    Serial.println(" RPM");*/
    Serial.print('\t');
    Serial.print("Theta: ");
    Serial.print(*thetaPTR);
    Serial.println(" Rad");
    Serial.print('\t');
    Serial.print("Theta Dot: ");
    Serial.print(*thetaDotPTR);
    Serial.println(" Rad/sec");
    Serial.print('\t');
    Serial.print("X ");
    Serial.print(*xPTR);
    Serial.println(" cm");
    Serial.print('\t');
    Serial.print("XDot ");
    Serial.print(*xDotPTR);
    Serial.println(" cm/s");
  //}
  
  encoderValue = 0;
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
