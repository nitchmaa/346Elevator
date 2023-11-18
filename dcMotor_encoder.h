#ifndef DCMOTOR_ENCODER_H
#define DCMOTOR_ENCODER_H

#include <Arduino.h>

void motorDown (void); //set direction down
void motorUp (void); //set direction up
void stopMotor (void); //stops elevator
void motorSpeed(int ein); //set motor speed

void readThetaAndX(float *thetaPTR, float *thetaDotPTR, float *xPTR, float *xDotPTR, long *previousMilllisPTR); //read theta and thetaDot
void updateEncoder(void); //Increment Value for Each pulse from Encoder 


#endif
