#ifndef DEFINEPINS_H
#define DEFINEPINS_H

#include <Arduino.h>

//SONAR
#define trigTop 48
#define echoTop 49
#define trigBot 52
#define echoBot 53

//STEPPER MOTOR
#define TS1  22 // IN1 on the ULN2003, top
#define TS2  23 // IN2 on the ULN2003, top
#define TS3  24 // IN3 on the ULN2003, top
#define TS4  25 // IN4 on the ULN2003, top
#define BS1  28 // IN1 on the ULN2003, bot
#define BS2  29 // IN2 on the ULN2003, bot
#define BS3  30 // IN3 on the ULN2003, bot
#define BS4  31 // IN4 on the ULN2003, bot

//DC MOTOR
#define MP1 10 //motor pin 1
#define MP2 12 //motor pin 2
#define MPEN 11 //motor pin enable

//ROTARY ENCODER
#define COUNT_REV 115 //Rodery encoder output pulse per rotation (Need to change depemnding on encoder)
#define ENCODER_IN 3 //Encoder output to Aurdino Interrupt pin
#define ENCODER_IN2 2

#endif
