//Sonar Libraries
#include <NewPing.h> //Sonar library
#include "ObjectDetection.h"
//DEFINE SONAR VARIABLES
#define trigTop 48
#define echoTop 49
#define trigBot 52
#define echoBot 53

void setup() {
  //SONAR pinout
  pinMode(trigTop,OUTPUT);
  pinMode(echoTop,INPUT);
  pinMode(trigBot,OUTPUT);
  pinMode(echoBot,INPUT);
  Serial.begin(9600);
}

void loop() {
delay(5000);
if(objectDetect()==0) {Serial.println("No object detected");
}
if(objectDetect()==1) {Serial.println("Object Detected on Top");
}
if(objectDetect()==2) {Serial.println("Object Detected on Bottom");
}
}
