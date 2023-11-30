#include <NewPing.h> //Sonar library
#include "ObjectDetection.h"

void setup() {
  Serial.begin(9600);
}

void loop() {
delay(1000);
if(objectDetect()==0) {Serial.println("No object detected");
}
if(objectDetect()==1) {Serial.println("Object Detected on Top");
}
if(objectDetect()==2) {Serial.println("Object Detected on Bottom");
}
}
