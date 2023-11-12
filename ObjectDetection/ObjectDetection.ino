
#include <NewPing.h> //Sonar library


int objectDetect (void); //returns 0 if no object, 1 if top, 2 if bottom

//DEFINE SONAR VARIABLES
#define trigTop 48
#define echoTop 49
#define trigBot 52
#define echoBot 53
#define MAX_DISTANCE 400 // Maximum distance we want to measure (in centimeters).
NewPing sonarTop(trigTop, echoTop, MAX_DISTANCE); // NewPing setup of pins and maximum distance.
NewPing sonarBot(trigBot, echoBot, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

void setup() {
  // put your setup code here, to run once:
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


int objectDetect(void){ //detect object at platforms; returns 0 if no object, 1 if top, 2 if bottom
  int objectPosition = 0; //stores position of object; value to be returned

  int distanceTop = sonarTop.ping_cm(); //Send ping for top, get distance in cm and print result (0 = outside set distance range)
  //Serial.print("Top distance: ");
  //Serial.print(distanceTop);
  //Serial.println("cm");
  
  int distanceBot = sonarBot.ping_cm(); //Send ping for bot, get distance in cm and print result (0 = outside set distance range)
  //Serial.print("Bottom distance: ");
  //Serial.print(distanceBot);
  //Serial.println("cm");
  
  if(distanceTop <= 5) objectPosition = 1; //top sensor triggered
  else if(distanceBot <= 5) objectPosition = 2; //bottom sensor triggered
  else objectPosition = 0; //neither sensor triggered, no object present
  
  return objectPosition;
}