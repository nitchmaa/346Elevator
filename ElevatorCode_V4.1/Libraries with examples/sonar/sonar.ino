#include <NewPing.h>
////////////////////////ALL SETUP FOR SONAR SENSORS//////////////////////////////////////////////////////////////////////
#define TRIGGER_PIN_BOT  52 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_BOT     53 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define TRIGGER_PIN_TOP  48 // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN_TOP     49 // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE      5 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
NewPing sonar[2] = {     // Sensor object array with the number of sensors
  NewPing(TRIGGER_PIN_BOT, ECHO_PIN_BOT, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping. 
  NewPing(TRIGGER_PIN_TOP, ECHO_PIN_TOP, MAX_DISTANCE) 
 };
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  Serial.begin(9600); // Open serial monitor at 9600 baud to see ping results.
}

void loop() { 
  if(sonar[0].ping_cm() > 0 ){Serial.print("Something on Bottom");}
  if(sonar[1].ping_cm() > 0 ){Serial.print("Something on Top");}
  Serial.println();
}