#include <Encoder.h>

Encoder encoder(2, 3);

void setup() {
  Serial.begin(9600);
}

long position  = -999;

void loop() {
  long newposition; 
  newposition = encoder.read();
  if (newposition != position) {
    Serial.print("Position = ");
    Serial.print(newposition);
    Serial.println();
    position = newposition;
  }
  // if a character is sent from the serial monitor,
  // reset both back to zero.
  if (Serial.available()) {
    Serial.read();
    Serial.println("Reset zero");
    encoder.write(0);
  }
}