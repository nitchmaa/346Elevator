int in1 = 10; //Declaring the pins where in1 in2 from the driver are wired 
int in2 = 12; //here they are wired with D9 and D8 from Arduino
int ConA = 11; //And we add the pin to control the speed after we remove its jumper 
               //Make sure it's connected to a pin that can deliver a PWM signal
 
void setup() {
  pinMode(in1, OUTPUT); //Declaring the pin modes, obviously they're outputs
  pinMode(in2, OUTPUT);
  pinMode(ConA, OUTPUT);
}

void loop() {
  TurnMotorA();  //Sequence: turning on low speed, stop, turning again in high speed and stop
  delay(2000);
 
  TurnOFFA();
  delay(2000);
}

 
void TurnMotorA(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(ConA,250);
}
 
void TurnOFFA(){
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(ConA,0);
}
