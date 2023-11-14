#define COUNT_REV 115 //Rodery encoder output pulse per rotation (Need to change depemnding on encoder)
#define ENCODER_IN 3 //Encoder output to Aurdino Interrupt pin
#define ENCODER_IN2 2

 volatile long encoderValue = 0; //Pulse count from encoder 
 int interval = 100; //1 second interval measuments 
 long previousMillis = 0;
 long currentMillis = 0;
 float rpm = 0;
 float theta =0;
 float thetaDot = 0;
 float x = 0;
 float xdot = 0;
 int encoderOverallValue;
 int b;
 
 


void setup() {
  
  Serial.begin(9600);

  pinMode (ENCODER_IN, INPUT_PULLUP);
  pinMode (ENCODER_IN2, INPUT_PULLUP);

  
  attachInterrupt(digitalPinToInterrupt(ENCODER_IN), updateEncoder, RISING); // Interupt to find X and X dot
  previousMillis = millis();

}

void loop() {

  
  currentMillis = millis();
  
  
  if (currentMillis - previousMillis > interval){
    int timefortheta = currentMillis - previousMillis;
    previousMillis = currentMillis;
  

   rpm = (float)(encoderValue * 60.0 / COUNT_REV);
   //theta = (float)(theta + ((rpm / 60.0) * 2.0 * 3.14159)* (timefortheta/1000)); // Radians 
   theta = ((float)(encoderOverallValue / COUNT_REV) * 2 * 3.14159);
   thetaDot = (float)((rpm /60.0) * 2.0 * 3.14159); //Radians Per Second
   x = (float)((theta * 1.358)); //However big the radius of the pully is goes here, x starts at 70 
   xdot = (float)(thetaDot * 1.358); //Radius of the pully goes in the second term 
  
  

  if(rpm > 0){
  
    Serial.print("Pulses: ");
    Serial.print(encoderValue);
    Serial.print('\t');
    Serial.print("Speed: ");
    Serial.print(rpm);
    Serial.println(" RPM");
    Serial.print('\t');
    Serial.print("Theta: ");
    Serial.print(theta);
    Serial.println(" Rad");
    Serial.print('\t');
    Serial.print("Theta Dot: ");
    Serial.print(thetaDot);
    Serial.println(" Rad/sec");
    Serial.print('\t');
    Serial.print("X ");
    Serial.print(x);
    Serial.println(" cm");
    Serial.print('\t');
    Serial.print("XDot ");
    Serial.print(xdot);
    Serial.println(" cm/s");
  }
  
  encoderValue = 0;

  }
  
}

void updateEncoder(){
  b = digitalRead(ENCODER_IN2);
  if (b == 0){
      encoderValue ++;
      encoderOverallValue ++;
  }else {
      encoderValue --;
      encoderOverallValue --;
  }
  
}
