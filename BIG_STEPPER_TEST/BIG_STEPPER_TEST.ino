#include <AccelStepper.h>
#define dirPin 6
#define stepPin 7
#define interface 1

AccelStepper stepper=AccelStepper(interface,stepPin,dirPin);

void setup()
{  
  stepper.setSpeed(500);
  stepper.setMaxSpeed(20000); //20000 steps/second max
  stepper.setAcceleration(50000); 
}

void loop()
{  
   stepper.moveTo(8000);
   stepper.runToPosition();
   delay (1000);

   stepper.moveTo(0);
   stepper.runToPosition();
   delay (1000);
}

 
// int reverseSwitch = 2;  // Push button for reverse
// int driverPUL = 7;    // PUL- pin
// int driverDIR = 6;    // DIR- pin
// int spd = A0;     // Potentiometer
 
// // Variables
 
// int pd = 500;       // Pulse Delay period
// boolean setdir = LOW; // Set Direction
 
// // Interrupt Handler
 
// void revmotor (){
 
//   setdir = !setdir;
  
// }
 
 
// void setup() {
 
//   pinMode (driverPUL, OUTPUT);
//   pinMode (driverDIR, OUTPUT);
//   attachInterrupt(digitalPinToInterrupt(reverseSwitch), revmotor, FALLING);
  
// }
 
// void loop() {
  
//     pd = map((analogRead(spd)),0,1023,2000,50);
//     digitalWrite(driverDIR,setdir);
//     digitalWrite(driverPUL,HIGH);
//     delayMicroseconds(pd);
//     digitalWrite(driverPUL,LOW);
//     delayMicroseconds(pd);
 
// }