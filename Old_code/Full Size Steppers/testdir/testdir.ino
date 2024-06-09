
#include <AccelStepper.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

#define dirPin1 6
#define stepPin1 7
#define dirPin2 8
#define stepPin2 9
#define interface 1

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define EARTH_RADIUS 6371000  // in meters


static const int RXPin = 12;  // Define GPS Module connections
static const int TXPin = 13;
static const uint32_t GPSBaud = 9600;  // Define GPS baud rate

AccelStepper stepper2 = AccelStepper(interface, stepPin1, dirPin1);
AccelStepper stepper1 = AccelStepper(interface, stepPin2, dirPin2);

void testMotorDirections(int steps, int speed) {
  // Test motor 1 (horizontal)
  Serial.println("Testing Motor 1 (Horizontal) Direction:");
  stepper1.setSpeed(speed); // Set a relatively fast speed for testing
  
  // Move motor 1 in one direction for a short distance
  stepper1.move(steps); // Adjust the number of steps as needed
  stepper1.runToPosition();
  delay(1000); // Wait for movement to complete
  
  // Move motor 1 in the opposite direction for the same distance
  stepper1.move(-steps); // Adjust the number of steps as needed
  stepper1.runToPosition();
  delay(1000); // Wait for movement to complete
  
  // Test motor 2 (vertical)
  Serial.println("Testing Motor 2 (Vertical) Direction:");
  stepper2.setSpeed(speed); // Set a relatively fast speed for testing
  
  // Move motor 2 in one direction for a short distance
  stepper2.move(steps); // Adjust the number of steps as needed
  stepper2.runToPosition();
  delay(1000); // Wait for movement to complete
  
  // Move motor 2 in the opposite direction for the same distance
  stepper2.move(-steps); // Adjust the number of steps as needed
  stepper2.runToPosition();
  delay(1000); // Wait for movement to complete
}


void setup() {
  Serial.begin(9600);
  Serial.println("HELLO");
  stepper1.setSpeed(10);
  stepper1.setMaxSpeed(10);  //20000 steps/second max
  stepper1.setAcceleration(5);
  stepper2.setSpeed(10);
  stepper2.setMaxSpeed(10);  //20000 steps/second max
  stepper2.setAcceleration(5);
}

void loop() {
      testMotorDirections(50,10);
      delay(1000);
}




