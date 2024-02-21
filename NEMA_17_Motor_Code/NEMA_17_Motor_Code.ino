#include <Stepper.h>


// Define stepper motor connections

//STEPPER 1: HORIZONTAL MOTOR
#define motor1_Pin1 8   // IN1 on NEMA 17 ==> Blue   on MOTOR 1
#define motor1_Pin2 9   // IN2 on NEMA 17 ==> Pink   on MOTOR 1
#define motor1_Pin3 10  // IN3 on NEMA 17 ==> Yellow on MOTOR 1
#define motor1_Pin4 11  // IN4 on NEMA 17 ==> Orange on MOTOR 1

//STEPPER 2: VERTICAL MOTOR
#define motor2_Pin1 4  // IN1 on NEMA 17 ==> Blue   on MOTOR 2
#define motor2_Pin2 5  // IN2 on NEMA 17 ==> Pink   on MOTOR 2
#define motor2_Pin3 6  // IN3 on NEMA 17 ==> Yellow on MOTOR 2
#define motor2_Pin4 7  // IN4 on NEMA 17 ==> Orange on MOTOR 2

#define vertical_limit_pin 2  // Limit switch for Motor 2

// Constants for steps per revolution and speed of stepper motors
#define STEPS_PER_REVOLUTION 200
#define SPEED 30
#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559
#define HALF_PI 1.5707963267948966192313216916398


struct Point {
  float x;
  float y;
  float z;

  String toString() const {
    return String("(") + String(x) + ", " + String(y) + ", " + String(z) + ")";
  }
};

Point testPoints[] = {
  { -1, -1, 10 },  // Point 1
  { 1, 1, 10 },    // Point 2
  { 1, 1, 20 },    // Point 3
  { 1, -1, 15 },   // Point 3
};
int numberOfPoints = sizeof(testPoints) / sizeof(testPoints[0]);

// Original Antenna Location
float x0 = 0;
float y0 = 0;
float z0 = 0;



// Original Antenna Orientation Vector
float xop = 1;
float yop = 0;
float zop = 0;

//Create boolean to change later when motors should stop
bool done_moving = false;

//Variabe to track last position (previous loop's theta absolute)
float old_theta = 0;
float old_phi = HALF_PI / 2;


//declare theta_abs (Global theta position based off of the set origin vector <1 0 0>
float theta_abs;


// Function to calculate magnitude of a vector
float mag(float vector[]) {
  return sqrt(pow(vector[0], 2) + pow(vector[1], 2));
}

// Initialize stepper motors

Stepper MOTOR1(STEPS_PER_REVOLUTION, motor1_Pin1, motor1_Pin3, motor1_Pin2, motor1_Pin4);
Stepper MOTOR2(STEPS_PER_REVOLUTION, motor2_Pin1, motor2_Pin3, motor2_Pin2, motor2_Pin4);

void setup() {
  Serial.begin(9600);

  Serial.println("INITIALIZING");

  // Set stepper motor speeds
  MOTOR1.setSpeed(SPEED);
  MOTOR2.setSpeed(SPEED);

  pinMode(vertical_limit_pin, INPUT);

//   MOTOR1.step(-200);
//   MOTOR1.step(400);
//   MOTOR1.step(-200);

//   while (digitalRead(vertical_limit_pin) == LOW) {  // Find bottom limit
//     MOTOR2.step(-1);
//   }
//   MOTOR2.step(25);  // Reset to 45°
}


int i = 0;
void loop() {

  while (i < numberOfPoints) {

    Serial.print("TEST POINT #: ");
    Serial.println(i + 1);

    Serial.print("Point: ");
    Serial.println(testPoints[i].toString());

    // Rocket Coordinates
    float xr = testPoints[i].x;
    float yr = testPoints[i].y;
    float zr = testPoints[i].z;

    // Components of Vector That Points at Rocket
    float dx = xr - x0;
    float dy = yr - y0;
    float dz = zr - z0;

    //------ PHI CALC ------
    // xz component of original antenna direction
    float xz_op[] = { xop, zop };
    // xz component of required pointing vector
    float dxz[] = { dx, dz };


    // Solve for phi
    float phi = acos((xz_op[0] * dxz[0] + xz_op[1] * dxz[1]) / (mag(xz_op) * mag(dxz)));
    float vertical_flip_offset = 0;

    Serial.print("Old Phi: ");
    Serial.println(phi);

    //***90 degree max for vertical motor here:
    if (phi > HALF_PI) {
      phi = PI - phi;
      Serial.print("Phi (V2): ");
      Serial.println(phi);
      vertical_flip_offset = PI;
    }



    //------ THETA CALC ------
    // xy component of original antenna direction
    float xy_op[] = { xop, yop };
    // xy component of required pointing vector
    float dxy[] = { dx, dy };



    // Solve for angle between them
    float theta = acos((xy_op[0] * dxy[0] + xy_op[1] * dxy[1]) / (mag(xy_op) * mag(dxy)));

    //Inverse cos can only give angle between 0 and 180,have to account for the negative y half of xy plane using dy indicator and adjusting:
    if (dy < 0) {
      theta_abs = TWO_PI - theta;
    } else {
      theta_abs = theta;
    }

    //OPTIMIZATION FOR THETA (Finds fastest route):
    //Theta_dif will be the actual movement angle for the horizontal motor
    float theta_dif = theta_abs - old_theta;
    float phi_dif = phi - old_phi;

    if ((abs(theta_dif)) > PI) {
      if (theta_dif < 0) {
        theta_dif = theta_dif + TWO_PI;

      } else {
        theta_dif = theta_dif - TWO_PI;
      }
    }

    //Update old position for next loop as calculated theta from this loop
    old_theta = theta_abs;
    old_phi = phi;

    Serial.print("Theta ");
    Serial.println(degrees(theta_abs));

    Serial.print("Phi ");
    Serial.println(degrees(phi));

    //Convert angle to motor step instruction
    int MOTOR1_steps = (theta_dif / TWO_PI * STEPS_PER_REVOLUTION);
    Serial.print("Motor 1 (Horizontal) Steps: ");
    Serial.println(MOTOR1_steps);


    int MOTOR2_steps = (phi_dif / TWO_PI * STEPS_PER_REVOLUTION);
    Serial.print("Motor 2 (Vertivcal) Steps: ");
    Serial.println(MOTOR2_steps);


    // Move stepper motors based on calculated angles
    MOTOR1.step(MOTOR1_steps);
    MOTOR2.step(MOTOR2_steps);



    delay(2000);  // Adjust delay as needed
    i++;
  }
}