#include <Stepper.h>

//STEPPER 1: HORIZONTAL MOTOR
//STEPPER 2: VERTICAL MOTOR

// Define stepper motor connections
#define motor1_Pin1 8   // IN1 on ULN2003_1 ==> Blue   on MOTOR 1
#define motor1_Pin2 9   // IN2 on ULN2004_1 ==> Pink   on MOTOR 1
#define motor1_Pin3 10  // IN3 on ULN2003_1 ==> Yellow on MOTOR 1
#define motor1_Pin4 11  // IN4 on ULN2003_1 ==> Orange on MOTOR 1

#define motor2_Pin1 4  // IN1 on ULN2003_2 ==> Blue   on MOTOR 2
#define motor2_Pin2 5  // IN2 on ULN2004_2 ==> Pink   on MOTOR 2
#define motor2_Pin3 6  // IN3 on ULN2003_2 ==> Yellow on MOTOR 2
#define motor2_Pin4 7  // IN4 on ULN2003_2 ==> Orange on MOTOR 2

// Constants for steps per revolution and speed of stepper motors
#define STEPS_PER_REVOLUTION 2048
#define SPEED 100

// Original Antenna Location
float x0 = 0;
float y0 = 0;
float z0 = 0;

// Rocket Coordinates
float xr = 1;
float yr = 1;
float zr = 1;

// Original Antenna Orientation Vector
float xop = 1;
float yop = 0;
float zop = 0;

//Create boolean to change later when motors should stop
bool done_moving = false;

//Variabe to track last position (previous loop's theta absolute)
float old_pos = 0;

//Inverse cos can only give angle between 0 and 180,have to account for the negative y half of xy plane using dy indicator and adjusting:
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

  // Set stepper motor speeds
  MOTOR1.setSpeed(SPEED);
  MOTOR2.setSpeed(SPEED);
}

void loop() {

  while (!done_moving) {
    // Components of Vector That Points at Rocket
    float dx = xr - x0;
    float dy = yr - y0;
    float dz = zr - z0;

    //------ THETA CALC ------
    // xy component of original antenna direction
    float xy_op[] = { xop, yop };
    // xy component of required pointing vector
    float dxy[] = { dx, dy };



    // Solve for angle between them
    float theta = acos((xy_op[0] * dxy[0] + xy_op[1] * dxy[1]) / (mag(xy_op) * mag(dxy)));

    if (dy < 1) {
      theta_abs = 360 - theta;
    } else {
      theta_abs = theta;
    }

    //OPTIMIZATION FOR THETA (Finds fastest route):
    //Theta_dif will be the actual movement angle for tthe horizontal motor
    float theta_dif = theta_abs - old_pos;

    if ((abs(theta_dif)) > 180) {
      if (theta_dif < 0) {
        theta_dif = theta_dif + 360;

      } else {
        theta_dif = theta_dif - 360;
      }
    }

    //Update old position for next loop as calculated theta from this loop
    old_pos = theta_abs;

    //------ PHI CALC ------
    // xz component of original antenna direction
    float xz_op[] = { xop, zop };
    // xz component of required pointing vector
    float dxz[] = { dx, dz };


    // Solve for phi
    float phi = acos((xz_op[0] * dxz[0] + xz_op[1] * dxz[1]) / (mag(xz_op) * mag(dxz)));

    // Convert angles to degrees
    theta_dif = degrees(theta_dif);
    phi = degrees(phi);

    Serial.print("Theta ");
    Serial.print(theta_dif);

    //***90 degree max for vertical motor here:
    if (phi > 90) {
      phi = 180 - phi;
      Serial.print("Phi (V2): ");
      Serial.print(phi);
    }

    //Convert angle to motor step instruction
    int MOTOR1_steps = (theta_dif / 360 * STEPS_PER_REVOLUTION);
    Serial.print("Motor 1 Steps");
    Serial.print(MOTOR1_steps);


    int MOTOR2_steps = (phi / 360 * STEPS_PER_REVOLUTION);
    Serial.print("MOTOR 2 steps");
    Serial.print(MOTOR2_steps);


    // Move stepper motors based on calculated angles
    MOTOR1.step(MOTOR1_steps);
    MOTOR2.step(MOTOR2_steps);

    delay(1000);  // Adjust delay as needed

    //Run loop once to make things simple for now
    done_moving = true;
  }
}
