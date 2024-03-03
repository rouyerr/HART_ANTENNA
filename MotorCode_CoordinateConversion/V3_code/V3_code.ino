#include <Stepper.h>

//Example for now, but this will be GPS location of antenna
float ref_lat = 44.56780; 
float ref_lon = 123.27506; 
float ref_alt = 0;

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define EARTH_RADIUS 6371000 // in meters 

//Will be read-in rocket coordinates, just test for now
float lat_test = 44.56527; // Example latitude for testing
float lon_test = 123.27597; // Example longitude for testing
float alt_test = 1000; // Example altitude for testing

void llaToEnu(float lat, float lon, float alt, float& east, float& north, float& up) {
    float delta_lat = lat - ref_lat;
    float delta_lon = lon - ref_lon;

    float cos_ref_lat = cos(ref_lat * DEG_TO_RAD);

    east = -delta_lon * DEG_TO_RAD * EARTH_RADIUS * cos_ref_lat;
    north = delta_lat * DEG_TO_RAD * EARTH_RADIUS;
    up = alt - ref_alt;

};

float east, north, up;


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

// Original Antenna Orientation Vector
float xop = 0;
float yop = 1;
float zop = 0;

//_____SET OLD PHI INITIALLY WITH STARTING ANGLE_____
float old_phi = 0;

//Variabe to track last position (previous loop's theta absolute)
float old_theta = 0;

//declare theta_abs (Global theta position based off of the set origin vector <1 0 0>
float theta_abs;

// Function to calculate magnitude of a vector
float mag(float vector[]) {
  return sqrt(pow(vector[0], 2) + pow(vector[1], 2));
}

//-------One rotation variable---------
boolean onerot = true;

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

void loop() {
while (onerot) {
 llaToEnu(lat_test, lon_test, alt_test, east, north, up);
 Serial.print("East" );
 Serial.print(east);
 Serial.print("North");
 Serial.print(north);
 Serial.print("up");
 Serial.print(up);
 
    // Rocket Coordinates
    float yr = north;
    float xr = east;
    float zr = up;
    
    // Components of Vector That Points at Rocket
    float dx = xr;
    float dy = yr;
    float dz = zr;

    //------ THETA CALC ------
    // xy component of original antenna direction
    float xy_op[] = { xop, yop };
    // xy component of required pointing vector
    float dxy[] = { dx, dy };
    // Solve for angle between them
    float theta = acos((xy_op[0] * dxy[0] + xy_op[1] * dxy[1]) / (mag(xy_op) * mag(dxy)));
    
    //Inverse cos can only give angle between 0 and 180,have to account for the positive x half of xy plane using dx indicator and adjusting:
    //ADJUST FOLLOWING CONDITION FOR MOTOR SIGN CONVENTION -- CW+: dx<0   CCW: dx>0
    if (dx < 0) {
      theta_abs = TWO_PI - theta;
      Serial.println("dx>0");
    } else {
      theta_abs = theta;
      Serial.println("dx>0");
    }

    //OPTIMIZATION FOR THETA (Finds fastest route):
    //Theta_dif will be the actual movement angle for the horizontal motor
    float theta_dif = theta_abs - old_theta;
   
    if ((abs(theta_dif)) > PI) {
      if (theta_dif < 0) {
        theta_dif = theta_dif + TWO_PI;
      } else {
        theta_dif = theta_dif - TWO_PI;
      }
    }

    
    //------ PHI CALC ------
    // Solve for phi using arctan with mag(dxy) as horizontal triangle leg and dz as vertical leg
    float phi = atan((dz)/(mag(dxy)));
    Serial.print("Phi (V1): ");
    Serial.println(degrees(phi));
    
    //***90 degree max for vertical motor here:
    if (phi > HALF_PI) {
      phi = PI - phi;
      Serial.print("Phi (V2): ");
      Serial.println(degrees(phi));
    }
    
    //Calculate how much phi should change from previous phi
     float phi_dif = phi - old_phi;
    
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
    onerot = false;
}
  }
