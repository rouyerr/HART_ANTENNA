#include <Stepper.h>
#include <math.h>

#define vertical_limit_pin 2                                            // Limit switch for Vertical Motor 
#define horizontal_limit_pin 3                                          // Limit switch for Horizontal Motor 

#define TWO_PI PI * 2                                                   // Math Definition 2Pi
#define HALF_PI PI / 2                                                  // Math Definition Pi/2

struct Point {                                                          // Define 3-Dimensional Point Variable and necessary operators
  float x;
  float y;
  float z;

  bool operator==(const Point& other) const {
    return this->x == other.x
           && this->y == other.y
           && this->z == other.z;
  }
  bool operator!=(const Point& other) const {
    return !(*this == other);
  }

  Point operator+(const Point& other) const {
    return { x + other.x, y + other.y, z + other.z };
  }
  Point operator-(const Point& other) const {
    return { x - other.x, y - other.y, z - other.z };
  }
  
  Point operator*(const float other) const {
    return { x * other, y * other, z * other };
  }
  Point operator/(const float other) const {
    return { x / other, y / other, z / other };
  }
};

// Point testPoints[] = {
//   { -1, -1, 10 },  // Point 1
//   { 1, 1, 10 },    // Point 2
//   { 1, 1, 20 },    // Point 3
//   { 1, -1, 15 },   // Point 3
// };

// int numberOfPoints = sizeof(testPoints) / sizeof(testPoints[0]);

Point Antenna_GPS = {0, 0, 0};                                        // Original Antenna GPS Location
Point Rocket_GPS = {0, 0, 0};                                         // Rocket GPS Loaction
Point Antenna_Direction = {0, 0, 0};                                  // Original Antenna Orientation Vector

void setup() {
  Serial.begin(9600);

  Serial.println("INITIALIZING");

  pinMode(vertical_limit_pin, INPUT);
  pinMode(horizontal_limit_pin, INPUT);

  Antenna_Direction = initialize_motors();                            // Zero Motors and get current orientation of motors

  Antenna_GPS = initialize_GPS();                                     // Get current GPS Coordinated of Antenna
  
}


int i = 0;
void loop() {

  Rocket_GPS;//=  RECIEVE GPS DATA FROM ROCKET

  Antenna_Direction = Move_to_Coordinates(Antenna_GPS, Rocket_GPS, Antenna_Direction);



  // while (i < numberOfPoints) {

  //   Serial.print("\nTEST POINT #: ");
  //   Serial.println(i + 1);

  //   Serial.print("Point: ");
  //   Serial.print(testPoints[i].x);
  //   Serial.print(", ");
  //   Serial.print(testPoints[i].y);
  //   Serial.print(", ");
  //   Serial.println(testPoints[i].z);

  //   // Rocket Coordinates
  //   float xr = testPoints[i].x;
  //   float yr = testPoints[i].y;
  //   float zr = testPoints[i].z;

  //   // Components of Vector That Points at Rocket
  //   float dx = xr - x0;
  //   float dy = yr - y0;
  //   float dz = zr - z0;

  //   //------ PHI CALC ------
  //   // xz component of original antenna direction
  //   float xz_op[] = { xop, zop };
  //   // xz component of required pointing vector
  //   float dxz[] = { dx, dz };

  //   // Solve for phi
  //   float phi = acos((xz_op[0] * dxz[0] + xz_op[1] * dxz[1]) / (mag(xz_op) * mag(dxz)));
  //   float vertical_flip_offset = 0;

  //   Serial.print("Old Phi: ");
  //   Serial.println(phi);

  //   //***90 degree max for vertical motor here:
  //   if (phi > HALF_PI) {
  //     phi = PI - phi;
  //     Serial.print("Phi (V2): ");
  //     Serial.println(phi);
  //     vertical_flip_offset = PI;
  //   }

  //   //------ THETA CALC ------
  //   // xy component of original antenna direction
  //   float xy_op[] = { xop, yop };
  //   // xy component of required pointing vector
  //   float dxy[] = { dx, dy };



  //   // Solve for angle between them
  //   float theta = acos((xy_op[0] * dxy[0] + xy_op[1] * dxy[1]) / (mag(xy_op) * mag(dxy)));

  //   //Inverse cos can only give angle between 0 and 180,have to account for the negative y half of xy plane using dy indicator and adjusting:
  //   if (dy < 0) {
  //     theta_abs = TWO_PI - theta;
  //   } else {
  //     theta_abs = theta;
  //   }

  //   //OPTIMIZATION FOR THETA (Finds fastest route):
  //   //Theta_dif will be the actual movement angle for the horizontal motor
  //   float theta_dif = theta_abs - old_theta;
  //   float phi_dif = phi - old_phi;

  //   if ((abs(theta_dif)) > PI) {
  //     if (theta_dif < 0) {
  //       theta_dif = theta_dif + TWO_PI;

  //     } else {
  //       theta_dif = theta_dif - TWO_PI;
  //     }
  //   }

  //   //Update old position for next loop as calculated theta from this loop
  //   old_theta = theta_abs;
  //   old_phi = phi;

  //   Serial.print("Theta ");
  //   Serial.println(degrees(theta_abs));

  //   Serial.print("Phi ");
  //   Serial.println(degrees(phi));

  //   //Convert angle to motor step instruction
  //   int MOTOR_H_steps = (theta_dif / TWO_PI * STEPS_PER_REVOLUTION);
  //   Serial.print("Motor 1 (Horizontal) Steps: ");
  //   Serial.println(MOTOR_H_steps);


  //   int MOTOR_V_steps = (phi_dif / TWO_PI * STEPS_PER_REVOLUTION);
  //   Serial.print("Motor 2 (Vertivcal) Steps: ");
  //   Serial.println(MOTOR_V_steps);


  //   // Move stepper motors based on calculated angles
  //   MOTOR_H.step(MOTOR_H_steps);
  //   MOTOR_V.step(MOTOR_V_steps);



  //   delay(5000);  // Adjust delay as needed
  //   i++;
  // }
}