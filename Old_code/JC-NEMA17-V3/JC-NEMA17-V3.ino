#include <Stepper.h>
#include <math.h>

// Define stepper motor connections
//STEPPER 1: HORIZONTAL MOTOR
#define motor_horizontal_Pin1 8
#define motor_horizontal_Pin2 9
#define motor_horizontal_Pin3 10
#define motor_horizontal_Pin4 11

//STEPPER 2: VERTICAL MOTOR
#define motor_vertical_Pin1 4
#define motor_vertical_Pin2 5
#define motor_vertical_Pin3 6
#define motor_vertical_Pin4 7

#define STEPS_PER_REVOLUTION 200  // Constants for steps per revolution and speed of stepper motors
#define SPEED 30

// Initialize stepper motors
Stepper MOTOR_H(STEPS_PER_REVOLUTION, motor_horizontal_Pin1, motor_horizontal_Pin3, motor_horizontal_Pin2, motor_horizontal_Pin4);
Stepper MOTOR_V(STEPS_PER_REVOLUTION, motor_vertical_Pin1, motor_vertical_Pin3, motor_vertical_Pin2, motor_vertical_Pin4);

#define vertical_limit_pin 2    // Limit switch for Vertical Motor
#define horizontal_limit_pin 3  // Limit switch for Horizontal Motor

#define TWO_PI PI * 2   // Math Definition 2Pi
#define HALF_PI PI / 2  // Math Definition Pi/2
#define RAD_EARTH 6371000 // in meters 

struct Point {  // Define 3-Dimensional Point Variable and necessary operators
  float x;
  float y;
  float z;

  // #define lat x
  // #define lng y
  // #define alt z

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

  Point operator*(const float& other) const {
    return { x * other, y * other, z * other };
  }
  Point operator/(const float& other) const {
    return { x / other, y / other, z / other };
  }

  float dot(const Point& other) const {  // Dot product of two points
    return x * other.x + y * other.y + z * other.z;
  }
  Point cross(const Point& other) const {  // Cross product of two points
    return { y * other.z - z * other.y,
             z * other.x - x * other.z,
             x * other.y - y * other.x };
  }
  Point unit() const {
    float magnitude = sqrt(x * x + y * y + z * z);
    return { x / magnitude, y / magnitude, z / magnitude };
  }

  String toString() const {

    return String("(") + String(x, 6) + ", " + String(y, 6) + ", " + String(z, 6) + ")";
  }
};

Point testPoints[] = {
  { 44.569206, 123.274863, 142 },  // Point 1
  { 44.34066, 123.16346, 300 },    // Point 2
  { 100, 100, 30 },                // Point 3
  { 100, 100, 40 },                // Point 4
};

// int numberOfPoints = sizeof(testPoints) / sizeof(testPoints[0]);

float Compass_Heading = 0.0;
Point Antenna_GPS = { 0.00000000, 0.000000000, 0.00000000 };  // Original Antenna GPS Location
Point Rocket_GPS = { 100, 100, 0.0 };                         // Rocket GPS Loaction
Point Antenna_Direction = { 0, 1, 0 };                        // Original Antenna Orientation Vector

void setup() {
  Serial.begin(9600);
  delay(500);

  Serial.println("\n\n-------------------------------------------------------------------------------------------------\nINITIALIZING");

  pinMode(vertical_limit_pin, INPUT);
  pinMode(horizontal_limit_pin, INPUT);

  Compass_Heading = initialize_compass(Compass_Heading);

  Antenna_GPS = initialize_GPS(Antenna_GPS);  // Get current GPS Coordinated of Antenna

  Antenna_Direction = initialize_motors(Antenna_Direction);  // Zero Motors and get current orientation of motors
}


int i = 0;
void loop() {
  Serial.print("\n\nTEST POINT ");
  Serial.println(i + 1);
  Rocket_GPS = Get_Rocket_GPS(testPoints[i]);

  Antenna_Direction = Aim_Antenna(Antenna_GPS, Rocket_GPS, Antenna_Direction);

  if (i < 2) {
    i++;
    delay(2000);
  } else {
    while (1)
      ;
  }
}