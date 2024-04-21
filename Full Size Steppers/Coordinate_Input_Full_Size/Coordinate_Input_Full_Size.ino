
#include <AccelStepper.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#define dirPin1 6
#define stepPin1 7
#define dirPin2 8
#define stepPin2 9
#define interface 1

AccelStepper stepper2 = AccelStepper(interface, stepPin1, dirPin1);
AccelStepper stepper1 = AccelStepper(interface, stepPin2, dirPin2);

//GPS initialization
static const int RXPin = 12;  // Define GPS Module connections
static const int TXPin = 13;
static const uint32_t GPSBaud = 9600;  // Define GPS baud rate

TinyGPSPlus gps;  // Define GPS Module object

SoftwareSerial ss(RXPin, TXPin);  // Define connection to GPS Module

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

Point initialize_GPS() {
  Serial.println("\nINITIALIZING GPS");

  Point antenna_gps;

  ss.begin(9600);

  Serial.print("Connecting to GPS satellites, please wait.");
  int i = 0;
  while (!ss.available() && i++ <= 30) {  // Check if able to connect to GPS
    delay(100);
    Serial.print(".");
  }
  Serial.print(".");
  Serial.println(i);
  if (i >= 25) {
    Serial.println("Unable to Connect to GPS Satellites.");
  } else {
    Serial.println("Reading GPS Satellites 0. "+String(antenna_gps.x));
    while (antenna_gps.x == 0.0) {
      Serial.println("Reading GPS Satellites 1. "+String(antenna_gps.x));
      while (ss.available() > 0) {
        if (gps.encode(ss.read())) {  // Read data from GPS
          Serial.print("Successfully Connected to ");
          Serial.print(gps.satellites.value());
          Serial.println(" Satellites.");
          if (gps.satellites.value()>4){
            antenna_gps = { gps.location.lat(), gps.location.lng(), gps.altitude.meters() };  // Store Antenna GPS location and return
          }
        }
      }
    }
  }

  Serial.print("Antenna GPS: ");
  Serial.println(antenna_gps.toString());
  return antenna_gps;
}

//Take values from antenna_gps to get reference location
float ref_lat;
float ref_lon;
float ref_alt;

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define EARTH_RADIUS 6371000  // in meters

//Create function that converts regular coordinate to East, North, Up
void llaToEnu(float lat, float lon, float alt, float& east, float& north, float& up) {
  float delta_lat = lat - ref_lat;
  float delta_lon = lon - ref_lon;

  float cos_ref_lat = cos(ref_lat * DEG_TO_RAD);

  east = -delta_lon * DEG_TO_RAD * EARTH_RADIUS * cos_ref_lat;
  north = delta_lat * DEG_TO_RAD * EARTH_RADIUS;
  up = alt - ref_alt;
};

void parseIncomingData(String data, float& lat, float& lon, float& alt) {
  int firstCommaIndex = data.indexOf(',');
  int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);

  lat = data.substring(0, firstCommaIndex).toFloat();
  lon = data.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
  alt = data.substring(secondCommaIndex + 1).toFloat();
}

float east, north, up;

// Constants for steps per revolution and speed of stepper motors
#define STEPS_PER_REVOLUTION 400
#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559
#define HALF_PI 1.5707963267948966192313216916398

//PHI_MIN CHANGE 1:
//Define desired starting phi offset (in degrees)
float offset = 45;
float phi_i_offset = offset * DEG_TO_RAD;
//Introduce boolean 'above45yet' to track
boolean above45yet = 0;

// Original Antenna Orientation Vector
float xop = 0;
float yop = 1;
float zop = 0;
static String incomingData = "";
//_____SET OLD PHI INITIALLY WITH STARTING ANGLE_____
//(NOW PHI DEFINED AS DIFFERENCE FROM INITIAL 45 POSITION)
float old_phi = 0;

//Variabe to track last position (previous loop's theta absolute)
float old_theta = 0;

//declare theta_abs (Global theta position based off of the set origin vector <1 0 0>
float theta_abs;

// Function to calculate magnitude of a vector
float mag(float vector[]) {
  return sqrt(pow(vector[0], 2) + pow(vector[1], 2));
}
int verbose = 0;
void setup() {
  Serial.begin(9600);
  Serial.println("INITIALIZING");
  // Set stepper motor speeds
  stepper1.setSpeed(500);
  stepper1.setMaxSpeed(2000);  //20000 steps/second max
  stepper1.setAcceleration(50000);
  stepper2.setSpeed(500);
  stepper2.setMaxSpeed(2000);  //20000 steps/second max
  stepper2.setAcceleration(50000);

  Point antenna_gps = initialize_GPS();

  float ref_lat = antenna_gps.x;
  float ref_lon = antenna_gps.y;
  float ref_alt = antenna_gps.z;

  //PHI_MIN CHANGE 3: Set at 45 initially
  Serial.println("Start steps calculated and moving stepper motor...");
  float start_steps = offset / 360 * STEPS_PER_REVOLUTION;
  stepper2.move(start_steps);
  stepper2.runToPosition();  // Reset to 45�
}

void loop() {
  

  if (Serial.available() > 0) {
    
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      // When a newline is received, parse the data
      float lat, lon, alt;
      parseIncomingData(incomingData, lat, lon, alt);

      // Now you have new lat, lon, and alt values
      // Use them for your llaToEnu and subsequent calculations
      llaToEnu(lat, lon, alt, east, north, up);
      if (verbose >0){
      Serial.print("East ");
      Serial.print(east);
      Serial.print("North ");
      Serial.print(north);
      Serial.print("up ");
      Serial.println(up);
      }
      // Components of Vector That Points at Rocket coordinate
      float dx = east;
      float dy = north;
      float dz = up;

      //------ THETA CALC ------

      // xy component of original antenna direction
      float xy_op[] = { xop, yop };
      // xy component of required pointing vector
      float dxy[] = { dx, dy };

      //PHI_MIN CHANGE 3: Set minimum z coordinate associated with PHI_MIN
      float z_min = mag(dxy) * tan(phi_i_offset);
      //Print current rocket angle for testing
      float rocket_angle = (atan((dz) / (mag(dxy)))) / (DEG_TO_RAD);
      if (verbose >0){
      Serial.print("Rocket Angle");
      Serial.println(rocket_angle);
      }
      //PHI_MIN CHANGE 4: If condition
      //Only continue once dz > zmin (Rocket above 45 deg angle)
      if (dz > z_min) {
        above45yet = 1;
      }
      //Put everything left in 'if' statement>>>
      if (1){//above45yet == 1) {
        if (verbose >0){
        Serial.println("Above 45");
        //Now start actual rocket tracking code:
        }

        // Solve for angle between them
        float theta = acos((xy_op[0] * dxy[0] + xy_op[1] * dxy[1]) / (mag(xy_op) * mag(dxy)));

        //Inverse cos can only give angle between 0 and 180,have to account for the positive x half of xy plane using dx indicator and adjusting:
        //ADJUST FOLLOWING CONDITION FOR MOTOR SIGN CONVENTION -- CW+: dx<0   CCW: dx>0
        if (dx < 0) {
          theta_abs = TWO_PI - theta;
          if (verbose >0){
          Serial.println("dx>0");
          }
        } else {
          theta_abs = theta;
          if (verbose >0){
          Serial.println("dx>0");
          }
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
        float phi = atan((dz) / (mag(dxy)));
        //Relative to 45 degree starting point
        phi = phi - phi_i_offset;
        if (verbose >0){
        Serial.print("Phi (V1): ");
        Serial.println(degrees(phi));
        }

        //***90 degree max for vertical motor here:
        if (phi > HALF_PI) {
          phi = PI - phi;
          if (verbose >0){
          Serial.print("Phi (V2): ");
          Serial.println(degrees(phi));
          }
        }

        //Calculate how much phi should change from previous phi
        float phi_dif = phi - old_phi;

        //Update old position to be used for next loop as calculated theta from this loop
        old_theta = theta_abs;
        old_phi = phi;
        if (verbose >0){
        Serial.print("Theta ");
        Serial.println(degrees(theta_abs));
        Serial.print("Phi ");
        Serial.println(degrees(phi));
        }
        //Convert angle to motor step instruction
        int MOTOR1_steps = (theta_dif / TWO_PI * STEPS_PER_REVOLUTION);
        if (verbose >0){
        Serial.print("Motor 1 (Horizontal) Steps: ");
        Serial.println(MOTOR1_steps);
        }
        int MOTOR2_steps = (phi_dif / TWO_PI * STEPS_PER_REVOLUTION);
        if (verbose >0){
        Serial.print("Motor 2 (Vertivcal) Steps: ");
        Serial.println(MOTOR2_steps);
        }

        // Move stepper motors based on calculated angles

        stepper1.move(MOTOR1_steps);
        stepper2.move(MOTOR2_steps);
        stepper1.runToPosition();
        stepper2.runToPosition();
        //delay(1000);  // Adjust delay as needed
      }

      // Reset incomingData for the next message
      incomingData = "";
    } else {
      // Append receivedChar to incomingData
      incomingData += receivedChar;
    }
  }
}
