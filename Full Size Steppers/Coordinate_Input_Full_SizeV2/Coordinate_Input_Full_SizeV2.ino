
#include <AccelStepper.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <cmath>

#define dirPin1 6
#define stepPin1 7
#define dirPin2 8
#define stepPin2 9
#define interface 1

#define DEG_TO_RAD 0.017453292519943295769236907684886
#define EARTH_RADIUS 6371000  // in meters

#define STEPS_PER_REVOLUTION 400
#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559
#define HALF_PI 1.5707963267948966192313216916398

const float BACKUP_LAT = 35.353333; // Approx Far CA Latitude
const float BACKUP_LON = -117.807167; // Approx Far CA Longitude
const float BACKUP_ALT = 30.0; // Approximate Far CAaltitude in meters

// Constants for calculating deltas
const float MAX_RADIUS = 60;  // Radius in km
const float LATITUDE_PER_KM = 1 / 111.0;  // Degrees per kilometer

// Assuming a latitude for calculation, e.g., 45 degrees North
const float LATITUDE = 45.0;

// Calculate max deltas
const float MAX_DELTA_LAT = MAX_RADIUS * LATITUDE_PER_KM;
const float MAX_DELTA_LON = MAX_RADIUS * LATITUDE_PER_KM / cos(LATITUDE * PI / 180.0);
const float MAX_DELTA_ALT = 80000;  // meters

static const int RXPin = 12;  // Define GPS Module connections
static const int TXPin = 13;
static const uint32_t GPSBaud = 9600;  // Define GPS baud rate

int verbose = 1;

float ref_lat;
float ref_lon;
float ref_alt;

AccelStepper stepper2 = AccelStepper(interface, stepPin1, dirPin1);
AccelStepper stepper1 = AccelStepper(interface, stepPin2, dirPin2);

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
  if (verbose >0){
    Serial.println("\nINITIALIZING GPS");
  }

  Point antenna_gps = {0.0,0.0,0.0};
  ss.begin(9600);
  if (verbose >0){
    Serial.print("Connecting to GPS satellites, please wait.");
  }
  int i = 0;
  int GPS_time_out = 60;
  while (!ss.available() && i++ < GPS_time_out) {  // Check if able to connect to GPS
    delay(200);
    if (verbose > 0){
      Serial.print(".");
    }
  }
  if (i >= GPS_time_out) {
    antenna_gps = {BACKUP_LAT, BACKUP_LON, BACKUP_ALT};
    if (verbose >0){
    Serial.println("Unable to Connect to GPS Satellites.");
    Serial.print("Using GPS coordingates: ");
    Serial.println(antenna_gps.toString());
    }
    return antenna_gps;
  }
  i=0;
  while (i++ < GPS_time_out) {
    if (verbose >2){
      Serial.println("Reading GPS Satellites ");
    }
    while (ss.available() > 0) {
      if (gps.encode(ss.read())) {  // Read data from GPS
        if (verbose >0){
          Serial.print("Successfully Connected to ");
          Serial.print(gps.satellites.value());
          Serial.println(" Satellites.");
        }
        if (gps.location.isValid() && gps.satellites.value()>5){
          antenna_gps = { gps.location.lat(), gps.location.lng(), gps.altitude.meters() };  // Store Antenna GPS location and return
          if (verbose >0){
            Serial.print("Antenna GPS: ");
            Serial.println(antenna_gps.toString());
          }
          return antenna_gps;
        }
        delay(20);
      }
    }
      delay(1000);
  }
  antenna_gps = {BACKUP_LAT, BACKUP_LON, BACKUP_ALT};
  if (verbose >0){
    Serial.println("Unable to Connect to GPS Satellites.");
    Serial.print("Using GPS coordingates: ");
    Serial.println(antenna_gps.toString());
  }
  return antenna_gps;
}

//Create function that converts regular coordinate to East, North, Up
void llaToEnu(float lat, float lon, float alt, float& east, float& north, float& up) {
  float delta_lat = lat - ref_lat;
  float delta_lon = lon - ref_lon;

  float cos_ref_lat = cos(ref_lat * DEG_TO_RAD);

  east = delta_lon * DEG_TO_RAD * EARTH_RADIUS * cos_ref_lat;
  north = delta_lat * DEG_TO_RAD * EARTH_RADIUS;
  up = alt - ref_alt;
  if (verbose >2){
    Serial.print("Delta Latitude: ");
    Serial.println(delta_lat);
    Serial.print("Delta Longitude: ");
    Serial.println(delta_lon);
    Serial.print("East: ");
    Serial.println(east);
    Serial.print("North: ");
    Serial.println(north);
    Serial.print("Up: ");
    Serial.println(up);
  }
};

bool parseIncomingData(String data, float& lat, float& lon, float& alt) {
  int firstCommaIndex = data.indexOf(',');
  if (firstCommaIndex == -1) {
    if (verbose >0){
      Serial.println("Error: No commas found in data: " + data);
    }
    return false; 
  }
  int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
  if (secondCommaIndex  == -1) {
    if (verbose > 0){
      Serial.println("Error: No second comma found in data: " + data);
    }
    return false; 
  }
  float temp_lat = data.substring(0, firstCommaIndex).toFloat();
  float temp_lon = data.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
  float temp_alt = data.substring(secondCommaIndex + 1).toFloat();
  
  if (fabs(temp_lat - REF_LAT) > MAX_DELTA_LAT) {
    if (verbose > 0){
      Serial.print("Error: Latitude change too large. Delta: ");
      Serial.println(fabs(temp_lat - REF_LAT));
    }
    return false;
  }
  if (fabs(temp_lon - REF_LON) > MAX_DELTA_LON) {
    if (verbose > 0){
      Serial.print("Error: Longitude change too large. Delta: ");
      Serial.println(fabs(temp_lon - REF_LON));
    }
    return false;
  }
  if (fabs(temp_alt - REF_ALT) > MAX_DELTA_ALT) {
    if (verbose > 0){
      Serial.print("Error: Altitude change too large. Delta: ");
      Serial.println(fabs(temp_alt - REF_ALT));
    }
    return false;
  }
  lat = temp_lat;
  lon = temp_lon;
  alt = temp_alt;
  return true;
}
String receiveData() {
  String incomingData = "";
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      // When a newline is received, return the data
      return incomingData;
    } else {
      // Append receivedChar to incomingData
      incomingData += receivedChar;
    }
  }
  // If no newline character is received yet, return an empty string
  return "";
}


float east, north, up;

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
//_____SET OLD PHI INITIALLY WITH STARTING ANGLE_____
//(NOW PHI DEFINED AS DIFFERENCE FROM INITIAL 45 POSITION)
float old_phi = 0;

//Variabe to track last position (previous loop's theta absolute)
float old_theta = 0;

//declare theta_abs (Global theta position based off of the set origin vector <1 0 0>
float theta_abs;
float fractionalStepResidue1 = 0.0;
float fractionalStepResidue2 = 0.0;

// Function to calculate magnitude of a vector
float mag(float vector[]) {
  return sqrt(pow(vector[0], 2) + pow(vector[1], 2));
}

void setup() {
  Serial.begin(9600);
  if (verbose >0){
    Serial.println("INITIALIZING");
  }
  // Set stepper motor speeds
  stepper1.setSpeed(10);
  stepper1.setMaxSpeed(10);  //20000 steps/second max
  stepper1.setAcceleration(5);
  stepper2.setSpeed(10);
  stepper2.setMaxSpeed(10);  //20000 steps/second max
  stepper2.setAcceleration(5);

  Point antenna_gps = initialize_GPS();

  ref_lat = antenna_gps.x;
  ref_lon = antenna_gps.y;
  ref_alt = antenna_gps.z;

  //Handshake
  
  //PHI_MIN CHANGE 3: Set at 45 initially
  if (verbose >0){
    Serial.println("Start steps calculated and moving stepper motor...");
  }
  float start_steps = offset / 360 * STEPS_PER_REVOLUTION;
  stepper2.move(start_steps);
  stepper2.runToPosition();  // Reset to 45
}

void point_motors(float lat, float lon, float alt) {
    // Convert geographic coordinates to local ENU coordinates
    llaToEnu(lat, lon, alt, east, north, up);

    if (verbose > 0) {
        Serial.print("East: "); Serial.print(east);
        Serial.print(" North: "); Serial.print(north);
        Serial.print(" Up: "); Serial.println(up);
    }

    // Compute directional vectors
    float dxy_magnitude = sqrt(east * east + north * north);
    float z_min = dxy_magnitude * tan(phi_i_offset);

    // Check if target is above the minimum elevation angle
    if (up > z_min) {
        above45yet = true;
        if (verbose > 2) {
            Serial.println("Above 45");
        }
    }
    update_orientation(east, north, up, dxy_magnitude);
}

void update_orientation(float dx, float dy, float dz, float dxy_magnitude) {
    // Calculate theta for horizontal direction
    float theta_diff = calculate_theta(dx, dy);

    // Calculate phi for vertical direction
    float phi_diff = calculate_phi(dz, dxy_magnitude);

    if (verbose > 1) {
        Serial.print("Theta: "); Serial.println(degrees(theta));
        Serial.print("Phi: "); Serial.println(degrees(phi));
    }
    // Send movement commands to motors
    command_motors(theta_diff, phi_diff);
}

float calculate_theta(float dx, float dy) {
    float theta = atan2(dy, dx);  // Calculate angle in radians
    float reference_theta = atan2(yop, xop);  // Reference direction of the antenna
    float theta_diff = theta - reference_theta;  // Difference between target and reference

    return theta_diff;
}

float calculate_phi(float dz, float dxy_magnitude) {

    float ref_xy_magnitude = sqrt(xop * xop + yop * yop);
    float ref_phi = atan2(zop, ref_xy_magnitude);
    float phi = atan2(dz, dxy_magnitude);
    float phi_diff = phi - ref_phi;

    return phi_diff;
}

void command_motors(float theta_diff, float phi_diff) {
    float totalSteps1 = theta_diff / TWO_PI * STEPS_PER_REVOLUTION;
    float totalSteps2 = phi_diff / TWO_PI * STEPS_PER_REVOLUTION;

    int MOTOR1_steps = int(totalSteps1 + fractionalStepResidue1 );
    int MOTOR2_steps = int(totalSteps2  + fractionalStepResidue2);
    
    // Update fractional residues
    fractionalStepResidue1 = (totalSteps1 + fractionalStepResidue1) - MOTOR1_steps;
    fractionalStepResidue2 = (totalSteps2 + fractionalStepResidue2) - MOTOR2_steps;

    if (verbose > 1) {
        Serial.print("Motor 1 (Horizontal) Steps: "); Serial.println(MOTOR1_steps);
        Serial.print("Motor 2 (Vertical) Steps: "); Serial.println(MOTOR2_steps);
    }

    stepper1.move(MOTOR1_steps);
    stepper1.runToPosition();
    stepper2.move(MOTOR2_steps);
    if (above45yet){
      stepper2.runToPosition();
    }
    xop = dx;
    yop = dy;
    zop = dz;
}

float mag(float components[]) {
    return sqrt(components[0] * components[0] + components[1] * components[1]);
}


void loop() {
      Serial.print('\x05');
      String incomingData = receiveData();
      float lat, lon, alt;
      parseIncomingData(incomingData, lat, lon, alt);
      if (verbose >2){
        Serial.print("incoming data from py ");
        Serial.print(incomingData);
        Serial.print("Lat: ");
        Serial.print(lat);
        Serial.print(" long: ");
        Serial.print(lon);
        Serial.print(" alt: ");
        Serial.println(alt);
      }
      point_motors( lat, lon, alt);
}
