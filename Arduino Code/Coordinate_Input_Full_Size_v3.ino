
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

#define STEPS_PER_REVOLUTION 400
#define PI 3.1415926535897932384626433832795
#define TWO_PI 6.283185307179586476925286766559
#define HALF_PI 1.5707963267948966192313216916398

// const float BACKUP_LAT = 35.353333; // Approx Far CA Latitude
// const float BACKUP_LON = -117.807167; // Approx Far CA Longitude
// const float BACKUP_ALT = 30.0; // Approximate Far CAaltitude in meters
//const float BACKUP_LAT = 44.57232; // Approx Remy Latitude
//const float BACKUP_LON = -123.18552;  // Approx Remy Longitude
//const float BACKUP_ALT = 65; // Approximate Remy in meters

//const float BACKUP_LAT = 44.56737; // Approx Covell Latitude
//const float BACKUP_LON = -123.27482;  // Approx Covell Longitude

const float BACKUP_LAT = 44.5596601; // Approx Covell Latitude
const float BACKUP_LON = -123.2787409;  // Approx Covell Longitude
const float BACKUP_ALT = 80; // Approximate covell in meters

const int MAX_VERT_ROT_NEG = (int) (-STEPS_PER_REVOLUTION * .25);
const int MAX_VERT_ROT_POS = STEPS_PER_REVOLUTION * 0;


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

int verbose = 0;
int HOR_DIR=1;
int VERT_DIR=1;
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
  
  if (fabs(temp_lat - ref_lat) > MAX_DELTA_LAT) {
    if (verbose > 0){
      Serial.print("Error: Latitude change too large. Delta: ");
      Serial.println(fabs(temp_lat - ref_lat));
    }
    return false;
  }
  if (fabs(temp_lon - ref_lon) > MAX_DELTA_LON) {
    if (verbose > 0){
      Serial.print("Error: Longitude change too large. Delta: ");
      Serial.println(fabs(temp_lon - ref_lon));
    }
    return false;
  }
  if (fabs(temp_alt - ref_alt) > MAX_DELTA_ALT) {
    if (verbose > 0){
      Serial.print("Error: Altitude change too large. Delta: ");
      Serial.println(fabs(temp_alt - ref_alt));
    }
    return false;
  }
  lat = temp_lat;
  lon = temp_lon;
  alt = temp_alt;
  return true;
}
String receiveData() {
  unsigned long startTime = millis(); // Start time for timeout
  const unsigned long timeout = 12000; // Timeout in milliseconds
  String incomingData = "";

  while (millis() - startTime < timeout) {
    while (Serial.available() > 0) {
      char receivedChar = Serial.read();
      if (receivedChar == '\n') {
        // When a newline is received, return the data
        return incomingData;
      } else {
        // Check if incomingData is about to overflow
        if (incomingData.length() < 255) {
          // Append receivedChar to incomingData
          incomingData += receivedChar;
        } else {
          // Handle buffer overflow
          Serial.println("Error: Incoming data exceeds buffer size");
          return "Buffer Overflow";
        }
      }
    }
    // Add a small delay to avoid busy-waiting
    delay(5);
  }
  // If no newline character is received within timeout, return an empty string
  if (verbose > 0) {
    Serial.println("Input timeout");
  }
  return "Timeout";
}


float east, north, up;

//PHI_MIN CHANGE 1:
//Define desired starting phi offset (in degrees)
float offset = 45;
float phi_i_offset = offset * DEG_TO_RAD;
//Introduce boolean 'above45yet' to track
boolean above45yet = 0;
bool INIT_GPS = false;
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

  /*String init_awk = receiveData() ;
  while(init_awk!="Start"){
    Serial.println("Bad start");
    Serial.println(init_awk);
    init_awk = receiveData();
  }*/
  if (verbose >0){
    Serial.println("INITIALIZING");
  }
  // Set stepper motor speeds
  stepper1.setSpeed(100);
  stepper1.setMaxSpeed(300);  //20000 steps/second max
  stepper1.setAcceleration(100);
  stepper2.setSpeed(100);
  stepper2.setMaxSpeed(400);  //20000 steps/second max
  stepper2.setAcceleration(100);
  stepper2.move(90);
       stepper2.runToPosition();
       stepper2.setCurrentPosition(0);

  if (INIT_GPS){
    Point antenna_gps = initialize_GPS();

    ref_lat = antenna_gps.x;
    ref_lon = antenna_gps.y;
    ref_alt = antenna_gps.z;
  }else{
    ref_lat = BACKUP_LAT;
    ref_lon = BACKUP_LON;
    ref_alt = BACKUP_ALT;
    if (verbose >0){
      Serial.println("Set to use backup coords.");
      Serial.print("Using GPS coordingates: ");
      Serial.print(ref_lat); Serial.print(", ");
      Serial.print(ref_lon); Serial.print(", ");
      Serial.print(ref_alt); Serial.println("m");
    }
  }



  //Handshake
  
  //PHI_MIN CHANGE 3: Set at 45 initially
  if (verbose >0){
    //Serial.println("Start steps calculated and moving stepper motor...");
  }
  //float start_steps = offset / 360 * STEPS_PER_REVOLUTION;
  //stepper2.move(start_steps);
  //stepper2.runToPosition();  // Reset to 45
}

void point_motors(float lat, float lon, float alt) {
    llaToEnu(lat, lon, alt, east, north, up);
    if (verbose >0){
      Serial.print("East " );
      Serial.print(east);
      Serial.print("  North ");
      Serial.print(north);
      Serial.print("  up ");
      Serial.println(up);
    }
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

    //PHI_MIN CHANGE 3: Set minimum z coordinate associated with PHI_MIN
    //float z_min = mag(dxy)*tan(phi_i_offset);
    //Print current rocket angle for testing
    //float rocket_angle = (atan((dz)/(mag(dxy))))/(DEG_TO_RAD);
    //Serial.print("Rocket Angle ");
    //Serial.println(rocket_angle);

    //PHI_MIN CHANGE 4: If conditions
    //Only continue once dz > zmin (Rocket above 45 deg angle)
    //if (dz > z_min) {
      //above45yet = 1;
   // }
    //Put everything left in 'if' statement>>>
  //  if (above45yet == 1) {
   // Serial.println("Above 45");
    //Now start actual rocket tracking code:

    
    // Solve for angle between them
    float theta = acos((xy_op[0] * dxy[0] + xy_op[1] * dxy[1]) / (mag(xy_op) * mag(dxy)));
    
    //Inverse cos can only give angle between 0 and 180,have to account for the positive x half of xy plane using dx indicator and adjusting:
    //ADJUST FOLLOWING CONDITION FOR MOTOR SIGN CONVENTION -- CW+: dx<0   CCW: dx>0
    if (dx < 0) {
      theta_abs = TWO_PI - theta;
      if (verbose >1){
        Serial.println("dx<0");
      }
    } else {
      theta_abs = theta;
      if (verbose >1){
        Serial.println("dx>0");
      }
    }
    if (verbose >1){
      Serial.print("theta_abs: ");
      Serial.println(degrees(theta_abs));
    }
    //OPTIMIZATION FOR THETA (Finds fastest route):
    //Theta_dif will be the actual movement angle for the horizontal motor
    float theta_dif = theta_abs - old_theta;
    if (verbose >0){
      Serial.print("theta_dif: ");
      Serial.println(degrees(theta_dif));
    }
    if ((abs(theta_dif)) > PI) {
      if (verbose >2){
        Serial.println("Optimizing Theta...");
      }
      if (theta_dif < 0) {
        theta_dif = theta_dif + TWO_PI;
      } else {
        theta_dif = theta_dif - TWO_PI;
      }
      if (verbose >2){
        Serial.print("Optimized theta_dif: ");
        Serial.println(degrees(theta_dif));
      }

    }

    
    //------ PHI CALC ------
    // Solve for phi using arctan with mag(dxy) as horizontal triangle leg and dz as vertical leg
    // Motor oritentaion: Add negative in front of phi calc if raising antenna is CCW for vertical motor
    float phi = atan((dz)/(mag(dxy)));

    //Relative to 45 degree starting point
    //If phi is negative>>
    //phi = phi + phi_i_offset
    //If phi is positive>>
    //phi = phi - phi_I_offset
    if (verbose >0){
    Serial.print("Phi_abs: ");
    Serial.println(degrees(phi));
    }
    
    //***90 degree max for vertical motor here:
    if (abs(phi) > HALF_PI) {
      phi = PI - abs(phi);
      if (verbose >1){
        Serial.print("Phi (V2): ");
        Serial.println(degrees(phi));
      }
    }
    
    //Calculate how much phi should change from previous phi
     float phi_dif = old_phi - phi ;
    
    //Update old position to be used for next loop as calculated theta from this loop
    old_theta = theta_abs;
    old_phi = phi;
    if (verbose >1){
      Serial.print("Phi_dif: ");
      Serial.println(degrees(phi_dif));
    }
    
    //Convert angle to motor step instruction
    int MOTOR1_steps = (theta_dif / TWO_PI * STEPS_PER_REVOLUTION);
    if (verbose >0){
      Serial.print("Motor 1 (Horizontal) Steps: ");
      Serial.println(MOTOR1_steps);
    }
    int MOTOR2_steps = (phi_dif / TWO_PI * STEPS_PER_REVOLUTION);
    if (verbose >0){
      Serial.print("Motor 2 (Vertical) Steps: ");
      Serial.println(MOTOR2_steps);
    }
    
    // Move stepper motors based on calculated angles
       stepper1.move(MOTOR1_steps);
       stepper1.runToPosition();
       stepper2.move(MOTOR2_steps);
        if (stepper2.targetPosition() >= MAX_VERT_ROT_NEG && stepper2.targetPosition() <= MAX_VERT_ROT_POS){
            stepper2.runToPosition();
        }else{
          if (verbose >0){
            Serial.print("Protection for vert steps cur pos: ");
            Serial.print(stepper2.currentPosition());
            Serial.print(" to pos: ");
            Serial.println(stepper2.targetPosition());
          }
        }
       
}

void loop() {
      Serial.print('\x05');
      String incomingData = receiveData();
      float lat, lon, alt;
      parseIncomingData(incomingData, lat, lon, alt);
      if (verbose >2){
        Serial.print("incoming data from py \"");
        Serial.print(incomingData);
        Serial.println("\"");
        Serial.print("Parsed data- Lat: ");
        Serial.print(lat);
        Serial.print(" , long: ");
        Serial.print(lon);
        Serial.print(" , alt: ");
        Serial.println(alt);
      }
      point_motors( lat, lon, alt);
}
