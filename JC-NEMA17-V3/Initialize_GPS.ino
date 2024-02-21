#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 3;                                                                   // Define GPS Module connections
static const int TXPin = 4;
static const uint32_t GPSBaud = 9600;                                                         // Define GPS baud rate

TinyGPSPlus gps;                                                                              // Define GPS Module object

SoftwareSerial ss(RXPin, TXPin);                                                              // Define connection to GPS Module

Point initialize_GPS() {
  Serial.println("Initialize GPS");

  ss.begin(9600);

  Serial.print("Connecting to GPS satellites, please wait."); 
  while (!ss.available()) {                                                                   // Check if able to connect to GPS
    delay(100);
    Serial.print(".");
  }
  Serial.println(".");

  gps.encode(ss.read());                                                                      // Read data from GPS

  Point antenna_location = {gps.location.lat(), gps.location.lng(), gps.altitude.feet()};     // Store Antenna GPS location and return
  return antenna_location;
}