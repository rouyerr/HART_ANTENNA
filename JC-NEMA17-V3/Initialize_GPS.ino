#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 33;                                                                    // Define GPS Module connections
static const int TXPin = 32; 
static const uint32_t GPSBaud = 9600;                                                           // Define GPS baud rate

TinyGPSPlus gps;                                                                                // Define GPS Module object

SoftwareSerial ss(RXPin, TXPin);                                                                // Define connection to GPS Module

Point initialize_GPS(Point antenna_gps) {
  Serial.println("\nINITIALIZING GPS");

  ss.begin(9600);

  Serial.print("Connecting to GPS satellites, please wait."); 
  int i = 0;
  while (!ss.available() && i++ <= 30) {                                                        // Check if able to connect to GPS
    delay(100);
    Serial.print(".");
  }
  Serial.print(".");
  Serial.println(i);
  if (i >= 15) {
    Serial.println("Unable to Connect to GPS Satellites.");
  }
  else {
  gps.encode(ss.read());                                                                        // Read data from GPS
  
  Serial.print("Successfully Connected to ");
  Serial.print(gps.satellites.value());
  Serial.println(" Satellites.");

  antenna_gps = {gps.location.lat(), gps.location.lng(), gps.altitude.feet()};                  // Store Antenna GPS location and return
  }

  Serial.print("Antenna GPS: ");
  Serial.println(antenna_gps.toString());
  return antenna_gps;
}