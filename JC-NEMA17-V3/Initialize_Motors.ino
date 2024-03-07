#include <Stepper.h>

Point initialize_motors(Point antenna_direction) {
  Serial.println("\nINITIALIZING MOTORS");

  MOTOR_H.setSpeed(SPEED);  // Set stepper motor speeds
  MOTOR_V.setSpeed(SPEED);

  // while (digitalRead(horizontal_limit_pin) == LOW)                                  // Find Zero Location of Horizontal Motor
  //   MOTOR_H.step(-1);
  // MOTOR_H.step(STEPS_PER_REVOLUTION);                                               // Reset Horizontal Motor to Center

  // while (digitalRead(vertical_limit_pin) == LOW)                                    // Find Zero Location of Vertical Motor
  //   MOTOR_V.step(-1);
  // MOTOR_V.step(STEPS_PER_REVOLUTION / 8);                                           // Reset to 45° above Horizontal

  float compass_heading = get_compass_heading();  // Example compass heading (east)
  float lift_angle = rad(45.0);                   // 45 degrees converted to radians

  antenna_direction = Compass_Lift_to_Unit_Vector(compass_heading, lift_angle);  // Define Orientation of antenna and Return as Unit Vector

  Serial.print("Antenna Direction: ");
  Serial.println(antenna_direction.toString());
  return antenna_direction;
}

Point Compass_Lift_to_Unit_Vector(float compass_heading, float lift_angle) {
  float azimuth_angle = compass_heading * PI / 180.0;  // Convert compass heading to azimuth angle in radians

  float x = sin(azimuth_angle) * cos(lift_angle);  // Convert azimuth angle and lift angle to Cartesian coordinates
  float y = cos(azimuth_angle) * cos(lift_angle);
  float z = sin(lift_angle);

  return { x, y, z };  // Return the unit vector
}
