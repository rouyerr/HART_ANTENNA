Point Aim_Antenna(Point& antenna_gps, Point& rocket_gps, Point& antenna_direction) {
  Serial.println("\nAIMING ANTENNA");

  Point diff_vector = rocket_gps - antenna_gps;                                                                                             // Create variable to find the difference between the GPS coordinates.
  Serial.print("Difference Vector: ");
  Serial.println(diff_vector.toString());

  Point target_direction = diff_vector.unit();                                                                                       // Calculate a unit vector for the path from the Antenna to the Rocket
  Serial.print("Target Direction Unit Vector: ");
  Serial.println(target_direction.toString());

  Serial.print("Antenna Direction Unit Vector: ");
  Serial.println(antenna_direction.toString());

  Point_Antenna(antenna_direction, target_direction);

  return target_direction;                                                                                                                       // Return the new direction of the antenna

}

void Point_Antenna(Point& from_direction, const Point& to_direction) {
  Serial.println("\nPOINTING ANTENNA");

  // Calculate the elevation angle (lift angle) between the current antenna direction and the target unit vector
  float dot_product_elevation = from_direction.x * to_direction.x + from_direction.y * to_direction.y;
  float cos_theta_elevation = dot_product_elevation / (sqrt(from_direction.x * from_direction.x + from_direction.y * from_direction.y) * sqrt(to_direction.x * to_direction.x + to_direction.y * to_direction.y));
  float theta_elevation = acos(cos_theta_elevation);

  // Convert elevation angle to motor steps for Motor_V (vertical motor)
  int MOTOR_V_steps = (theta_elevation / PI * STEPS_PER_REVOLUTION);
  Serial.print("Vertical Steps: ");
  Serial.println(MOTOR_V_steps);

  // Calculate the azimuth angle between the current antenna direction and the target unit vector
  float cross_product_azimuth = from_direction.x * to_direction.y - from_direction.y * to_direction.x;
  float sin_theta_azimuth = cross_product_azimuth / (sqrt(from_direction.x * from_direction.x + from_direction.y * from_direction.y) * sqrt(to_direction.x * to_direction.x + to_direction.y * to_direction.y));
  float cos_theta_azimuth = dot_product_elevation / (sqrt(from_direction.x * from_direction.x + from_direction.y * from_direction.y) * sqrt(to_direction.x * to_direction.x + to_direction.y * to_direction.y));
  float theta_azimuth = acos(cos_theta_azimuth);

  // Determine the sign of the azimuth angle based on the sign of the cross product
  if (cross_product_azimuth < 0) {
    theta_azimuth = -theta_azimuth;
  }

  // Convert azimuth angle to motor steps for Motor_H (horizontal motor)
  int MOTOR_H_steps = (theta_azimuth / TWO_PI * STEPS_PER_REVOLUTION);
  Serial.print("Horizontal Steps: ");
  Serial.println(MOTOR_H_steps);

  // Move stepper motors to point the antenna in the new direction
  MOTOR_V.step(MOTOR_V_steps);
  MOTOR_H.step(MOTOR_H_steps);

}