Point Aim_Antenna(Point& antenna_gps, Point& rocket_gps, Point& antenna_direction) {
  Serial.println("\nAIMING ANTENNA");

  Point diff_vector = rocket_gps - antenna_gps;                                                                                             // Create variable to find the difference between the GPS coordinates.
  Serial.print("Difference Vector: ");
  Serial.println(diff_vector.toString());

  float vector_magnitude = sqrt((diff_vector.x * diff_vector.x) + (diff_vector.y * diff_vector.y) + (diff_vector.z * diff_vector.z));       // Calculate Magnitude of the difference vector
  Serial.print("Difference Vector Magnitude: ");
  Serial.println(vector_magnitude);

  Point unit_vector = diff_vector / vector_magnitude;                                                                                       // Calculate a unit vector for the path from the Antenna to the Rocket
  Serial.print("Target Unit Vector: ");
  Serial.println(unit_vector.toString());

  Point_Antenna(antenna_direction, unit_vector);

  return unit_vector;                                                                                                                       // Return the new direction of the antenna

}

void Point_Antenna(Point& antenna_direction, const Point& unit_vector) {
  Serial.println("\nPOINTING ANTENNA");

  // Calculate the elevation angle (lift angle) between the current antenna direction and the target unit vector
  float dot_product_elevation = antenna_direction.x * unit_vector.x + antenna_direction.y * unit_vector.y;
  float cos_theta_elevation = dot_product_elevation / (sqrt(antenna_direction.x * antenna_direction.x + antenna_direction.y * antenna_direction.y) * sqrt(unit_vector.x * unit_vector.x + unit_vector.y * unit_vector.y));
  float theta_elevation = acos(cos_theta_elevation);

  // Convert elevation angle to motor steps for Motor_V (vertical motor)
  int MOTOR_V_steps = (theta_elevation / PI * STEPS_PER_REVOLUTION);
  Serial.print("Vertical Motor= Steps: ");
  Serial.println(MOTOR_V_steps);

  // Calculate the azimuth angle between the current antenna direction and the target unit vector
  float cross_product_azimuth = antenna_direction.x * unit_vector.y - antenna_direction.y * unit_vector.x;
  float sin_theta_azimuth = cross_product_azimuth / (sqrt(antenna_direction.x * antenna_direction.x + antenna_direction.y * antenna_direction.y) * sqrt(unit_vector.x * unit_vector.x + unit_vector.y * unit_vector.y));
  float cos_theta_azimuth = dot_product_elevation / (sqrt(antenna_direction.x * antenna_direction.x + antenna_direction.y * antenna_direction.y) * sqrt(unit_vector.x * unit_vector.x + unit_vector.y * unit_vector.y));
  float theta_azimuth = acos(cos_theta_azimuth);

  // Determine the sign of the azimuth angle based on the sign of the cross product
  if (cross_product_azimuth < 0) {
    theta_azimuth = -theta_azimuth;
  }

  // Convert azimuth angle to motor steps for Motor_H (horizontal motor)
  int MOTOR_H_steps = (theta_azimuth / TWO_PI * STEPS_PER_REVOLUTION);
  Serial.print("Horizontal Motor Steps: ");
  Serial.println(MOTOR_H_steps);

  // Move stepper motors to point the antenna in the new direction
  MOTOR_V.step(MOTOR_V_steps);
  MOTOR_H.step(MOTOR_H_steps);

}