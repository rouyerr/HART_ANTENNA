Point Aim_Antenna(Point antenna_gps, Point rocket_gps, Point antenna_direction) {
  Serial.println("\nAIMING ANTENNA");

  Point diff_vector = rocket_gps - antenna_gps;  // Create variable to find the difference between the GPS coordinates.
  // Serial.print("Difference Vector: ");
  // Serial.println(diff_vector.toString());

  Point rocket_xyz = {
    rad(-diff_vector.x) * RAD_EARTH * cos(rad(antenna_gps.y)),
    rad(diff_vector.y) * RAD_EARTH,
    diff_vector.z
  };
  Serial.print("Rocket XYZ: ");
  Serial.println(rocket_xyz.toString());

  Point_Antenna(antenna_direction, rocket_xyz);

  return rocket_xyz;  // Return the new direction of the antenna
}

void Point_Antenna(Point antenna_direction, Point rocket_xyz){

}

float rad(float deg) {
  return (deg * PI / 180);
}