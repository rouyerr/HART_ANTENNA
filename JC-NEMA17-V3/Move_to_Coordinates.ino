Point Move_to_Coordinates(Point& antenna_gps, Point& rocket_gps, Point& antenna_direction) {
  Point diff_vector = rocket_gps - antenna_gps;                                                                                             // Create variable to find the difference between the GPS coordinates.
  float vector_magnitude = sqrt((diff_vector.x * diff_vector.x) + (diff_vector.y * diff_vector.y) + (diff_vector.z * diff_vector.z));       // Calculate Magnitude of the difference vector
  Point unit_vector = diff_vector / vector_magnitude;                                                                                       // Calculate a unit vector for the path from the Antenna to the Rocket

  



  return unit_vector;                                                                                                                       // Return the new direction of the antenna

}