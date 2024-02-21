float initialize_compass(float compass_heading) {
  Serial.println("\nINITIALIZING COMPASS");


  Serial.print("Compass Heading: ");
  Serial.println(compass_heading);
  return compass_heading;
}

float get_compass_heading() {

  float compass_heading = 190.0;

  Serial.print("Compass Heading: ");
  Serial.println(compass_heading);
  return compass_heading;
}
