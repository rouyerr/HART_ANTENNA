Point Get_Rocket_GPS(Point rocket_gps) {
  Serial.println("\nGETTING ROCKET GPS");

  // FOR REMY

  // rocket_gps = {rocket.latitude, rocket.longitude, rocket.altitude};

  // GET ROCKET GPS DATA AND RETURN

  Serial.print("Rocket GPS: ");
  Serial.println(rocket_gps.toString());
  return rocket_gps;

}