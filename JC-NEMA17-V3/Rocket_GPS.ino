void setup() {
  Serial.begin(9600);
}

Point Get_Rocket_GPS(Point rocket_gps) {
  Serial.println("\nGETTING ROCKET GPS");

  if (Serial.available() > 0) {
    String rocket_gps = Serial.readString(); 
    

  // GET ROCKET GPS DATA AND RETURN
  
  Serial.print("Rocket GPS: ");
  Serial.println(rocket_gps.toString());
  return rocket_gps;
  }
}
