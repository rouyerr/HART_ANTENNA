// Define stepper motor connections

//STEPPER 1: HORIZONTAL MOTOR
#define motor_horizontal_Pin1 8       // IN1 on NEMA 17 ==> Blue   on MOTOR 1
#define motor_horizontal_Pin2 9       // IN2 on NEMA 17 ==> Pink   on MOTOR 1
#define motor_horizontal_Pin3 10      // IN3 on NEMA 17 ==> Yellow on MOTOR 1
#define motor_horizontal_Pin4 11      // IN4 on NEMA 17 ==> Orange on MOTOR 1

//STEPPER 2: VERTICAL MOTOR
#define motor_vertical_Pin1 4         // IN1 on NEMA 17 ==> Blue   on MOTOR 2
#define motor_vertical_Pin2 5         // IN2 on NEMA 17 ==> Pink   on MOTOR 2
#define motor_vertical_Pin3 6         // IN3 on NEMA 17 ==> Yellow on MOTOR 2
#define motor_vertical_Pin4 7         // IN4 on NEMA 17 ==> Orange on MOTOR 2

#define STEPS_PER_REVOLUTION 200      // Constants for steps per revolution and speed of stepper motors
#define SPEED 30

// Initialize stepper motors
Stepper MOTOR_H(STEPS_PER_REVOLUTION, motor_horizontal_Pin1, motor_horizontal_Pin3, motor_horizontal_Pin2, motor_horizontal_Pin4);
Stepper MOTOR_V(STEPS_PER_REVOLUTION, motor_vertical_Pin1, motor_vertical_Pin3, motor_vertical_Pin2, motor_vertical_Pin4);

Point initialize_motors() {
  MOTOR_H.setSpeed(SPEED);                                      // Set stepper motor speeds
  MOTOR_V.setSpeed(SPEED);

  while (digitalRead(horizontal_limit_pin) == LOW)              // Find Zero Location of Horizontal Motor
    MOTOR_H.step(-1);
  MOTOR_H.step(STEPS_PER_REVOLUTION);                           // Reset Horizontal Motor to Center

  while (digitalRead(vertical_limit_pin) == LOW)                // Find Zero Location of Vertical Motor
    MOTOR_V.step(-1);
  MOTOR_V.step(STEPS_PER_REVOLUTION / 8);                       // Reset to 45° above Horizontal

  Point dir_uni_vec = {1, 1, 1};                                // Define Orientation of antenna and Return
  return dir_uni_vec / sqrt(3);
}
