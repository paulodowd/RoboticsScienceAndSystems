/**
 * @brief Ideal paired motor-response model for the simulated robot.
 * @note This deliberately linear mapping is a starting assumption, not a
 * calibrated model of the physical motors.
 */
class MotorsModel_c {
  // Deliberately simple starting assumption for student-led refinement.
  final float maximumWheelSpeedMmPerMs = 0.25;
  MotorModel_c left = new MotorModel_c();
  MotorModel_c right = new MotorModel_c();

  /** @brief Stores requested left and right PWM values after limiting them. */
  void setPWM(float leftPwm, float rightPwm) {
    left.setPWM(leftPwm);
    right.setPWM(rightPwm);
  }

  /** @brief Updates both ideal wheel-speed estimates for the current PWM. */
  void update() {
    left.update(maximumWheelSpeedMmPerMs);
    right.update(maximumWheelSpeedMmPerMs);
  }
}

/** @brief Ideal response model for one motor. */
class MotorModel_c {
  float pwm = 0;
  float speedMmPerMs = 0;

  /** @brief Limits and stores a requested PWM value in [-200, +200]. */
  void setPWM(float value) { pwm = constrain(value, -200, 200); }

  /** @brief Maps the requested PWM linearly to wheel speed.
   * @param maximumSpeed Maximum wheel speed in mm/ms.
   */
  void update(float maximumSpeed) {
    speedMmPerMs = map(pwm, -200, 200, -maximumSpeed, maximumSpeed);
  }
}
