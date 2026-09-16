/**
 * @brief Top-level model of the robot, excluding the environment.
 * @details Coordinates independent motor, surface-sensor, and odometry
 * models. Its public methods deliberately mirror the subset of Robot_c used
 * by Controller_c, while the model components remain visible for revision.
 */
class RobotSimulation_c {
  /** @brief Radius used only to draw the robot body, in mm. */
  final float bodyRadiusMm = 50.0;
  final float wheelRadiusMm;
  final float wheelSeparationMm;
  final float encoderCountsPerRevolution = 358.3;

  /** @brief Simulated clock, advanced explicitly in milliseconds. */
  long simulationMillis = 0;
  PImage surfaceImage;
  RobotSurface_c surface = new RobotSurface_c();
  MotorsModel_c motors = new MotorsModel_c();
  OdometryModel_c odometry;
  SurfaceSensorModel_c[] sensors = new SurfaceSensorModel_c[5];

  /**
   * @param wheelRadiusMmIn Wheel radius used by the odometry model, in mm.
   * @param wheelSeparationMmIn Distance between wheels, in mm.
   * @param surfaceIn Environment image sampled by the five surface sensors.
   */
  RobotSimulation_c(float wheelRadiusMmIn, float wheelSeparationMmIn,
    PImage surfaceIn) {
    wheelRadiusMm = wheelRadiusMmIn;
    wheelSeparationMm = wheelSeparationMmIn;
    surfaceImage = surfaceIn;
    odometry = new OdometryModel_c(wheelRadiusMm, encoderCountsPerRevolution);
    sensors[0] = new SurfaceSensorModel_c("DN1", 45, 45);
    sensors[1] = new SurfaceSensorModel_c("DN2", 45, 14);
    sensors[2] = new SurfaceSensorModel_c("DN3", 45, 0);
    sensors[3] = new SurfaceSensorModel_c("DN4", 45, -14);
    sensors[4] = new SurfaceSensorModel_c("DN5", 45, -45);
  }

  /** @return Current simulated time in milliseconds. */
  long getMillis() { return simulationMillis; }
  /** @brief Advances the simulated clock.
   * @param dtMs Positive elapsed model time in milliseconds.
   */
  void advanceTime(int dtMs) { simulationMillis += dtMs; }

  /** @brief Samples all five ideal surface sensors into surface.reading. */
  void getSurfaceSensors() {
    RobotPose_c pose = odometry.pose;
    for (int i = 0; i < sensors.length; i++) {
      surface.reading[i] = sensors[i].sample(surfaceImage, pose.x, pose.y,
        pose.theta);
    }
  }

  /** @brief Compatibility method; encoder values update during physics steps. */
  void getEncoders() { }
  /** @brief Compatibility method; pose values update during physics steps. */
  void getPose() { }

  /** @brief Stores requested left and right motor PWM values.
   * @param leftPwm Requested left PWM, nominally -400 to +400.
   * @param rightPwm Requested right PWM, nominally -400 to +400.
   */
  void setMotorPWM(float leftPwm, float rightPwm) {
    motors.setPWM(leftPwm, rightPwm);
  }

  /** @brief Resets pose and encoder state for a controlled run.
   * @param x Initial x position in mm.
   * @param y Initial y position in mm.
   * @param theta Initial heading in radians.
   * @param leftEncoder Initial left encoder count.
   * @param rightEncoder Initial right encoder count.
   */
  void resetForRun(float x, float y, float theta, float leftEncoder,
    float rightEncoder) {
    odometry.reset(x, y, theta, leftEncoder, rightEncoder);
    setMotorPWM(0, 0);
  }

  /** @brief Advances motor response and differential-drive odometry.
   * @param dtMs Fixed simulation step in milliseconds.
   */
  void updatePhysics(int dtMs) {
    motors.update();
    float leftDistance = motors.left.speedMmPerMs * dtMs;
    float rightDistance = motors.right.speedMmPerMs * dtMs;
    odometry.update(leftDistance, rightDistance, wheelSeparationMm);
  }

  /** @return The current five-channel ideal surface-sensor readings. */
  float[] sensorReadings() { return surface.reading; }

  void draw() {
    RobotPose_c pose = odometry.pose;
    pushMatrix();
    translate(pose.x, pose.y);
    rotate(pose.theta);
    fill(190, 205, 220);
    stroke(40);
    strokeWeight(2);
    ellipse(0, 0, bodyRadiusMm, bodyRadiusMm);
    stroke(40, 100, 220);
    strokeWeight(3);
    line(0, 0, bodyRadiusMm * .72, 0);
    fill(40, 100, 220);
    noStroke();
    triangle(bodyRadiusMm * .92, 0, bodyRadiusMm * .62, bodyRadiusMm * .14,
      bodyRadiusMm * .62, -bodyRadiusMm * .14);
    for (int i = 0; i < sensors.length; i++) sensors[i].draw(surface.reading[i]);
    popMatrix();
  }
}
