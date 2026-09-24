/**
 * @brief Nominal line-following controller shared conceptually with StampC3.
 * @details The controller reads cached robot values at a fixed interval and
 * commands requested PWM values. It is intentionally separate from the
 * physical-response models so those models can be revised independently.
 */
class Controller_c {
  /** @brief Minimum interval between control/acquisition updates, in ms. */
  final int UPDATE_INTERVAL_MS = 10;
  /** @brief Interval reserved for telemetry work, in ms. */
  final int TELEMETRY_INTERVAL_MS = 50;
  final int RECOVERY_TIMEOUT_MS = 5000;
  final float LINE_THRESHOLD = 1400;
  // Keep these values aligned with StampC3_Template/Controller.h.
  final float BASE_PWM = 45;
  final float LINE_FOLLOW_GAIN = 0.01;
  final float RECOVERY_LEFT_PWM = -35.0;
  final float RECOVERY_RIGHT_PWM = 35.0;
  final int WAITING = 0, FOLLOWING = 1, SEARCHING = 2, STOPPED = 3;
  int signal = 0, mode = WAITING;
  long lastUpdateMs = 0, lastTelemetryMs = 0, recoveryStartMs = 0;

  /** @brief Returns the controller to its inactive waiting state. */
  void reset() {
    signal = 0;
    mode = WAITING;
    lastUpdateMs = lastTelemetryMs = recoveryStartMs = 0;
  }
  /** @return The latched start signal: 0 while waiting, 1 once started. */
  int getSignal() {
    return signal;
  }
  /**
   * @brief Updates the latched start signal.
   * @param requested Zero resets the controller; a non-zero value starts it.
   */
  void setSignal(int requested) {
    if (requested == 0) {
      reset();
      return;
    }
    if (signal == 0) {
      signal = 1;
      mode = FOLLOWING;
      recoveryStartMs = 0;
    }
  }
  /**
   * @brief Runs at most one periodic controller update.
   * @param robot Simulated robot supplying cached readings and accepting PWM.
   * @note No update occurs until UPDATE_INTERVAL_MS has elapsed in simulation
   * time.
   */
  void update(RobotSimulation_c robot) {
    long now = robot.getMillis();
    if (now - lastUpdateMs < UPDATE_INTERVAL_MS) return;
    lastUpdateMs = now;
    robot.getSurfaceSensors();
    robot.getEncoders();
    robot.getPose();
    if (signal == 1) runLineFollower(robot, now);
    else robot.setMotorPWM(0, 0);
    if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
      lastTelemetryMs = now;
      publishTelemetry(robot);
    }
  }
  /** @brief Tests whether any surface channel is at or above the line threshold. */
  boolean lineDetected(RobotSimulation_c robot) {
    if (robot.surface.reading[2] >= LINE_THRESHOLD) return true;
    return false;
  }
  /** @brief Applies following, search, or stopped motor commands. */
  void runLineFollower(RobotSimulation_c robot, long now) {
    boolean detected = lineDetected(robot);
    if (!detected && mode == FOLLOWING) {
      mode = SEARCHING;
      recoveryStartMs = now;
    }
    
    if (mode == SEARCHING && detected) {
      mode = FOLLOWING;
    }
    if (mode == SEARCHING && now - recoveryStartMs >= RECOVERY_TIMEOUT_MS) {
      mode = STOPPED;
    }

    if (mode == STOPPED) {
      robot.setMotorPWM(0, 0);
      return;
    }
    if (mode == SEARCHING) {
      robot.setMotorPWM(RECOVERY_LEFT_PWM, RECOVERY_RIGHT_PWM);
      return;
    }
    
    float turn = (robot.surface.reading[1] - robot.surface.reading[3]) * LINE_FOLLOW_GAIN;
    robot.setMotorPWM(BASE_PWM - turn, BASE_PWM + turn);
  }
  /**
   * @brief Placeholder matching the StampC3 controller's telemetry task.
   * @note Simulator owns TCP reception and graphing. This empty method keeps
   * the 50 ms controller task visible without creating a second transport.
   */
  void publishTelemetry(RobotSimulation_c robot) {
  }
}
