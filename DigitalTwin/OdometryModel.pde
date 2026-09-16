/** @brief Cached left and right encoder counts. */
class RobotEncoders_c {
  float left = 0;
  float right = 0;
}

/** @brief Robot pose in world coordinates: x/y in mm and heading in radians. */
class RobotPose_c {
  float x = 0;
  float y = 0;
  float theta = 0;
}

/**
 * @brief Ideal differential-drive pose and encoder model.
 * @details The model integrates wheel travel over each fixed simulation step.
 * It makes no claim that its geometry or encoder resolution is calibrated to
 * a physical robot.
 */
class OdometryModel_c {
  final float mmPerEncoderCount;
  RobotEncoders_c encoders = new RobotEncoders_c();
  RobotPose_c pose = new RobotPose_c();

  /**
   * @param wheelRadiusMm Wheel radius used to convert revolutions to mm.
   * @param encoderCountsPerRevolution Encoder resolution used for count output.
   */
  OdometryModel_c(float wheelRadiusMm, float encoderCountsPerRevolution) {
    mmPerEncoderCount = TWO_PI * wheelRadiusMm / encoderCountsPerRevolution;
  }

  /** @brief Sets the complete pose and encoder state for a new run. */
  void reset(float x, float y, float theta, float leftEncoder,
    float rightEncoder) {
    pose.x = x;
    pose.y = y;
    pose.theta = theta;
    encoders.left = leftEncoder;
    encoders.right = rightEncoder;
  }

  /**
   * @brief Integrates one differential-drive motion increment.
   * @param leftDistance Left wheel travel in mm over the step.
   * @param rightDistance Right wheel travel in mm over the step.
   * @param wheelSeparationMm Distance between wheels in mm.
   */
  void update(float leftDistance, float rightDistance, float wheelSeparationMm) {
    float distance = (leftDistance + rightDistance) / 2.0;
    float deltaTheta = (rightDistance - leftDistance) / wheelSeparationMm;
    pose.x += distance * cos(pose.theta);
    pose.y += distance * sin(pose.theta);
    pose.theta += deltaTheta;
    encoders.left += leftDistance / mmPerEncoderCount;
    encoders.right += rightDistance / mmPerEncoderCount;
  }
}
