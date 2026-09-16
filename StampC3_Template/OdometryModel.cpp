#include "OdometryModel.h"

#include <Arduino.h>

OdometryModel_c::OdometryModel_c(float wheelRadiusMm, float separationMm, float countsPerRevolution)
  : leftWheelRadiusMm(wheelRadiusMm),
    rightWheelRadiusMm(wheelRadiusMm),
    wheelSeparationMm(separationMm),
    encoderCountsPerRevolution(countsPerRevolution) {
  reset();
}

void OdometryModel_c::setLeftWheelRadiusMm( float mm ) {
  leftWheelRadiusMm = mm;
}

void OdometryModel_c::setRightWheelRadiusMm( float mm ) {
  rightWheelRadiusMm = mm;
}

void OdometryModel_c::reset(float x, float y, float theta) {
  pose = {x, y, theta};
  initialised = false;
  previousLeftCount = 0;
  previousRightCount = 0;
}

bool OdometryModel_c::update(int32_t leftCount, int32_t rightCount) {
  if (!isfinite(leftWheelRadiusMm) || leftWheelRadiusMm <= 0.0f ||
      !isfinite(rightWheelRadiusMm) || rightWheelRadiusMm <= 0.0f ||
      !isfinite(wheelSeparationMm) || wheelSeparationMm <= 0.0f ||
      !isfinite(encoderCountsPerRevolution) ||
      encoderCountsPerRevolution <= 0.0f) {
    return false;
  }

  // Absolute counts may be non-zero when the StampC3 starts. They are not
  // distance travelled since the start of this particular pose estimate.
  if (!initialised) {
    previousLeftCount = leftCount;
    previousRightCount = rightCount;
    initialised = true;
    return false;
  }

  // Subtract as integers before converting to float, preserving small
  // increments even when the absolute count has grown large.
  float deltaLeftCounts = countChange(leftCount, previousLeftCount);
  float deltaRightCounts = countChange(rightCount, previousRightCount);
  previousLeftCount = leftCount;
  previousRightCount = rightCount;

  // Distance = revolutions * circumference, separately for each wheel.
  float leftDistance = deltaLeftCounts * TWO_PI * leftWheelRadiusMm
                       / encoderCountsPerRevolution;
  float rightDistance = deltaRightCounts * TWO_PI * rightWheelRadiusMm
                        / encoderCountsPerRevolution;
  float distance = (leftDistance + rightDistance) / 2.0f;
  float deltaTheta = (rightDistance - leftDistance) / wheelSeparationMm;

  // First-order Euler: use the OLD heading for both position increments,
  // then update heading. This matches DigitalTwin/OdometryModel.pde.
  // During curved motion this approximates an arc by short straight steps;
  // integration interval, wheel slip, and geometry can all affect accuracy.
  pose.x += distance * cosf(pose.theta);
  pose.y += distance * sinf(pose.theta);
  pose.theta += deltaTheta;
  return true;
}

int64_t OdometryModel_c::countChange(int32_t current, int32_t previous) {
  int64_t difference = int64_t(current) - int64_t(previous);
  const int64_t counterRange = int64_t(1) << 32;
  if (difference > INT32_MAX) difference -= counterRange;
  if (difference < INT32_MIN) difference += counterRange;
  return difference;
}
