#ifndef ODOMETRY_MODEL_H
#define ODOMETRY_MODEL_H

#include <math.h>
#include <stdint.h>

/**
 * @brief Converts measured encoder changes to pose using first-order Euler.
 *
 * This is a student-editable model, independent of the 32U4 pose estimate.
 * Its nominal geometry matches the Processing demonstration; calibrate it
 * against physical distance and heading observations for your own robot.
 * Positive wheel counts mean forward travel. At heading zero, forward is +x;
 * positive heading turns towards +y (anticlockwise when viewed from above).
 */
class OdometryModel_c {
  public:
    /** @brief Position in millimetres and unwrapped heading in radians. */
    struct Pose {
      float x;
      float y;
      float theta;
    };

    // Start with equal radii. Change both for a shared-radius calibration;
    // change them separately only when the measurements support that choice.
    float leftWheelRadiusMm;
    float rightWheelRadiusMm;
    float wheelSeparationMm;
    /** @brief Counts per complete wheel revolution, including gearing. */
    float encoderCountsPerRevolution;
    Pose pose;

    /** @brief Nominal starting values, not measured calibration results. */
    OdometryModel_c(float wheelRadiusMm = 16.0, float separationMm = 90.0, float countsPerRevolution = 358.3);

    void setLeftWheelRadiusMm( float mm );
    void setRightWheelRadiusMm( float mm );

    /**
     * @brief Sets the local starting pose and forgets the previous counts.
     * @note The next update establishes a count baseline without adding motion.
     * Reset while stationary before each trial, after changing parameters, or
     * after restarting/resetting the encoder source. This sends no I2C command.
     */
    void reset(float x = 0.0, float y = 0.0, float theta = 0.0);

    /**
     * @brief Processes one successfully acquired pair of absolute wheel counts.
     * @return True after a pose update; false for the first count baseline or
     * invalid geometry. Geometry values must be finite and greater than zero.
     * @note Call only after successful acquisition, using the named Robot_c
     * getters. Counts already measure movement over the interval, so no extra
     * multiplication by dt is needed. Keep timestamps in experimental logs.
     */
    bool update(int32_t leftCount, int32_t rightCount);

  private:
    bool initialised;
    int32_t previousLeftCount;
    int32_t previousRightCount;

    /** @brief Handles signed 32-bit counter wrap without signed overflow.
     * @note Assumes fewer than 2^31 counts of travel between observations.
     */
    static int64_t countChange(int32_t current, int32_t previous);
};

#endif
