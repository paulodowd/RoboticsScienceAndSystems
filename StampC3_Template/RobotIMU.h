#ifndef ROBOT_IMU_H
#define ROBOT_IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

/**
 * @brief One timestamped set of raw readings from the 3Pi+ inertial sensors.
 *
 * Values are kept as the signed 16-bit readings produced by the sensor ICs.
 * Keeping the raw values visible lets an investigation state and check its own
 * conversion, bias estimate, filtering, and integration rather than receiving
 * a pre-corrected orientation from this interface.
 */
struct RobotIMUSample_t {
  uint32_t timestamp_us;
  uint32_t sequence;
  int16_t acceleration[3];
  int16_t gyroscope[3];
  int16_t magnetic_field[3];
};

/**
 * @brief Direct StampC3 interface to the IMU sensors on the Pololu 3Pi+ 32U4.
 *
 * RobotIMU_c communicates directly with the LSM6DS33 accelerometer/gyroscope
 * and LIS3MDL magnetometer using Pololu's Arduino libraries. These devices
 * share the I2C bus with the 32U4, but their readings do not pass through the
 * opaque robot middleware.
 *
 * The class deliberately does not remove bias, filter readings, integrate
 * acceleration or angular velocity, or calculate an orientation. Those steps
 * change the evidence and therefore belong in the student's investigation.
 */
class RobotIMU_c {
  public:
    /** Axis indices used by RobotIMUSample_t's three-element arrays. */
    enum Axis {
      X_AXIS = 0,
      Y_AXIS = 1,
      Z_AXIS = 2
    };

    /**
     * @brief Creates an interface with no acquired samples.
     * @note Call initialise() after the shared I2C bus has been started.
     */
    RobotIMU_c();

    /**
     * @brief Detects and configures the 3Pi+ inertial sensors.
     * @return true when the LSM6DS33 accelerometer/gyroscope was found.
     *
     * The LSM6 is configured using the Pololu library defaults: +/-2 g for
     * acceleration and +/-245 degrees/s for angular velocity. The LIS3MDL is
     * configured for +/-4 gauss if it is present. A missing magnetometer does
     * not prevent the acceleration and gyro exercises from running.
     *
     * Wire.begin() is intentionally not called here because Robot_c owns the
     * initialisation of the shared bus. Calling it again could alter the bus
     * configuration used to communicate with the 32U4.
     */
    bool initialise();

    /** @brief Returns whether the accelerometer/gyroscope was detected. */
    bool isAvailable() const;

    /** @brief Returns whether the optional magnetometer was detected. */
    bool isMagnetometerAvailable() const;

    /**
     * @brief Checks whether a new accelerometer and gyro sample is available.
     * @return true when both LSM6DS33 data-ready flags are set.
     * @note Polling this method avoids repeatedly logging the same sensor data.
     */
    bool isSampleReady();

    /**
     * @brief Acquires and caches one raw IMU sample.
     * @return true when the LSM6DS33 was available and a sample was acquired.
     *
     * The timestamp is taken immediately after the I2C reads. It is a StampC3
     * acquisition timestamp, not an exact timestamp generated inside either
     * sensor. The magnetometer fields remain zero if that device is absent.
     */
    bool read();

    /** @brief Returns the most recently acquired sample. */
    const RobotIMUSample_t &getSample() const;

    /**
     * @brief Converts a raw default-range acceleration reading to g.
     * @note The LSM6DS33 sensitivity at +/-2 g is 0.061 mg per count.
     */
    static float accelerationToG(int16_t raw_value);

    /**
     * @brief Converts a raw default-range gyro reading to degrees per second.
     * @note The LSM6DS33 sensitivity at +/-245 dps is 8.75 mdps per count.
     */
    static float gyroscopeToDegreesPerSecond(int16_t raw_value);

    /**
     * @brief Converts a raw default-range magnetic reading to gauss.
     * @note The LIS3MDL sensitivity at +/-4 gauss is 6842 counts per gauss.
     */
    static float magneticFieldToGauss(int16_t raw_value);

  private:
    LSM6 lsm6;
    LIS3MDL magnetometer;
    bool lsm6_available;
    bool magnetometer_available;
    uint32_t sample_sequence;
    RobotIMUSample_t sample;
};

#endif
