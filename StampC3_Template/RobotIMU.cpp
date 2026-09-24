#include "RobotIMU.h"

namespace {
// Choose a 104 Hz sensor output rate close to the controller's nominal 100 Hz
// update rate. This configures the sensors only: the supplied controller does
// not read the IMU. Investigation code must call isSampleReady() and read().
constexpr uint8_t LSM6_ACCELEROMETER_104_HZ_2_G = 0x40;
constexpr uint8_t LSM6_GYROSCOPE_104_HZ_245_DPS = 0x40;
constexpr uint8_t LSM6_BLOCK_UPDATE_AND_AUTO_INCREMENT = 0x44;
}

RobotIMU_c::RobotIMU_c()
  : lsm6_available(false),
    magnetometer_available(false),
    sample_sequence(0) {
  memset(&sample, 0, sizeof(sample));
}

bool RobotIMU_c::initialise() {
  lsm6.setBus(&Wire);
  lsm6_available = lsm6.init(LSM6::device_DS33, LSM6::sa0_auto);
  if (lsm6_available) {
    // Establish the library's known baseline, then replace its 1.66 kHz output
    // rates with settings suited to polling near the controller's 10 ms interval.
    // This does not add IMU acquisition to the controller.
    lsm6.enableDefault();
    lsm6.writeReg(
      LSM6::CTRL1_XL,
      LSM6_ACCELEROMETER_104_HZ_2_G
    );
    lsm6.writeReg(
      LSM6::CTRL2_G,
      LSM6_GYROSCOPE_104_HZ_245_DPS
    );
    lsm6.writeReg(
      LSM6::CTRL3_C,
      LSM6_BLOCK_UPDATE_AND_AUTO_INCREMENT
    );
  }

  // The Pololu LIS3MDL library uses the default Wire bus, which is also the
  // bus used by Robot_c and the LSM6 above.
  magnetometer_available = magnetometer.init(
    LIS3MDL::device_LIS3MDL,
    LIS3MDL::sa1_auto
  );
  if (magnetometer_available) {
    magnetometer.enableDefault();
  }

  sample_sequence = 0;
  memset(&sample, 0, sizeof(sample));
  return lsm6_available;
}

bool RobotIMU_c::isAvailable() const {
  return lsm6_available;
}

bool RobotIMU_c::isMagnetometerAvailable() const {
  return magnetometer_available;
}

bool RobotIMU_c::isSampleReady() {
  if (!lsm6_available) {
    return false;
  }

  const uint8_t ACCELEROMETER_READY = 0x01;
  const uint8_t GYROSCOPE_READY = 0x02;
  uint8_t status = lsm6.readReg(LSM6::STATUS_REG);
  return (status & (ACCELEROMETER_READY | GYROSCOPE_READY)) ==
         (ACCELEROMETER_READY | GYROSCOPE_READY);
}

bool RobotIMU_c::read() {
  if (!lsm6_available) {
    return false;
  }

  lsm6.read();
  sample.acceleration[X_AXIS] = lsm6.a.x;
  sample.acceleration[Y_AXIS] = lsm6.a.y;
  sample.acceleration[Z_AXIS] = lsm6.a.z;
  sample.gyroscope[X_AXIS] = lsm6.g.x;
  sample.gyroscope[Y_AXIS] = lsm6.g.y;
  sample.gyroscope[Z_AXIS] = lsm6.g.z;

  if (magnetometer_available) {
    magnetometer.read();
    sample.magnetic_field[X_AXIS] = magnetometer.m.x;
    sample.magnetic_field[Y_AXIS] = magnetometer.m.y;
    sample.magnetic_field[Z_AXIS] = magnetometer.m.z;
  }

  sample.timestamp_us = micros();
  sample.sequence = ++sample_sequence;
  return true;
}

const RobotIMUSample_t &RobotIMU_c::getSample() const {
  return sample;
}

float RobotIMU_c::accelerationToG(int16_t raw_value) {
  return raw_value * 0.000061f;
}

float RobotIMU_c::gyroscopeToDegreesPerSecond(int16_t raw_value) {
  return raw_value * 0.00875f;
}

float RobotIMU_c::magneticFieldToGauss(int16_t raw_value) {
  return raw_value / 6842.0f;
}
