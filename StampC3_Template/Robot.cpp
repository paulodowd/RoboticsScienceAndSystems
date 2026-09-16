#include "Robot.h"

Robot_c::Robot_c() {
  memset(&teamname, 0, sizeof(teamname));
  memset(&encoders, 0, sizeof(encoders));
  memset(&last_encoders, 0, sizeof(last_encoders));
  memset(&surface, 0, sizeof(surface));
  memset(&motors, 0, sizeof(motors));
  memset(&buzzer, 0, sizeof(buzzer));
  memset(&pose, 0, sizeof(pose));
  memset(&motion_status, 0, sizeof(motion_status));
  memset(&self_test, 0, sizeof(self_test));
  button_debounce_ts = millis();
}

void Robot_c::initialise() {
  Wire.begin();
  Wire.setClock(400000);
  setupLED();
  pinMode(3, INPUT_PULLUP);
}

bool Robot_c::initialise(char* teamname_in) {
  initialise();
  delay(50);
  return setTeamname(teamname_in);
}

bool Robot_c::isButtonPressed() {
  if (millis() - button_debounce_ts > debounce_delay_ms) {
    if (digitalRead(3) == LOW) {
      button_debounce_ts = millis();
      return true;
    }
  }
  return false;
}

unsigned long Robot_c::getMillis() {
  return millis();
}

bool Robot_c::getSurfaceSensors() {
  return requestSnapshot(REGISTER_GET_SURFACE, surface);
}

bool Robot_c::getEncoders() {
  RobotEncoders_t new_encoders;
  if (!requestSnapshot(REGISTER_GET_ENCODERS, new_encoders)) return false;

  last_encoders = encoders;
  encoders = new_encoders;
  return true;
}

bool Robot_c::getPose() {
  return requestSnapshot(REGISTER_GET_POSE, pose);
}

bool Robot_c::startMoveDistance(float distance_mm) {
  RobotMotionDistance_t request_data;
  request_data.distance_mm = distance_mm;
  return sendMotionRequest(REGISTER_SET_MOTION_DISTANCE, request_data);
}

bool Robot_c::startRotateAngle(float angle_rad) {
  RobotMotionRotation_t request_data;
  request_data.angle_rad = angle_rad;
  return sendMotionRequest(REGISTER_SET_MOTION_ROTATION, request_data);
}

bool Robot_c::getMotionStatus() {
  return requestSnapshot(REGISTER_GET_MOTION_STATUS, motion_status);
}

bool Robot_c::isMotionHelperRunning() {
  return motion_status.state == MOTION_STATUS_RUNNING;
}

bool Robot_c::startSelfTest() {
  RobotSelfTestCommand_t command;
  command.start = 1;
  command.reserved = 0;
  return writeRegisterPayload(REGISTER_START_SELF_TEST, command);
}

bool Robot_c::getSelfTestResult() {
  return requestSnapshot(REGISTER_GET_SELF_TEST, self_test);
}

bool Robot_c::isSelfTestRunning() {
  return self_test.state == SELF_TEST_STATUS_RUNNING;
}

bool Robot_c::didMotorPassSelfTest(uint8_t motor_index) {
  return motor_index < 2 && (self_test.motor_pass_mask & (1U << motor_index));
}

bool Robot_c::didEncoderPassSelfTest(uint8_t encoder_index) {
  return encoder_index < 2 && (self_test.encoder_pass_mask & (1U << encoder_index));
}

bool Robot_c::didSurfaceSensorPassSelfTest(uint8_t sensor_index) {
  return sensor_index < 5 && (self_test.surface_pass_mask & (1U << sensor_index));
}

int32_t Robot_c::getLeftEncoderCount() {
  return encoders.count[0];
}

int32_t Robot_c::getRightEncoderCount() {
  return encoders.count[1];
}

int16_t Robot_c::getLeftMotorPWM() {
  return motors.pwm[0];
}

int16_t Robot_c::getRightMotorPWM() {
  return motors.pwm[1];
}

bool Robot_c::setTeamname(char* teamname_in) {
  RobotTeamname_t new_teamname;
  snprintf(new_teamname, sizeof(new_teamname), "%s", teamname_in);
  if (!writeRegisterPayload(REGISTER_SET_TEAMNAME, new_teamname)) return false;

  memcpy(teamname, new_teamname, sizeof(teamname));
  return true;
}

bool Robot_c::setMotorPWM(float l, float r) {
  RobotMotors_t new_motors;
  new_motors.pwm[0] = (int16_t)l;
  new_motors.pwm[1] = (int16_t)r;
  if (!writeRegisterPayload(REGISTER_SET_MOTORS, new_motors)) return false;

  motors = new_motors;
  return true;
}

bool Robot_c::setPose(float x, float y, float theta) {
  RobotPose_t new_pose;
  new_pose.x = x;
  new_pose.y = y;
  new_pose.theta = theta;
  return writeRegisterPayload(REGISTER_SET_POSE, new_pose);
}

bool Robot_c::playTone(uint16_t freq, uint16_t duration_ms) {
  RobotBuzzer_t new_tone;
  new_tone.freq = freq;
  new_tone.duration_ms = duration_ms;
  
  if (!writeRegisterPayload(REGISTER_SET_BUZZER, new_tone)) return false;

  buzzer = new_tone;
  return true;
}

void Robot_c::setLED(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness) {
  r = (uint8_t)constrain(r, 0, 255);
  g = (uint8_t)constrain(g, 0, 255);
  b = (uint8_t)constrain(b, 0, 255);
  brightness = constrain(brightness, 0, 255);

  uint8_t p[3];
  p[LED_rOffset] = r * brightness / 255;
  p[LED_gOffset] = g * brightness / 255;
  p[LED_bOffset] = b * brightness / 255;

  uint32_t color = p[0] << 16 | p[1] << 8 | p[2];
  for (int bit = 0; bit < 24; bit++) {
    if (color & (1 << (23 - bit))) {
      led_data[bit].level0 = 1;
      led_data[bit].duration0 = 8;
      led_data[bit].level1 = 0;
      led_data[bit].duration1 = 4;
    } else {
      led_data[bit].level0 = 1;
      led_data[bit].duration0 = 4;
      led_data[bit].level1 = 0;
      led_data[bit].duration1 = 8;
    }
  }
  updateLED();
}

bool Robot_c::selectRegister(RobotRegister_t request) {
  Wire.beginTransmission(ROBOT_I2C_ADDR);
  const size_t written = Wire.write((byte*)&request, sizeof(request));
  const uint8_t error = Wire.endTransmission();
  return written == sizeof(request) && error == 0;
}

template<typename Payload_t>
bool Robot_c::writeRegisterPayload(RobotRegister_t request, const Payload_t& payload) {
  if (!selectRegister(request)) return false;

  Wire.beginTransmission(ROBOT_I2C_ADDR);
  const size_t written = Wire.write((const byte*)&payload, sizeof(payload));
  const uint8_t error = Wire.endTransmission();
  return written == sizeof(payload) && error == 0;
}

template<typename Snapshot_t>
bool Robot_c::requestSnapshot(RobotRegister_t request, Snapshot_t& snapshot) {
  if (!selectRegister(request)) return false;

  Snapshot_t received_snapshot;
  const size_t received = Wire.requestFrom(ROBOT_I2C_ADDR, sizeof(received_snapshot));
  if (received != sizeof(received_snapshot)) {
    // Discard any partial response so it cannot contaminate a later read.
    while (Wire.available()) Wire.read();
    return false;
  }

  const size_t copied = Wire.readBytes(
    (byte*)&received_snapshot,
    sizeof(received_snapshot)
  );
  if (copied != sizeof(received_snapshot)) return false;

  snapshot = received_snapshot;
  return true;
}

template<typename MotionRequest_t>
bool Robot_c::sendMotionRequest(RobotRegister_t request, const MotionRequest_t& request_data) {
  return writeRegisterPayload(request, request_data);
}

bool Robot_c::setupLED() {
  LED_rOffset = (0x12 >> 4) & 0x03;
  LED_gOffset = (0x12 >> 2) & 0x03;
  LED_bOffset = 0x12 & 0x03;
  rmt_reserve_memsize_t rmt_mem = RMT_MEM_NUM_BLOCKS_1;
  if (rmtInit(2, RMT_TX_MODE, rmt_mem, 10000000) == false) {
    return false;
  }

  for (int i = 0; i < 24; i++) {
    led_data[i].level0 = 1;
    led_data[i].duration0 = 4;
    led_data[i].level1 = 0;
    led_data[i].duration1 = 8;
  }

  setLED(255, 128, 178, 20);
  return true;
}

esp_err_t Robot_c::updateLED() {
  return rmtWrite(2, led_data, 24, RMT_WAIT_FOR_EVER);
}
