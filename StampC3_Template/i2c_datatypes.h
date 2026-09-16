#ifndef I2C_DATATYPES_H
#define I2C_DATATYPES_H

/**
 * @file i2c_datatypes.h
 * @brief Packed I2C protocol types shared by the StampC3 and robot middleware.
 *
 * These layouts form a binary protocol. Do not reorder fields or change their
 * widths without making the corresponding change on both I2C endpoints.
 */

#pragma pack(push, 1)

/** @brief I2C address used by the robot middleware. */
#define ROBOT_I2C_ADDR 0x08

/** @brief One-byte selector for an I2C request or command. */
typedef uint8_t RobotRegister_t;

/** @brief No-operation register value. */
#define REGISTER_NULL 0x00
/** @brief Requests one RobotSurface_t snapshot. */
#define REGISTER_GET_SURFACE 0x01
/** @brief Reserved bump-sensor request. */
#define REGISTER_GET_BUMP 0x02
/** @brief Requests one RobotEncoders_t snapshot. */
#define REGISTER_GET_ENCODERS 0x03
/** @brief Requests one RobotPose_t snapshot. */
#define REGISTER_GET_POSE 0x04
/** @brief Sends one RobotPose_t value. */
#define REGISTER_SET_POSE 0x05
/** @brief Sends one RobotMotors_t command. */
#define REGISTER_SET_MOTORS 0x06
/** @brief Sends one RobotBuzzer_t command. */
#define REGISTER_SET_BUZZER 0x07
/** @brief Sends one RobotTeamname_t value. */
#define REGISTER_SET_TEAMNAME 0x08
/** @brief Starts one fixed-amplitude nominal straight-line motion. */
#define REGISTER_SET_MOTION_DISTANCE 0x09
/** @brief Starts one fixed-amplitude nominal in-place rotation. */
#define REGISTER_SET_MOTION_ROTATION 0x0A
/** @brief Requests one RobotMotionStatus_t snapshot. */
#define REGISTER_GET_MOTION_STATUS 0x0B
/** @brief Starts the built-in motor, encoder, and surface-sensor activity test. */
#define REGISTER_START_SELF_TEST 0x0C
/** @brief Requests the most recent RobotSelfTestResult_t snapshot. */
#define REGISTER_GET_SELF_TEST 0x0D

/** @brief No helper motion has been requested since reset. */
#define MOTION_STATUS_IDLE 0x00
/** @brief A helper motion currently owns the motors. */
#define MOTION_STATUS_RUNNING 0x01
/** @brief Both encoder targets were reached. */
#define MOTION_STATUS_COMPLETE 0x02
/** @brief The helper stopped after its safety timeout. */
#define MOTION_STATUS_TIMEOUT 0x03
/** @brief A later ordinary motor command interrupted the helper. */
#define MOTION_STATUS_CANCELLED 0x04

/** @brief No helper motion mode is associated with the status. */
#define MOTION_MODE_NONE 0x00
/** @brief The helper is carrying out a nominal straight-line motion. */
#define MOTION_MODE_DISTANCE 0x01
/** @brief The helper is carrying out a nominal in-place rotation. */
#define MOTION_MODE_ROTATION 0x02

/** @brief No self-test has run since reset. */
#define SELF_TEST_STATUS_IDLE 0x00
/** @brief The self-test currently owns the motors. */
#define SELF_TEST_STATUS_RUNNING 0x01
/** @brief The self-test completed and its pass masks are valid. */
#define SELF_TEST_STATUS_COMPLETE 0x02
/** @brief A newer motor-owning command interrupted the self-test. */
#define SELF_TEST_STATUS_CANCELLED 0x03

/**
 * @brief Five-channel downward surface-sensor snapshot.
 *
 * @var RobotSurface_t::timestamp_us Timestamp supplied by the robot interface.
 * @var RobotSurface_t::reading Five raw surface-reading values.
 */
typedef struct {
  uint32_t timestamp_us;
  uint16_t reading[5];
} RobotSurface_t;

/**
 * @brief Pose estimate supplied by the robot interface.
 * @note This estimate should be checked against independent physical references
 * when students investigate odometry.
 */
typedef struct {
  float x;
  float y;
  float theta;
} RobotPose_t;

/**
 * @brief Pair of wheel encoder counts supplied by the robot interface.
 * @note Channel order is part of the I2C protocol; use Robot_c getter methods
 * when controller code needs named left or right counts.
 */
typedef struct {
  int32_t count[2];
} RobotEncoders_t;

/**
 * @brief Pair of requested motor PWM values for the I2C motor command.
 * @note Use Robot_c::setMotorPWM rather than assuming a raw array index maps to
 * a particular physical wheel.
 */
typedef struct {
  int16_t pwm[2];
} RobotMotors_t;

/** @brief Buzzer-command payload. */
typedef struct {
  uint16_t freq;
  uint16_t duration_ms;
} RobotBuzzer_t;

/** @brief Payload for a nominal straight-line helper request in millimetres. */
typedef struct {
  float distance_mm;
} RobotMotionDistance_t;

/**
 * @brief Payload for a nominal in-place rotation helper request in radians.
 * @note Positive angles request a left turn.
 */
typedef struct {
  float angle_rad;
} RobotMotionRotation_t;

/**
 * @brief Snapshot of the most recently requested helper motion.
 *
 * @var RobotMotionStatus_t::state Current helper state.
 * @var RobotMotionStatus_t::mode Type of the most recent helper request.
 * @var RobotMotionStatus_t::elapsed_ms Elapsed helper time in milliseconds.
 * @note Record encoder snapshots separately with Robot_c::getEncoders() before,
 * during, and after a helper motion. Completion is not an independent
 * measurement of physical distance or heading.
 */
typedef struct {
  uint8_t state;
  uint8_t mode;
  uint32_t elapsed_ms;
} RobotMotionStatus_t;

/** @brief Command payload used to start or restart the built-in self-test. */
typedef struct {
  uint8_t start;
  // A two-byte payload cannot be mistaken for a register-selection transaction.
  uint8_t reserved;
} RobotSelfTestCommand_t;

/**
 * @brief Result of the most recent built-in hardware activity test.
 *
 * During the two-second test the robot rotates on the spot. Encoder movement
 * is the only onboard evidence of wheel activity, so corresponding motor and
 * encoder bits intentionally agree; the test cannot distinguish a failed
 * motor from a failed encoder on the same wheel without external evidence.
 * Surface bits 0..4 correspond to DN1..DN5 and pass when the raw RC reading
 * varied during the test. The ranges and encoder deltas support diagnosis.
 */
typedef struct {
  uint8_t state;
  uint8_t motor_pass_mask;
  uint8_t encoder_pass_mask;
  uint8_t surface_pass_mask;
  uint32_t elapsed_ms;
  int32_t encoder_delta[2];
  uint16_t surface_range[5];
} RobotSelfTestResult_t;

/** @brief Null-terminated team-name payload with space for 31 characters plus terminator. */
typedef char RobotTeamname_t[32];

#pragma pack(pop)

#endif
