#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include "i2c_datatypes.h"

/**
 * @brief Provides the StampC3 application's public interface to the 3Pi+ robot.
 *
 * Robot_c exchanges packed requests and snapshots with the robot over I2C. Its
 * public data members are local caches, not live hardware registers. Successful
 * get...() requests store received snapshots; successful writes cache requested
 * motor PWM, team name, buzzer settings, or pose. Failed transactions leave the
 * corresponding caches unchanged. Cached commands are not measurements of the
 * physical response, which remains a property for students to measure.
 */
class Robot_c {
  public:
    /** @brief Cached team name most recently sent to the robot. */
    RobotTeamname_t teamname;
    /** @brief Most recent encoder snapshot received from the robot. */
    RobotEncoders_t encoders;
    /** @brief Encoder snapshot before the most recent successful getEncoders() call. */
    RobotEncoders_t last_encoders;
    /** @brief Most recent five-channel surface-sensor snapshot. */
    RobotSurface_t surface;
    /** @brief Most recent motor PWM request successfully sent over I2C. */
    RobotMotors_t motors;
    /** @brief Most recent buzzer request successfully sent over I2C. */
    RobotBuzzer_t buzzer;
    /** @brief Pose cached by the most recent successful getPose() or setPose(). */
    RobotPose_t pose;
    /** @brief Most recent fixed-amplitude motion-helper status snapshot. */
    RobotMotionStatus_t motion_status;
    /** @brief Most recently received motion-helper geometry snapshot. */
    RobotMotionGeometry_t motion_geometry;
    /** @brief Most recent built-in hardware self-test result snapshot. */
    RobotSelfTestResult_t self_test;

    /** @brief Creates a zero-initialised interface and starts button debounce timing. */
    Robot_c();

    /**
     * @brief Initialises I2C, the on-board LED, and the StampC3 button input.
     * @note This must be called from setup() before requesting robot data.
     */
    void initialise();

    /**
     * @brief Initialises the interface and sends a team name to the robot.
     * @param teamname_in Null-terminated team name.
     * @return true when the name is valid and the I2C transaction succeeds.
     */
    bool initialise(const char* teamname_in);

    /**
     * @brief Returns a rate-limited active-low reading from the StampC3 button.
     * @return true when a low input is accepted; false otherwise.
     * @note The input is active-low and uses a 250 ms debounce interval. A
     * button held down can therefore produce another true result after the
     * debounce interval; this method does not wait for release.
     */
    bool isButtonPressed();

    /** @brief Returns the StampC3 millisecond clock. */
    unsigned long getMillis();

    /**
     * @brief Requests and stores one five-channel surface-sensor snapshot.
     * @return true when a complete snapshot is received.
     * @note The result is written to surface and remains cached until refreshed.
     */
    bool getSurfaceSensors();


    /**
     * @brief Requests and stores one encoder-count snapshot.
     * @return true when a complete snapshot is received.
     * @note After a complete response, the previous encoder snapshot is copied
     * into last_encoders and the newly received pair becomes encoders. Neither
     * cache changes after a failed or incomplete response.
     */
    bool getEncoders();

    /**
     * @brief Requests and stores one robot pose snapshot.
     * @return true when a complete snapshot is received.
     * @note Pose is an interface-provided estimate, not an independent reference
     * measurement of physical position or heading.
     */
    bool getPose();

    /**
     * @brief Starts a relative straight-line helper motion.
     * @param distance_mm Requested travel in millimetres; negative values move backwards.
     * Values from -2000 to +2000 mm are accepted.
     * @return true when the value is valid and both I2C writes are acknowledged.
     * @note The helper uses a fixed internal PWM and its currently configured geometry.
     * It bypasses the normal motor middleware path, so it is an experimental
     * apparatus aid rather than a replacement for ordinary motor commands.
     */
    bool startMoveDistance(float distance_mm);

    /**
     * @brief Starts a nominal in-place rotation helper motion.
     * @param angle_rad Requested angle in radians; positive values request a left turn.
     * Values from -2*pi to +2*pi radians are accepted.
     * @return true when the value is valid and both I2C writes are acknowledged.
     * @note A subsequent setMotorPWM() command cancels this helper immediately.
     */
    bool startRotateAngle(float angle_rad);

    /**
     * @brief Sends geometry used to calculate future motion-helper targets.
     * @param left_wheel_radius_mm Effective left wheel radius in millimetres.
     * @param right_wheel_radius_mm Effective right wheel radius in millimetres.
     * @param wheel_separation_mm Effective wheel separation in millimetres.
     * @return true when both I2C writes are acknowledged.
     * @note The middleware accepts finite radii from 5 to 50 mm and a finite
     * separation from 20 to 200 mm, and only applies a write while no helper is
     * running. This does not alter the middleware pose model. Call
     * getMotionHelperGeometry() afterwards to verify the accepted values.
     */
    bool setMotionHelperGeometry( float left_wheel_radius_mm,float right_wheel_radius_mm,float wheel_separation_mm );

    /**
     * @brief Requests the geometry currently used by the motion helpers.
     * @return true when a complete snapshot is received.
     * @note The result is written to motion_geometry.
     */
    bool getMotionHelperGeometry();

    /**
     * @brief Requests and stores the current helper-motion status.
     * @return true when a complete status is received.
     * @note Encoder, sensor, and pose requests can be made while the helper runs.
     */
    bool getMotionStatus();

    /** @brief Returns true when the cached helper status is running. */
    bool isMotionHelperRunning();

    /**
     * @brief Starts or restarts the middleware's two-second hardware self-test.
     * @return true when both I2C writes are acknowledged.
     * @warning The robot rotates immediately. Place it on a clear, varied
     * surface and keep hands, cables, and obstacles outside its swept area.
     * @note The self-test becomes the current motor owner and cancels any
     * ordinary command or motion helper that has not completed.
     */
    bool startSelfTest();

    /**
     * @brief Requests and caches the current or most recent self-test result.
     * @return true when a complete result is received.
     */
    bool getSelfTestResult();

    /** @brief Returns true while the cached self-test status is running. */
    bool isSelfTestRunning();

    /** @brief Returns the cached pass result for the named motor channel. */
    bool didMotorPassSelfTest(uint8_t motor_index);

    /** @brief Returns the cached pass result for the named encoder channel. */
    bool didEncoderPassSelfTest(uint8_t encoder_index);

    /** @brief Returns the cached pass result for DN1..DN5 using index 0..4. */
    bool didSurfaceSensorPassSelfTest(uint8_t sensor_index);


    /** @brief Returns the left encoder count from the cached encoder snapshot. */
    int32_t getLeftEncoderCount();

    /** @brief Returns the right encoder count from the cached encoder snapshot. */
    int32_t getRightEncoderCount();

    /** @brief Returns the cached left motor PWM value used for telemetry. */
    int16_t getLeftMotorPWM();

    /** @brief Returns the cached right motor PWM value used for telemetry. */
    int16_t getRightMotorPWM();

    /**
     * @brief Stores and sends a team name through the robot I2C interface.
     * @param teamname_in Null-terminated team name.
     * @return true when the name is valid and both I2C writes are acknowledged.
     * @note Names must be non-empty printable ASCII. Long names are truncated
     * to fit RobotTeamname_t.
     */
    bool setTeamname(const char* teamname_in);

    /**
     * @brief Sends requested PWM values to the left and right motor channels.
     * @param l Requested left-motor PWM value.
     * @param r Requested right-motor PWM value.
     * @return true when both I2C writes are acknowledged.
     * @note This method sends an I2C request. It does not guarantee immediate
     * wheel-speed response, which depends on the physical and middleware system.
     * If a helper motion is running, this newer motor command cancels it.
     */
    bool setMotorPWM(float l, float r);

    /**
     * @brief Sends a requested pose value to the robot interface.
     * @param x Requested x coordinate.
     * @param y Requested y coordinate.
     * @param theta Requested heading.
     * @return true when both I2C writes are acknowledged.
     * @note This is useful for controlled demonstrations; it is not a physical
     * measurement or calibration operation.
     */
    bool setPose(float x, float y, float theta);

    /**
     * @brief Sends one buzzer-tone request to the robot.
     * @param freq Tone frequency.
     * @param duration_ms Tone duration in milliseconds.
     * @return true when both I2C writes are acknowledged.
     */
    bool playTone(uint16_t freq, uint16_t duration_ms);

    /**
     * @brief Sets the StampC3's on-board RGB LED colour and brightness.
     * @param r Red component from 0 to 255.
     * @param g Green component from 0 to 255.
     * @param b Blue component from 0 to 255.
     * @param brightness Overall brightness from 0 to 255.
     * @note Pass values within 0..255. Arguments are converted to uint8_t before
     * the function runs, so its internal constrain calls cannot clamp an
     * out-of-range caller value such as 300 to 255. The low-level implementation
     * is adapted from the Freenove WS2812 library included with this template.
     */
    void setLED(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);

  private:
    unsigned long button_debounce_ts;
    const unsigned long debounce_delay_ms = 250;

    // Adapted from Freenove_WS2812_Lib_for_ESP32 to avoid a first-session
    // library-installation requirement for the single on-board RGB LED.
    uint8_t LED_rOffset;
    uint8_t LED_gOffset;
    uint8_t LED_bOffset;
    rmt_data_t led_data[24];

    /** @brief Selects one middleware register and verifies its acknowledgement. */
    bool selectRegister(RobotRegister_t request);

    /** @brief Sends a packed payload only after its register was selected. */
    // These private templates are defined and used only in Robot.cpp.
    template<typename Payload_t>
    bool writeRegisterPayload(RobotRegister_t request, const Payload_t& payload);

    /** @brief Requests one exact packed snapshot without altering a cache on failure. */
    template<typename Snapshot_t>
    bool requestSnapshot(RobotRegister_t request, Snapshot_t& snapshot);

    /** @brief Sends a packed helper-motion payload after selecting its register. */
    template<typename MotionRequest_t>
    bool sendMotionRequest(RobotRegister_t request, const MotionRequest_t& request_data);

    /** @brief Configures the on-board RGB LED transport and initial status colour. */
    bool setupLED();

    /** @brief Transmits the prepared RGB LED bit pattern. */
    esp_err_t updateLED();
};

#endif
