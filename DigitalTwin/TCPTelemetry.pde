/**
 * @brief One timestamped CSV telemetry record received from the StampC3.
 * @details The expected record has 14 comma-separated fields in the order
 * timestamp, pose, requested PWM, encoders, five surface readings, and signal.
 */
class TCPTelemetry_c {
  long timestampMs = -1;
  float x, y, theta;
  float pwmLeft, pwmRight;
  float encoderLeft, encoderRight;
  float[] sensorReadings = new float[5];
  int signal = 0;
  int receivedAtMs = -1;

  /**
   * @brief Parses and stores one complete CSV telemetry record.
   * @param line Input line without an assumed trailing newline.
   * @param arrivalMs Local Processing time at receipt, in ms.
   * @return True only when all 14 fields are present and numeric.
   */
  boolean parse(String line, int arrivalMs) {
    String[] fields = split(line, ',');
    if (fields.length != 14) return false;
    float[] values = new float[14];
    for (int i = 0; i < fields.length; i++) {
      values[i] = parseFloat(trim(fields[i]));
      if (Float.isNaN(values[i])) return false;
    }
    timestampMs = (long)values[0];
    x = values[1]; y = values[2]; theta = values[3];
    pwmLeft = values[4]; pwmRight = values[5];
    encoderLeft = values[6]; encoderRight = values[7];
    for (int i = 0; i < 5; i++) sensorReadings[i] = values[8 + i];
    signal = (int)values[13];
    receivedAtMs = arrivalMs;
    return true;
  }

  /** @return True if this sample arrived less than 100 ms ago. */
  boolean isRecent(int nowMs) { return receivedAtMs >= 0 && nowMs - receivedAtMs < 100; }

  /** @return An independent copy suitable for a buffered live comparison. */
  TCPTelemetry_c copy() {
    TCPTelemetry_c result = new TCPTelemetry_c();
    result.timestampMs = timestampMs; result.x = x; result.y = y; result.theta = theta;
    result.pwmLeft = pwmLeft; result.pwmRight = pwmRight;
    result.encoderLeft = encoderLeft; result.encoderRight = encoderRight;
    for (int i = 0; i < 5; i++) result.sensorReadings[i] = sensorReadings[i];
    result.signal = signal; result.receivedAtMs = receivedAtMs;
    return result;
  }
}
