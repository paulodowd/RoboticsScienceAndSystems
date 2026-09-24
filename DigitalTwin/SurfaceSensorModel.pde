/** @brief Cached five-channel surface-sensor data exposed to the controller. */
class RobotSurface_c {
  float[] reading = {400, 400, 400, 400, 400};
}

/**
 * @brief Ideal image-sampling model for one downward-facing surface sensor.
 * @details Local coordinates are in mm in the robot frame. The source image
 * is treated as 1 pixel per mm, centred on its configured world offset, with
 * image y inverted relative to world y.
 */
class SurfaceSensorModel_c {
  String name;
  float localX;
  float localY;

  /**
   * @param nameIn Human-readable channel name, such as DN1.
   * @param radiusMm Sensor distance from the robot centre, in mm.
   * @param angleDegrees Sensor angle in the robot frame, in degrees.
   */
  SurfaceSensorModel_c(String nameIn, float radiusMm, float angleDegrees) {
    name = nameIn;
    localX = radiusMm * cos(radians(angleDegrees));
    localY = radiusMm * sin(radians(angleDegrees));
  }

  /**
   * @brief Converts an image brightness sample into an ideal sensor reading.
   * @param image Surface image, centred on the supplied world offset.
   * @param surfaceOffsetX Surface-image x offset in world millimetres.
   * @param surfaceOffsetY Surface-image y offset in world millimetres.
   * @param robotX Robot x position in mm.
   * @param robotY Robot y position in mm.
   * @param robotTheta Robot heading in radians.
   * @return 2000 for black, 400 for white or a location outside the image.
   */
  float sample(PImage image, float surfaceOffsetX, float surfaceOffsetY,
    float robotX, float robotY, float robotTheta) {
    float worldX = robotX + localX * cos(robotTheta) - localY * sin(robotTheta);
    float worldY = robotY + localX * sin(robotTheta) + localY * cos(robotTheta);
    int pixelX = floor(image.width / 2.0 + worldX - surfaceOffsetX);
    int pixelY = floor(image.height / 2.0 - worldY + surfaceOffsetY);
    if (pixelX < 0 || pixelX >= image.width || pixelY < 0 || pixelY >= image.height) {
      return 400;
    }
    return map(brightness(image.get(pixelX, pixelY)), 0, 255, 2000, 400);
  }

  /** @brief Draws a small sensor marker in the robot frame.
   * @param reading Current ideal sensor reading, used to set marker shade.
   */
  void draw(float reading) {
    pushStyle();
    fill(map(reading, 400, 2000, 220, 20));
    stroke(20);
    rectMode(CENTER);
    rect(localX, localY, 3, 3);
    popStyle();
  }
}
