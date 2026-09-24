/**
 * @brief Owns the standalone application: timing, robot model, view, and TCP.
 * @details World positions use millimetres, with +x forward/right and +y
 * upward. Rendering may run at a variable frame rate; model/controller
 * updates use a fixed 10 ms interval.
 */
class Simulator {
  /** @brief Number of milliseconds visible in each telemetry graph. */
  final int graphWindowMs = 5000;
  final int graphSamplePeriodMs = 20;
  // Mirrors the physical robot's 10 ms update interval.
  final int simulationUpdateIntervalMs = 10;
  final int maximumCatchUpSteps = 10;
  /** @brief Default StampC3 access-point address for live comparison. */
  final String tcpHost = "192.168.4.1";
  final int tcpPort = 80;
  final int tcpRetryPeriodMs = 2000;
  final int tcpConnectTimeoutMs = 250;
  final float minimumZoom = 0.25;
  final float maximumZoom = 4.0;
  final int SANDBOX = 0, LIVE_COMPARISON = 1;

  /** @brief Surface-image centre in world coordinates, in mm. */
  final float surfaceOffsetX;
  final float surfaceOffsetY;

  PImage surfaceImage;
  // Cache only the currently visible part of the 1 px/mm map. This avoids
  // redrawing the complete source image at 1:1 scale.
  PImage surfaceViewportCache;
  int surfaceCacheX = -1;
  int surfaceCacheY = -1;
  int surfaceCacheWidth = 0;
  int surfaceCacheHeight = 0;
  PApplet applet;
  /** @brief Robot-only Digital Twin model; the surface image is its environment. */
  RobotSimulation_c robot;
  Controller_c controller;
  TimeSeriesGraph motorGraph;
  TimeSeriesGraph sensorGraph;
  TimeSeriesGraph encoderGraph;

  boolean showTelemetry = false;
  boolean trackRobot = false;
  int simulatorMode = SANDBOX;
  boolean liveArmed = false;
  boolean liveRunning = false;
  int previousTcpSignal = 0;
  long liveStartTcpMs = -1;
  long liveStartSimulationMs = -1;
  long lastTcpTimestampMs = -1;
  TCPTelemetry_c armedPose = new TCPTelemetry_c();
  TCPTelemetry_c ghost = new TCPTelemetry_c();
  ArrayList<TCPTelemetry_c> tcpQueue = new ArrayList<TCPTelemetry_c>();
  float zoom = 1.0;
  float viewCentreX = 0.0;
  float viewCentreY = 0.0;

  int lastGraphSampleMs = 0;
  int lastSimulationUpdateMs = 0;
  int simulationAccumulatorMs = 0;
  int lastTcpAttemptMs = -tcpRetryPeriodMs;

  Client telemetryClient;
  TCPTelemetry_c tcpTelemetry = new TCPTelemetry_c();
  String tcpStatus = "TCP: not connected";
  String lastTcpLine = "";
  volatile boolean tcpAttemptInProgress = false;
  volatile boolean tcpAttemptFailed = false;
  volatile Socket pendingTcpSocket = null;

  /**
   * @brief Creates the supplied baseline Digital Twin application.
   * @param appletIn Parent Processing application used for networking.
   * @param surfaceOffsetXIn Surface-image x offset in world millimetres.
   * @param surfaceOffsetYIn Surface-image y offset in world millimetres.
   */
  Simulator(PApplet appletIn, float surfaceOffsetXIn, float surfaceOffsetYIn) {
    applet = appletIn;
    surfaceOffsetX = surfaceOffsetXIn;
    surfaceOffsetY = surfaceOffsetYIn;
    ellipseMode(RADIUS);
    strokeJoin(ROUND);
    surfaceImage = loadSurfaceImage();
    robot = new RobotSimulation_c(16.0, 90.00, surfaceImage,
      surfaceOffsetX, surfaceOffsetY);
    controller = new Controller_c();
    controller.setSignal(1); // Sandbox starts immediately.
    lastSimulationUpdateMs = millis();

    motorGraph = new TimeSeriesGraph("Motor PWM", graphWindowMs, true);
    motorGraph.addSeries("Sim L", color(220, 70, 70));
    motorGraph.addSeries("Sim R", color(70, 100, 220));
    motorGraph.addSeries("TCP L", color(145, 60, 190));
    motorGraph.addSeries("TCP R", color(20, 150, 150));
    sensorGraph = new TimeSeriesGraph("Surface sensors", graphWindowMs, false);
    for (int i = 0; i < 5; i++) sensorGraph.addSeries("Sim " + (i + 1), sensorColour(i));
    for (int i = 0; i < 5; i++) sensorGraph.addSeries("TCP " + (i + 1), tcpSensorColour(i));
    encoderGraph = new TimeSeriesGraph("Encoder counts", graphWindowMs, false);
    encoderGraph.addSeries("Sim L", color(220, 70, 70));
    encoderGraph.addSeries("Sim R", color(70, 100, 220));
    encoderGraph.addSeries("TCP L", color(145, 60, 190));
    encoderGraph.addSeries("TCP R", color(20, 150, 150));

  }

  /** @brief Updates TCP, advances fixed-step model time, and records graphs. */
  void update() {
    updateTcpConnection();
    updateRobotModel();

    if (trackRobot) {
      viewCentreX = robot.odometry.pose.x;
      viewCentreY = robot.odometry.pose.y;
    }
    recordTelemetry();
  }

  /**
   * @brief Accumulates elapsed wall-clock time into fixed model updates.
   * @note Long pauses are capped to prevent an unbounded catch-up simulation.
   */
  void updateRobotModel() {
    int now = millis();
    int elapsedMs = now - lastSimulationUpdateMs;
    lastSimulationUpdateMs = now;

    // Do not try to replay an unbounded amount of time after a breakpoint or
    // long application pause.
    elapsedMs = min(elapsedMs, simulationUpdateIntervalMs * maximumCatchUpSteps);
    simulationAccumulatorMs += elapsedMs;

    int steps = 0;
    while (simulationAccumulatorMs >= simulationUpdateIntervalMs
      && steps < maximumCatchUpSteps) {
      robot.advanceTime(simulationUpdateIntervalMs);
      updateLiveClock();
      controller.update(robot);
      robot.updatePhysics(simulationUpdateIntervalMs);
      simulationAccumulatorMs -= simulationUpdateIntervalMs;
      steps++;
    }
  }

  /** @brief Advances the physical-robot ghost only to matching telemetry time. */
  void updateLiveClock() {
    if (simulatorMode != LIVE_COMPARISON || !liveRunning) return;
    long targetTcpTime = liveStartTcpMs + (robot.getMillis() - liveStartSimulationMs);
    while (tcpQueue.size() > 0 && tcpQueue.get(0).timestampMs <= targetTcpTime) {
      ghost = tcpQueue.remove(0);
    }
  }

  /** @brief Renders the surface, simulated robot, optional ghost, and UI. */
  void draw() {

    background(180);
    updateSurfaceViewportCache();

    pushMatrix();
    translate(width / 2.0, height / 2.0);
    scale(zoom, -zoom);
    translate(-viewCentreX, -viewCentreY);
    imageMode(CENTER);
    drawSurfaceViewportCache();
    robot.draw();
    if (simulatorMode == LIVE_COMPARISON && ghost.timestampMs >= 0) drawGhost();

    popMatrix();

    drawStatus();
    if (showTelemetry) drawTelemetryPanel();
  }

  void updateSurfaceViewportCache() {
    // Convert the visible world rectangle to source-image pixels. The image
    // is centred on the configured offset, and its y axis is opposite the
    // world y axis.
    float halfViewWidth = width / (2.0 * zoom);
    float halfViewHeight = height / (2.0 * zoom);
    int visibleX0 = floor(surfaceImage.width / 2.0
      + viewCentreX - surfaceOffsetX - halfViewWidth);
    int visibleX1 = ceil(surfaceImage.width / 2.0
      + viewCentreX - surfaceOffsetX + halfViewWidth);
    int visibleY0 = floor(surfaceImage.height / 2.0
      - viewCentreY + surfaceOffsetY - halfViewHeight);
    int visibleY1 = ceil(surfaceImage.height / 2.0
      - viewCentreY + surfaceOffsetY + halfViewHeight);

    visibleX0 = constrain(visibleX0, 0, surfaceImage.width);
    visibleX1 = constrain(visibleX1, 0, surfaceImage.width);
    visibleY0 = constrain(visibleY0, 0, surfaceImage.height);
    visibleY1 = constrain(visibleY1, 0, surfaceImage.height);
    if (visibleX1 <= visibleX0 || visibleY1 <= visibleY0) {
      surfaceViewportCache = null;
      return;
    }

    boolean cacheContainsView = surfaceViewportCache != null
      && visibleX0 >= surfaceCacheX
      && visibleY0 >= surfaceCacheY
      && visibleX1 <= surfaceCacheX + surfaceCacheWidth
      && visibleY1 <= surfaceCacheY + surfaceCacheHeight;
    if (cacheContainsView) return;

    // The margin lets the robot move or the view pan a little before a new
    // crop is made. It keeps cache creation out of the normal draw path.
    int cacheMargin = ceil(constrain(max(halfViewWidth, halfViewHeight) * 0.15,
      40, 250));
    surfaceCacheX = max(0, visibleX0 - cacheMargin);
    surfaceCacheY = max(0, visibleY0 - cacheMargin);
    int cacheRight = min(surfaceImage.width, visibleX1 + cacheMargin);
    int cacheBottom = min(surfaceImage.height, visibleY1 + cacheMargin);
    surfaceCacheWidth = cacheRight - surfaceCacheX;
    surfaceCacheHeight = cacheBottom - surfaceCacheY;
    surfaceViewportCache = surfaceImage.get(surfaceCacheX, surfaceCacheY,
      surfaceCacheWidth, surfaceCacheHeight);
  }

  void drawSurfaceViewportCache() {
    if (surfaceViewportCache == null) return;

    float worldX = surfaceOffsetX + surfaceCacheX + surfaceCacheWidth / 2.0
      - surfaceImage.width / 2.0;
    float worldY = surfaceOffsetY + surfaceImage.height / 2.0
      - (surfaceCacheY + surfaceCacheHeight / 2.0);
    image(surfaceViewportCache, worldX, worldY);
  }

  void drawGhost() {
    float alpha = millis() - ghost.receivedAtMs > 100 ? 128 : 255;
    pushStyle(); pushMatrix();
    translate(ghost.x, ghost.y); rotate(ghost.theta);
    fill(235, 125, 35, alpha); stroke(180, 70, 15, alpha);
    ellipse(0, 0, 50, 50); line(0, 0, 36, 0);
    popMatrix(); popStyle();
  }

  /**
   * @brief Handles the documented keyboard controls.
   * @param pressedKey Character key reported by Processing.
   * @param pressedKeyCode Processing key code for arrow-key panning.
   */
  void handleKey(char pressedKey, int pressedKeyCode) {
    if (pressedKey == 'g' || pressedKey == 'G') showTelemetry = !showTelemetry;
    if (pressedKey == 't' || pressedKey == 'T') trackRobot = !trackRobot;
    if ((pressedKey == 'm' || pressedKey == 'M') && !liveRunning) toggleMode();
    if (pressedKey == '+' || pressedKey == '=') zoom = min(maximumZoom, zoom * 1.2);
    if (pressedKey == '-') zoom = max(minimumZoom, zoom / 1.2);

    if (pressedKey == CODED) {
      // Arrow-key panning intentionally disables tracking so the requested
      // view remains visible after the key press.
      float panStepMm = 50.0 / zoom;
      if (pressedKeyCode == LEFT) panBy(-panStepMm, 0);
      if (pressedKeyCode == RIGHT) panBy(panStepMm, 0);
      if (pressedKeyCode == UP) panBy(0, panStepMm);
      if (pressedKeyCode == DOWN) panBy(0, -panStepMm);
    }
  }

  /** @brief Switches between standalone sandbox and live-comparison modes. */
  void toggleMode() {
    simulatorMode = simulatorMode == SANDBOX ? LIVE_COMPARISON : SANDBOX;
    resetLiveSession();
    if (simulatorMode == SANDBOX) controller.setSignal(1);
  }

  void panBy(float deltaX, float deltaY) {
    trackRobot = false;
    viewCentreX += deltaX;
    viewCentreY += deltaY;
  }

  void drawStatus() {
    int margin = uiMargin();
    int contentWidth = statusWidth();
    float lineHeight = max(18, 16 * uiScale());

    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(255, 255, 255, 205);
    rect(margin - 6, margin - 6, contentWidth + 12, lineHeight * 4 + 12, 5);
    fill(30);
    textSize(max(12, 13 * uiScale()));
    text("Robot pose: " + nf(robot.odometry.pose.x, 0, 1) + ", "
      + nf(robot.odometry.pose.y, 0, 1) + ", " + nf(robot.odometry.pose.theta, 0, 2),
      margin, margin + lineHeight, contentWidth, lineHeight);
    text("+/-: zoom   Arrows: pan   T: tracking "
      + (trackRobot ? "on" : "off") + "   G: telemetry   M: mode",
      margin, margin + lineHeight * 2, contentWidth, lineHeight);
    text(tcpStatus, margin, margin + lineHeight * 3, contentWidth, lineHeight);
    text(liveStatus(), margin, margin + lineHeight * 4, contentWidth, lineHeight);
    popStyle();
  }

  String liveStatus() {
    if (simulatorMode == SANDBOX) return "Mode: Sandbox";
    if (telemetryClient == null) return "Mode: Live comparison — Offline";
    if (!liveArmed) return "Mode: Live comparison — Awaiting fresh signal 0";
    if (!liveRunning) return "Mode: Live comparison — Armed";
    return "Mode: Live comparison — Running; ghost = robot-reported odometry";
  }

  /** @brief Adds current simulation and, when recent, TCP values to graphs. */
  void recordTelemetry() {
    int now = millis();
    if (now - lastGraphSampleMs < graphSamplePeriodMs) return;
    lastGraphSampleMs = now;
    float[] simulatedSensors = robot.sensorReadings();
    if (tcpTelemetry.isRecent(now)) {
    motorGraph.addSample(now, new float[] {robot.motors.left.pwm, robot.motors.right.pwm,
        tcpTelemetry.pwmLeft, tcpTelemetry.pwmRight});
      sensorGraph.addSample(now, combine(simulatedSensors, tcpTelemetry.sensorReadings));
      encoderGraph.addSample(now, new float[] {robot.odometry.encoders.left,
        robot.odometry.encoders.right, tcpTelemetry.encoderLeft, tcpTelemetry.encoderRight});
    } else {
      motorGraph.addSample(now, new float[] {
        robot.motors.left.pwm, robot.motors.right.pwm
      });
      sensorGraph.addSample(now, simulatedSensors);
      encoderGraph.addSample(now, new float[] {
        robot.odometry.encoders.left, robot.odometry.encoders.right
      });
    }
  }

  float[] combine(float[] first, float[] second) {
    float[] combined = new float[first.length + second.length];
    for (int i = 0; i < first.length; i++) combined[i] = first[i];
    for (int i = 0; i < second.length; i++) combined[first.length + i] = second[i];
    return combined;
  }

  void drawTelemetryPanel() {
    int margin = uiMargin();
    int panelWidth = telemetryPanelWidth();
    int panelX = width - panelWidth;
    int graphWidth = panelWidth - 2 * margin;
    float titleHeight = max(28, 24 * uiScale());
    float graphGap = margin;
    float graphHeight = (height - titleHeight - margin * 4 - graphGap * 2) / 3.0;

    // The panel deliberately fills the available height. This keeps graph
    // backgrounds attached to their plots after a resize, including when the
    // window is made smaller than its normal working size.
    graphHeight = max(1, graphHeight);
    float graphY = titleHeight + margin;
    pushStyle();
    rectMode(CORNER);
    noStroke();
    fill(248, 248, 248, 235);
    rect(panelX, 0, panelWidth, height);
    fill(30);
    textSize(max(12, 13 * uiScale()));
    text("Telemetry (press G to hide)", panelX + margin, titleHeight - margin / 2.0);
    motorGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    graphY += graphHeight + graphGap;
    sensorGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    graphY += graphHeight + graphGap;
    encoderGraph.draw(panelX + margin, graphY, graphWidth, graphHeight);
    popStyle();
  }

  float uiScale() {
    return constrain(min(width / 1000.0, height / 800.0), 0.85, 1.5);
  }

  int uiMargin() {
    return max(10, round(12 * uiScale()));
  }

  int telemetryPanelWidth() {
    int availableWidth = max(180, width - 2 * uiMargin());
    int preferredWidth = round(width * 0.32);
    return min(max(260, preferredWidth), availableWidth);
  }

  int statusWidth() {
    int availableRight = showTelemetry ? width - telemetryPanelWidth() : width;
    return max(1, availableRight - 2 * uiMargin());
  }

  color sensorColour(int index) {
    color[] colours = {color(160, 60, 60), color(210, 120, 50),
      color(80, 150, 80), color(60, 120, 190), color(140, 70, 170)};
    return colours[index];
  }

  color tcpSensorColour(int index) {
    color[] colours = {color(225, 110, 110), color(245, 180, 70),
      color(105, 190, 105), color(80, 160, 230), color(185, 100, 210)};
    return colours[index];
  }

  /**
   * @brief Loads surface.png or creates the supplied A3 circular-line fallback.
   * @return A 1 pixel/mm surface image positioned using the configured offset.
   */
  PImage loadSurfaceImage() {
    PImage image = loadImage("surface.png");
    if (image != null && image.width > 0 && image.height > 0) return image;

    // A3 landscape is 420 x 297 mm. At 1 px/mm this generated test surface
    // has the same physical dimensions as the printed student sheet.
    image = createImage(420, 297, RGB);
    image.loadPixels();
    float circleRadius = 125.0;
    float lineWidth = 19.0;
    for (int py = 0; py < image.height; py++) {
      for (int px = 0; px < image.width; px++) {
        float dx = px - image.width / 2.0;
        float dy = py - image.height / 2.0;
        float distanceFromCentre = sqrt(dx * dx + dy * dy);
        boolean onLine = abs(distanceFromCentre - circleRadius) <= lineWidth / 2.0;
        image.pixels[py * image.width + px] = onLine ? color(0) : color(255);
      }
    }
    image.updatePixels();
    return image;
  }

  /** @brief Maintains the TCP connection and consumes bounded CSV input. */
  void updateTcpConnection() {
    int now = millis();
    if (telemetryClient != null && !telemetryClient.active()) {
      telemetryClient.stop();
      telemetryClient = null;
      tcpStatus = "TCP: disconnected; retrying";
      resetLiveSession();
    }

    if (pendingTcpSocket != null) {
      Socket connectedSocket = pendingTcpSocket;
      pendingTcpSocket = null;
      tcpAttemptInProgress = false;
      try {
        telemetryClient = new Client(applet, connectedSocket);
        tcpStatus = "TCP: connected to " + tcpHost + ":" + tcpPort;
      }
      catch (Exception error) {
        telemetryClient = null;
        tcpStatus = "TCP: unavailable; retrying";
        closeSocket(connectedSocket);
      }
    }

    if (tcpAttemptFailed) {
      tcpAttemptFailed = false;
      tcpAttemptInProgress = false;
      tcpStatus = "TCP: unavailable; retrying";
    }

    if (telemetryClient == null && !tcpAttemptInProgress
      && now - lastTcpAttemptMs >= tcpRetryPeriodMs) {
      beginTcpConnectionAttempt();
    }

    int linesRead = 0;
    while (telemetryClient != null && telemetryClient.available() > 0 && linesRead < 20) {
      String line = telemetryClient.readStringUntil('\n');
      if (line == null) break;
      lastTcpLine = trim(line);
      if (tcpTelemetry.parse(lastTcpLine, now)) handleTcpSample(tcpTelemetry);
      linesRead++;
    }
  }

  /**
   * @brief Uses a parsed physical-robot record for graphing and live alignment.
   * @param sample Newly received StampC3 telemetry record.
   */
  void handleTcpSample(TCPTelemetry_c sample) {
    if (lastTcpTimestampMs >= 0 && sample.timestampMs < lastTcpTimestampMs) {
      resetLiveSession();
    }
    lastTcpTimestampMs = sample.timestampMs;
    TCPTelemetry_c copy = sample.copy();
    if (simulatorMode != LIVE_COMPARISON) return;

    if (liveRunning) tcpQueue.add(copy);

    if (sample.signal == 0) {
      armedPose = sample.copy();
      liveArmed = true;
    }
    if (liveArmed && !liveRunning && previousTcpSignal == 0 && sample.signal == 1) {
      controller.reset();
      controller.setSignal(1);
      robot.resetForRun(armedPose.x, armedPose.y, armedPose.theta,
        armedPose.encoderLeft, armedPose.encoderRight);
      liveStartTcpMs = sample.timestampMs;
      liveStartSimulationMs = robot.getMillis();
      ghost = armedPose.copy();
      liveRunning = true;
      tcpQueue.clear();
    }
    previousTcpSignal = sample.signal;
  }

  /** @brief Clears buffered TCP state and disarms the live-comparison run. */
  void resetLiveSession() {
    tcpQueue.clear();
    liveArmed = false;
    liveRunning = false;
    previousTcpSignal = 0;
    lastTcpTimestampMs = -1;
    ghost = new TCPTelemetry_c();
    if (simulatorMode == LIVE_COMPARISON) controller.reset();
  }

  /** @brief Starts a short asynchronous connection attempt to the StampC3. */
  void beginTcpConnectionAttempt() {
    lastTcpAttemptMs = millis();
    tcpAttemptInProgress = true;
    tcpStatus = "TCP: connecting to " + tcpHost + ":" + tcpPort;
    new Thread(new Runnable() {
      public void run() {
        connectTcpInBackground();
      }
    }).start();
  }

  void connectTcpInBackground() {
    Socket socket = new Socket();
    try {
      socket.connect(new InetSocketAddress(tcpHost, tcpPort), tcpConnectTimeoutMs);
      pendingTcpSocket = socket;
    }
    catch (Exception error) {
      closeSocket(socket);
      tcpAttemptFailed = true;
    }
  }

  void closeSocket(Socket socket) {
    try {
      socket.close();
    }
    catch (Exception ignored) {
    }
  }
}
