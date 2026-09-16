/**
 * @brief Rolling multi-series graph used by the simulator overlay.
 * @details Samples are retained only for the configured time window.
 */
class TimeSeriesGraph {
  String title;
  int windowMs;
  boolean signedData;
  ArrayList<GraphSeries> series = new ArrayList<GraphSeries>();

  /**
   * @param titleIn Graph title.
   * @param windowMsIn Retained history duration in ms.
   * @param signedDataIn True to draw a symmetric range around zero.
   */
  TimeSeriesGraph(String titleIn, int windowMsIn, boolean signedDataIn) {
    title = titleIn;
    windowMs = windowMsIn;
    signedData = signedDataIn;
  }

  /** @brief Adds one labelled data series to the graph. */
  void addSeries(String name, color lineColour) {
    series.add(new GraphSeries(name, lineColour));
  }

  /**
   * @brief Adds one value per configured series and discards expired points.
   * @param timestampMs Sample time in local Processing milliseconds.
   * @param values Values in the same order as the added series.
   */
  void addSample(int timestampMs, float[] values) {
    int count = min(values.length, series.size());
    for (int i = 0; i < count; i++) series.get(i).add(timestampMs, values[i]);
    int cutoff = timestampMs - windowMs;
    for (GraphSeries s : series) s.removeBefore(cutoff);
  }

  /** @brief Draws the graph in screen coordinates. */
  void draw(float x, float y, float w, float h) {
    pushStyle();
    rectMode(CORNER);
    fill(255);
    stroke(130);
    strokeWeight(1);
    rect(x, y, w, h);

    float maximum = signedData ? maximumAbsValue() : maximumValue();
    if (maximum <= 0.0) maximum = 1.0;
    float minimum = signedData ? -maximum : 0.0;
    float left = x + 42;
    float right = x + w - 8;
    float top = y + 24;
    // Sensor telemetry has ten series. Use more columns when a student has
    // made the window short, preserving useful room for the plotted data.
    int legendColumns = h < 165 ? 5 : 3;
    int legendRows = ceil(series.size() / (float) legendColumns);
    float legendHeight = legendRows * 13;
    float bottom = y + h - 22 - legendHeight;

    fill(30);
    text(title, x + 8, y + 16);
    text(nf(maximum, 0, 1), x + 5, top + 5);
    text(nf(minimum, 0, 1), x + 5, bottom);
    stroke(215);
    line(left, top, right, top);
    line(left, bottom, right, bottom);
    if (minimum < 0.0) {
      float zeroY = map(0, minimum, maximum, bottom, top);
      line(left, zeroY, right, zeroY);
      fill(80);
      text("0", x + 18, zeroY + 4);
    }

    int newest = newestTimestamp();
    int oldest = newest - windowMs;
    for (GraphSeries s : series) {
      stroke(s.lineColour);
      strokeWeight(2);
      noFill();
      beginShape();
      for (GraphPoint p : s.points) {
        float px = map(p.timestampMs, oldest, newest, left, right);
        float py = map(constrain(p.value, minimum, maximum),
          minimum, maximum, bottom, top);
        vertex(px, py);
      }
      endShape();
    }

    drawLegend(left, bottom + 13, right, legendColumns);
    fill(30);
    text("0 s", left, y + h - 5);
    text(nf(windowMs / 1000.0, 0, 1) + " s", right - 25, y + h - 5);
    popStyle();
  }

  void drawLegend(float left, float y, float right, int columns) {
    float columnWidth = (right - left) / columns;
    textSize(10);
    for (int i = 0; i < series.size(); i++) {
      int column = i % columns;
      int row = i / columns;
      float labelX = left + column * columnWidth;
      float labelY = y + row * 13;
      GraphSeries s = series.get(i);
      stroke(s.lineColour);
      strokeWeight(3);
      line(labelX, labelY - 3, labelX + 10, labelY - 3);
      fill(30);
      text(s.name, labelX + 14, labelY);
    }
  }

  float maximumValue() {
    float result = 0.0;
    for (GraphSeries s : series)
      for (GraphPoint p : s.points) result = max(result, p.value);
    return result;
  }

  float maximumAbsValue() {
    float result = 0.0;
    for (GraphSeries s : series)
      for (GraphPoint p : s.points) result = max(result, abs(p.value));
    return result;
  }

  int newestTimestamp() {
    for (GraphSeries s : series)
      if (s.points.size() > 0)
        return s.points.get(s.points.size() - 1).timestampMs;
    return millis();
  }
}

/** @brief One labelled, coloured sequence of graph points. */
class GraphSeries {
  String name;
  color lineColour;
  ArrayList<GraphPoint> points = new ArrayList<GraphPoint>();

  GraphSeries(String nameIn, color lineColourIn) {
    name = nameIn;
    lineColour = lineColourIn;
  }

  /** @brief Appends one timestamped value. */
  void add(int timestampMs, float value) {
    points.add(new GraphPoint(timestampMs, value));
  }

  void removeBefore(int cutoff) {
    while (points.size() > 0 && points.get(0).timestampMs < cutoff)
      points.remove(0);
  }
}

/** @brief One graph sample. */
class GraphPoint {
  int timestampMs;
  float value;

  GraphPoint(int timestampMsIn, float valueIn) {
    timestampMs = timestampMsIn;
    value = valueIn;
  }
}
