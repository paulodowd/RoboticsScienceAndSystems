# Benchmarking TODO

Measure what the platform can actually do before relying on it in the final project.
Each item should end with a number (with spread), the method used, and where the raw data lives.

**Conventions**
- Raw data and scripts go in `bench/<item>/` (CSV + the script/sketch that produced it).
- Report distributions, not single values: median, p95, p99, max, and sample count.
- Record conditions: robot ID, battery state, WiFi distance/obstacles, firmware commit.
- Tick an item only when its result is written in the **Results** column.

---

## 1. Communication

| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 1.1 | **TCP round-trip latency** (laptop ↔ StampC3) | Laptop sends timestamped ping over TCP; StampC3 echoes immediately from `loop()`. Measure RTT distribution idle, and with the controller + telemetry running. One-way ≈ RTT/2. | |
| [ ] 1.2 | **TCP throughput, robot → laptop** | Stream fixed-size records as fast as possible; vary record size (CSV line vs packed binary via `sendBuffer`). Measure sustained bytes/s and records/s. | |
| [ ] 1.3 | **TCP throughput, laptop → robot** | Same, reversed; check the StampC3 keeps up without stalling `loop()`. | |
| [ ] 1.4 | **Cost of sending on the robot** | Time `server.printf` / `sendBuffer` calls with `micros()`. Does `write()` block when the laptop is slow or out of range? | |
| [ ] 1.5 | **WiFi range and dropouts** | Latency/throughput vs distance and with other robots' APs nearby; time to detect disconnect and reconnect. | |
| [ ] 1.6 | **USB Serial baseline** | Repeat 1.1–1.2 over Serial at 115200 (and higher baud) for comparison. | |
| [ ] 1.7 | **I2C transaction time** (StampC3 ↔ 3Pi+) | Time each `Robot_c` call (`getSurfaceSensors`, `getEncoders`, `getPose`, `setMotorPWM`) with `micros()`. Record failure rate at 400 kHz. | |
| [ ] 1.8 | **Maximum control-loop rate** | Remove the 10 ms `TaskTimer_c` gate; measure achievable cycle rate with a full sense → act cycle. | |

## 2. End-to-end latency and timing

| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 2.1 | **Remote command → wheel motion** | Laptop sends a motor command over TCP; robot timestamps receipt, I2C send, and first encoder change. Split into: network, StampC3 processing, I2C, motor/encoder response. | |
| [ ] 2.2 | **Sensor event → motor command** (on-board) | Move a black/white edge under a surface sensor; timestamp first reading change and resulting `setMotorPWM`. | |
| [ ] 2.3 | **Control-loop period and jitter** | Log `micros()` at each accepted `Controller_c::update`; histogram of actual period vs nominal 10 ms, with and without WiFi client. | |
| [ ] 2.4 | **Data freshness from the middleware** | Compare `surface.timestamp_us` across consecutive reads: how often does the 3Pi+ publish new data vs return a repeated snapshot? | |
| [ ] 2.5 | **Motor step response** | Step PWM 0 → N; log encoder counts at max rate. Extract dead time and time constant per wheel. | |
| [ ] 2.6 | **PWM → wheel speed curve** | Sweep PWM; steady-state speed per wheel. Find deadband (minimum PWM that moves), linearity, left/right asymmetry. | |
| [ ] 2.7 | **Motion-helper accuracy** | `startMoveDistance` / `startRotateAngle` for a range of targets; measure physically. Also time until `MOTION_STATUS_COMPLETE`. | |

## 3. Sensors

### Surface (line) sensors DN1–DN5
| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 3.1 | **Static noise** | Robot stationary on white, then on black: mean and std dev per sensor over ≥ 1000 samples. | |
| [ ] 3.2 | **Range and per-sensor offsets** | Min/max per sensor on the course surface; derive per-sensor normalisation. | |
| [ ] 3.3 | **Line threshold validity** | Is `LINE_THRESHOLD = 1400` separable for every sensor, under different room lighting? | |
| [ ] 3.4 | **Spatial response** | Slide an edge across each sensor in known mm steps; sensor footprint and effective lateral resolution. | |
| [ ] 3.5 | **Sim vs real** | Compare real readings with `SurfaceSensorModel` in DigitalTwin for the same positions. | |

### Encoders and odometry
| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 3.6 | **Counts per revolution** | Rotate a wheel exactly N turns by hand; verify 358.3 counts/rev. | |
| [ ] 3.7 | **Wheel radius and separation calibration** | Straight-line runs (radius) and in-place rotations (separation) against physical measurement. | |
| [ ] 3.8 | **Odometry drift** | Repeated square paths (UMBmark-style), clockwise and anticlockwise; end-point error distribution. | |
| [ ] 3.9 | **Local vs middleware pose** | Log `controller.odometry.pose` and `robot.pose` side by side; quantify divergence over a run. | |
| [ ] 3.10 | **Integration interval effect** | Odometry error vs update interval (10, 20, 50 ms) on curved paths. | |

### IMU (LSM6DS33 accel/gyro, LIS3MDL magnetometer)
| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 3.11 | **Achievable sample rate** | Time `imu.read()`; how often `isSampleReady()` is true; effect on the control loop. | |
| [ ] 3.12 | **Gyro bias and noise** | Stationary for several minutes: bias, noise density, bias drift (Allan variance if time allows). | |
| [ ] 3.13 | **Gyro scale factor** | Known rotations (e.g. 10 × 360°); integrated gyro heading vs truth. | |
| [ ] 3.14 | **Gyro vs encoder heading** | Heading from gyro vs odometry during turns and line following; which drifts faster? | |
| [ ] 3.15 | **Accelerometer noise and vibration** | Stationary noise; noise while driving at different PWMs. | |
| [ ] 3.16 | **Magnetometer usefulness** | Heading while rotating on the spot, motors off vs on; interference from motors and the surroundings. | |

### Clocks and power
| # | Item | Method | Results |
|---|------|--------|---------|
| [ ] 3.17 | **Clock alignment** | Drift between StampC3 `millis()`, 3Pi+ `timestamp_us`, and the laptop clock over a long run. | |
| [ ] 3.18 | **Battery effect** | Battery voltage isn't exposed over I2C. Track speed at fixed PWM across a session as a proxy (see 2.6). | |
