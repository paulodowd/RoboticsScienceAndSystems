import socket
import logging
import errno
from rich.table import Table
from rich.live import Live
from rich.logging import RichHandler
import math
import rerun as rr
import rerun.blueprint as rrb

# Robot: "192.168.4.1", 80. Local test with nc: "127.0.0.1", 9000.
HOST = "192.168.4.1"
PORT = 80
TIMEOUT_S = 3
USE_RERUN = True          # stream every record to a Rerun viewer
HEADING_ARROW_MM = 30     # length of the heading arrow drawn at the robot

# Field order sent by Controller_c::publishTelemetry (see Controller.h).
COLUMNS = [
    "t_ms", "x_mm", "y_mm", "theta_rad",
    "pwm_left", "pwm_right", "enc_left", "enc_right",
    "dn1", "dn2", "dn3", "dn4", "dn5", "signal",
]
NUM_FIELDS = len(COLUMNS)

log = logging.getLogger(__name__)


def parse_line(line):
    fields = line.split(",")

    if len(fields) != NUM_FIELDS:
        log.warning("expected %d fields, got %d: %r", NUM_FIELDS, len(fields), line)
        return None

    for field in fields:
        try:
            float(field)
        except ValueError:
            log.warning("not a number %r in: %r", field, line)
            return None

    return fields


def make_table(fields):
    """Builds the dashboard from the latest record, or a placeholder before the first one."""
    table = Table(title="SLAMDunk telemetry")
    table.add_column("Field")
    table.add_column("Value", justify="right")

    if fields is None:
        table.add_row("status", "waiting for data...")
        return table

    for name, value in zip(COLUMNS, fields):
        table.add_row(name, value)
    return table


def make_blueprint():
    """Rerun viewer layout: robot path on the left, time series on the right."""
    # Show every point up to the time cursor, so the path leaves a trail.
    trail = rrb.VisibleTimeRange(
        "robot_time",
        start=rrb.TimeRangeBoundary.infinite(),
        end=rrb.TimeRangeBoundary.cursor_relative(),
    )
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(origin="world", name="Path", time_ranges=[trail]),
            rrb.Vertical(
                rrb.TimeSeriesView(origin="pose", name="Pose"),
                rrb.TimeSeriesView(origin="motors", name="Motor PWM"),
                rrb.TimeSeriesView(origin="encoders", name="Encoders"),
                rrb.TimeSeriesView(origin="surface/series", name="Surface sensors"),
                rrb.BarChartView(origin="surface/now", name="Surface now"),
            ),
        ),
    )


def log_to_rerun(fields):
    """Sends one validated record to Rerun, stamped with the robot's own clock."""
    values = dict(zip(COLUMNS, (float(f) for f in fields)))

    rr.set_time("robot_time", duration=values["t_ms"] / 1000)

    # Rerun's 2D view has +y pointing down; the robot's +y is anticlockwise
    # (up), so y is negated for drawing only. The pose/ series keep true values.
    x, y, theta = values["x_mm"], values["y_mm"], values["theta_rad"]
    rr.log("world/trail", rr.Points2D([[x, -y]], radii=0.5))
    rr.log("world/robot", rr.Arrows2D(
        origins=[[x, -y]],
        vectors=[[HEADING_ARROW_MM * math.cos(theta), -HEADING_ARROW_MM * math.sin(theta)]],
    ))

    for name in ("x_mm", "y_mm", "theta_rad"):
        rr.log(f"pose/{name}", rr.Scalars(values[name]))
    for name in ("pwm_left", "pwm_right"):
        rr.log(f"motors/{name}", rr.Scalars(values[name]))
    for name in ("enc_left", "enc_right"):
        rr.log(f"encoders/{name}", rr.Scalars(values[name]))

    surface = [values[f"dn{i}"] for i in range(1, 6)]
    for i, reading in enumerate(surface, start=1):
        rr.log(f"surface/series/dn{i}", rr.Scalars(reading))
    rr.log("surface/now", rr.BarChart(surface))


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(message)s",
        handlers=[RichHandler()],
    )

    if USE_RERUN:
        rr.init("slamdunk_logger", spawn=True, default_blueprint=make_blueprint())

    try:
        sock = socket.create_connection((HOST, PORT), timeout=TIMEOUT_S)
    except ConnectionRefusedError:
        log.error("Server not listening. Robot must be booting.")
        return
    except socket.timeout:
        log.error("robot did not answer within %s s: robot off or out of range.", TIMEOUT_S)
        return
    except OSError as e:
        if e.errno in (errno.EHOSTUNREACH, errno.ENETUNREACH):
            log.error("not on the robot's WiFi (%s)", e)
        else:
            log.exception("unexpected connect error")
        return

    buffer = b""
    latest = None

    with Live(make_table(latest), refresh_per_second=10) as live:
        while True:
            try:
                data = sock.recv(4096)
                if not data:
                    raise ConnectionError("Closed by the server.")

            except socket.timeout:
                log.error("no data for %s s: robot reset or out of range.", TIMEOUT_S)
                break
            except ConnectionError as e:
                log.info("connection ended: %s", e)
                break
            except OSError:
                log.exception("unexpected error while reading")
                break

            buffer += data
            parts = buffer.split(b"\n")
            for line in parts[:-1]:
                text = line.decode("utf-8", errors="replace").strip()
                fields = parse_line(text)
                if fields is not None:
                    latest = fields
                    if USE_RERUN:
                        log_to_rerun(fields)
            buffer = parts[-1]

            live.update(make_table(latest))

    if buffer:
        log.warning("discarding incomplete last line: %r", buffer)

    sock.close()
    log.info("logger stopped.")


if __name__ == "__main__":
    main()
