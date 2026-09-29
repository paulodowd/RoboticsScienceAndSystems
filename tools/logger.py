import socket
import logging
import errno

# Robot: "192.168.4.1", 80. Local test with nc: "127.0.0.1", 9000.
HOST = "127.0.0.1"
PORT = 9000
TIMEOUT_S = 3
NUM_FIELDS = 14

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

def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )

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
                print(text)     

        buffer = parts[-1]

    if buffer:
        log.warning("discarding incomplete last line: %r", buffer)

    sock.close()
    log.info("logger stopped.")


if __name__ == "__main__":
    main()