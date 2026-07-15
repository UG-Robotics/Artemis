"""Artemis web control: serves a teleop UI + MJPEG camera stream from the Pi.

    python -m robot.web.app        # from src/, or with PYTHONPATH=src


"""

import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from robot.web.camera_feed import make_camera_source
from robot.web.teleop import Teleop
from robot.web.telemetry import Telemetry

HOST, PORT = "0.0.0.0", 8000
WATCHDOG_TIMEOUT = 0.6  # seconds without a command before we cut the throttle
HERE = os.path.dirname(os.path.abspath(__file__))

teleop = Teleop()
telemetry = Telemetry()
camera = make_camera_source(teleop)


def _watchdog():
    while True:
        teleop.watchdog(WATCHDOG_TIMEOUT)
        time.sleep(0.1)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # keep the console quiet

    def _send(self, code, body=b"", ctype="text/plain"):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if body:
            self.wfile.write(body)

    def _json(self, payload):
        self._send(200, json.dumps(payload).encode(), "application/json")

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "index.html"), "rb") as f:
                return self._send(200, f.read(), "text/html; charset=utf-8")
        if self.path == "/stream.mjpg":
            return self._stream()
        if self.path == "/state":
            return self._json(teleop.state())
        if self.path == "/sensors":
            return self._json(telemetry.read())
        return self._send(404, b"not found")

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0) or 0)
        try:
            data = json.loads(self.rfile.read(n) or b"{}") if n else {}
        except Exception:
            data = {}
        if self.path == "/control":
            teleop.command(throttle=data.get("throttle"), steer=data.get("steer"),
                           speed_scale=data.get("speed_scale"))
            return self._json(teleop.state())
        if self.path == "/stop":
            teleop.stop()
            return self._json(teleop.state())
        return self._send(404, b"not found")

    def _stream(self):
        self.send_response(200)
        self.send_header("Age", "0")
        self.send_header("Cache-Control", "no-cache, private")
        self.send_header("Pragma", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
        self.end_headers()
        try:
            for jpg in camera.frames():
                self.wfile.write(b"--FRAME\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                self.wfile.flush()  # push each frame out now, don't let it buffer
        except (BrokenPipeError, ConnectionResetError):
            pass  # client closed the stream


def main():
    threading.Thread(target=_watchdog, daemon=True).start()
    httpd = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"Artemis web control: http://{HOST}:{PORT}  (simulated={teleop.simulated})")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.close()
        telemetry.close()


if __name__ == "__main__":
    main()
