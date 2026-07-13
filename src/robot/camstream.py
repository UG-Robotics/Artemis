import io, time, threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.outputs import FileOutput

PORT = 8080
lock = threading.Lock()
frame_buf = None


class StreamOutput(io.BufferedIOBase):
    def write(self, buf):
        global frame_buf
        with lock:
            frame_buf = bytes(buf)
        return len(buf)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a): pass

    def do_GET(self):
        if self.path == "/":
            html = b"<html><body style='background:#111'><img src=/stream style='width:100%'></body></html>"
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", len(html))
            self.end_headers()
            self.wfile.write(html)
        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with lock:
                        f = frame_buf
                    if f:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + f + b"\r\n")
                    time.sleep(0.04)
            except Exception:
                pass


cam = Picamera2()
config = cam.create_video_configuration(main={"size": (640, 480)})
cam.configure(config)
output = StreamOutput()
cam.start_recording(MJPEGEncoder(), FileOutput(output))

print("Stream at http://0.0.0.0:{}/".format(PORT))
try:
    HTTPServer(("", PORT), Handler).serve_forever()
finally:
    cam.stop_recording()
