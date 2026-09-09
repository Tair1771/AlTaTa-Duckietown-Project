"""Read-only camera client used by the Windows companion."""

from dataclasses import dataclass
import queue
import threading
import time
import urllib.error
import urllib.parse
import urllib.request


@dataclass(frozen=True)
class CameraFrame:
    ppm: bytes
    view: str
    age: float
    captured_at: float
    diagnostic: str


class CameraClient:
    VIEWS = ("normal", "mask", "overlay")

    def __init__(self, url="http://127.0.0.1:8766", token=""):
        parsed = urllib.parse.urlparse(url)
        if parsed.scheme != "http" or parsed.hostname not in ("127.0.0.1", "localhost"):
            raise ValueError("Camera viewer must use a loopback SSH tunnel")
        self.url = url.rstrip("/")
        self.headers = ({"Authorization": "Bearer " + token} if token else {})

    def frame(self, view="normal", timeout=1.5):
        if view not in self.VIEWS:
            raise ValueError("Unknown camera view")
        request = urllib.request.Request(self.url + "/camera?view=" + view,
                                         headers=self.headers)
        try:
            with urllib.request.urlopen(request, timeout=timeout) as response:
                if response.headers.get_content_type() != "image/x-portable-pixmap":
                    raise RuntimeError("Camera returned an unexpected image format")
                data = response.read(4 * 1024 * 1024 + 1)
                if len(data) > 4 * 1024 * 1024:
                    raise RuntimeError("Camera frame is too large")
                age = float(response.headers.get("X-Camera-Age", "nan"))
                captured_at = float(response.headers.get("X-Captured-At", "nan"))
                if not data.startswith(b"P6\n") or not 0 <= age <= 30:
                    raise RuntimeError("Camera returned invalid frame metadata")
                return CameraFrame(data, view, age, captured_at,
                                   response.headers.get("X-Diagnostic", ""))
        except (urllib.error.URLError, TimeoutError, ValueError) as error:
            raise RuntimeError("Camera connection unavailable or timed out") from error


class CameraPoller:
    """Latest-frame worker. Network reads never block the Tk event loop."""

    def __init__(self, client, interval=0.2):
        self.client = client
        self.interval = max(0.2, float(interval))
        self._view = "normal"
        self._stop = threading.Event()
        self._thread = None
        self._queue = queue.Queue(maxsize=1)

    def set_view(self, view):
        if view not in self.client.VIEWS:
            raise ValueError("Unknown camera view")
        self._view = view

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _put_latest(self, item):
        try:
            self._queue.get_nowait()
        except queue.Empty:
            pass
        self._queue.put_nowait(item)

    def _run(self):
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                self._put_latest(("frame", self.client.frame(self._view)))
            except Exception as error:
                self._put_latest(("error", str(error)))
            self._stop.wait(max(0.0, self.interval - (time.monotonic() - started)))

    def latest(self):
        try:
            return self._queue.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
