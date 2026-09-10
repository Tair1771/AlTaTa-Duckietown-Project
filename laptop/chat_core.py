"""HTTP transport shared by the live companion and robot gateways."""
import json
import time
import urllib.error
import urllib.request
import uuid

def request_json(url, payload=None, headers=None, timeout=5):
    body = None if payload is None else json.dumps(payload, allow_nan=False).encode("utf8")
    req = urllib.request.Request(url, data=body, headers={
        "Content-Type": "application/json", **(headers or {})})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        try:
            details = json.load(error)
            reason = details.get("error", "Request rejected")
            if isinstance(reason, dict):
                reason = reason.get("message", "Request rejected")
        except (ValueError, TypeError):
            reason = "Request rejected"
        raise RuntimeError(str(reason)) from None
    except (urllib.error.URLError, TimeoutError) as error:
        raise RuntimeError("Connection unavailable or timed out") from error


class RobotTransport:
    def __init__(self, url="http://127.0.0.1:8765", token=""):
        if not url.startswith(("http://", "https://")):
            raise ValueError("Use an http:// or https:// gateway address")
        self.client_id = str(uuid.uuid4())
        self.url = url.rstrip("/")
        self.headers = {"Authorization": "Bearer "+token} if token else {}

    def status(self):
        return request_json(self.url+"/status", headers=self.headers)

    def heartbeat(self):
        return request_json(self.url+"/heartbeat", {
            "client_id": self.client_id, "issued_at": time.time()}, self.headers, timeout=1)

    def poll_status(self):
        self.heartbeat()
        return self.status()

    def send(self, action, command_id=None, **kwargs):
        payload = dict(id=command_id or str(uuid.uuid4()), action=action, client_id=self.client_id,
                       issued_at=time.time(), **kwargs)
        return request_json(self.url+"/command", payload, self.headers)
