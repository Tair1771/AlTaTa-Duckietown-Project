#!/usr/bin/env python3
"""Small authenticated HTTP bridge for the Windows client; wheel commands stay in ROS."""
import hmac
import json
import os
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
try:
    import rospy
    from std_msgs.msg import String
except ImportError:
    rospy = None  # ROS 2 supplies its transport explicitly.

class Gateway:
    def __init__(self, vehicle, runtime=None):
        self.runtime = runtime if runtime is not None else rospy
        self.String = runtime.String if runtime is not None else String
        self.condition = threading.Condition()
        self.current = None
        self.updated = 0.0
        self.acks = {}
        self.publisher = self.runtime.Publisher(
            "/%s/lane_follower/command" % vehicle, self.String, queue_size=10)
        self.subscriber = self.runtime.Subscriber(
            "/%s/lane_follower/status" % vehicle, self.String, self.receive, queue_size=10)

    def receive(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
        except (ValueError, TypeError):
            return
        with self.condition:
            self.current = status
            self.updated = time.monotonic()
            ack = status.get("last_command")
            if isinstance(ack, dict) and isinstance(ack.get("id"), str):
                self.acks[ack["id"]] = ack
                if len(self.acks) > 100:
                    del self.acks[next(iter(self.acks))]
            self.condition.notify_all()

    def status(self):
        with self.condition:
            if self.current is None or time.monotonic()-self.updated > 1.5:
                raise RuntimeError("No fresh status from duck2")
            return dict(self.current)

    def heartbeat(self, command):
        if not isinstance(command, dict) or not isinstance(command.get("client_id"), str):
            raise ValueError("Heartbeat needs a client id")
        if self.publisher.get_num_connections() == 0:
            raise RuntimeError("Lane follower is not connected")
        payload = dict(command, action="heartbeat", id=str(uuid.uuid4()))
        payload.setdefault("issued_at", time.time())
        self.publisher.publish(self.String(data=json.dumps(payload, allow_nan=False)))
        return {"forwarded": True}

    def send(self, command):
        if not isinstance(command, dict):
            raise ValueError("Expected a command object")
        identifier = command.get("id", str(uuid.uuid4()))
        if not isinstance(identifier, str) or not 1 <= len(identifier) <= 100:
            raise ValueError("Invalid command id")
        if self.publisher.get_num_connections() == 0:
            raise RuntimeError("Lane follower is not connected")
        payload = dict(command, id=identifier)
        payload.setdefault("issued_at", time.time())
        self.publisher.publish(self.String(data=json.dumps(payload, allow_nan=False)))
        with self.condition:
            deadline = time.monotonic()+3
            while identifier not in self.acks:
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise RuntimeError("No acknowledgment; command outcome is unknown")
                self.condition.wait(remaining)
            return self.acks[identifier]

def handler_for(gateway, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, body):
            data = json.dumps(body, allow_nan=False).encode("utf8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def authorized(self):
            # This is a native desktop client. Web pages must not control the bot.
            if self.headers.get("Origin"):
                self.reply(403, {"error": "Browser requests are not accepted"})
                return False
            if token and not hmac.compare_digest(
                    self.headers.get("Authorization", ""), "Bearer "+token):
                self.reply(401, {"error": "Control token is missing or incorrect"})
                return False
            return True

        def do_GET(self):
            if not self.authorized():
                return
            if self.path != "/status":
                self.reply(404, {"error": "Not found"})
                return
            try:
                self.reply(200, gateway.status())
            except RuntimeError as error:
                self.reply(503, {"error": str(error)})

        def do_POST(self):
            if not self.authorized():
                return
            if self.path not in ("/command", "/heartbeat"):
                self.reply(404, {"error": "Not found"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8192:
                    raise ValueError("Invalid request size")
                if self.headers.get_content_type() != "application/json":
                    raise ValueError("Expected application/json")
                command = json.loads(self.rfile.read(length))
                self.reply(200, gateway.heartbeat(command) if self.path == "/heartbeat"
                           else gateway.send(command))
            except (ValueError, TypeError) as error:
                self.reply(400, {"error": str(error)})
            except RuntimeError as error:
                self.reply(503, {"error": str(error)})
    return Handler

def main():
    rospy.init_node("duck2_command_gateway")
    host = rospy.get_param("~listen_host", "127.0.0.1")
    port = int(rospy.get_param("~port", 8765))
    token = os.environ.get("DUCK2_CONTROL_TOKEN", "")
    if host not in ("127.0.0.1", "localhost") and not token:
        raise ValueError("Set DUCK2_CONTROL_TOKEN when exposing the gateway beyond loopback")
    gateway = Gateway(os.environ["VEHICLE_NAME"])
    server = ThreadingHTTPServer((host, port), handler_for(gateway, token))
    server.daemon_threads = True
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    rospy.on_shutdown(server.shutdown)
    rospy.loginfo("Command gateway listening on %s:%s", host, port)
    rospy.spin()
    server.server_close()

if __name__ == "__main__":
    main()
