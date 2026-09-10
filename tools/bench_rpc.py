"""Framed stdio HTTP relay within network-none; only fixed loopback services."""
import base64
import concurrent.futures
import json
import sys
import threading
import urllib.request
import urllib.error

lock = threading.Lock()


def request(item):
    try:
        path = item["path"]
        if path in ("/status", "/heartbeat", "/command"):
            port = 8765
        elif path in ("/camera?view=normal", "/camera?view=mask", "/camera?view=overlay"):
            port = 8766
        elif path == "/bench/scene":
            port = 8768
        else:
            raise ValueError("Unsupported bench endpoint")
        body = base64.b64decode(item["body"]) if item.get("body") else None
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (port, path), data=body,
              headers={"Content-Type": "application/json"}, method=item["method"])
        try:
            response = urllib.request.urlopen(req, timeout=4)
        except urllib.error.HTTPError as error:
            response = error
        with response:
            result = {"id": item["id"], "code": response.code,
                "headers": dict(response.headers), "body": base64.b64encode(response.read(4*1024*1024)).decode()}
    except Exception as error:
        result = {"id": item.get("id"), "code": 503, "headers": {"Content-Type": "application/json"},
                  "body": base64.b64encode(json.dumps({"error": str(error)}).encode()).decode()}
    with lock:
        print(json.dumps(result), flush=True)


if __name__ == "__main__":
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        for line in sys.stdin:
            pool.submit(request, json.loads(line))
