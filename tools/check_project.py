"""Offline source/startup/documentation checks. Never connects to duck2."""
import argparse
import ast
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]


def check():
    errors = []
    count = 0
    for folder in ("laptop", "tools", "tests", "packages"):
        for path in (ROOT / folder).rglob("*.py"):
            if "__pycache__" in path.parts:
                continue
            try:
                ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
                count += 1
            except (SyntaxError, UnicodeError) as error:
                errors.append(str(error))
    for path in ROOT.rglob("*.md"):
        if any(part in (".git", ".venv", ".runtime") for part in path.parts):
            continue
        for link in re.findall(r"\]\(([^)]+)\)", path.read_text(encoding="utf-8")):
            link = unquote(link.split("#", 1)[0].split(' "', 1)[0].strip("<>"))
            if not link or "://" in link or link.startswith(("mailto:", "/", "\\")):
                continue
            if not (path.parent / link).exists():
                errors.append("Broken link in %s: %s" % (path.relative_to(ROOT), link))
    for path in list((ROOT / "laptop").glob("*.cmd")) + list((ROOT / "tools").glob("*.cmd")):
        text = path.read_text(encoding="utf-8")
        for target in re.findall(r'%~dp0([^"\r\n]+\.(?:py|ps1))', text):
            if not (path.parent / target).exists():
                errors.append("Missing launcher target: %s -> %s" % (path.name, target))
    if errors:
        raise RuntimeError("\n".join(errors))
    print("PASS: %d Python sources, relative documentation links and Windows launcher targets" % count)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tests", action="store_true", help="Run native regression suite (ROS script is separate)")
    args = parser.parse_args()
    check()
    if args.tests:
        import os
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(str(ROOT / part) for part in (
            "tests", "laptop", "tools", "packages/duckie_lane_follower/src"))
        modules = sorted(path.stem for path in (ROOT / "tests").glob("test_*.py")
                         if path.name != "test_ros_transport.py")
        subprocess.run([sys.executable, "-m", "unittest"] + modules, cwd=ROOT, env=env, check=True)


if __name__ == "__main__":
    main()
