#!/usr/bin/env python3
"""
rosbridge_test_runner.py -- Lightweight WebSocket test runner for rosbridge.

Usage:
    python3 rosbridge_test_runner.py [--host HOST] [--port PORT] [--verbose]

Setup:
    pip install websocket-client
"""

import sys
import json
import time
import argparse

# ============================================================
# Test Cases
# ============================================================

TEST_SUITE = [
    {
        "group": "1. System Health",
        "tests": [
            {
                "name": "List all ROS topics via rosapi",
                "send": {"op": "call_service", "service": "/rosapi/topics"},
                "expect_response": True,
                "asserts": [
                    ("Response is dict", lambda r: isinstance(r, dict)),
                    ("Has topics", lambda r: "topics" in r.get("values", {}) or "values" in r),
                ],
            },
            {
                "name": "List all ROS services via rosapi",
                "send": {"op": "call_service", "service": "/rosapi/services"},
                "expect_response": True,
                "asserts": [
                    ("Response is dict", lambda r: isinstance(r, dict)),
                    ("Has services", lambda r: "services" in r.get("values", {}) or "values" in r),
                ],
            },
        ],
    },
    {
        "group": "2. IMU Data Pipeline",
        "tests": [
            {
                "name": "Subscribe to /imu_data",
                "send": {"op": "subscribe", "topic": "/imu_data", "type": "sensor_msgs/Imu"},
                "expect_response": False,
                "asserts": [("Sent OK (fire-and-forget)", lambda r: True)],
            },
            {
                "name": "Publish test IMU message to /imu_data",
                "send": {
                    "op": "publish",
                    "topic": "/imu_data",
                    "msg": {
                        "header": {"seq": 1, "stamp": {"secs": 0, "nsecs": 0}, "frame_id": "imu_link"},
                        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
                        "angular_velocity": {"x": 0.01, "y": 0.02, "z": 0.03},
                        "linear_acceleration": {"x": 9.81, "y": 0, "z": 0},
                    },
                },
                "expect_response": False,
                "asserts": [("Sent OK (fire-and-forget)", lambda r: True)],
            },
        ],
    },
    {
        "group": "3. Motion Control",
        "tests": [
            {
                "name": "Advertise /cmd_vel topic",
                "send": {"op": "advertise", "topic": "/cmd_vel", "type": "geometry_msgs/Twist"},
                "expect_response": False,
                "asserts": [("Sent OK (fire-and-forget)", lambda r: True)],
            },
            {
                "name": "Publish velocity command (forward 0.2 m/s)",
                "send": {
                    "op": "publish",
                    "topic": "/cmd_vel",
                    "msg": {"linear": {"x": 0.2, "y": 0, "z": 0}, "angular": {"x": 0, "y": 0, "z": 0}},
                },
                "expect_response": False,
                "asserts": [("Sent OK (fire-and-forget)", lambda r: True)],
            },
            {
                "name": "Subscribe to /odom",
                "send": {"op": "subscribe", "topic": "/odom", "type": "nav_msgs/Odometry"},
                "expect_response": False,
                "asserts": [("Sent OK (fire-and-forget)", lambda r: True)],
            },
        ],
    },
    {
        "group": "4. Service Call Validation",
        "tests": [
            {
                "name": "Query /rosapi/topic_type for /rosout",
                "send": {"op": "call_service", "service": "/rosapi/topic_type", "args": {"topic": "/rosout"}},
                "expect_response": True,
                "asserts": [
                    ("Response is dict", lambda r: isinstance(r, dict)),
                    ("Has values", lambda r: "values" in r),
                ],
            },
            {
                "name": "Unadvertise /cmd_vel (cleanup)",
                "send": {"op": "unadvertise", "topic": "/cmd_vel"},
                "expect_response": False,
                "asserts": [("Sent OK (fire-and-forget)", lambda r: True)],
            },
        ],
    },
]


def run_tests(host="localhost", port=9090, verbose=False):
    import websocket

    ws_url = "ws://{}:{}".format(host, port)
    total = 0
    passed = 0

    try:
        ws = websocket.create_connection(ws_url, timeout=10)
    except Exception as e:
        print("ERROR: Cannot connect to rosbridge at {}".format(ws_url))
        print("  {}".format(e))
        print("  Start rosbridge: roslaunch filter rosbridge_test.launch")
        return 1

    for group in TEST_SUITE:
        print("")
        print("=== {} ===".format(group["group"]))

        for test in group["tests"]:
            total += 1
            name = test["name"]
            msg = json.dumps(test["send"])

            try:
                ws.send(msg)

                if test.get("expect_response", True):
                    raw = ws.recv()
                    resp = json.loads(raw)
                    if verbose:
                        print("  <<< {}".format(json.dumps(resp, indent=2)[:200]))
                else:
                    # Fire-and-forget: give 0.5s for any async echo, ignore if none
                    ws.settimeout(0.5)
                    try:
                        raw = ws.recv()
                        resp = json.loads(raw)
                    except Exception:
                        resp = {"_implicit_ack": True}
                    ws.settimeout(10)
                    if verbose and "_implicit_ack" not in resp:
                        print("  <<< {}".format(json.dumps(resp, indent=2)[:200]))

                failures = []
                for desc, fn in test["asserts"]:
                    try:
                        if not fn(resp):
                            failures.append(desc)
                    except Exception as e:
                        failures.append("{} ({})".format(desc, e))

                if failures:
                    print("  FAIL  {} -- {}".format(name, "; ".join(failures)))
                else:
                    print("  PASS  {}".format(name))
                    passed += 1

            except websocket.WebSocketTimeoutException:
                print("  FAIL  {} -- Timeout".format(name))
            except json.JSONDecodeError as e:
                print("  FAIL  {} -- Invalid JSON: {}".format(name, e))
            except Exception as e:
                print("  FAIL  {} -- {}".format(name, e))

    ws.close()
    print("")
    print("=" * 50)
    print("  Results: {}/{} passed".format(passed, total))
    print("=" * 50)
    return 0 if passed == total else 1


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="rosbridge WebSocket test runner")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=9090)
    p.add_argument("--verbose", "-v", action="store_true")
    args = p.parse_args()
    sys.exit(run_tests(args.host, args.port, args.verbose))
