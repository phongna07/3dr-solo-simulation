"""Release MPC publisher ownership before the existing LAND command."""

import json
import sys
from pathlib import Path

import rclpy
from std_srvs.srv import SetBool

manifest = json.loads(Path("/workspace/runs/latest/manifest.json").read_text())
if manifest.get("profile") == "mpc":
    rclpy.init()
    node = rclpy.create_node("release_mpc")
    try:
        for service in ("/mpc/enable", "/target/run"):
            client = node.create_client(SetBool, service)
            if not client.wait_for_service(timeout_sec=3.0):
                print(f"{service} unavailable; proceeding with LAND", file=sys.stderr)
                continue
            future = client.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
            if not future.done() or not future.result().success:
                print(f"{service} release failed; proceeding with LAND", file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()
