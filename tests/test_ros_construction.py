"""Catch conflicts with rclpy Node APIs before attempting a flight command."""
import pytest

rclpy = pytest.importorskip('rclpy')

from solo_bringup.flight import Flight


def test_flight_node_can_create_command_clients():
    rclpy.init()
    node = None
    try:
        node = Flight([{'name': 'construction_test'}], timeout=1)
        assert len(tuple(node.clients)) == 3
        assert len(tuple(node.subscriptions)) == 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
