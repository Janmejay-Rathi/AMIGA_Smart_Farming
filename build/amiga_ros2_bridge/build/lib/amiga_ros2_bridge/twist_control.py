#!/usr/bin/env python3
# Copyright (c) farm-ng, inc.

from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import asyncio
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory  # ✅ added

from farm_ng.canbus.canbus_pb2 import Twist2d
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList, SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file
from geometry_msgs.msg import TwistStamped


def cmd_vel_callback(
    node: Node, twist_stamped: TwistStamped, queue: asyncio.Queue
) -> None:
    """Callback for the /amiga/cmd_vel topic."""
    node.get_logger().info("cmd_vel_callback triggered")
    twist = Twist2d()
    twist.linear_velocity_x = twist_stamped.twist.linear.x
    twist.linear_velocity_y = twist_stamped.twist.linear.y
    twist.angular_velocity = twist_stamped.twist.angular.z

    try:
        queue.put_nowait(twist)
        node.get_logger().info("Twist2d added to queue")
    except asyncio.QueueFull:
        node.get_logger().warn("Queue full, dropping Twist message")


async def command_task(node: Node, client: EventClient, queue: asyncio.Queue) -> None:
    node.get_logger().info("command_task started")
    while rclpy.ok():
        try:
            twist = await queue.get()
            node.get_logger().info(f"Twist2d RECEIVED: {twist}")
            await client.request_reply("/twist", twist)
            node.get_logger().info("Twist2d sent to canbus")
        except Exception as e:
            node.get_logger().error(f"Failed to send twist: {e}")


async def run(node: Node, service_config: Path) -> None:
    config_list = proto_from_json_file(service_config, EventServiceConfigList())
    clients = {}
    subscriptions = []

    for config in config_list.configs:
        if config.port != 0 and config.name == "canbus":
            clients[config.name] = EventClient(config)
            node.get_logger().info(f"Client added for: {config.name}")
        else:
            subscriptions = config.subscriptions

    queue = asyncio.Queue(maxsize=1)
    qos_profile = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)

    node.create_subscription(
        TwistStamped,
        "/amiga/cmd_vel",
        lambda msg: cmd_vel_callback(node, msg, queue),
        qos_profile,
    )
    node.get_logger().info("Subscribed to /amiga/cmd_vel")

    tasks = []
    for subscription in subscriptions:
        service_name = subscription.uri.query.split("=")[-1]
        path = subscription.uri.path
        if service_name == "canbus" and path == "/twist":
            tasks.append(asyncio.create_task(command_task(node, clients[service_name], queue)))

    await asyncio.gather(*tasks)


def main(args=None):
    rclpy.init(args=args)
    node = Node("twist_control_node")
    node.get_logger().info("amiga_twist_control started!")

    # ✅ Use ROS 2 share directory to get service_config.json path
    try:
        pkg_path = get_package_share_directory("amiga_ros2_bridge")
        service_config = Path(pkg_path) / "include" / "service_config.json"
        node.get_logger().info(f"Service config set to: {service_config}")
    except Exception as e:
        node.get_logger().error(f"Failed to locate service config: {e}")
        return

    loop = asyncio.get_event_loop()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        asyncio.ensure_future(run(node, service_config))
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
            loop.run_until_complete(asyncio.sleep(0.1))
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        loop.close()


if __name__ == "__main__":
    main()

