#!/usr/bin/env python3
# Copyright (c) farm-ng, inc.
#
# Licensed under the Amiga Development Kit License (the "License");
# https://github.com/farm-ng/amiga-dev-kit/blob/main/LICENSE

from __future__ import annotations

import asyncio
import os
from pathlib import Path

# ROS 2 libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from ament_index_python.packages import get_package_share_directory

# Farm-ng
from farm_ng.core.event_client import EventClient
from farm_ng.core.event_service_pb2 import EventServiceConfigList, SubscribeRequest
from farm_ng.core.events_file_reader import proto_from_json_file

# Your own ROS publisher logic
from .farmng_ros_pipelines import create_ros_publisher


async def run(node: Node, service_config: Path) -> None:
    # Load config from file
    config_list: EventServiceConfigList = proto_from_json_file(
        service_config, EventServiceConfigList()
    )

    # Setup event clients and subscriptions
    clients: dict[str, EventClient] = {}
    subscriptions: list[SubscribeRequest] = []

    for config in config_list.configs:
        if config.port != 0:
            clients[config.name] = EventClient(config)
        else:
            subscriptions = config.subscriptions

    # Create tasks for ROS publishers
    tasks: list[asyncio.Task] = []
    for subscription in subscriptions:
        service_name = subscription.uri.query.split("=")[-1]
        task = asyncio.create_task(
            create_ros_publisher(node, clients[service_name], subscription)
        )
        tasks.append(task)

    await asyncio.gather(*tasks)


def main(args=None):
    # ✅ Dynamically find the path to your config file inside the installed package
    try:
        pkg_path = get_package_share_directory("amiga_ros2_bridge")
        service_config = Path(os.path.join(pkg_path, "include", "service_config.json"))
    except Exception as e:
        print(f"[amiga_streams.py] Failed to locate package share directory: {e}")
        return

    # Start ROS node
    rclpy.init(args=args)
    node = Node("amiga_streams_node")
    node.get_logger().info("amiga_streams_node started!")

    try:
        loop = asyncio.get_event_loop()
        loop.run_until_complete(run(node, service_config))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.get_logger().info("Shutting down amiga_streams_node")
        rclpy.shutdown()


if __name__ == "__main__":
    main()

