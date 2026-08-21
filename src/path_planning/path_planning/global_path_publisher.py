#!/usr/bin/env python3

import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg


FRAME_ID = "slam_map"
TOPIC = "/global_path"


class GlobalPathPublisher(Node):

    def __init__(self):
        super().__init__("global_path_publisher")

        package_share = Path(
            get_package_share_directory("path_planning")
        )

        self.csv_path = (
            package_share
            / "data"
            / "lpa_autodrive_smooth_final.csv"
        )

        self.publisher = self.create_publisher(
            PathMsg,
            TOPIC,
            10,
        )

        self.points = self.load_csv()

        self.timer = self.create_timer(
            0.5,
            self.publish_path,
        )

        self.get_logger().info(
            f"Trayectoria cargada: {len(self.points)} puntos"
        )

        self.get_logger().info(
            f"Publicando en {TOPIC}, frame={FRAME_ID}"
        )

    def load_csv(self):

        if not self.csv_path.exists():
            raise FileNotFoundError(
                f"No se encontró el CSV: {self.csv_path}"
            )

        points = []

        with self.csv_path.open(
            "r",
            encoding="utf-8",
        ) as file:

            reader = csv.DictReader(file)

            for row in reader:
                points.append(
                    (
                        float(row["world_x"]),
                        float(row["world_y"]),
                        float(row["yaw"]),
                    )
                )

        return points

    def publish_path(self):

        now = self.get_clock().now().to_msg()

        path = PathMsg()

        path.header.stamp = now
        path.header.frame_id = FRAME_ID

        for x, y, yaw in self.points:

            pose = PoseStamped()

            pose.header.stamp = now
            pose.header.frame_id = FRAME_ID

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            pose.pose.orientation.z = math.sin(
                yaw / 2.0
            )

            pose.pose.orientation.w = math.cos(
                yaw / 2.0
            )

            path.poses.append(pose)

        self.publisher.publish(path)


def main(args=None):

    rclpy.init(args=args)

    node = GlobalPathPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
