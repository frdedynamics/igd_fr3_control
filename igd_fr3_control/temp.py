#!/usr/bin/env python3
"""
tag_translation_publisher.py

Replaces:
    ros2 run tf2_ros tf2_echo fr3_link0 tag0 | grep Translation
    ros2 run tf2_ros tf2_echo fr3_link0 tag0 | grep degree

Instead of parsing tf2_echo's text output, this node uses the tf2_ros
Buffer/TransformListener API directly to look up the transform from
'fr3_link0' to 'tag0' and publishes:
  - the translation component as a geometry_msgs/msg/Vector3
  - the rotation component (roll/pitch/yaw, in degrees, matching the
    "RPY (degree)" line tf2_echo prints) as a geometry_msgs/msg/Vector3

Usage:
    ros2 run <your_package> tag_translation_publisher.py
    or
    python3 tag_translation_publisher.py

Then, in other terminals:
    ros2 topic echo /tag0_translation
    ros2 topic echo /tag0_rotation_deg
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import Vector3
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def quaternion_to_euler_deg(x, y, z, w):
    """
    Convert a quaternion into roll, pitch, yaw in degrees (RPY, extrinsic
    XYZ), matching the convention tf2_echo uses for its
    'Rotation: in RPY (degree)' line.
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))  # clamp to avoid domain errors
    pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


class TagTranslationPublisher(Node):
    def __init__(self):
        super().__init__('tag_translation_publisher')

        # ---- Parameters (overridable via CLI / launch file) ----
        self.declare_parameter('source_frame', 'fr3_link0')
        self.declare_parameter('target_frame', 'tag0')
        self.declare_parameter('translation_topic', 'tag0_translation')
        self.declare_parameter('rotation_topic', 'tag0_rotation_deg')
        self.declare_parameter('publish_rate_hz', 20.0)

        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        translation_topic = self.get_parameter('translation_topic').get_parameter_value().string_value
        rotation_topic = self.get_parameter('rotation_topic').get_parameter_value().string_value
        rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

        # ---- TF2 buffer/listener ----
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Publishers ----
        self.translation_pub = self.create_publisher(Vector3, translation_topic, 10)
        self.rotation_pub = self.create_publisher(Vector3, rotation_topic, 10)

        # ---- Timer ----
        period = 1.0 / rate_hz if rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Publishing '{self.source_frame}' -> '{self.target_frame}': "
            f"translation on '/{translation_topic}', "
            f"rotation (RPY, degrees) on '/{rotation_topic}', at {rate_hz:.1f} Hz"
        )

    def on_timer(self):
        try:
            # Time() with zero seconds/nanoseconds = "latest available transform"
            transform = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"Could not look up transform '{self.source_frame}' -> "
                f"'{self.target_frame}': {e}",
                throttle_duration_sec=2.0,
            )
            return

        t = transform.transform.translation
        q = transform.transform.rotation

        translation_msg = Vector3()
        translation_msg.x = t.x
        translation_msg.y = t.y
        translation_msg.z = t.z
        self.translation_pub.publish(translation_msg)
        print(translation_msg)

        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_deg(q.x, q.y, q.z, q.w)
        rotation_msg = Vector3()
        rotation_msg.x = roll_deg
        rotation_msg.y = pitch_deg
        rotation_msg.z = yaw_deg
        self.rotation_pub.publish(rotation_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TagTranslationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()