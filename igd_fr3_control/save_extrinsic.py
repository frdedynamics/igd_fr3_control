#!/usr/bin/env python3
"""
save_extrinsic.py -- run in the ROS 2 env. Saves the task->camera 4x4 matrix
(the "extrinsic" integrate() wants) to a .npy for offline_test.py.

    python3 save_extrinsic.py --camera camera_color_optical_frame --task task \
        --out /tmp/extrinsic.npy
"""
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import tf2_ros


def quat_to_mat(x, y, z, w):
    n = x*x + y*y + z*z + w*w
    s = 0.0 if n < 1e-12 else 2.0 / n
    xx, yy, zz = x*x*s, y*y*s, z*z*s
    xy, xz, yz = x*y*s, x*z*s, y*z*s
    wx, wy, wz = w*x*s, w*y*s, w*z*s
    return np.array([[1-(yy+zz), xy-wz, xz+wy],
                     [xy+wz, 1-(xx+zz), yz-wx],
                     [xz-wy, yz+wx, 1-(xx+yy)]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="camera_color_optical_frame")
    ap.add_argument("--task", default="task")
    ap.add_argument("--out", default="/tmp/extrinsic.npy")
    args = ap.parse_args()

    rclpy.init()
    node = Node("save_extrinsic")
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    # lookup_transform(target=camera, source=task) == task->camera == extrinsic
    tf = None
    for _ in range(200):
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            tf = buf.lookup_transform(args.camera, args.task, Time(),
                                      timeout=Duration(seconds=0.1))
            break
        except Exception:
            continue
    if tf is None:
        print("Failed to look up transform. Is the TF tree connected?")
        return

    t = tf.transform.translation
    q = tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    np.save(args.out, T)
    print(f"saved {args.out}:\n{T}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()