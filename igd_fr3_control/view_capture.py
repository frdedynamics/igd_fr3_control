#!/usr/bin/env python3
import argparse
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
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


def lookup(buf, node, target, source, stamp, tries=200):
    last_err = None
    for _ in range(tries):
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            return buf.lookup_transform(target, source, Time.from_msg(stamp),
                                         timeout=Duration(seconds=0.1))
        except Exception as e:
            last_err = e
    node.get_logger().warn(f"lookup_transform({target}, {source}) failed: {last_err}")
    return None


def tf_to_matrix(tf):
    t = tf.transform.translation
    q = tf.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="camera_color_optical_frame")
    ap.add_argument("--task", default="task")
    ap.add_argument("--tag", default=None,
                     help="apriltag_ros frame name to also record for cross-checking "
                          "(e.g. 'tag0'); requires apriltag_ros running and the tag "
                          "visible in this view. Omit to skip.")
    ap.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
    ap.add_argument("--info-topic", default="/camera/camera/aligned_depth_to_color/camera_info",
                     help="camera_info matching --depth-topic. Because the depth is "
                          "aligned to colour, these are the COLOUR intrinsics, and they "
                          "change with the streaming profile -- never hardcode them.")
    ap.add_argument("--out-prefix", required=True,
                     help="writes <prefix>_depth.npy, <prefix>_extrinsic.npy, "
                          "<prefix>_intrinsic.npy, and <prefix>_tag_extrinsic.npy "
                          "if --tag is given")
    ap.add_argument("--warmup-seconds", type=float, default=3.0,
                     help="time to let the TF buffer fill before grabbing a frame")
    args = ap.parse_args()

    rclpy.init()
    node = Node("save_view")
    bridge = CvBridge()
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf, node)

    got = []
    node.create_subscription(Image, args.depth_topic, lambda m: got.append(m), 10)

    got_info = []
    node.create_subscription(CameraInfo, args.info_topic, lambda m: got_info.append(m), 10)

    node.get_logger().info(f"Warming up for {args.warmup_seconds:.1f}s "
                            f"(letting TF buffer + topics fill)...")
    warmup_end = node.get_clock().now() + Duration(seconds=args.warmup_seconds)
    while node.get_clock().now() < warmup_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    got.clear()

    node.get_logger().info("Waiting for a fresh depth frame + camera_info...")
    while not got or not got_info:
        rclpy.spin_once(node, timeout_sec=0.05)
    msg = got[0]
    info = got_info[0]
    stamp = msg.header.stamp

    tf_task = lookup(buf, node, args.camera, args.task, stamp)
    if tf_task is None:
        print("Failed to look up camera->task transform.")
        node.destroy_node()
        rclpy.shutdown()
        return

    tf_tag = None
    if args.tag is not None:
        tf_tag = lookup(buf, node, args.camera, args.tag, stamp)
        if tf_tag is None:
            print(f"WARNING: could not look up camera->{args.tag}; "
                  f"is apriltag_ros running and is the tag visible in this view?")

    depth = bridge.imgmsg_to_cv2(msg, "passthrough")
    np.save(f"{args.out_prefix}_depth.npy", depth)

    # Intrinsics belong with the frame they describe. Saving them here is what
    # stops a stale --fx/--cx from a different streaming profile being passed on
    # the command line, which silently rescales the whole reconstruction.
    if (info.width, info.height) != (depth.shape[1], depth.shape[0]):
        node.get_logger().error(
            f"camera_info says {info.width}x{info.height} but the depth image is "
            f"{depth.shape[1]}x{depth.shape[0]}. --info-topic and --depth-topic "
            f"are not the same stream; fix this before trusting the capture.")
    np.save(f"{args.out_prefix}_intrinsic.npy",
            np.array([info.width, info.height,
                      info.k[0], info.k[4], info.k[2], info.k[5]], dtype=np.float64))
    node.get_logger().info(
        f"intrinsics {info.width}x{info.height} "
        f"fx={info.k[0]:.2f} fy={info.k[4]:.2f} cx={info.k[2]:.2f} cy={info.k[5]:.2f}")

    T_task = tf_to_matrix(tf_task)
    np.save(f"{args.out_prefix}_extrinsic.npy", T_task)

    print(f"saved {args.out_prefix}_depth.npy {depth.shape} {depth.dtype}")
    print(f"saved {args.out_prefix}_extrinsic.npy:\n{T_task}")

    if tf_tag is not None:
        T_tag = tf_to_matrix(tf_tag)
        np.save(f"{args.out_prefix}_tag_extrinsic.npy", T_tag)
        print(f"saved {args.out_prefix}_tag_extrinsic.npy (camera->{args.tag}):\n{T_tag}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# import argparse
# import numpy as np
# import rclpy
# from rclpy.node import Node
# from rclpy.time import Time
# from rclpy.duration import Duration
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge
# import tf2_ros


# def quat_to_mat(x, y, z, w):
#     n = x*x + y*y + z*z + w*w
#     s = 0.0 if n < 1e-12 else 2.0 / n
#     xx, yy, zz = x*x*s, y*y*s, z*z*s
#     xy, xz, yz = x*y*s, x*z*s, y*z*s
#     wx, wy, wz = w*x*s, w*y*s, w*z*s
#     return np.array([[1-(yy+zz), xy-wz, xz+wy],
#                       [xy+wz, 1-(xx+zz), yz-wx],
#                       [xz-wy, yz+wx, 1-(xx+yy)]])


# def lookup(buf, node, target, source, stamp, tries=200):
#     last_err = None
#     for _ in range(tries):
#         rclpy.spin_once(node, timeout_sec=0.05)
#         try:
#             return buf.lookup_transform(target, source, Time.from_msg(stamp),
#                                          timeout=Duration(seconds=0.1))
#         except Exception as e:
#             last_err = e
#     node.get_logger().warn(f"lookup_transform({target}, {source}) failed: {last_err}")
#     return None


# def tf_to_matrix(tf):
#     t = tf.transform.translation
#     q = tf.transform.rotation
#     T = np.eye(4)
#     T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
#     T[:3, 3] = [t.x, t.y, t.z]
#     return T


# def main():
#     ap = argparse.ArgumentParser()
#     ap.add_argument("--camera", default="camera_color_optical_frame")
#     ap.add_argument("--task", default="task")
#     ap.add_argument("--tag", default=None)
#     ap.add_argument("--depth-topic", default="/camera/camera/aligned_depth_to_color/image_raw")
#     ap.add_argument("--info-topic", default="/camera/camera/aligned_depth_to_color/camera_info")
#     ap.add_argument("--out-prefix", required=True)
#     ap.add_argument("--warmup-seconds", type=float, default=3.0)
#     args = ap.parse_args()

#     rclpy.init()
#     node = Node("save_view")
#     bridge = CvBridge()
#     buf = tf2_ros.Buffer()
#     tf2_ros.TransformListener(buf, node)

#     got = []
#     node.create_subscription(Image, args.depth_topic, lambda m: got.append(m), 10)

#     got_info = []
#     node.create_subscription(CameraInfo, args.info_topic, lambda m: got_info.append(m), 10)

#     node.get_logger().info(f"Warming up for {args.warmup_seconds:.1f}s...")
#     warmup_end = node.get_clock().now() + Duration(seconds=args.warmup_seconds)
#     while node.get_clock().now() < warmup_end:
#         rclpy.spin_once(node, timeout_sec=0.05)

#     got.clear()
#     got_info.clear()

#     node.get_logger().info("Waiting for a fresh depth frame and camera info...")
#     while not got or not got_info:
#         rclpy.spin_once(node, timeout_sec=0.05)
    
#     msg = got[0]
#     info_msg = got_info[0]
#     stamp = msg.header.stamp

#     tf_task = lookup(buf, node, args.camera, args.task, stamp)
#     if tf_task is None:
#         print("Failed to look up camera->task transform.")
#         node.destroy_node()
#         rclpy.shutdown()
#         return

#     tf_tag = None
#     if args.tag is not None:
#         tf_tag = lookup(buf, node, args.camera, args.tag, stamp)

#     # 1. Lagre dybdebilde (Uendret)
#     depth = bridge.imgmsg_to_cv2(msg, "passthrough")
#     np.save(f"{args.out_prefix}_depth.npy", depth)
#     print(f"saved {args.out_prefix}_depth.npy")

#     # 2. Lagre kamera-info (Nye linjer for å unngå Open3D her)
#     # Lagrer width, height, fx, fy, cx, cy til en tekstfil
#     with open(f"{args.out_prefix}_intrinsics.txt", "w") as f:
#         f.write(f"{info_msg.width}\n")
#         f.write(f"{info_msg.height}\n")
#         f.write(f"{info_msg.k[0]}\n")  # fx
#         f.write(f"{info_msg.k[4]}\n")  # fy
#         f.write(f"{info_msg.k[2]}\n")  # cx
#         f.write(f"{info_msg.k[5]}\n")  # cy
#     print(f"saved {args.out_prefix}_intrinsics.txt")

#     # 3. Lagre ekstrinsics (Uendret)
#     T_task = tf_to_matrix(tf_task)
#     np.save(f"{args.out_prefix}_extrinsic.npy", T_task)
#     print(f"saved {args.out_prefix}_extrinsic.npy")

#     if tf_tag is not None:
#         T_tag = tf_to_matrix(tf_tag)
#         np.save(f"{args.out_prefix}_tag_extrinsic.npy", T_tag)

#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()

