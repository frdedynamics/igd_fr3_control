#!/usr/bin/env python3
"""
tsdf_grasp_node.py  --  runs in the ROS 2 Jazzy environment (system Python).

Collects aligned depth frames + camera poses (from TF) and ships them to the
IGD inference server (igd_inference_server.py, running in your conda env)
over ZMQ. Publishes the returned grasps and fused scene cloud for RViz.

No Open3D / torch needed on this side. Only rclpy, cv_bridge, numpy, pyzmq.

Services (std_srvs/Trigger):
  ~/reset     clear all captured frames
  ~/capture   store the latest depth frame + camera pose (call once per viewpoint)
  ~/predict   send frames to the IGD server, publish grasps, return best grasp

Publishers:
  ~/grasps        geometry_msgs/PoseArray   all predicted grasps (task frame)
  ~/best_grasp    geometry_msgs/PoseStamped highest-scoring grasp (task frame)
  ~/scene_cloud   sensor_msgs/PointCloud2   fused TSDF cloud (task frame) - use
                                            this in RViz to verify calibration!
  ~/workspace     visualization_msgs/Marker wireframe of the 30cm task cube

Typical flow (with your eye-in-hand camera):
  1. move robot to scan pose 1 (spacemouse or scripted)  ->  call ~/capture
  2. optionally repeat for 1-2 more viewpoints           ->  ~/capture again
  3. call ~/predict
"""

import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time

import zmq
import pickle

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from visualization_msgs.msg import Marker
from std_srvs.srv import Trigger
from std_msgs.msg import Header

import tf2_ros


def quat_to_mat(x, y, z, w):
    """Quaternion (xyzw) -> 3x3 rotation matrix."""
    n = x * x + y * y + z * z + w * w
    s = 0.0 if n < 1e-12 else 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz, xz + wy],
        [xy + wz, 1.0 - (xx + zz), yz - wx],
        [xz - wy, yz + wx, 1.0 - (xx + yy)],
    ])


def tfmsg_to_matrix(tf_msg):
    """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix."""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class TSDFGraspNode(Node):
    def __init__(self):
        super().__init__("tsdf_grasp_node")

        # ---------------- parameters ----------------
        self.declare_parameter("depth_topic",
                               "/camera/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic",
                               "/camera/camera/color/camera_info")
        # Frame the 30cm workspace cube is defined in. Publish it yourself:
        #   ros2 run tf2_ros static_transform_publisher x y z 0 0 0 fr3_link0 task
        # where (x,y,z) is the MIN corner of the cube in fr3_link0 coordinates.
        self.declare_parameter("task_frame", "task")
        # Leave empty to use the frame_id from the depth image header
        # (usually camera_color_optical_frame - correct for aligned depth).
        self.declare_parameter("camera_frame", "")
        self.declare_parameter("zmq_endpoint", "tcp://127.0.0.1:5555")
        self.declare_parameter("workspace_size", 0.30)   # must match training
        self.declare_parameter("depth_trunc", 1.2)        # metres, ignore farther pixels

        self.depth_topic = self.get_parameter("depth_topic").value
        self.info_topic = self.get_parameter("camera_info_topic").value
        self.task_frame = self.get_parameter("task_frame").value
        self.camera_frame_override = self.get_parameter("camera_frame").value
        self.zmq_endpoint = self.get_parameter("zmq_endpoint").value
        self.ws_size = self.get_parameter("workspace_size").value
        self.depth_trunc = self.get_parameter("depth_trunc").value

        # ---------------- state ----------------
        self.bridge = CvBridge()
        self.latest_depth = None          # last Image msg
        self.camera_info = None           # last CameraInfo msg
        self.frames = []                  # list of dicts sent to the server

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ZMQ (lazy connect; REQ/REP)
        self.zmq_ctx = zmq.Context()
        self.zmq_sock = None

        # ---------------- I/O ----------------
        self.create_subscription(Image, self.depth_topic,
                                 self._depth_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic,
                                 self._info_cb, qos_profile_sensor_data)

        self.pub_grasps = self.create_publisher(PoseArray, "~/grasps", 1)
        self.pub_best = self.create_publisher(PoseStamped, "~/best_grasp", 1)
        self.pub_cloud = self.create_publisher(PointCloud2, "~/scene_cloud", 1)
        self.pub_ws = self.create_publisher(Marker, "~/workspace", 1)

        self.create_service(Trigger, "~/reset", self._srv_reset)
        self.create_service(Trigger, "~/capture", self._srv_capture)
        self.create_service(Trigger, "~/predict", self._srv_predict)

        self.create_timer(1.0, self._publish_workspace_marker)

        self.get_logger().info(
            f"Ready. depth={self.depth_topic}  info={self.info_topic}  "
            f"task_frame={self.task_frame}  server={self.zmq_endpoint}")

    # ---------------- subscriptions ----------------
    def _depth_cb(self, msg: Image):
        self.latest_depth = msg

    def _info_cb(self, msg: CameraInfo):
        self.camera_info = msg

    # ---------------- services ----------------
    def _srv_reset(self, req, res):
        self.frames.clear()
        res.success = True
        res.message = "Cleared captured frames."
        self.get_logger().info(res.message)
        return res

    def _srv_capture(self, req, res):
        if self.latest_depth is None or self.camera_info is None:
            res.success = False
            res.message = "No depth image / camera_info received yet."
            return res

        msg = self.latest_depth
        cam_frame = self.camera_frame_override or msg.header.frame_id

        # --- depth image -> float32 metres ---
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if depth.dtype == np.uint16:          # 16UC1, millimetres (RealSense)
            depth = depth.astype(np.float32) / 1000.0
        else:                                  # 32FC1, metres
            depth = depth.astype(np.float32)
        depth = np.nan_to_num(depth, nan=0.0, posinf=0.0, neginf=0.0)
        depth[depth > self.depth_trunc] = 0.0  # 0 = "no measurement" for TSDF

        # --- camera pose: extrinsic maps task-frame points into camera frame ---
        # lookup_transform(target=camera, source=task) == T_cam_task, which is
        # exactly the "extrinsic" the VGN/GIGA TSDFVolume.integrate() expects.
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                cam_frame, self.task_frame, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.5))
        except tf2_ros.ExtrapolationException:
            # stamp slightly ahead/behind TF cache -> fall back to latest
            tf_msg = self.tf_buffer.lookup_transform(
                cam_frame, self.task_frame, Time(),
                timeout=Duration(seconds=0.5))
        T_cam_task = tfmsg_to_matrix(tf_msg)

        K = self.camera_info.k  # row-major 3x3
        self.frames.append({
            "depth": depth,
            "width": int(self.camera_info.width),
            "height": int(self.camera_info.height),
            "fx": float(K[0]), "fy": float(K[4]),
            "cx": float(K[2]), "cy": float(K[5]),
            "T_cam_task": T_cam_task,
        })
        res.success = True
        res.message = f"Captured frame #{len(self.frames)} from '{cam_frame}'."
        self.get_logger().info(res.message)
        return res

    def _srv_predict(self, req, res):
        if not self.frames:
            res.success = False
            res.message = "No frames captured. Call ~/capture first."
            return res

        request = {"cmd": "predict",
                   "size": self.ws_size,
                   "frames": self.frames}
        try:
            reply = self._zmq_request(request, timeout_s=60.0)
        except Exception as e:
            res.success = False
            res.message = f"Inference server error: {e}"
            self.get_logger().error(res.message)
            return res

        if reply.get("error"):
            res.success = False
            res.message = f"Server-side error: {reply['error']}"
            self.get_logger().error(res.message)
            return res

        grasps = reply.get("grasps", [])
        cloud = reply.get("cloud", None)

        # --- publish fused cloud (task frame) for RViz sanity checking ---
        if cloud is not None and len(cloud) > 0:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.task_frame
            self.pub_cloud.publish(
                point_cloud2.create_cloud_xyz32(header, np.asarray(cloud, np.float32)))

        # --- publish grasps ---
        pa = PoseArray()
        pa.header.frame_id = self.task_frame
        pa.header.stamp = self.get_clock().now().to_msg()
        for g in grasps:
            p = Pose()
            p.position.x, p.position.y, p.position.z = g["position"]
            (p.orientation.x, p.orientation.y,
             p.orientation.z, p.orientation.w) = g["quaternion_xyzw"]
            pa.poses.append(p)
        self.pub_grasps.publish(pa)

        if grasps:
            best = max(grasps, key=lambda g: g["score"])
            ps = PoseStamped()
            ps.header = pa.header
            ps.pose = pa.poses[grasps.index(best)]
            self.pub_best.publish(ps)
            res.success = True
            res.message = json.dumps({
                "n_grasps": len(grasps),
                "best": {k: (list(v) if isinstance(v, (list, tuple, np.ndarray)) else v)
                         for k, v in best.items()},
                "inference_time_s": reply.get("toc"),
            })
        else:
            res.success = False
            res.message = "Server returned no grasps (scene empty or below threshold)."
        self.get_logger().info(res.message)
        return res

    # ---------------- helpers ----------------
    def _zmq_request(self, obj, timeout_s=60.0):
        # fresh socket per request: REQ sockets lock up after a failed cycle
        sock = self.zmq_ctx.socket(zmq.REQ)
        sock.setsockopt(zmq.LINGER, 0)
        sock.connect(self.zmq_endpoint)
        try:
            sock.send(pickle.dumps(obj, protocol=4))
            if sock.poll(int(timeout_s * 1000)) == 0:
                raise TimeoutError(f"no reply within {timeout_s}s "
                                   f"(is igd_inference_server.py running?)")
            return pickle.loads(sock.recv())
        finally:
            sock.close()

    def _publish_workspace_marker(self):
        m = Marker()
        m.header.frame_id = self.task_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = "workspace", 0, Marker.CUBE, Marker.ADD
        s = self.ws_size
        m.pose.position.x = m.pose.position.y = m.pose.position.z = s / 2.0
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = s
        m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.1, 0.15
        self.pub_ws.publish(m)


def main():
    rclpy.init()
    node = TSDFGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
