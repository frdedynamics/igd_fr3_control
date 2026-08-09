# #!/usr/bin/env python3
# """
# Interactive TF fine-tuning node for eye-in-hand camera calibration.

# Publishes a LIVE transform (fr3_link8 -> camera_link) whose x/y/z and
# roll/pitch/yaw are ROS parameters. Change them on the fly with
# `ros2 param set` and watch the point cloud slide into alignment with
# the robot mesh in RViz, without ever restarting anything.

# Run it (it starts from your current calibration by default):

#     python3 tf_tuner.py

# or, as a ROS 2 node with overrides:

#     ros2 run <your_pkg> tf_tuner.py --ros-args \
#         -p x:=0.017583 -p y:=-0.022985 -p z:=0.054449

# Then tune live, e.g.:

#     ros2 param set /tf_tuner roll 0.02      # nudge by ~1 degree
#     ros2 param set /tf_tuner x 0.019

# Every change is logged along with the equivalent quaternion and a
# ready-to-copy static_transform_publisher command, so once RViz looks
# right you can paste that command back into your launch setup.

# ADDITIONAL cam_roll / cam_pitch / cam_yaw PARAMETERS
# -----------------------------------------------------
# roll/pitch/yaw above rotate about the PARENT frame's (fr3_link8) axes
# -- fine for a first pass, but if your camera is mounted at some odd
# angle, "roll about fr3_link8's x-axis" won't correspond to a twist
# about the camera's own boresight/optical axis.

# cam_roll / cam_pitch / cam_yaw instead apply a small rotation in the
# CAMERA's OWN local frame, composed on top of (after) the base
# roll/pitch/yaw. Use these to isolate and correct a pure twist about
# the camera's own axis -- e.g. the "always shifts to the same side
# after a 180 degree wrist rotation" symptom, which points at a roll
# error about the camera's boresight:

#     ros2 param set /tf_tuner cam_roll 0.02   # ~1 deg twist about the
#                                               # camera's own optical axis
#     ros2 param set /tf_tuner cam_pitch 0.0
#     ros2 param set /tf_tuner cam_yaw 0.0

# Workflow: leave cam_roll/pitch/yaw at 0 until the base roll/pitch/yaw
# + xyz look reasonable, then use cam_roll specifically to kill the
# side-dependent-on-rotation shift. Once it's gone regardless of wrist
# angle, the printed "final combined quaternion" already has everything
# folded in -- copy that into your static_transform_publisher command.
# """

# import math
# import rclpy
# from rclpy.node import Node
# from rcl_interfaces.msg import SetParametersResult
# from geometry_msgs.msg import TransformStamped
# from tf2_ros import TransformBroadcaster


# def euler_to_quat(roll, pitch, yaw):
#     cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
#     cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
#     cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
#     qw = cr * cp * cy + sr * sp * sy
#     qx = sr * cp * cy - cr * sp * sy
#     qy = cr * sp * cy + sr * cp * sy
#     qz = cr * cp * sy - sr * sp * cy
#     return qx, qy, qz, qw


# def quat_mult(q1, q2):
#     """Hamilton product q1 * q2, both as (x, y, z, w). Applying q2 as a
#     right-multiply rotates about q1's OWN (local/child) axes -- this is
#     how we compose a local camera-frame twist on top of a base
#     orientation expressed in the parent frame."""
#     x1, y1, z1, w1 = q1
#     x2, y2, z2, w2 = q2
#     return (
#         w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
#         w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
#         w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
#         w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
#     )


# def quat_to_euler(qx, qy, qz, qw):
#     sinr_cosp = 2 * (qw * qx + qy * qz)
#     cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
#     roll = math.atan2(sinr_cosp, cosr_cosp)

#     sinp = max(-1.0, min(1.0, 2 * (qw * qy - qz * qx)))
#     pitch = math.asin(sinp)

#     siny_cosp = 2 * (qw * qz + qx * qy)
#     cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
#     yaw = math.atan2(siny_cosp, cosy_cosp)
#     return roll, pitch, yaw


# class TfTuner(Node):
#     def __init__(self):
#         super().__init__('tf_tuner')

#         self.declare_parameter('parent_frame', 'fr3_link8')
#         self.declare_parameter('child_frame', 'camera_link')
#         self.declare_parameter('x', 0.0)
#         self.declare_parameter('y', 0.05)
#         self.declare_parameter('z', 0.065)

#         # Seed roll/pitch/yaw from your existing quaternion so you start
#         # from the current calibration instead of zero.
#         qx0, qy0, qz0, qw0 = 0.5, -0.5, 0.5, 0.5
#         r0, p0, yy0 = quat_to_euler(qx0, qy0, qz0, qw0)
#         self.declare_parameter('roll', r0)
#         self.declare_parameter('pitch', p0)
#         self.declare_parameter('yaw', yy0)

#         # Local camera-frame twist, composed AFTER the base rotation above.
#         # Use cam_roll to fix a pure boresight-axis twist.
#         self.declare_parameter('cam_roll', 0.0)
#         self.declare_parameter('cam_pitch', 0.0)
#         self.declare_parameter('cam_yaw', 0.0)

#         self.br = TransformBroadcaster(self)
#         self.add_on_set_parameters_callback(self._on_param_change)
#         self.timer = self.create_timer(0.05, self._broadcast)  # 20 Hz
#         self._print_current()

#     def _combined_quat(self):
#         q_base = euler_to_quat(self._d('roll'), self._d('pitch'), self._d('yaw'))
#         q_cam = euler_to_quat(self._d('cam_roll'), self._d('cam_pitch'), self._d('cam_yaw'))
#         # Right-multiply => q_cam is applied about the camera's OWN
#         # (already-rotated) local axes, not the parent's.
#         return quat_mult(q_base, q_cam)

#     def _d(self, name):
#         return self.get_parameter(name).get_parameter_value().double_value

#     def _s(self, name):
#         return self.get_parameter(name).get_parameter_value().string_value

#     def _broadcast(self):
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = self._s('parent_frame')
#         t.child_frame_id = self._s('child_frame')
#         t.transform.translation.x = self._d('x')
#         t.transform.translation.y = self._d('y')
#         t.transform.translation.z = self._d('z')
#         qx, qy, qz, qw = self._combined_quat()
#         t.transform.rotation.x = qx
#         t.transform.rotation.y = qy
#         t.transform.rotation.z = qz
#         t.transform.rotation.w = qw
#         self.br.sendTransform(t)

#     def _on_param_change(self, params):
#         self._print_current()
#         return SetParametersResult(successful=True)

#     def _print_current(self):
#         x, y, z = self._d('x'), self._d('y'), self._d('z')
#         roll, pitch, yaw = self._d('roll'), self._d('pitch'), self._d('yaw')
#         cr, cp, cyaw = self._d('cam_roll'), self._d('cam_pitch'), self._d('cam_yaw')
#         qx, qy, qz, qw = self._combined_quat()
#         self.get_logger().info(
#             "\n--- current calibration ---\n"
#             f"xyz: {x:.6f} {y:.6f} {z:.6f}\n"
#             f"base rpy (rad): {roll:.4f} {pitch:.4f} {yaw:.4f}  "
#             f"(deg: {math.degrees(roll):.2f} {math.degrees(pitch):.2f} {math.degrees(yaw):.2f})\n"
#             f"cam-local rpy (rad): {cr:.4f} {cp:.4f} {cyaw:.4f}  "
#             f"(deg: {math.degrees(cr):.2f} {math.degrees(cp):.2f} {math.degrees(cyaw):.2f})\n"
#             f"COMBINED quat: qx={qx:.6f} qy={qy:.6f} qz={qz:.6f} qw={qw:.6f}\n"
#             "copy-paste command once you're happy (includes base + cam-local twist):\n"
#             f"ros2 run tf2_ros static_transform_publisher "
#             f"--x {x:.6f} --y {y:.6f} --z {z:.6f} "
#             f"--qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} "
#             f"--frame-id {self._s('parent_frame')} --child-frame-id {self._s('child_frame')}"
#         )


# def main():
#     rclpy.init()
#     node = TfTuner()
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
"""
Interactive TF fine-tuning node for eye-in-hand camera calibration.

Publishes a LIVE transform (fr3_hand -> camera_color_optical_frame) whose x/y/z and
roll/pitch/yaw are ROS parameters. Change them on the fly with
`ros2 param set` and watch the point cloud slide into alignment with
the robot mesh in RViz, without ever restarting anything.
"""

import math
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def quat_mult(q1, q2):
    """Hamilton product q1 * q2, both as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


class TfTuner(Node):
    def __init__(self):
        super().__init__('tf_tuner')

        # Oppdatert til riktige standard-rammer for Franka Hand og RealSense RGB Optical
        self.declare_parameter('parent_frame', 'fr3_hand')
        self.declare_parameter('child_frame', 'camera_color_optical_frame')
        
        # Basert på din tekniske tegning (X=sentrert, Y=50mm ut på siden, Z=65mm frem)
        self.declare_parameter('x', 0.03185)
        self.declare_parameter('y', 0.000000)  # Endre til -0.050000 hvis montert på motsatt side
        self.declare_parameter('z', 0.065000)

        # Baseline optisk rotasjon (Z fremover, X til høyre, Y ned) uttrykt i Euler-vinkler
        self.declare_parameter('roll', 1.570796)   # 90 grader
        self.declare_parameter('pitch', 0.000000)  # 0 grader
        self.declare_parameter('yaw', 1.570796)    # 90 grader

        # Lokale kamera-justeringer (nyttig for finjustering live)
        self.declare_parameter('cam_roll', 0.0)
        self.declare_parameter('cam_pitch', 0.0)
        self.declare_parameter('cam_yaw', 0.0)

        self.br = TransformBroadcaster(self)
        self.add_on_set_parameters_callback(self._on_param_change)
        
        # Timer for TF-broadcasting (20 Hz)
        self.timer = self.create_timer(0.05, self._broadcast)
        
        # Timer for forsinket utskrift for å unngå ROS 2 parameter-lag i terminalen
        self.print_timer = None
        self._print_current()

    def _combined_quat(self):
        q_base = euler_to_quat(self._d('roll'), self._d('pitch'), self._d('yaw'))
        q_cam = euler_to_quat(self._d('cam_roll'), self._d('cam_pitch'), self._d('cam_yaw'))
        return quat_mult(q_base, q_cam)

    def _d(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _s(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._s('parent_frame')
        t.child_frame_id = self._s('child_frame')
        t.transform.translation.x = self._d('x')
        t.transform.translation.y = self._d('y')
        t.transform.translation.z = self._d('z')
        qx, qy, qz, qw = self._combined_quat()
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

    def _on_param_change(self, params):
        # Trigger en tidsforsinket utskrift slik at parameterne rekker å bli lagret først
        if self.print_timer is not None:
            self.print_timer.destroy()
        self.print_timer = self.create_timer(0.2, self._print_callback)
        return SetParametersResult(successful=True)

    def _print_callback(self):
        self._print_current()
        if self.print_timer is not None:
            self.print_timer.destroy()
            self.print_timer = None

    def _print_current(self):
        x, y, z = self._d('x'), self._d('y'), self._d('z')
        roll, pitch, yaw = self._d('roll'), self._d('pitch'), self._d('yaw')
        cr, cp, cyaw = self._d('cam_roll'), self._d('cam_pitch'), self._d('cam_yaw')
        qx, qy, qz, qw = self._combined_quat()
        self.get_logger().info(
            "\n--- gjeldende kalibrering ---\n"
            f"xyz: {x:.6f} {y:.6f} {z:.6f}\n"
            f"base rpy (rad): {roll:.4f} {pitch:.4f} {yaw:.4f}  "
            f"(deg: {math.degrees(roll):.2f} {math.degrees(pitch):.2f} {math.degrees(yaw):.2f})\n"
            f"cam-local rpy (rad): {cr:.4f} {cp:.4f} {cyaw:.4f}  "
            f"(deg: {math.degrees(cr):.2f} {math.degrees(cp):.2f} {math.degrees(cyaw):.2f})\n"
            f"KOMBINERT kvaternion: qx={qx:.6f} qy={qy:.6f} qz={qz:.6f} qw={qw:.6f}\n"
            "Kopier denne kommandoen når du er fornøyd i RViz:\n"
            f"ros2 run tf2_ros static_transform_publisher "
            f"--x {x:.6f} --y {y:.6f} --z {z:.6f} "
            f"--qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} "
            f"--frame-id {self._s('parent_frame')} --child-frame-id {self._s('child_frame')}"
        )


def main():
    rclpy.init()
    node = TfTuner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
