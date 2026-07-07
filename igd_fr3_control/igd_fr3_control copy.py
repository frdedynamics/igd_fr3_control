#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, Pose, Twist
from sensor_msgs.msg import JointState

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    MotionPlanRequest,
    PlanningOptions,
)
from shape_msgs.msg import SolidPrimitive

import tf_transformations


class FR3Commander(Node):

    def __init__(self):
        super().__init__('fr3_commander')

        # ---- Parameters (match your MoveIt config / SRDF) ----------------
        self.declare_parameter('arm_group_name', 'fr3_arm')
        self.declare_parameter('ee_link', 'fr3_hand_tcp')
        self.declare_parameter('base_frame', 'fr3_link0')
        self.declare_parameter('velocity_scaling', 0.3)
        self.declare_parameter('acceleration_scaling', 0.3)
        self.declare_parameter('planning_time', 5.0)

        self.arm_group_name = self.get_parameter('arm_group_name').value
        self.ee_link = self.get_parameter('ee_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.vel_scale = self.get_parameter('velocity_scaling').value
        self.acc_scale = self.get_parameter('acceleration_scaling').value
        self.planning_time = self.get_parameter('planning_time').value

        # ---- Action client to the already-running move_group -------------
        self._client = ActionClient(self, MoveGroup, '/move_action')

        # ---- Feedback state ------------------------------------------------
        self._current_pose = None
        self._current_joint_state = None

        self.create_subscription(PoseStamped, '/franka_robot_state_broadcaster/current_pose', self._pose_cb, 10)
        self.create_subscription(JointState, '/franka/joint_states', self._joint_cb, 10)
        self.create_subscription(Twist, '/spacemouse/twist', self._spacemouse_cb, 10)

        self.sm_linear_scale = 0.02
        self.sm_angular_scale = 0.05
        self.spacemouse_enabled = True

        self.get_logger().info('Waiting for /move_action server...')
        self._client.wait_for_server()
        self.get_logger().info('FR3Commander ready.')

        self.constraints = []

    # ------------------------------------------------------------------ #
    # Feedback callbacks
    # ------------------------------------------------------------------ #
    def _pose_cb(self, msg: PoseStamped):
        self._current_pose = msg

    def _joint_cb(self, msg: JointState):
        self._current_joint_state = msg

    def get_current_pose(self) -> PoseStamped:
        return self._current_pose

    # ------------------------------------------------------------------ #
    # Joint-space control
    # ------------------------------------------------------------------ #
    def move_to_joint_positions(self, joint_positions: dict, tolerance=0.01) -> bool:
        """
        joint_positions: {'fr3_joint1': 0.0, ..., 'fr3_joint7': 0.785}
        """
        constraints = Constraints()
        for name, pos in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        self.get_logger().info(f'Sending joint-space goal: {joint_positions}')
        return self._send_goal(constraints)

    # ------------------------------------------------------------------ #
    # Task-space (Cartesian pose) control
    # ------------------------------------------------------------------ #
    def move_to_pose(self, pose: Pose, frame_id: str = None,
                      pos_tol=0.005, orient_tol=0.01) -> bool:
        frame_id = frame_id or self.base_frame

        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name = self.ee_link
        pc.target_point_offset.x = 0.0
        pc.target_point_offset.y = 0.0
        pc.target_point_offset.z = 0.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]

        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = self.ee_link
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = orient_tol
        oc.absolute_y_axis_tolerance = orient_tol
        oc.absolute_z_axis_tolerance = orient_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        self.get_logger().info(
            f'Sending task-space goal: pos=({pose.position.x:.3f}, '
            f'{pose.position.y:.3f}, {pose.position.z:.3f}) frame="{frame_id}"'
        )
        return self._send_goal(constraints)

    def move_relative(self, dx=0.0, dy=0.0, dz=0.0,
                       droll=0.0, dpitch=0.0, dyaw=0.0,
                       frame_id: str = None) -> bool:
        """Jog from the current measured EE pose. Call this from a spacemouse."""
        current = self.get_current_pose()
        if current is None:
            self.get_logger().warn('No current pose yet — cannot jog.')
            return False

        frame_id = frame_id or current.header.frame_id or self.base_frame
        p = current.pose

        target = Pose()
        target.position.x = p.position.x + dx
        target.position.y = p.position.y + dy
        target.position.z = p.position.z + dz

        q_current = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        q_delta = tf_transformations.quaternion_from_euler(droll, dpitch, dyaw)
        q_target = tf_transformations.quaternion_multiply(q_current, q_delta)

        target.orientation.x = q_target[0]
        target.orientation.y = q_target[1]
        target.orientation.z = q_target[2]
        target.orientation.w = q_target[3]

        return self.move_to_pose(target, frame_id=frame_id)

    # ------------------------------------------------------------------ #
    # Spacemouse hook
    # ------------------------------------------------------------------ #
    def _spacemouse_cb(self, msg: Twist):
        print("Received spacemouse msg", msg)
        if not self.spacemouse_enabled:
            return

        dx = msg.linear.x * self.sm_linear_scale
        dy = msg.linear.y * self.sm_linear_scale
        dz = msg.linear.z * self.sm_linear_scale
        droll = msg.angular.x * self.sm_angular_scale
        dpitch = msg.angular.y * self.sm_angular_scale
        dyaw = msg.angular.z * self.sm_angular_scale

        if max(abs(dx), abs(dy), abs(dz), abs(droll), abs(dpitch), abs(dyaw)) < 1e-4:
            return

        self.move_relative(dx, dy, dz, droll, dpitch, dyaw)

    # ------------------------------------------------------------------ #
    # Internal: send a MoveGroup goal and block until done
    # ------------------------------------------------------------------ #
    def _send_goal(self, constraints: Constraints) -> bool:
        req = MotionPlanRequest()
        req.group_name = self.arm_group_name
        req.goal_constraints.append(constraints)
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        req.num_planning_attempts = 5
        req.start_state.is_diff = True  # use robot's current state

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False  # plan AND execute

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by move_group.')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        success = (result.error_code.val == 1)  # moveit_msgs/MoveItErrorCodes.SUCCESS
        if success:
            self.get_logger().info('Motion succeeded.')
        else:
            self.get_logger().error(f'Motion failed, error code: {result.error_code.val}')
        return success


def main(args=None):
    rclpy.init(args=args)
    node = FR3Commander()

    # ---- Example usage --------------------------------------------------
    try: 
        joint_goal = {
                'fr3_joint1': 0.2313,
                'fr3_joint2': -0.6396,
                'fr3_joint3': -0.0370,
                'fr3_joint4': -2.7108,
                'fr3_joint5': -0.1908,
                'fr3_joint6': 2.0329,
                'fr3_joint7': 1.0460,
        }
        node.move_to_joint_positions(joint_goal)

        # node.move_relative(dx=0.05, dz=0.10)  # nudge 5cm forward, 10cm up
        # rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl-C")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()