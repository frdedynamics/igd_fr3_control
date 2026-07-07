import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
import tf_transformations


class MoveitNonBlockingClient(Node):
    def __init__(self):
        super().__init__('moveit_non_blocking_node')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        self.current_pose = None
        self.create_subscription(
            PoseStamped, '/franka_robot_state_broadcaster/current_pose',
            self._pose_cb, 10)
        self.create_subscription(Twist, '/spacemouse/twist', self._spacemouse_cb, 10)

        self.linear_scale = 0.02
        self.angular_scale = 0.05
        self.busy = False  # ignore new spacemouse deltas while a goal is in flight

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _spacemouse_cb(self, msg: Twist):
        if self.busy or self.current_pose is None:
            return

        dx = msg.linear.x * self.linear_scale
        dy = msg.linear.y * self.linear_scale
        dz = msg.linear.z * self.linear_scale
        droll = msg.angular.x * self.angular_scale
        dpitch = msg.angular.y * self.angular_scale
        dyaw = msg.angular.z * self.angular_scale

        if max(abs(dx), abs(dy), abs(dz), abs(droll), abs(dpitch), abs(dyaw)) < 1e-4:
            return

        p = self.current_pose.pose
        target = Pose()
        target.position.x = p.position.x + dx
        target.position.y = p.position.y + dy
        target.position.z = p.position.z + dz

        q = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        q_delta = tf_transformations.quaternion_from_euler(droll, dpitch, dyaw)
        q_new = tf_transformations.quaternion_multiply(q, q_delta)
        target.orientation = Quaternion(x=q_new[0], y=q_new[1], z=q_new[2], w=q_new[3])

        self.send_pose_goal(target, frame_id=self.current_pose.header.frame_id)

    def send_pose_goal(self, target_pose: Pose, frame_id='fr3_link0'):
        self._action_client.wait_for_server()

        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name = 'fr3_hand_tcp'
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])
        pc.constraint_region.primitives.append(sphere)
        pc.constraint_region.primitive_poses.append(target_pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = 'fr3_hand_tcp'
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 0.01
        oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'fr3_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = False

        self.busy = True
        self.get_logger().info('Sending planning goal asynchronously...')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def send_joint_goal(self, joint_positions: dict, tolerance=0.01):
        """joint_positions: {'fr3_joint1': 0.0, ..., 'fr3_joint7': 0.785}"""
        self._action_client.wait_for_server()

        constraints = Constraints()
        for name, pos in joint_positions.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'fr3_arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.start_state.is_diff = True
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = False

        self.busy = True
        self.get_logger().info(f'Sending joint-space goal: {joint_positions}')
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the MoveIt server.')
            self.busy = False
            return

        self.get_logger().info('Goal accepted! Executing motion...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Action completed with error code: {result.error_code.val}')
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = MoveitNonBlockingClient()

    joint_goal = {
                'fr3_joint1': 0.2313,
                'fr3_joint2': -0.6396,
                'fr3_joint3': -0.0370,
                'fr3_joint4': -2.7108,
                'fr3_joint5': -0.1908,
                'fr3_joint6': 2.0329,
                'fr3_joint7': 1.0460,
        }
    node.send_joint_goal(joint_goal)

    # target_pose = Pose(
    #     position=Point(x=0.5, y=0.0, z=0.5),
    #     orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    # )
    # node.send_pose_goal(target_pose, frame_id='fr3_link0')

    # Spin to keep processing goal callbacks AND spacemouse callbacks in the background
    rclpy.spin(node)


if __name__ == '__main__':
    main()