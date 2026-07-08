#!/usr/bin/env python3
"""
spacemouse_publisher.py

Simple ROS 2 (Jazzy) node that reads a 3Dconnexion SpaceMouse via spnav
and publishes its state as a geometry_msgs/TwistStamped on
/servo_node/delta_twist_cmds, for consumption by moveit_servo's servo_node.

Requires: pip install spnav  (and spacenavd running on the system)
"""

import time
from threading import Thread, Event
from collections import defaultdict

import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState

from franka_msgs.action import Move, Grasp, Homing
from rclpy.action import ActionClient

from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent


class Spacemouse(Thread):
    def __init__(self, max_value=500, deadzone=(0, 0, 0, 0, 0, 0), dtype=np.float32):
        """
        Continuously listen to 3D connection space navigator events
        and update the latest state.
        max_value: {300, 500} 300 for wired version and 500 for wireless
        deadzone: [0,1], number or tuple, axis with value lower than this value will stay at 0
        front
        z
        ^   _
        |  (O) space mouse
        |
        *----->x right
        y
        """
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        super().__init__()
        self.stop_event = Event()
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.motion_event = SpnavMotionEvent([0, 0, 0], [0, 0, 0], 0)
        self.button_state = defaultdict(lambda: False)
        self.tx_zup_spnav = np.array([
            [0, 0, -1],
            [1, 0, 0],
            [0, 1, 0]
        ], dtype=dtype)

    def get_motion_state(self):
        me = self.motion_event
        state = np.array(me.translation + me.rotation,
                          dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state

    def get_motion_state_transformed(self):
        """
        Return in right-handed coordinate
        z
        *------>y right
        |   _
        |  (O) space mouse
        v
        x
        back
        """
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]
        return tf_state

    def is_button_pressed(self, button_id):
        return self.button_state[button_id]

    def stop(self):
        self.stop_event.set()
        self.join()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def run(self):
        spnav_open()
        try:
            while not self.stop_event.is_set():
                event = spnav_poll_event()
                if isinstance(event, SpnavMotionEvent):
                    self.motion_event = event
                elif isinstance(event, SpnavButtonEvent):
                    self.button_state[event.bnum] = event.press
                else:
                    time.sleep(1 / 200)
        finally:
            spnav_close()


class SpaceMousePublisher(Node):

    def __init__(self):
        super().__init__('spacemouse_publisher')

        self.declare_parameter('topic', '/servo_node/delta_twist_cmds')
        self.declare_parameter('frame_id', 'fr3_hand_tcp')  # must match robot_link_command_frame in servo_config.yaml
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('max_value', 500)       # 300 wired, 500 wireless
        self.declare_parameter('deadzone', 0.3)

        topic = self.get_parameter('topic').value
        self.frame_id = self.get_parameter('frame_id').value
        rate = self.get_parameter('publish_rate').value
        max_value = self.get_parameter('max_value').value
        deadzone = self.get_parameter('deadzone').value

        # Gripper commands:
        self.move_client = ActionClient(self, Move, '/franka_gripper/move')
        self.grasp_client = ActionClient(self, Grasp, '/franka_gripper/grasp')
        self.home_client = ActionClient(self, Homing, '/franka_gripper/homing')

        self.gripper_state = True # open
        self.gripper_latest_positions = {}

        self._pub = self.create_publisher(TwistStamped, topic, 10)
        self.subscription = self.create_subscription(JointState, '/franka_gripper/joint_states', self.gripper_joint_state_callback,10)


        self._configure_servo()

        self._sm = Spacemouse(max_value=max_value, deadzone=deadzone)
        self._sm.start()

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(f'Publishing SpaceMouse TwistStamped on "{topic}" at {rate} Hz.')

    def _configure_servo(self):
        """One-time setup calls to servo_node, blocking (safe: runs before rclpy.spin())."""
        cmd_type_client = self.create_client(ServoCommandType, '/servo_node/switch_command_type')
        pause_client = self.create_client(SetBool, '/servo_node/pause_servo')

        self.get_logger().info('Waiting for servo_node services...')
        cmd_type_client.wait_for_service()
        pause_client.wait_for_service()

        req = ServoCommandType.Request()
        req.command_type = 1  # TWIST -- confirmed via: ros2 service call .../switch_command_type "{command_type: 1}"
        future = cmd_type_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'switch_command_type(TWIST) -> success={future.result().success}')

        req = SetBool.Request()
        req.data = False  # ensure not paused
        future = pause_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f'pause_servo(False) -> {future.result().message}')

    def _on_timer(self):
        state = self._sm.get_motion_state_transformed()
        button_state = self._sm.is_button_pressed(1)

        print(button_state)

        if button_state:
            if self.gripper_state:
                self.grasp()
                print("Closing gripper")
            else:
                self.open_gripper()
                print("Openning gripper")

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = float(state[0])
        msg.twist.linear.y = float(state[1])
        msg.twist.linear.z = float(state[2])
        msg.twist.angular.x = float(state[3])
        msg.twist.angular.y = float(state[4])
        msg.twist.angular.z = float(state[5])

        self._pub.publish(msg)

    def destroy_node(self):
        self._sm.stop()
        super().destroy_node()

    def gripper_joint_state_callback(self, msg: JointState):

        self.gripper_latest_positions = msg.position
 


    def open_gripper(self, width=0.1, speed=0.1):
        self.move_client.wait_for_server()
        goal = Move.Goal(width=width, speed=speed)
        self.gripper_state = False
        return self.move_client.send_goal_async(goal)

    def grasp(self, width=0.02, speed=0.1, force=20.0, epsilon_inner=0.1, epsilon_outer=0.1):
        self.grasp_client.wait_for_server()
        goal = Grasp.Goal(width=width, speed=speed, force=force)
        goal.epsilon.inner = epsilon_inner
        goal.epsilon.outer = epsilon_outer
        self.gripper_state = True
        return self.grasp_client.send_goal_async(goal)


def main(args=None):
    rclpy.init(args=args)
    node = SpaceMousePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()