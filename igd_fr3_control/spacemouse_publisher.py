import rclpy,time
from rclpy.node import Node

from geometry_msgs.msg import Twist

from SpacemouseClass import Spacemouse


class SpacemousePublisher(Node):

    def __init__(self):
        super().__init__('spacemouse_publisher')
        self.publisher_ = self.create_publisher(Twist, '/spacemouse/twist', 10)
        timer_period = 0.001  # seconds
        self.timer = self.create_timer(timer_period, self.spacemouse_pub_callback)
        self.sm = Spacemouse()
        self.sm.start()
        self.get_logger().info('Spacemouse thread started')

    def spacemouse_pub_callback(self):
        msg = Twist()
        state = self.sm.get_motion_state()
        msg.linear.x = float(state[0])
        msg.linear.y = float(state[1])
        msg.linear.z = float(state[2])
        msg.angular.x = float(state[3])
        msg.angular.y = float(state[4])
        msg.angular.z = float(state[5])
        # print(state)

        self.publisher_.publish(msg)
    
    def destroy_node(self):
        self.sm.stop()  # signals stop_event and joins the thread
        super().destroy_node()

    
def main(args=None):
    rclpy.init(args=args)
    node = SpacemousePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
