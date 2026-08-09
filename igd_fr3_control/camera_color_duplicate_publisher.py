import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import qos_profile_sensor_data

class FrameRepublisher(Node):
    def __init__(self):
        super().__init__('frame_republisher')
        self.new_frame_id = self.declare_parameter('new_frame_id', 'dup_camera_color_optical_frame').value

        # Reliable (default) QoS — matches what apriltag_node expects
        self.img_pub = self.create_publisher(Image, '/camera/color_dup/image_rect', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/color_dup/camera_info', 10)

        # Best Effort — matches what RealSense actually publishes
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, qos_profile_sensor_data)

    def img_cb(self, msg):
        msg.header.frame_id = self.new_frame_id
        self.img_pub.publish(msg)

    def info_cb(self, msg):
        msg.header.frame_id = self.new_frame_id
        self.info_pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(FrameRepublisher())

if __name__ == '__main__':
    main()