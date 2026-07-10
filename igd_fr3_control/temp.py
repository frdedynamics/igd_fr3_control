import rclpy, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
rclpy.init(); n=Node('grab'); b=CvBridge(); got=[]
def cb(m):
    d=b.imgmsg_to_cv2(m,'passthrough')
    got.append(d); 
sub=n.create_subscription(Image,'/camera/camera/aligned_depth_to_color/image_raw',cb,10)
while not got: rclpy.spin_once(n)
np.save('/tmp/depth.npy', got[0]); print('saved', got[0].shape, got[0].dtype)