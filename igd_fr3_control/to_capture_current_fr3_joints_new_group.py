import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
rclpy.init(); n = Node("grab"); got = []
n.create_subscription(JointState, "/joint_states", lambda m: got.append(m), 10)
while not got: rclpy.spin_once(n)
jp = dict(zip(got[0].name, got[0].position))
name = "grasp_view1"   # <-- rename per pose
print(f"""  <group_state name=\"{name}\" group=\"fr3_arm\">""")
for i in range(1, 8):
    j = f"fr3_joint{i}"
    print(f"""    <joint name=\"{j}\" value=\"{jp[j]:.4f}\"/>""")
print("  </group_state>")
n.destroy_node(); rclpy.shutdown()