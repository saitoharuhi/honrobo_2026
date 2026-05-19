import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomBridge(Node):
    def __init__(self):
        super().__init__('odom_bridge')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom', self.callback, 10)

    def callback(self, msg):
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    rclpy.spin(OdomBridge())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
