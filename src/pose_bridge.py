import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class PoseBridge(Node):
    def __init__(self):
        super().__init__('pose_bridge')
        self.sub = self.create_subscription(Odometry, '/fastlio2/lio_odom', self.callback, 10)
        self.pub = self.create_publisher(PoseStamped, '/nav3d/current_pose', 10)
        self.get_logger().info('定位桥接节点已启动')

    def callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.pub.publish(pose)

def main():
    rclpy.init()
    node = PoseBridge()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
