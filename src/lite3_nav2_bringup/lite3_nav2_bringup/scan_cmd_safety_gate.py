import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener


class ScanCmdSafetyGate(Node):
    """Fail-closed command gate; starts disabled and publishes zero when unhealthy."""

    def __init__(self):
        super().__init__("scan_cmd_safety_gate")
        self.declare_parameter("input_cmd", "/scan_cmd_vel")
        self.declare_parameter("output_cmd", "/cmd_vel")
        self.declare_parameter("odom_topic", "/fastlio2/lio_odom")
        self.declare_parameter("cloud_topic", "/fastlio2/body_cloud")
        self.declare_parameter("command_timeout", 0.30)
        self.declare_parameter("sensor_timeout", 0.50)
        self.declare_parameter("tf_timeout", 0.50)
        self.declare_parameter("max_vx", 0.15)
        self.declare_parameter("max_vy", 0.10)
        self.declare_parameter("max_wz", 0.25)
        self.enabled = False
        self.last_cmd = Twist()
        self.last_cmd_ns = self.last_odom_ns = self.last_cloud_ns = 0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, self.get_parameter("output_cmd").value, 10)
        self.ready_pub = self.create_publisher(Bool, "/scan_navigation_ready", 10)
        self.create_subscription(Twist, self.get_parameter("input_cmd").value, self.cmd_cb, 10)
        self.create_subscription(Odometry, self.get_parameter("odom_topic").value, self.odom_cb, 10)
        self.create_subscription(PointCloud2, self.get_parameter("cloud_topic").value, self.cloud_cb, 10)
        self.create_subscription(Bool, "/scan_navigation_enable", self.enable_cb, 10)
        self.timer = self.create_timer(0.05, self.tick)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def cmd_cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_ns = self.now_ns()

    def odom_cb(self, _msg):
        self.last_odom_ns = self.now_ns()

    def cloud_cb(self, _msg):
        self.last_cloud_ns = self.now_ns()

    def enable_cb(self, msg):
        self.enabled = bool(msg.data)
        text = "Motion gate ENABLED" if self.enabled else "Motion gate disabled; commanding zero"
        self.get_logger().warning(text)

    @staticmethod
    def clamp(value, limit):
        return max(-limit, min(limit, value)) if math.isfinite(value) else 0.0

    def healthy(self):
        now = self.now_ns()
        sensor_ns = int(self.get_parameter("sensor_timeout").value * 1e9)
        cmd_ns = int(self.get_parameter("command_timeout").value * 1e9)
        if now - self.last_odom_ns > sensor_ns or now - self.last_cloud_ns > sensor_ns:
            return False
        if now - self.last_cmd_ns > cmd_ns:
            return False
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time(), timeout=Duration(seconds=0.02))
        except TransformException:
            return False
        tf_stamp = rclpy.time.Time.from_msg(tf.header.stamp).nanoseconds
        if tf_stamp and now - tf_stamp > int(self.get_parameter("tf_timeout").value * 1e9):
            return False
        return True

    def tick(self):
        ready = self.healthy()
        self.ready_pub.publish(Bool(data=ready))
        output = Twist()
        if self.enabled and ready:
            output.linear.x = self.clamp(self.last_cmd.linear.x, self.get_parameter("max_vx").value)
            output.linear.y = self.clamp(self.last_cmd.linear.y, self.get_parameter("max_vy").value)
            output.angular.z = self.clamp(self.last_cmd.angular.z, self.get_parameter("max_wz").value)
        self.pub.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = ScanCmdSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
