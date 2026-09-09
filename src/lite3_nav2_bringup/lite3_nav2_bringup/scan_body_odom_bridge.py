import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class ScanBodyOdomBridge(Node):
    """Publish the base_link pose in odom for SCAN's body_pose input."""

    def __init__(self):
        super().__init__("scan_body_odom_bridge")
        self.declare_parameter("input_odom", "/fastlio2/lio_odom")
        self.declare_parameter("output_odom", "/scan/body_odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            Odometry, self.get_parameter("output_odom").value, 20)
        self.subscription = self.create_subscription(
            Odometry, self.get_parameter("input_odom").value, self.odom_cb, 20)
        self.last_warning_ns = 0

    def odom_cb(self, source):
        odom_frame = self.get_parameter("odom_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            tf = self.tf_buffer.lookup_transform(
                odom_frame, base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_warning_ns > 2_000_000_000:
                self.get_logger().warning(f"Waiting for {odom_frame}->{base_frame}: {exc}")
                self.last_warning_ns = now_ns
            return

        msg = Odometry()
        msg.header.stamp = source.header.stamp
        msg.header.frame_id = odom_frame
        msg.child_frame_id = base_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        msg.pose.covariance = source.pose.covariance
        msg.twist = source.twist
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanBodyOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
