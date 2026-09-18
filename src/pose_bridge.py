import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class PoseBridge(Node):
    """Publish the robot body pose in map coordinates for Nav3D."""

    def __init__(self):
        super().__init__("pose_bridge")
        self.declare_parameter("input_odom", "/fastlio2/lio_odom")
        self.declare_parameter("output_pose", "/nav3d/current_pose")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("body_frame", "base_link")

        self.global_frame = self.get_parameter("global_frame").value
        self.body_frame = self.get_parameter("body_frame").value
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(
            PoseStamped, self.get_parameter("output_pose").value, 10
        )
        self.sub = self.create_subscription(
            Odometry,
            self.get_parameter("input_odom").value,
            self.callback,
            10,
        )
        self.get_logger().info(
            f"Pose bridge started: {self.global_frame} -> {self.body_frame}"
        )

    def callback(self, _msg):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.body_frame,
                Time(),
                timeout=Duration(seconds=0.0),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f"Waiting for {self.global_frame} -> {self.body_frame}: {exc}",
                throttle_duration_sec=2.0,
            )
            return

        pose = PoseStamped()
        pose.header = transform.header
        pose.header.frame_id = self.global_frame
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = PoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
