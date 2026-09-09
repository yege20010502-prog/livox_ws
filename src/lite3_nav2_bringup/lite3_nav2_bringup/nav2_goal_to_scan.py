import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformException, TransformListener


def quat_multiply(a, b):
    return (
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3],
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],
    )


def rotate(q, p):
    inverse = (-q[0], -q[1], -q[2], q[3])
    return quat_multiply(quat_multiply(q, (p[0], p[1], p[2], 0.0)), inverse)[:3]


class Nav2GoalToScan(Node):
    """Request a Nav2 global path and publish a latest-TF odom path to SCAN."""

    def __init__(self):
        super().__init__("nav2_goal_to_scan")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("scan_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("planner_id", "GridBased")
        self.tf_buffer = Buffer(cache_time=Duration(seconds=20.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, "/initial_path", qos)
        self.goal_sub = self.create_subscription(PoseStamped, "/goal_pose", self.goal_cb, 10)
        self.plan_client = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def transform_pose(self, pose, target_frame):
        source_frame = pose.header.frame_id
        if not source_frame:
            raise ValueError("pose frame_id is empty")
        if source_frame == target_frame:
            out = PoseStamped()
            out.pose = pose.pose
            out.header.frame_id = target_frame
            out.header.stamp = self.get_clock().now().to_msg()
            return out
        tf = self.tf_buffer.lookup_transform(
            target_frame, source_frame, rclpy.time.Time(),
            timeout=Duration(seconds=0.3))
        tq = (tf.transform.rotation.x, tf.transform.rotation.y,
              tf.transform.rotation.z, tf.transform.rotation.w)
        pq = (pose.pose.orientation.x, pose.pose.orientation.y,
              pose.pose.orientation.z, pose.pose.orientation.w)
        rp = rotate(tq, (pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
        out = PoseStamped()
        out.header.frame_id = target_frame
        out.header.stamp = self.get_clock().now().to_msg()
        out.pose.position.x = rp[0] + tf.transform.translation.x
        out.pose.position.y = rp[1] + tf.transform.translation.y
        out.pose.position.z = rp[2] + tf.transform.translation.z
        oq = quat_multiply(tq, pq)
        out.pose.orientation.x = oq[0]
        out.pose.orientation.y = oq[1]
        out.pose.orientation.z = oq[2]
        out.pose.orientation.w = oq[3]
        return out

    def goal_cb(self, incoming):
        map_frame = self.get_parameter("map_frame").value
        base_frame = self.get_parameter("base_frame").value
        try:
            goal_pose = self.transform_pose(incoming, map_frame)
            base_tf = self.tf_buffer.lookup_transform(
                map_frame, base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.3))
        except (TransformException, ValueError) as exc:
            self.get_logger().error(f"Reject goal: current TF unavailable: {exc}")
            return
        if not self.plan_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Reject goal: /compute_path_to_pose is unavailable")
            return
        start = PoseStamped()
        start.header.frame_id = map_frame
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = base_tf.transform.translation.x
        start.pose.position.y = base_tf.transform.translation.y
        start.pose.position.z = base_tf.transform.translation.z
        start.pose.orientation = base_tf.transform.rotation
        request = ComputePathToPose.Goal()
        request.goal = goal_pose
        request.start = start
        request.use_start = True
        request.planner_id = self.get_parameter("planner_id").value
        future = self.plan_client.send_goal_async(request)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Nav2 rejected the global planning request")
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(self.path_result_cb)

    def path_result_cb(self, future):
        result = future.result().result
        if not result.path.poses:
            self.get_logger().error("Nav2 returned an empty path")
            return
        scan_frame = self.get_parameter("scan_frame").value
        output = Path()
        output.header.frame_id = scan_frame
        output.header.stamp = self.get_clock().now().to_msg()
        try:
            output.poses = [self.transform_pose(p, scan_frame) for p in result.path.poses]
        except (TransformException, ValueError) as exc:
            self.get_logger().error(f"Cannot transform Nav2 path to {scan_frame}: {exc}")
            return
        for pose in output.poses:
            pose.header = output.header
        self.path_pub.publish(output)
        self.get_logger().info(
            f"Published {len(output.poses)} Nav2 waypoints to SCAN in {scan_frame}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2GoalToScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
