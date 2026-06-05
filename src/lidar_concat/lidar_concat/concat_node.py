import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

TOPICS = [
    '/sensing/lidar/bottom/front/outlier_filtered/pointcloud',
    '/sensing/lidar/middle/front/outlier_filtered/pointcloud',
    '/sensing/lidar/middle/left/outlier_filtered/pointcloud',
    '/sensing/lidar/middle/right/outlier_filtered/pointcloud',
    '/sensing/lidar/top/rear/outlier_filtered/pointcloud',
]
OUTPUT_TOPIC = '/concatenated_pointcloud'


class ConcatNode(Node):
    def __init__(self):
        super().__init__('lidar_concat')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('max_age_sec', 0.5)
        publish_rate = self.get_parameter('publish_rate').value
        self.max_age = self.get_parameter('max_age_sec').value

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest = [None] * len(TOPICS)
        self._subs = []
        for i, topic in enumerate(TOPICS):
            sub = self.create_subscription(
                PointCloud2, topic,
                lambda msg, idx=i: self._on_msg(idx, msg),
                qos_profile=sub_qos,
            )
            self._subs.append(sub)

        self.pub = self.create_publisher(PointCloud2, OUTPUT_TOPIC, 10)
        self.create_timer(1.0 / publish_rate, self._tick)

        self.published = 0
        self.create_timer(5.0, self._report)

        self.get_logger().info(
            f'Subscribed to {len(TOPICS)} LiDAR topics; '
            f'publishing on {OUTPUT_TOPIC} at {publish_rate} Hz; '
            f'max_age={self.max_age}s'
        )

    def _on_msg(self, idx, msg):
        self.latest[idx] = msg

    def _stamp_seconds(self, msg):
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _tick(self):
        msgs = [m for m in self.latest if m is not None]
        if len(msgs) < len(TOPICS):
            return

        now_stamp = self._stamp_seconds(max(msgs, key=self._stamp_seconds))
        fresh = [
            m for m in msgs
            if (now_stamp - self._stamp_seconds(m)) <= self.max_age
        ]
        if len(fresh) < 2:
            return

        first = fresh[0]
        if not all(m.point_step == first.point_step for m in fresh):
            self.get_logger().warn(
                'point_step mismatch across LiDARs; skipping this tick'
            )
            return

        out = PointCloud2()
        out.header.stamp = max(fresh, key=self._stamp_seconds).header.stamp
        out.header.frame_id = first.header.frame_id
        out.fields = first.fields
        out.is_bigendian = first.is_bigendian
        out.point_step = first.point_step
        out.height = 1
        total_pts = sum(m.width * m.height for m in fresh)
        out.width = total_pts
        out.row_step = out.point_step * total_pts
        out.data = b''.join(bytes(m.data) for m in fresh)
        out.is_dense = all(m.is_dense for m in fresh)

        self.pub.publish(out)
        self.published += 1

    def _report(self):
        present = sum(m is not None for m in self.latest)
        self.get_logger().info(
            f'topics with messages: {present}/{len(TOPICS)}; '
            f'concatenated published: {self.published}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ConcatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
