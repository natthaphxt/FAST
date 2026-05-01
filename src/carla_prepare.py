#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Imu, Image
import sensor_msgs_py.point_cloud2 as pc2
import math
import numpy as np

class CarlaLivoPrep(Node):
    def __init__(self):
        super().__init__('carla_livo_prep')
        
        self.create_subscription(Imu, '/carla/ego_vehicle/imu', self.imu_cb, 100)
        self.create_subscription(PointCloud2, '/carla/ego_vehicle/lidar', self.lidar_cb, 10)
        self.create_subscription(Image, '/carla/ego_vehicle/rgb_left/image', self.image_cb, 10)
        
        self.imu_pub = self.create_publisher(Imu, '/sync/imu', 100)
        self.lid_pub = self.create_publisher(PointCloud2, '/velodyne_points', 10)
        self.img_pub = self.create_publisher(Image, '/sync/image', 10)
        
        self.get_logger().info('LIVO Data Preparer Started!')

    def imu_cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub.publish(msg)

    def image_cb(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.img_pub.publish(msg)

    def lidar_cb(self, msg):
        now = self.get_clock().now().to_msg()

        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        if not points:
            return

        cloud_data = np.array(points)

        if cloud_data.dtype.names is not None:
            x = cloud_data['x'].astype(np.float32)
            y = cloud_data['y'].astype(np.float32)
            z = cloud_data['z'].astype(np.float32)
            intensity = cloud_data['intensity'].astype(np.float32)
        else:
            x = cloud_data[:, 0].astype(np.float32)
            y = cloud_data[:, 1].astype(np.float32)
            z = cloud_data[:, 2].astype(np.float32)
            intensity = cloud_data[:, 3].astype(np.float32)

        dist = np.sqrt(x*x + y*y)
        valid_mask = dist >= 0.1
        x, y, z, intensity, dist = (
            x[valid_mask], y[valid_mask], z[valid_mask],
            intensity[valid_mask], dist[valid_mask]
        )

        num_points = len(x)
        if num_points == 0:
            return

        lower_fov_rad = -10.0 * math.pi / 180.0
        total_fov_rad =  25.0 * math.pi / 180.0
        num_channels = 32

        v_angle = np.arctan2(z, dist)
        ring = ((v_angle - lower_fov_rad) / total_fov_rad * (num_channels - 1))
        ring = np.clip(np.round(ring), 0, num_channels - 1).astype(np.uint16)
        time_off = np.linspace(0.0, 0.05, num_points, endpoint=False, dtype=np.float32)

        struct_dtype = np.dtype([
            ('x', np.float32), ('y', np.float32), ('z', np.float32),
            ('intensity', np.float32), ('time', np.float32),
            ('ring', np.uint16), ('pad', np.uint16)
        ])

        out_data = np.zeros(num_points, dtype=struct_dtype)
        out_data['x'] = x
        out_data['y'] = y
        out_data['z'] = z
        out_data['intensity'] = intensity
        out_data['time'] = time_off
        out_data['ring'] = ring

        fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time',      offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring',      offset=20, datatype=PointField.UINT16,  count=1),
        ]

        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.header.stamp = now
        new_msg.height = 1
        new_msg.width = num_points
        new_msg.fields = fields
        new_msg.is_bigendian = False
        new_msg.point_step = 24
        new_msg.row_step = 24 * num_points
        new_msg.data = out_data.tobytes()
        new_msg.is_dense = True
        self.lid_pub.publish(new_msg)

def main():
    rclpy.init()
    node = CarlaLivoPrep()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
