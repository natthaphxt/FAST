#!/usr/bin/env python3
"""
M2DGR Right Fisheye Camera Decompression Node
Subscribes to /camera/right/image_raw/compressed
Publishes to /camera/right/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class RightCameraDecompressor(Node):
    def __init__(self):
        super().__init__('right_camera_decompressor')
        
        self.bridge = CvBridge()
        
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/right/image_raw/compressed',
            self.image_callback,
            10
        )
        
        self.pub = self.create_publisher(
            Image,
            '/camera/right/image_raw',
            10
        )
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Right Fisheye Camera Decompressor Started')
        self.get_logger().info('Resolution: 1280x1024')
        self.get_logger().info('Subscribe: /camera/right/image_raw/compressed')
        self.get_logger().info('Publish: /camera/right/image_raw')
        self.get_logger().info('=' * 50)
    
    def image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                return
            
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
            ros_image.header = msg.header
            
            self.pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RightCameraDecompressor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()