#!/usr/bin/env python3
"""
M2DGR Color Camera (RealSense D435i) Decompression Node
Subscribes to /camera/color/image_raw/compressed
Publishes to /camera/color/image_raw
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorCameraDecompressor(Node):
    def __init__(self):
        super().__init__('color_camera_decompressor')
        
        self.bridge = CvBridge()
        
        # Subscribe to RealSense D435i color camera (compressed)
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/color/image_raw/compressed',
            self.image_callback,
            10
        )
        
        # Publish decompressed raw images
        self.pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('Color Camera Decompressor Started')
        self.get_logger().info('Camera: RealSense D435i (640x480)')
        self.get_logger().info('Subscribe: /camera/color/image_raw/compressed')
        self.get_logger().info('Publish: /camera/color/image_raw')
        self.get_logger().info('=' * 50)
    
    def image_callback(self, msg):
        try:
            # Decompress JPEG
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is None:
                self.get_logger().warn('Failed to decode image')
                return
            
            # Convert BGR to RGB (ROS convention)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Convert to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')
            ros_image.header = msg.header  # Preserve timestamp
            
            # Publish
            self.pub.publish(ros_image)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ColorCameraDecompressor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()