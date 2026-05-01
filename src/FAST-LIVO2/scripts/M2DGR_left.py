#!/usr/bin/env python3
"""
M2DGR Image Decompression Node
Subscribes to compressed images and republishes as raw images for FAST-LIVO2
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class M2DGRImageDecompressor(Node):
    def __init__(self):
        super().__init__('m2dgr_image_decompressor')
        
        self.bridge = CvBridge()
        
        # Subscribe to compressed left camera
        self.sub = self.create_subscription(
            CompressedImage,
            '/camera/left/image_raw/compressed',
            self.image_callback,
            10
        )
        
        # Publish decompressed images
        self.pub = self.create_publisher(
            Image,
            '/camera/left/image_raw',
            10
        )
        
        self.get_logger().info('M2DGR Image Decompressor started')
        self.get_logger().info('Subscribed to: /camera/left/image_raw/compressed')
        self.get_logger().info('Publishing to: /camera/left/image_raw')
    
    def image_callback(self, msg):
        try:
            # Decompress the image
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
    node = M2DGRImageDecompressor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()