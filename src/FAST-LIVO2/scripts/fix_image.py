#!/usr/bin/env python3
"""Fix image encoding for FAST-LIVO2 - ensure clean rgb8 with correct step.

Also masks out the car hood at the bottom of the frame so feature extraction
ignores static-on-vehicle pixels.

Set env var FIX_IMAGE_NO_MASK=1 to disable hood mask (for ablation test).
"""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import struct

DISABLE_HOOD_MASK = os.environ.get('FIX_IMAGE_NO_MASK', '0') == '1'

# Hood top edge — list of (x, y) anchors describing the top boundary of the
# car hood from left to right. Everything BELOW this curve is masked black.
# Image is 672×376. Center is highest (windshield wiper peak), sides slope down.
# Tune these if the mask cuts road or leaves visible hood.
HOOD_TOP_POINTS = [
    (0,   340),
    (75,  325),
    (250, 295),
    (336, 275),   # wiper peak (image center)
    (422, 295),
    (597, 325),
    (672, 340),
]

def build_hood_mask(width, height):
    """Boolean mask: True for pixels under the hood polygon (to be zeroed)."""
    xs, ys = zip(*HOOD_TOP_POINTS)
    y_top = np.interp(np.arange(width), xs, ys).astype(np.int32)
    rows = np.arange(height).reshape(-1, 1)
    return rows >= y_top.reshape(1, -1)


class FixImage(Node):
    def __init__(self):
        super().__init__('fix_image')
        self.sub = self.create_subscription(Image, '/zed2/camera/left/image_raw', self.cb, 10)
        self.pub = self.create_publisher(Image, '/camera/image_fixed', 10)
        self.count = 0
        self.hood_mask = None  # (height, width) bool, lazily built once

    def cb(self, msg):
        try:
            raw = bytes(msg.data)
            
            # Try as bgr8 (3 bytes per pixel)
            expected_3ch = msg.width * msg.height * 3
            # Try as bgra8 (4 bytes per pixel)  
            expected_4ch = msg.width * msg.height * 4
            
            if self.count == 0:
                self.get_logger().info(f'Image: {msg.width}x{msg.height}, encoding={msg.encoding}, step={msg.step}')
                self.get_logger().info(f'Data len={len(raw)}, expected 3ch={expected_3ch}, expected 4ch={expected_4ch}')
            
            if len(raw) == expected_4ch or msg.step == msg.width * 4:
                # Actually 4 channel! Convert bgra8 -> rgb8
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                rgb = arr[:, :, 2::-1].copy()  # BGRA -> RGB (take first 3 channels, reverse)
                if self.count == 0:
                    self.get_logger().info('Detected 4-channel image, converting BGRA->RGB')
            elif msg.step != msg.width * 3:
                # Step mismatch - there's padding per row
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(msg.height, msg.step)
                bgr = arr[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
                rgb = bgr[:, :, ::-1].copy()
                if self.count == 0:
                    self.get_logger().info(f'Step mismatch detected (step={msg.step} vs width*3={msg.width*3}), fixing')
            else:
                # Normal bgr8 -> rgb8
                arr = np.frombuffer(raw, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                rgb = arr[:, :, ::-1].copy()
                if self.count == 0:
                    self.get_logger().info('Normal bgr8->rgb8 conversion')
            
            # Mask out the car hood (polygon-shaped, computed once)
            if not DISABLE_HOOD_MASK:
                if self.hood_mask is None or self.hood_mask.shape != (msg.height, msg.width):
                    self.hood_mask = build_hood_mask(msg.width, msg.height)
                rgb[self.hood_mask] = 0
            elif self.count == 0:
                self.get_logger().info('FIX_IMAGE_NO_MASK=1 — hood mask DISABLED')

            out = Image()
            out.header = msg.header
            out.height = msg.height
            out.width = msg.width
            out.encoding = 'rgb8'
            out.is_bigendian = 0
            out.step = msg.width * 3
            out.data = rgb.tobytes()
            self.pub.publish(out)
            self.count += 1
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main():
    rclpy.init()
    node = FixImage()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
