# Created by: Shaun Altmann
# Sourced: https://www.youtube.com/watch?v=6e94ZnYnO_U

# import opencv
import cv2

# import ros packages
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

# create subcriber
class SubscriberNodeClass(Node):
    def __init__(self):
        super().__init__('subscriber_node')

        # bridge opencv images to ros
        self.bridge = CvBridge()

        self.create_subscription(Image, 'camera_image', self.read_image, 20)

    def read_image(self, msg):
        self.get_logger().info('The image frame is received')
        img = self.bridge.imgmsg_to_cv2(msg)
        cv2.imshow("Camera Video", img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNodeClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()