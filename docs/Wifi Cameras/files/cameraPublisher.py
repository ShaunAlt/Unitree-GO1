# Created by: Shaun Altmann
# Sourced: https://www.youtube.com/watch?v=6e94ZnYnO_U

# import opencv
import cv2

# used for environment variables
import os

# import ros packages
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

# create publisher node
class PublisherNodeClass(Node):
    # constructor
    def __init__(self):
        super().__init__('publisher_node')

        # set rtsp connection string
        self.rtsp = ''
        try:
            self.rtsp = os.environ['CAMERA_RTSP']
        except:
            raise KeyError(
                f'Unable to get RTSP connection string from environment variable `CAMERA_RTSP`'
            )
        
        # create camera
        self.camera = cv2.VideoCapture(self.rtsp)

        # create image bridge
        self.bridge = CvBridge()

        # create publisher
        self.publisher = self.create_publisher(Image, 'camera_image', 20)

        # create timer
        self.timer = self.create_timer(1.0 / 30.0, self.publish_image)

        # counter for how many images were published
        self.i = 0

    def publish_image(self):
        # get camera data
        success, frame = self.camera.read()
        # resize the image
        # frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)

        # if able to read the frame - publish message
        if success == True:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame))
            
            # log publish
            self.get_logger().info(f'Publishing Image #{self.i:05}')
            self.i += 1

# main function
def main(args=None):
    rclpy.init(args=args)
    node = PublisherNodeClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
