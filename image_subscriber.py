import rclpy
from rclpy.node import Node
# 1. Import the standard, uncompressed Image message type
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RawImageSubscriber(Node):
    def __init__(self):
        super().__init__('raw_image_subscriber')
        
        # Subscribe to the uncompressed image topic
        self.subscription = self.create_subscription(
            Image,
            '/img',
            self.listener_callback,
            10
        )
        
        # Initialize the CvBridge
        self.bridge = CvBridge()
        self.get_logger().info("Raw Image Subscriber Node has started and is listening to /img")

    def listener_callback(self, data):
        try:
            # 2. Use imgmsg_to_cv2 (instead of compressed_imgmsg_to_cv2)
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
            # Display the image window
            cv2.imshow("ROS2 Raw Image from Gazebo", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Failed to convert raw image: {e}")

def main(args=None):
    rclpy.init(args=args)
    raw_image_subscriber = RawImageSubscriber()
    
    try:
        rclpy.spin(raw_image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up windows and shutdown node safely
        cv2.destroyAllWindows()
        raw_image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()