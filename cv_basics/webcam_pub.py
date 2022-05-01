# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
 
class ImagePublisher(Node):
  def __init__(self):

    super().__init__('image_publisher')
      
    self.publisher_ = self.create_publisher(Image, 'video_frames', 10)
      
    timer_period = 0.1  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    self.cap = cv2.VideoCapture(0)
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def timer_callback(self):

    # Capture frame-by-frame
    # This method returns True/False as well
    ret, frame = self.cap.read()
          
    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV image to a ROS 2 image message
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
      
    else:
      self.get_logger().info('dont recognize the camera')
      
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  rclpy.spin(image_publisher)
  
  image_publisher.destroy_node()
  
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
