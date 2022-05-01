# Import the necessary libraries
from typing import Any
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import sys
from torch import hub # Hub contains other models like FasterRCNN

thres = 0.45 # Threshold to detect object

classFile = 'src/prueba-camara/cv_basics/cv_basics/coco.names'
with open(classFile,'rt') as f:
  classNames = f.read().rstrip('\n').split('\n')

configPath = 'src/prueba-camara/cv_basics/cv_basics/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = 'src/prueba-camara/cv_basics/cv_basics/frozen_inference_graph.pb'

net = cv2.dnn_DetectionModel(weightsPath,configPath)
net.setInputSize(320,320)
net.setInputScale(1.0/ 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)


class ImageSubscriber(Node):
  def __init__(self):

    super().__init__('image_subscriber')
      
    #Subscribe the token
    self.subscription = self.create_subscription(
      Image, 
      'video_frames', 
      self.listener_callback, 
      10)
    self.subscription
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
   
  def listener_callback(self, data):
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)

    # Detect objects
    classIds, confs, bbox = net.detect(current_frame,confThreshold=thres)
 
    #Put the rectangle around the object and the text
    if len(classIds) != 0:
      for classId, confidence,box in zip(classIds.flatten(),confs.flatten(),bbox):
          cv2.rectangle(current_frame,box,color=(0,255,0),thickness=2)
          cv2.putText(current_frame,classNames[classId-1].upper(),(box[0]+10,box[1]+30),
          cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
          cv2.putText(current_frame,str(round(confidence*100,2)),(box[0]+200,box[1]+30),
          cv2.FONT_HERSHEY_COMPLEX,1,(0,255,0),2)
    
      cv2.imshow("camera", current_frame)
    
      cv2.waitKey(1)
  
def main(args=None):

  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  
  rclpy.spin(image_subscriber)

  image_subscriber.destroy_node()
  
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
