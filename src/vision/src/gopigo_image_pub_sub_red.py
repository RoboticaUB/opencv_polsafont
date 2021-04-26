#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


import numpy as np

global bridge


class image_detection:

    def __init__(self):

        rospy.init_node("rubot_vision", anonymous=False)

        self._objectColor = rospy.get_param("~object_color")
        self._objectShape = rospy.get_param("~object_shape")
        self._objectSize = rospy.get_param("~object_size")

        self._cmdDetection = rospy.Publisher("/cmd_detection", String, queue_size=10)

        self._image_sub = rospy.Subscriber("/gopigo/camera1/image_raw",Image, self.image_callback)

        self._r = rospy.Rate(5)


    def image_callback(ros_image):
      #convert ros_image into an opencv-compatible image
      try:
        cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
      except CvBridgeError as e:
          print(e)
      #from now on, you can work exactly like with opencv
      (rows,cols,channels) = cv_image.shape
      msg = String()
      # TODO analize image and send detection message

      # preprocess image

      # blurring
      blur_cv_image = cv2.GaussianBlur(cv_image, (3, 3), 0)
      # color filtering
      redLower =(20, 20, 150)
      redUpper = (50, 50, 255)
      #binary_image_mask = filter_color(blur_cv_image, redLower, redUpper)

      # ball detection

      #contours = getContours(binary_image_mask)

      #draw ball
      
      #draw_ball_contour(binary_image_mask, rgb_image,contours)
      
      # image drawing and image   

      #set message: positive detection
      msg.data = "positive"

      #set message: negative detection
      msg.data = "negative"


      #publish on _cmdDetection topic         
      
      cmdDetection.publish(msg)

if __name__ == '__main__':
    try:
        bridge = CvBridge()
        vision = image_detection()
        rospy.spin()
    except rospy.ROSInterruptException: pass
    cv2.destroyAllWindows()
  
