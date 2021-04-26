#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from ball_detection import *

bridge = CvBridge()
# if not declared here callback does not detect it
detection_pub = rospy.Publisher('/cmd_detection', String, queue_size=10)


def image_callback(ros_image):
  global bridge, detection_pub

  targetFound = False

  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  (rows,cols,channels) = cv_image.shape
  #if cols > 200 and rows > 200 :
  #    cv2.circle(cv_image, (100,100),90, 255)
  

  
  
  ## 
  font = cv2.FONT_HERSHEY_SIMPLEX
  cv2.putText(cv_image,'Searching for target',(10,40), font, 1,(255,255,255),2,cv2.LINE_AA)
  cv2.imshow("Image window", cv_image)


  cv2.waitKey(3)
  msg = String()
  #detect_red_ball(cv_image)

  if targetFound:
    #set message: positive detection
    msg.data = "positive"
    cv2.imwrite("images/copy/"+ cv_image +"-red_ball.jpg",cv_image)
  else:
    #set message: negative detection
    msg.data = "negative"

  
  detection_pub.publish(msg)


def detect_red_ball(image):
      ## HSV o BGR ¿?
    redLower =(360, 0, 0)
    redUpper = (360, 100, 100)
    #redLower =(10, 10, 200)
    #redUpper = (20, 20, 255)

    # blurring
    # blur_cv_image = cv2.GaussianBlur(cv_image, (3, 3), 0)
    rgb_image = image
    binary_image_mask = filter_color(rgb_image, redLower, redUpper)

    contours = getContours(binary_image_mask)
    draw_ball_contour(binary_image_mask, rgb_image,contours)

  
def main(args):
  rospy.init_node('rubot_vision', anonymous=True)

  #detection_pub = rospy.Publisher('/cmd_detection', String, queue_size=10)
  image_sub = rospy.Subscriber("/gopigo/camera1/image_raw",Image, image_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
