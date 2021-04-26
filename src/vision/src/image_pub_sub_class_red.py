#!/usr/bin/env python
from __future__ import print_function

import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import imutils

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    self.detection_pub = rospy.Publisher('/cmd_detection', String, queue_size=10)


    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/gopigo/camera1/image_raw",Image, self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    detectionMsg = String()
    detectionMsg.data = "negative"

    # IMAGE PROCESSING
    (rows,cols,channels) = cv_image.shape
    # resize the frame, blur it, and convert it to the HSV
    # color space

    blurred = cv2.GaussianBlur(cv_image, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "red", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    redLower = (0, 86, 6)
    redUpper = (10, 255, 255)
    mask1 = cv2.inRange(hsv, redLower, redUpper)

    redLower = (160, 86, 6)
    redUpper = (170, 255, 255)
    mask2 = cv2.inRange(hsv, redLower, redUpper)
    mask = mask1 + mask2
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    # find contours in the mask and initialize the current
    # (x, y) center of the ball
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    # only proceed if at least one contour was found
    if len(cnts) > 0:
      # find the largest contour in the mask, then use
      # it to compute the minimum enclosing circle and
      # centroid
      c = max(cnts, key=cv2.contourArea)
      ((x, y), radius) = cv2.minEnclosingCircle(c)
      M = cv2.moments(c)
      center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

      # compute permiter of the contour, ignoring possible artifacts 
      peri = cv2.arcLength(c, True)

      # geometric contruction of the approximated contour
      approx = cv2.approxPolyDP(c, 0.04 * peri, True)

      # only proceed if the radius meets a minimum size and a minimum number of verices
      if radius > 50 and len(approx) > 5:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv2.circle(cv_image, (int(x), int(y)), int(radius),
          (0, 255, 255), 2)
        cv2.circle(cv_image, center, 5, (0, 0, 255), -1)

        detectionMsg.data = "positive"
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_image,'Target found',(10,40), font, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.imwrite("images/copy/cv_image-target.jpg",cv_image)


    cv2.imshow("Mask window", mask)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.detection_pub.publish(detectionMsg)   
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)