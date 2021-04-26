#!/usr/bin/env python

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class GoPiGo3:

    def __init__(self):

        rospy.init_node("rubot_nav", anonymous=False)

        self._distanceLaser = rospy.get_param("~distance_laser")
        self._speedFactor = rospy.get_param("~speed_factor")
        self._forwardSpeed = rospy.get_param("~forward_speed")
        self._backwardSpeed = rospy.get_param("~backward_speed")
        self._rotationSpeed = rospy.get_param("~rotation_speed")

        self._msg = Twist()
        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callbackLaser)
        rospy.on_shutdown(self.shutdown)

        self._r = rospy.Rate(5)

    def start(self):

        while not rospy.is_shutdown():
            self._cmdVel.publish(self._msg)
            self._r.sleep()

    def callbackLaser(self, scan):

        closestDistance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges) if scan.range_min < val < scan.range_max
        )
        angleClosestDistance = self.__wrapAngle(elementIndex) # only 360 points in simulated lidar!!!
        #angleClosestDistance = self.__wrapAngle(elementIndex / 2) # Real YDLidar with 720 points!!!
        rospy.loginfo("Closest distance of %5.2f m at %5.1f degrees.",
                      closestDistance, angleClosestDistance)
       
        # CAS 1 OBSTACLE FRONTAL: EVITAR
        if closestDistance <  1.5* self._distanceLaser and -60 < angleClosestDistance < 60:
            self._msg.linear.x = self._backwardSpeed * self._speedFactor
            self._msg.angular.z = self.__sign(
                angleClosestDistance) * self._rotationSpeed * self._speedFactor
            rospy.logwarn("Within laser distance threshold. Rotating (z=%4.1f)...", self._msg.angular.z)
        
        
        # CAS 2 OBSTACLE DRETA: SEGUIR PARET DRETA    
        elif   (2*self._distanceLaser <= closestDistance < 5 *self._distanceLaser) and ( 45 <= angleClosestDistance <= 135 ):
            if angleClosestDistance < 90:
                 ## change angular to follow wall
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = 1 * self._rotationSpeed * self._speedFactor
                rospy.logwarn("FOLLOW rigth wall  45-90 degrees. Rotating(z=%4.1f)...", self._msg.angular.z)

            
            if angleClosestDistance >= 90:
                ## change angular to follow wall
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = -1 * self._rotationSpeed * self._speedFactor      
                rospy.logwarn("FOLLOW rigth wall 90-135 degrees. Rotating (z=%4.1f)...", self._msg.angular.z)
              
     
         # CAS 3 OBSTACLE DRETA: SEGUIR PARET ESQUERRA
        
        elif   (2*self._distanceLaser <= closestDistance < 5 *self._distanceLaser) and ( -135 <= angleClosestDistance <= -45 ):
            if angleClosestDistance < -90:
                 ## change angular to follow wall
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = 1 * self._rotationSpeed * self._speedFactor
                rospy.logwarn("FOLLOW left wall  -45 to -90 degrees. Rotating(z=%4.1f)...", self._msg.angular.z)

            
            if angleClosestDistance >= -90:
                ## change angular to follow wall
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.angular.z = -1 * self._rotationSpeed * self._speedFactor      
                rospy.logwarn("FOLLOW left wall -90to  -135 degrees. Rotating (z=%4.1f)...", self._msg.angular.z)

          
        # CAS 4 CAP OBSTACLE ENDAVANT
        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0
            rospy.logwarn("No obstacles. Moving forward (x=%4.1f)...", self._msg.linear.x)


    def __sign(self, val):

        if val >= 0:
            return 1
        else:
            return -1

    def __wrapAngle(self, angle):
        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360

    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)

if __name__ == '__main__':
    try:
        gpg = GoPiGo3()
        gpg.start()
        rospy.spin()
    except rospy.ROSInterruptException: pass