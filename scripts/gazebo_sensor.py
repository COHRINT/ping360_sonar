#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import copy
 
rospy.init_node("ping_gazebo_sonar")

 
class SonarPing:
    """
    Takes the raw sonar data that shows all 360 of pings every update and makes it so the sonar only sees
    whats in the sonars range of view. Also makes a service that can change the settings of the sonar
    """
    def __init__(self):
        """
        Subscribes to the etddf estimates and the raw sonar data
        """
        self.z = None
        ownship = "etddf/estimate" + rospy.get_namespace()[:-1]

        #subscribes to the estimates and the sonar data
        rospy.Subscriber(ownship,Odometry,self.own_pose_callback)
        rospy.Subscriber("ping360raw",LaserScan,self.sonar_callback)

        #Initiates the publishers
        self.filtered_sonarpub = rospy.Publisher("sonar_filtered",LaserScan,queue_size=10)

            
    def sonar_callback(self,msg):
        """
        Takes in the raw sonar measurment and publishes what the sonar can actually see given during a given time

        Args:
            msg (LaserScan): [Raw sonar data]
        """
        if self.z == None:
            return
        self.sonar = msg
        #this pulls from the message the horizontal samples and vertical samples
        #assumes the sonar is 360
        self.horz_count = int(round(np.pi*2 / msg.angle_increment))
        self.vert_count = int(len(msg.ranges)/self.horz_count)
        #this makes an array, one spot for every horizontal angle, and puts the shortest range in that value
        self.detect = [-1 for i in range(self.horz_count)]
        for i in range(len(msg.ranges)):
            if msg.ranges[i]<=msg.range_max:
                if self.detect[i%self.horz_count] == -1 or self.detect[i%self.horz_count]>msg.ranges[i]:
                    if not self.above_water(i,msg.ranges[i]):
                        self.detect[i%self.horz_count] = msg.ranges[i]
        
        total_view = copy.deepcopy(self.sonar)
        total_view.ranges = self.detect

        self.filtered_sonarpub.publish(total_view)
    def own_pose_callback(self,msg):
        self.z = msg.pose.pose.position.z
    def above_water(self,idx,rng):
        if idx < (self.horz_count*self.vert_count)/2:
            return False
        angIdx = int(idx/self.horz_count)
        ang = (-12.5*np.pi/180) + (angIdx *np.pi/180.)
        z = self.z + rng*np.sin(ang)
        if z <= 0:
            return False
        return True

        

 
if __name__ == "__main__":
    sp = SonarPing()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()