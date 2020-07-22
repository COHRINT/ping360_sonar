#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import tf
from ping360_sonar.srv import SetSonarSettings, SetSonarMode
from ping360_sonar.msg import SonarSettings
from minau.msg import SonarTargetList, SonarTarget
import numpy as np

THIRTY_DEG = np.pi/6
SEARCH_DISTANCE = 10
SCAN_ANGLES_RANGE = 20
 
class ScannerControl:
    def __init__(self):
        self.own_yaw = None
        ownship = "etddf/estimate" + rospy.get_namespace()[:-1]        
        rospy.Subscriber(ownship,Odometry,self.pose_callback)
        # rospy.Subscriber("ping_360_target",SonarTargetList,self.check_for_landmark)
        self.landmark_x = rospy.get_param("~landmark_x",10)
        self.landmark_y = rospy.get_param("~landmark_y",0)

        # self.seen_landmark = False
        # self.previous = None

        rospy.Service("ping360_node/sonar/set_scan_mode", SetSonarMode, self.handle_scan_mode)

    def handle_scan_mode(self, req):

        print("Waiting for ping360_sonar/sonar/set_sonar_settings")
        rospy.wait_for_service("ping360_sonar/sonar/set_sonar_settings")
        set_sonar = rospy.ServiceProxy("ping360_node/sonar/set_sonar_settings",SetSonarSettings)

        min_scan_angle = None
        max_scan_angle = None
        # SCAN360
        if req.mode.mode == req.mode.SCAN360:
            min_scan_angle = 0
            max_scan_angle = 360
        else: # TRACK
            # Use our heading to determine where the object is
            if req.object == "landmark":
                position = self.ownship_pose.pose.position
                angle = np.arctan2(self.landmark_y - position.y,self.landmark_x - position.x)
                target_angle = angle - self.ownship_yaw
                min_scan_angle = target_angle - SCAN_ANGLES_RANGE / 2
                max_scan_angle = target_angle + SCAN_ANGLES_RANGE / 2
            else:
                raise NotImplementedError("Tracking of: " + req.object)

        try:
            print("Configuring settings of: " + str(min_scan_angle) + " - " + str(max_scan_angle))
            # resp = set_sonar(SonarSettings(min_scan_angle, max_scan_angle, SEARCH_DISTANCE))
            # if resp == True:
            #     print("Successfully changed settings")
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        return True

    def pose_callback(self,msg):
        """
        Takes from etddf estimate the ownship pose estimate

        Args:
            msg (Odometry): etddf ownship estimate
        """
        self.ownship_pose = msg
        (r, p, self.ownship_yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
 
    # def check_for_landmark(self,msg):
    #     for i in range(len(msg.targets)):
    #         print("I saw a " + msg.targets[i].id)
    #         if msg.targets[i].id == "landmark":
    #             self.seen_landmark = True
 
    # def run(self):
    #     """
    #     Read in settings from 
    #     """
    #     rate = rospy.Rate(5)
    #     while True:
    #         if self.own_yaw == None:
    #             rate.sleep()
    #             continue
    #         if self.own_pose.pose.covariance[0] + self.own_pose.pose.covariance[7] > self.threshold:
    #             print("Passed threshold")
    #             self.previous = self.get_sonar().settings
    #             distance = self.get_dist([self.own_pose.pose.pose.position.x,self.own_pose.pose.pose.position.y])
    #             relative_diff = [self.landmark_loc[0]-self.own_pose.pose.pose.position.x,self.landmark_loc[1]-self.own_pose.pose.pose.position.y]
    #             angle = np.arctan2(relative_diff[1],relative_diff[0])
    #             # print(relative_diff)
    #             # print(angle)
    #             angle = angle - self.own_yaw
    #             # print(angle)
    #             self.seen_landmark = False
    #             num = 2
    #             while not self.seen_landmark:
    #                 settings = SonarSettings()
    #                 settings.range = distance+np.sqrt(self.own_pose.pose.covariance[0])*num
    #                 #make sonar point turn toward where we think landmark is
    #                 settings.min_angle = angle-THIRTY_DEG
    #                 settings.max_angle = angle+THIRTY_DEG
    #                 self.set_sonar(settings).time-.5
    #                 #make sonar go 360
    #                 settings.min_angle = -4
    #                 settings.max_angle = 4
    #                 wait_time = self.set_sonar(settings).time-.5
    #                 start_time = convert_ros_time(rospy.get_rostime())
    #                 rate2 = rospy.Rate(10)
    #                 while (convert_ros_time(rospy.get_rostime()) - start_time < wait_time) and not self.seen_landmark:
    #                     rate2.sleep()
    #                 #if the landmark hasn't been seen, check a wider area
    #                 if not self.seen_landmark:
    #                     print('Expanding Range')
    #                     num+=1
    #                 else:
    #                     print('I saw the landmark!')
    #                     self.set_sonar(self.previous)
 
if __name__ == "__main__":
    rospy.init_node("sonar_control")
    sc = ScannerControl()
    rospy.spin()