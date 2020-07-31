#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import tf
from ping360_sonar.srv import SetSonarSettings, SetSonarMode
from ping360_sonar.msg import SonarSettings
from minau.msg import SonarTargetList, SonarTarget
import numpy as np
from cuprint.cuprint import CUPrint

THIRTY_DEG = np.pi/6
DEFAULT_SCAN_360_RANGE_M = 5
SCAN_ANGLES_RANGE = 20 * (np.pi / 180)
ADDITIONAL_SEARCH_ROOM = 2 # When tracking an object or asset, add this distance to the scan range
 
class ScannerControl:
    def __init__(self):
        self.cuprint = CUPrint("Sonar Control")
        self.own_yaw = None
        # ownship = "etddf/estimate" + rospy.get_namespace()[:-1]
        ownship = rospy.get_namespace()[:-1] + "/pose_gt"
        rospy.Subscriber(ownship,Odometry,self.pose_callback)
        rospy.Subscriber("etddf/estimate/red_actor_0", Odometry, self.red_actor_callback)
        # rospy.Subscriber("ping_360_target",SonarTargetList,self.check_for_landmark)
        self.landmark_x = rospy.get_param("~landmark_x",10)
        self.landmark_y = rospy.get_param("~landmark_y",0)
        self.scan_update_rate = rospy.get_param("~scan_update_rate_hz")
        self.scan_range_360 = rospy.get_param("~360_scan_range_m", DEFAULT_SCAN_360_RANGE_M)

        self._red_team_names = rospy.get_param('~red_team_names',[])
        self._red_agent_id = self._red_team_names[0]

        self.cuprint("waiting for set_sonar_settings service")
        rospy.wait_for_service("ping360_node/sonar/set_sonar_settings")
        self.set_sonar = rospy.ServiceProxy("ping360_node/sonar/set_sonar_settings",SetSonarSettings)
        self.cuprint("...service found")

        self.last_req = None

        # self.seen_landmark = False
        # self.previous = None

        rospy.Service("ping360_node/sonar/set_scan_mode", SetSonarMode, self.handle_scan_mode)

    def run(self):
        normalize_angle = lambda angle : np.mod( angle + np.pi, 2*np.pi) - np.pi # [-pi,pi]

        r = rospy.Rate(self.scan_update_rate)
        while not rospy.is_shutdown():

            req = self.last_req
            if req != None:
                min_scan_angle = None
                max_scan_angle = None
                scan_range = None
                # SCAN360
                if req.mode.mode == req.mode.SCAN360:
                    min_scan_angle = 0
                    max_scan_angle = 360
                    scan_range = self.scan_range_360
                elif req.mode.object == "landmark": # TRACK LANDMARK
                    # Use our heading to determine where the object is
                    position = self.ownship_pose.pose.pose.position
                    angle = np.arctan2(self.landmark_y - position.y,self.landmark_x - position.x)
                    target_angle = angle - self.ownship_yaw
                    min_scan_angle = normalize_angle( target_angle - SCAN_ANGLES_RANGE / 2 )
                    max_scan_angle = normalize_angle( target_angle + SCAN_ANGLES_RANGE / 2 )
                    # Convert to degrees
                    min_scan_angle *= (180/np.pi)
                    max_scan_angle *= (180/np.pi)
                    scan_range = int(np.linalg.norm([self.landmark_x - position.x, self.landmark_y - position.y])) + ADDITIONAL_SEARCH_ROOM
                elif req.mode.object == self._red_agent_id:
                    # Use our heading to determine where the object is
                    position = self.ownship_pose.pose.pose.position
                    red_position = self.red_actor_pose.pose.pose.position
                    angle = np.arctan2(red_position.y - position.y,red_position.x - position.x)
                    target_angle = angle - self.ownship_yaw
                    min_scan_angle = normalize_angle( target_angle - SCAN_ANGLES_RANGE / 2 )
                    max_scan_angle = normalize_angle( target_angle + SCAN_ANGLES_RANGE / 2 )
                    # Convert to degrees
                    min_scan_angle *= (180/np.pi)
                    max_scan_angle *= (180/np.pi)
                    scan_range = int(np.linalg.norm([red_position.x - position.x, red_position.y - position.y])) + ADDITIONAL_SEARCH_ROOM
                try:
                    print("Configuring settings of: " + str(min_scan_angle) + " - " + str(max_scan_angle))
                    resp = self.set_sonar(SonarSettings(min_scan_angle, max_scan_angle, scan_range))
                    if resp == True:
                        print("Successfully changed settings")
                except rospy.ServiceException as exc:
                    print("Service did not process request: " + str(exc))

            r.sleep()


    def handle_scan_mode(self, req):
        if req.mode.mode == req.mode.SCAN360:
            self.cuprint("Scanning 360")
        elif req.mode.object == "landmark":
            self.cuprint("Tracking Landmark")
        elif req.mode.object == "red_actor_0":
            self.cuprint("Tracking Red Actor")
        else:
            raise NotImplementedError("Tracking of: " + req.object)
        self.last_req = req
        return True

    def red_actor_callback(self, msg):
        self.red_actor_pose = msg

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
    sc.run()