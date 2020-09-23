#!/usr/bin/env python
from __future__ import division
import rospy
from nav_msgs.msg import Odometry
import tf
from ping360_sonar.srv import SetSonarSettings, SetSonarMode, SetSonarModeRequest
from ping360_sonar.msg import SonarSettings, SonarMode
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
        rospy.wait_for_message(ownship, Odometry)
        rospy.Subscriber("etddf/estimate/red_actor_0", Odometry, self.red_actor_callback)
        # rospy.Subscriber("ping_360_target",SonarTargetList,self.check_for_landmark)
        self.landmark_dict = rospy.get_param("~landmarks")
        default_track = rospy.get_param("~default_track")
        print(default_track)
        if default_track != "None":
            self.last_req = SetSonarModeRequest()
            self.last_req.mode.object = default_track
            self.last_req.mode.mode = self.last_req.mode.TRACK
            self.cuprint("Tracking Landmark: " + self.last_req.mode.object[len("landmark_"):])
        else:
            self.last_req = None
        self.scan_update_rate = rospy.get_param("~scan_update_rate_hz")
        self.scan_range_360 = rospy.get_param("~360_scan_range_m", DEFAULT_SCAN_360_RANGE_M)

        self._red_team_names = rospy.get_param('~red_team_names',[])
        if len(self._red_team_names) > 0:
            self._red_agent_id = self._red_team_names[0]
        else:
            self._red_agent_id = -1

        self.cuprint("waiting for set_sonar_settings service")
        rospy.wait_for_service("ping360_node/sonar/set_sonar_settings")
        self.set_sonar = rospy.ServiceProxy("ping360_node/sonar/set_sonar_settings",SetSonarSettings)
        self.cuprint("...service found")

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
                elif "landmark" in req.mode.object: # TRACK LANDMARK
                    # Use our heading to determine where the object is
                    position = self.ownship_pose.pose.pose.position
                    landmark_name = req.mode.object[len("landmark_"):]
                    landmark_x, landmark_y, landmark_z = self.landmark_dict[landmark_name]
                    angle = np.arctan2(landmark_y - position.y,landmark_x - position.x)
                    target_angle = angle - self.ownship_yaw
                    min_scan_angle = normalize_angle( target_angle - SCAN_ANGLES_RANGE / 2 )
                    max_scan_angle = normalize_angle( target_angle + SCAN_ANGLES_RANGE / 2 )
                    # Convert to degrees
                    min_scan_angle *= (180/np.pi)
                    max_scan_angle *= (180/np.pi)
                    # scan_range = int(np.linalg.norm([landmark_x - position.x, landmark_y - position.y])) + ADDITIONAL_SEARCH_ROOM
                    scan_range = 10
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
        elif "landmark" in req.mode.object :
            self.cuprint("Tracking Landmark: " + req.mode.object[len("landmark_"):])
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
 
if __name__ == "__main__":
    rospy.init_node("sonar_control")
    sc = ScannerControl()
    sc.run()