#!/usr/bin/env python
import rospy
from ping360_sonar.msg import SonarEcho
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from etddf.msg import NetworkEstimate, AssetEstimate
from minau.msg import SonarTargetList, SonarTarget
from scipy import stats
import cv2 as cv 
import csv
import os
import tf


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2*np.pi
    while angle < -np.pi:
        angle += 2*np.pi
    return angle



class Detection:
    def __init__(self):
        self.own_yaw = None
        ownship = "etddf/estimate" + rospy.get_namespace()[:-1]
        otherAsset = "etddf/estimate/bluerov2_4"

        self.landmark_pos = None
        self.is_landmark = rospy.get_param("~is_landmark")
        if self.is_landmark:
            x = rospy.get_param("~landmark_x")
            y = rospy.get_param("~landmark_y")
            self.landmark_pos = np.array([x,y])
        
        
        

        #Subscribe to the sonar data
        rospy.Subscriber("ping360_node/sonar/data",SonarEcho,self.data_callback)
        rospy.Subscriber("ping360_node/sonar/images",Image,self.image_callback)
        rospy.Subscriber(ownship,Odometry,self.own_pose_callback)
        rospy.Subscriber("etddf/estimate/network",NetworkEstimate,self.network_callback)

        #Set up publisher to republish image with detections marked as red on them
        self.image_pub = rospy.Publisher("ping360_node/sonar/modified_image",Image,queue_size=10)
        self._sonar_targets_publisher = rospy.Publisher(
                        'ping_360_target',
                        SonarTargetList, queue_size=10)

        self.raw_image = None
        self.avg = [0]*200
        self.done = False
        self.count = 0
        self._sequential_observation_id = 1
        # detections is an array of array of detections, with each detection having a angle and index
        # ex: [[[ang1,id1],[ang2,id2],[ang3,id3]],[[ang11,id11],[ang12,id12],[ang13,id13]]]
        # all the first 3 belong together, and the second 3 belong together
        self.detections = []
        # the confirmed detections are ones that have passed all the tests and that we are going to draw
        self.confirmed_detections = []
        self.bridge = CvBridge()
        self.num_samp = None
        self.range = None
    
    def own_pose_callback(self,msg):
        """
        Takes from etddf estimate the ownship pose estimate

        Args:
            msg (Odometry): etddf ownship estimate
        """
        self.own_pose = msg
        self.own_position = msg.pose.pose.position
        (r, p, self.own_yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # print(self.own_yaw)


    def data_callback(self,msg):
        """Takes in the sonar data and determines if there is a detection

        Args:
            msg (SonarEcho): raw data from ping360 sonar
        """
        if self.own_yaw==None:
            return
        if rospy.get_rostime().secs < 6:
            return
        self.range = msg.range
        self.num_samp = len(msg.intensities)

        #this reads in the raw intensity data and puts it in an array, use ord because it thinks 8 bit unsigned ints are characters
        intens = [0]*self.num_samp
        for i in range(self.num_samp):
            intens[i] = ord(msg.intensities[i])
        
        #initializes prevous intensities we are going to check against
        prevIntens = [255,255,255] 

        count = 0
        high_intensity = False
        hi_intens_idx = -1
        #loops through the intensity array
        for i in range(len(intens)):
            if i < 15:
                continue
            #checks for spike in intensity
            if not high_intensity:
                intensity = intens[i]
                minPrev = min(prevIntens)
                if intensity > (minPrev+80):
                    hi_intens_idx = i
                    high_intensity = True
                prevIntens.pop(0)
                prevIntens.append(intens[i])
            else:
                count += 1
                if count == 10:
                    break
                if intens[i] > intens[hi_intens_idx]:
                    hi_intens_idx = i

        #if there was a spike, see if end of array has a lot of zeros
        if high_intensity:
            num_to_look = int((self.num_samp-hi_intens_idx)/2)
            total = 0
            for i in range(num_to_look):
                total += intens[-1-i]
            avg = (total*1.0)/num_to_look
            # if the average of the end of array is really low, classify as detection
            if avg < 2:
                # if msg.angle > 155 and msg.angle < 170:
                #     print(str(msg.angle) + ' : '+ str(hi_intens_idx)+' : '+str(avg))
                #     print(intens)
                #     if msg.angle == 93:
                #         plt.plot(intens)
                #         plt.xlabel('index')
                #         plt.ylabel('intensity')
                #         plt.savefig('error.png')
                single_detect = [msg.angle,hi_intens_idx]
                foundMatch = False
                # see if detection matches any we already saw and add it to corresponding or if not found make new detection
                for i in range(len(self.detections)):
                    for j in range(len(self.detections[i])):
                        if np.linalg.norm(self.detections[i][j][0]-single_detect[0]) < 5 and np.linalg.norm(self.detections[i][j][1]-single_detect[1]) < 20:
                            self.detections[i].append(single_detect)
                            foundMatch = True
                            break
                    if foundMatch:
                        break
                if not foundMatch:
                    self.detections.append([single_detect])
        #checks all detections to see if we havent see them in a bit, if there are 3 or more angles in the detection, assume it is real detection
        for i in range(len(self.detections)):
            close = False
            for j in range(len(self.detections[i])):
                if np.linalg.norm(msg.angle-self.detections[i][j][0]) < 5:
                    close = True
                    break
            if not close:
                det = self.detections.pop(i)
                if len(det) >= 3:
                    self.confirmed_detections.append(self.avg_detection(det))
                # else:
                #     print(str(msg.angle)+' : '+str(len(det)))
                break
    def avg_detection(self,det):
        """Takes in a detection and averages the distance and angle of grouped detection

        Args:
            det (array of detection angles and ranges): [description]

        Returns:
            [float,float]: [average angle and average range index]
        """
        ang_tot = 0
        idx_tot = 0
        for i in range(len(det)):
            ang_tot+=det[i][0]
            idx_tot+=det[i][1]
        ang_avg = (ang_tot*1.0)/len(det)
        idx_avg = (idx_tot*1.0)/len(det)
        dist_to = (float(idx_avg)/self.num_samp) * self.range
        self.determine_asset_detected(dist_to,ang_avg)
        print('Detection at angle ' + str(ang_avg) + ' with an index of '+str(idx_avg))
        return [ang_avg,idx_avg]
    def image_callback(self,msg):
        """Takes in an image and draws cirles where the detections are

        Args:
            msg (image): the raw sonar image
        """
        if self.num_samp == None:
            return
        self.raw_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")

        modified_image = self.raw_image
        for i in range(len(self.confirmed_detections)):
            center = self.find_center(self.confirmed_detections[i])
            cv.circle(modified_image,center, 2, (0,0,255), -1)

        image_message = self.bridge.cv2_to_imgmsg(modified_image,"bgr8")
        self.image_pub.publish(image_message)
    def find_center(self,detection):
        """Finds where the circle should be drawn on the image for a detection

        Args:
            detection ([float,float]): angle and range index of detection

        Returns:
            [(float,float)]: x,y where it should go on the image
        """
        center_img = 250
        rad_ang = detection[0]*np.pi/200
        range_d = (250 * float(detection[1]))/self.num_samp
        x = int(250 + range_d * np.sin(rad_ang))
        y = int(250 + range_d * np.cos(rad_ang))
        return (x,y)
    def network_callback(self,msg):
        """Takes etddf estimate of the other asset

        Args:
            msg (Odometry): etddf estimate of other asset
        """
        self.network = msg

                

    def determine_asset_detected(self,dist,ang):
        ang_rad = (ang-200) * np.pi/200
        ang_world = ang_rad + self.own_yaw
        ang_world = normalize_angle(ang_world)
        x = self.own_position.x + dist*np.cos(ang_world)
        y = self.own_position.y + dist*np.sin(ang_world)
        pos = np.array([x,y])
        p_network = []
        for i in range(len(self.network.assets)):
            if self.network.assets[i].name != rospy.get_namespace()[1:-1]:
                p = self.prob(self.network.assets[i].odom,pos)
                p_network.append([self.network.assets[i].name,p])

        p_asset = p_network[0]
        for i in range(len(p_network)):
            if p_network[i][1] > p_asset[1]:
                p_asset = p_network[i]
        
        p_landmark = 0
        if self.is_landmark:
            own_location = self.find_own_location(pos)
            p_landmark = self.prob(self.own_pose,own_location)
        
        location = " at " + str(pos[0]) + ', ' + str(pos[1])
        asset_id = None
        uuv_class = None
        if p_landmark < .05 and p_asset[1] < .05:
            print('I think I just saw the red asset'+location)
            asset_id = 'red_asset'
            uuv_class = SonarTarget.UUV_CLASS_RED
        else:
            if p_landmark > p_asset[1]:
                print('I think I just saw the landmark'+location)
                asset_id = 'landmark'
                uuv_class = SonarTarget.UUV_CLASS_UNKNOWN
            else:
                print('I think I just saw the other blue asset'+location)
                asset_id = p_asset[0]
                uuv_class = SonarTarget.UUV_CLASS_BLUE
        
        self.publish_detection(asset_id,[dist,ang_rad],uuv_class)

    def prob(self,odom,detection):
        """Determines probablity that a distribution could have a point with the malanobis distance or farther away as the detection

        Args:
            odom (Odometry): Odometry of the distribution we are checking the detection against
            detection ([float,float]): Location of detection we are checking

        Returns:
            [float]: the probablity that the distribution could have a point beloning to it that has a
            malonobis distance equal to or farther away as the detection
        """
        nav_cov = np.array(odom.pose.covariance).reshape(6,6)
        cov = np.zeros((2,2))
        cov[:2,:2] = nav_cov[:2,:2]
        if np.linalg.det(cov) < 0.1:
            cov = np.eye(2)*0.1
        estimate = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y])
        m_dist_x = np.dot((detection-estimate).transpose(),np.linalg.inv(cov))
        m_dist_x = np.dot(m_dist_x, (detection-estimate))
        return (1-stats.chi2.cdf(m_dist_x, 2))
    
    def find_own_location(self,detection):
        """Assumes detection is the pole and determines where the rovs ownship position would be if that were the pole

        Args:
            detection ([float,float]): x,y,z or detection based on ownship own estimate at the time

        Returns:
            [np.array(float,float)]: Ownship location if that is indeed the pole
        """
        own_pose = np.array([self.own_position.x,self.own_position.y])
        diff = self.landmark_pos - detection
        return own_pose+diff
    
    def publish_detection(self,id, detection,uuv_class):
        """Publishes the detection with OL message type

        Args:avg
            id (string): asset id
            detection ([float(range),float(bearing)]): where the detection was
        """
        target = SonarTarget()
        target.id = id
        target.elevation_rad = 0.0
        target.elevation_variance = 0.0
        target.bearing_rad = detection[1]
        target.bearing_variance = 2*np.pi/180
        target.range_m = detection[0]
        target.range_variance = 0.15**2
        target.associated = True
        target.type = None
        if id == "landmark":
            target.type = SonarTarget.TARGET_TYPE_OBJECT
        else:
            target.type = SonarTarget.TARGET_TYPE_UUV
        target.uuv_classification = uuv_class
        target_list = SonarTargetList()
        target_list.header.stamp = rospy.Time.now()
        target_list.header.frame_id = os.path.join(rospy.get_namespace(),"baselink")
        target_list.header.seq = self._sequential_observation_id
        self._sequential_observation_id += 1
        target_list.targets.append(target)
        self._sonar_targets_publisher.publish(target_list)


               


rospy.init_node("detector_node")

if __name__ == "__main__":
    detect = Detection()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
