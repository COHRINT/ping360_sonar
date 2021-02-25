#!/usr/bin/env python3
import rospy
from ping360_sonar.msg import SonarEcho
import matplotlib.pyplot as plt
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2 as cv 
import csv



class Detection:
    def __init__(self):

        #Subscribe to the sonar data
        rospy.Subscriber("/ping360_node/sonar/data",SonarEcho,self.data_callback)
        rospy.Subscriber("/ping360_node/sonar/images",Image,self.image_callback)

        #Set up publisher to republish image with detections marked as red on them
        self.image_pub = rospy.Publisher("/ping360_node/sonar/modified_image",Image,queue_size=10)


        self.raw_image = None
        self.avg = [0]*200
        self.done = False
        self.count = 0
        # detections is an array of array of detections, with each detection having a angle and index
        # ex: [[[ang1,id1],[ang2,id2],[ang3,id3]],[[ang11,id11],[ang12,id12],[ang13,id13]]]
        # all the first 3 belong together, and the second 3 belong together
        self.detections = []
        # the confirmed detections are ones that have passed all the tests and that we are going to draw
        self.confirmed_detections = []
        self.bridge = CvBridge()
        self.num_samp = None
    def data_callback(self,msg):
        """Takes in the sonar data and determines if there is a detection

        Args:
            msg (SonarEcho): raw data from ping360 sonar
        """

        self.num_samp = msg.number_of_samples

        #this reads in the raw intensity data and puts it in an array, use ord because it thinks 8 bit unsigned ints are characters
        intens = [0]*msg.number_of_samples
        for i in range(msg.number_of_samples):
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







            


rospy.init_node("detector_node")

if __name__ == "__main__":
    detect = Detection()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()
