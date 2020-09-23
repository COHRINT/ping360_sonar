#!/usr/bin/env python

import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from minau.msg import SonarTargetList, SonarTarget
from std_msgs.msg import Float64MultiArray
import struct

THRESHOLD = 130
MIN_DIST = 1.5

rospy.init_node("sonar_detector")
pub = rospy.Publisher("sonar_processing/target_list", SonarTargetList, queue_size=10)
pub2 = rospy.Publisher("sonar_processing/averages2", Float64MultiArray, queue_size=10)

NUM_BINS = 20

averages_mat = np.zeros((NUM_BINS,200))

def callback(msg):
    global pub, pub2, THRESHOLD, MIN_DIST, NUM_BINS
    
    # values = [x for x in msg.intensities)]
    vals = []
    for i in range(len(msg.intensities)):
        vals.append(struct.unpack("<B", msg.intensities[i])[0])
    
    num_pts = len(vals) / NUM_BINS
    bin_avgs = []
    for i in range(NUM_BINS):
        bin_avgs.append( np.mean( vals[i*num_pts: (i+1)*num_pts] ))

    averages_mat[:,int(msg.angle / 2)] = np.array(bin_avgs)
    cnt = np.count_nonzero(averages_mat)
    print(cnt)
    if cnt >= NUM_BINS * 200:
        print("FULL")
        for i in range(4,10): # 10 bins seemed to be the max
            mean_radial = np.mean(averages_mat[i,:])
            std_radial = np.std(averages_mat[i,:])
            thresh = mean_radial - 1.5*std_radial
            if bin_avgs[i] < thresh:
                print(str(msg.angle) + " DETECTION! in bin: " + str(i))
                print([mean_radial, std_radial, bin_avgs[i]])
                angle = msg.angle * (np.pi / 200.0) # Convert grad to rad
                detection_range = i * (10 / float(NUM_BINS))
                target = SonarTarget("detection", angle, 0.1, 0, 0.1, detection_range, 0.1, False, 
                    SonarTarget().TARGET_TYPE_UNKNOWN, 0)
                stl = SonarTargetList(msg.header, [target])
                pub.publish(stl)
        

rospy.Subscriber("/ping360_node/sonar/data", SonarEcho, callback)
print("loaded")
rospy.spin()