#!/usr/bin/env python

import os
import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from minau.msg import SonarTargetList, SonarTarget
from std_msgs.msg import Float64MultiArray
import struct
import pickle

THRESHOLD = 130
MIN_DIST = 1.5
COUNTER = 0
STEP = 4

FILTER_EXTENDED_SHADOW = True

rospy.init_node("sonar_detector")
pub = rospy.Publisher("sonar_processing/target_list", SonarTargetList, queue_size=10)

pickle_path = rospy.get_param('~pickle_path', os.path.join('..','cfg','detection_averages.pickle'))

NUM_BINS = 20

saved = False

if os.path.exists(pickle_path):
    averages_mat = pickle.load(open(pickle_path, 'rb'))
    saved = True
else:
    averages_mat = np.zeros((NUM_BINS,(400 / STEP)))
print(averages_mat.shape)

def callback(msg):
    global pub, pub2, THRESHOLD, MIN_DIST, NUM_BINS, COUNTER, saved, pickle_path
    
    # values = [x for x in msg.intensities)]
    vals = []
    for i in range(len(msg.intensities)):
        vals.append(struct.unpack("<B", msg.intensities[i])[0])
    
    num_pts = len(vals) / NUM_BINS
    bin_avgs = []
    for i in range(NUM_BINS):
        bin_avgs.append( np.mean( vals[i*num_pts: (i+1)*num_pts] ))

    if np.count_nonzero(bin_avgs) != NUM_BINS:
        return

    averages_mat[:,COUNTER] = np.array(bin_avgs)

    COUNTER = (COUNTER + 1) % (400 / STEP)
    cnt = np.count_nonzero(averages_mat)
    if cnt >= NUM_BINS * (400 / STEP):
        if not saved:
            pickle.dump(averages_mat, open(pickle_path, 'wb+'))
            print('Saved averages to pickle')
            saved = True

        num_lower = 0
        for i in range(10,NUM_BINS):
            mean_radial = np.mean(averages_mat[i,:])
            std_radial = np.std(averages_mat[i,:])
            thresh = mean_radial - 0.5*std_radial
            # print([bin_avgs[i], thresh])
            if bin_avgs[i] < thresh:
                num_lower += 1
        print(num_lower)
        if num_lower >= 0.6 * (NUM_BINS - 10):
            print("DETECTION!")
            samples_per_meter = int(len(vals) / 10.0)
            max_index = np.argmax(vals[samples_per_meter:]) + samples_per_meter # skip first meter
            angle = msg.angle * (np.pi / 200.0) # Convert grad to rad
            angle += np.pi # Transform to baselink frame
            detection_range = max_index / float(samples_per_meter)
            target = SonarTarget("detection", angle, 0.1, 0, 0.1, detection_range, 0.1, False, 
                SonarTarget().TARGET_TYPE_UNKNOWN, 0)
            stl = SonarTargetList(msg.header, [target])
            pub.publish(stl)
            print("Dist: " + str(max_index / float(samples_per_meter)))
        print("---")
        # for i in range(4,10): # 10 bins seemed to be the max
        #     mean_radial = np.mean(averages_mat[i,:])
        #     std_radial = np.std(averages_mat[i,:])
        #     thresh = mean_radial - 1.5*std_radial
        #     if bin_avgs[i] < thresh:
        #         print("DETECTION! in bin: " + str(i))
        #         angle = msg.angle * (np.pi / 200.0) # Convert grad to rad
        #         angle += np.pi # Transform to baselink frame
        #         detection_range = i * (10 / float(NUM_BINS))
        #         target = SonarTarget("detection", angle, 0.1, 0, 0.1, detection_range, 0.1, False, 
        #             SonarTarget().TARGET_TYPE_UNKNOWN, 0)
        #         stl = SonarTargetList(msg.header, [target])
        #         pub.publish(stl)
    else:
        print("counting...")

rospy.Subscriber("ping360_node/sonar/data", SonarEcho, callback)
print("loaded")
rospy.spin()