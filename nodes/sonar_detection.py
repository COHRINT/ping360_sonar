#!/usr/bin/env python

import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from minau.msg import SonarTargetList, SonarTarget
import struct

THRESHOLD = 100
MIN_DIST = 1.5

rospy.init_node("sonar_detector")
pub = rospy.Publisher("sonar_processing/target_list", SonarTargetList, queue_size=10)

def callback(msg):
    global pub, THRESHOLD, MIN_DIST
    
    # values = [x for x in msg.intensities)]
    vals = []
    for i in range(len(msg.intensities)):
        vals.append(struct.unpack("<B", msg.intensities[i])[0])
    detections = np.array(vals) > THRESHOLD
    ranges = np.linspace(0, msg.range, msg.number_of_samples)
    valid_detections = np.where( detections * (ranges > MIN_DIST) == True)[0]
    if len(valid_detections) > 0:
        argmin = valid_detections[0]
        detection_range = ranges[argmin]
        angle = msg.angle * (np.pi / 200) # Convert grad to rad
        target = SonarTarget("detection", msg.angle, 0.1, 0, 0.1, detection_range, 0.1, False, 
            SonarTarget().TARGET_TYPE_UNKNOWN, 0)
        stl = SonarTargetList(msg.header, [target])
        pub.publish(stl)

rospy.Subscriber("/ping360_node/sonar/data", SonarEcho, callback)
print("loaded")
rospy.spin()