#!/usr/bin/env python

import os
import rospy
from ping360_sonar.msg import SonarEcho
import numpy as np
from minau.msg import SonarTargetList, SonarTarget
from std_msgs.msg import Float64MultiArray
import struct
import pickle
import sys

THRESHOLD = 130
MIN_DIST = 1.5
COUNTER = 0
STEP = 4

FILTER_EXTENDED_SHADOW = True

rospy.init_node("sonar_detector")
pub = rospy.Publisher("sonar_processing/target_list", SonarTargetList, queue_size=10)

pickle_path = rospy.get_param('~pickle_path', os.path.join('..','cfg','logit_model.p')) # Open logistic regression model

NUM_BINS = 20

saved = False

if not os.path.exists(pickle_path):
    rospy.logfatal("Could not open sonar detection model...quitting")
    sys.exit(-1)
else:
    logit_clf = pickle.load(open(pickle_path, 'rb'))

def callback(msg):
    global pub, logit_clf
    
    # values = [x for x in msg.intensities)]
    vals = []
    for i in range(len(msg.intensities)):
        vals.append(struct.unpack("<B", msg.intensities[i])[0])
    
    X = np.array([vals]).reshape(1,-1)
    pred = logit_clf.predict(X)
    if pred > 0:
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

rospy.Subscriber("ping360_node/sonar/data", SonarEcho, callback)
print("loaded")
rospy.spin()