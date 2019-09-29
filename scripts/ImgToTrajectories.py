#!/usr/bin/python

import weave
import numpy as np
from imutils import resize
import matplotlib.pyplot as plt
import cv2
import rosbag
import os

import rospy
import rospkg
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseArray
import geometry_msgs.msg as gmsgs
from std_srvs.srv import Empty, EmptyResponse
from kuka_cv.srv import *
from kuka_cv.msg import *

def convertText(data):
	pass


if __name__ == "__main__":

    rospy.init_node("text_converter")
    textConverterService = rospy.Service("/convert_text", Empty, convertText)
    canvasCient = rospy.ServiceProxy('/request_canvas', RequestCanvas)
    rospy.loginfo("Start image preprocessor!")

    while not rospy.is_shutdown():
        rospy.spin();
