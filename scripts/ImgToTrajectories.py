#!/usr/bin/python

import numpy as np
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

from ContourProcessing.ContourProcessing import *
from ContourProcessing.ContourFinder import *
from ContourProcessing.EdgeCutter import *
import matplotlib.pyplot as mp

tresholdMin = 200 #min for Canny
tresholdMax = 255 #max for Canny
binMin = 70 #min for Threshold in %
binMax = 100 #max for Threshold in %
PAPER_WIDTH = 1.0 # in meters
PAPER_HEIGHT = 1.0 # in meters

rospack = rospkg.RosPack()
packagePath = rospack.get_path('picture_preprocessing') + "/"

def convertText(data):

    # while not rospy.is_shutdown():
    # 	try:
    #         # Get information about canvas dimensions
    #         cnvsResp = canvasCient(1)
    #         PAPER_WIDTH = cnvsResp.width
    #         PAPER_HEIGHT = cnvsResp.height
	# 		pass
    #         break;
    #     except rospy.ServiceException, e:
    #         print "Service call failed: {}".fromat(e)
	rospy.loginfo("Canvas dimensions: {0}x{1} m".format(PAPER_WIDTH, PAPER_HEIGHT))
	imageFile = packagePath + "images/techstarslogo.jpg";
	if (not os.path.exists(imageFile)):
		rospy.logerr("Error: file '{}' does not exist!".format(imageFile))
		return False
	src = cv2.imread(imageFile)

	filt = Filter(tresholdMin, tresholdMax)
	cf = ContourFinder(binMin, binMax)

	binImage = filt.getBinaryImage(src)

	heigh, width = binImage.shape
	ec = EdgeCutter([width, heigh],[PAPER_WIDTH, PAPER_HEIGHT],0.006, 10) #Init edge cutter(image size, paper size, brush size, scale factor)
	cp = ContourProcessing([width, heigh],[PAPER_WIDTH, PAPER_HEIGHT], 0, [0,0]); #Init contour processing(image size, paper size, rotation, bias)
	resizedImage = ec.normalizeImage(binImage); #Rescale image for good cutting

	cv2.imshow('Cutting image', resizedImage)
	cv2.waitKey(0)
	avg_colors = ec.getAvgColor(resizedImage) #Check avg color in picture for finish cutting

	step =0
	while(avg_colors < 255):
		step+=1
		cont,hier = cf.getContours(resizedImage)
		resizedImage = ec.cutEdges(resizedImage, cont) #Cutting image edges with step(brush size)
		cv2.imshow('Cutting image', resizedImage)
		avg_colors = ec.getAvgColor(resizedImage)
		print("[STEP ", str(step),"]Average color in picture: ",avg_colors)
		paths = cp.sort(cont,hier) #Sort image
		resizedImage = filt.getBlurBinaryImage(resizedImage,7)
		# for path in paths:
		# listx=[]
		# listy=[]
		# for path in paths:
		# 	for point in path:
		# 		listx.append(point[0])
		# 		listy.append(point[1])
		# 	mp.plot(listx,listy)
		# 	mp.show()
		# break


if __name__ == "__main__":

    rospy.init_node("text_converter")
    textConverterService = rospy.Service("/convert_text", Empty, convertText)
    canvasCient = rospy.ServiceProxy('/request_canvas', RequestCanvas)
    rospy.loginfo("Start image preprocessor!")

    while not rospy.is_shutdown():
        rospy.spin();
