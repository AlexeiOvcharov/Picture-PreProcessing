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
PAPER_WIDTH = 0.27 # in meters
PAPER_HEIGHT = 0.13 # in meters
BRUSH_SIZE = 0.006 # in meters

# Settings of path separator
eps = 0.04;
minDiff = eps/2;

rospack = rospkg.RosPack()
packagePath = rospack.get_path('picture_preprocessing') + "/"
BAG_FILE_PATH = packagePath + "data/"

def showPahts(paths):
	for path in paths:
		listx=[]
		listy=[]
		for point in path:
			listx.append(point[0])
			listy.append(point[1])
		mp.plot(listx,listy)
	mp.show()

def calculateTotalLength(path):
	length = 0;
	for i in xrange(1, len(path)):
		diff = np.sqrt((path[i][0] - path[i-1][0])**2 + (path[i][1] - path[i-1][1])**2)

		if diff > minDiff:
			print('Error! To hight distance between points!')
			break

		length += diff;
	return length

def separatePaths(paths):

	separatedPaths = []
	for path in paths:
		# Calculate all length of path
		pathLength = calculateTotalLength(path)
		segments = int(np.ceil(1.0*pathLength/eps))
		print("Path length: {}; segments: {}".format(pathLength, segments))

		separatedPath = [np.array(path[0])] # Get initial point
		summation = 0
		segmentLength = pathLength/(segments + 1)
		for i in xrange(1, len(path)):
			summation += np.sqrt((path[i][0] - path[i-1][0])**2 + (path[i][1] - path[i-1][1])**2)
			separatedPath.append(np.array(path[i]))

			if (summation >= segmentLength):
				separatedPaths.append(np.array(separatedPath))
				summation = 0
				separatedPath = [np.array(path[i])]
				# print('a')

	return np.array(separatedPaths)

def convertText(data):

	while not rospy.is_shutdown():
		try:
			# Get information about canvas dimensions
			cnvsResp = canvasCient(1)
			PAPER_WIDTH = cnvsResp.width
			PAPER_HEIGHT = cnvsResp.height
			break;
		except rospy.ServiceException, e:
			print "Service call failed: {}".fromat(e)

	rospy.loginfo("Canvas dimensions: {0}x{1} m".format(PAPER_WIDTH, PAPER_HEIGHT))
	imageFile = packagePath + "images/techstarslogo.jpg";
	if (not os.path.exists(imageFile)):
		rospy.logerr("Error: file '{}' does not exist!".format(imageFile))
		return False

	# Create rosbag file
	bag = rosbag.Bag(BAG_FILE_PATH + 'test.bag', 'w')

	# Intialize
	filt = Filter(tresholdMin, tresholdMax)
	cf = ContourFinder(binMin, binMax)

	src = cv2.imread(imageFile)
	binImage = filt.getBinaryImage(src)

	heigh, width = binImage.shape
	ec = EdgeCutter([width, heigh],[PAPER_WIDTH, PAPER_HEIGHT], BRUSH_SIZE, 10) #Init edge cutter(image size, paper size, brush size, scale factor)
	cp = ContourProcessing([width, heigh],[PAPER_WIDTH, PAPER_HEIGHT], 0, [0, 0]); #Init contour processing(image size, paper size, rotation, bias)
	resizedImage = ec.normalizeImage(binImage); #Rescale image for good cutting

	# cv2.imshow('Cutting image', resizedImage)
	# cv2.waitKey(0)
	avg_colors = ec.getAvgColor(resizedImage) #Check avg color in picture for finish cutting

	step = 0
	trajectorysNum = 0;
	while(avg_colors < 254.95):
		step += 1
		cont,hier = cf.getContours(resizedImage)
		resizedImage = ec.cutEdges(resizedImage, cont) #Cutting image edges with step(brush size)
		avg_colors = ec.getAvgColor(resizedImage)
		print("[STEP {}] Average color in picture: {}".format(step, avg_colors))
		paths = cp.sort(cont,hier) #Sort image
		resizedImage = filt.getBlurBinaryImage(resizedImage, 7)

		# cv2.imshow('Cutting image', resizedImage)
		# showPahts(paths)
		separatedPaths = separatePaths(paths)
		showPahts(separatedPaths)

		try:
			for path in separatedPaths:
				trajectory = gmsgs.PoseArray()

				for p in path:
					pose = gmsgs.Pose()
					pose.position.x = p[0]
					pose.position.y = p[1]

					trajectory.poses.append(pose)

				trajectorysNum += 1
				bag.write('/path', trajectory)
		except Exception, e:
			print("Error: {}".format(e))

	bag.close()
	rospy.loginfo("Trajectory generated: {}".format(trajectorysNum))
	rospy.loginfo("All trajectorys was write to file {}".format(BAG_FILE_PATH + 'test.bag'))

	return EmptyResponse()


if __name__ == "__main__":

	rospy.init_node("text_converter")
	textConverterService = rospy.Service("/convert_text", Empty, convertText)
	canvasCient = rospy.ServiceProxy('/request_canvas', RequestCanvas)
	rospy.loginfo("Start image preprocessor!")

	while not rospy.is_shutdown():
		rospy.spin();
