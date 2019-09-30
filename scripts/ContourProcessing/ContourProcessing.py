import numpy as np
from math import *
import cv2

class ContourProcessing:

    def __init__(self, __imageSize, __areaSize, __rotation, __bias):
        self.__areaSize = __areaSize
        self.__imageSize = __imageSize
        self.__st = max(__areaSize)/max(__imageSize);
        self.__transformMatrix = np.matrix([\
        [-self.__st*cos(__rotation),-sin(__rotation),__bias[0]],\
        [+sin(__rotation), -self.__st*cos(__rotation),__bias[1]],\
        [0              , 0                        , 1      ]])
        print(self.__transformMatrix)

    def sort(self, contours, hierarchy):
        paths = [];
        for j in range(len(contours)):
            path = []
            if cv2.contourArea(contours[j]) < min(self.__imageSize)/100.0:
                continue
            for i in range(len(contours[j])):
                point = contours[j][i][0] - np.array(self.__imageSize)/2
                point3d = np.matrix(np.append([np.matrix(point)], [[1]])).T
                trpoint3d= np.dot(self.__transformMatrix, np.matrix(point3d))
                path.append([trpoint3d.item(0), trpoint3d.item(1)])
            paths.append(np.array(path))
        return paths
