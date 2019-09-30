import numpy as np
import cv2


class Filter:

    def __init__(self, __minTr, __maxTr):
        self.__minTr = __minTr
        self.__maxTr = __maxTr

    def getGrayImage(self, sourceImage):
        grayImage = cv2.cvtColor(sourceImage.copy(), cv2.COLOR_BGR2GRAY)
        return grayImage

    def getBinaryImage(self, sourceImage):
        ret1, binaryImage = cv2.threshold(self.getGrayImage(sourceImage), self.__minTr,self.__maxTr, cv2.THRESH_BINARY)
        return binaryImage

    def getBlurBinaryImage(self, sourceImage, kernel):
        bluredImage = cv2.medianBlur(sourceImage.copy(),kernel)
        ret1, binaryImage = cv2.threshold(bluredImage, self.__minTr,self.__maxTr, cv2.THRESH_BINARY)
        return binaryImage


class ContourFinder:

    def __init__(self, __canyTr1, __canyTr2):
        self.__canyTr1 = __canyTr1
        self.__canyTr2 = __canyTr2

    def getContours(self, binaryImage):
        edges = cv2.Canny(binaryImage, self.__canyTr1, self.__canyTr2)
        _, contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        return contours, hierarchy
