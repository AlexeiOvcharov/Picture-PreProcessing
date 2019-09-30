import numpy as np
import cv2

class EdgeCutter:

    def __init__(self,__imageSize, __areaSize, __step, __scaleFactor = 10):
        self.__areaSize = __areaSize
        self.__step = __step
        self.__scaleFactor = __scaleFactor
        self.__corImageScale = 1
        self.__imageSize = __imageSize
        self.calcImageScaleNorm()

    def calcImageScaleNorm(self):
        scaleTransform = np.diag([self.__imageSize[0],self.__imageSize[1]])*np.linalg.inv(np.diag(self.__areaSize))
        vec2dStep = scaleTransform * np.matrix([[self.__step], [self.__step]])
        pixel_step = int(np.linalg.norm(vec2dStep))
        self.__corImageScale = self.__scaleFactor/(1.0*pixel_step)

    def getImageScaleNorm(self):
        return self.__corImageScale

    def setScaleFactor(self, scaleFactor):
        self.__scaleFactor = scaleFactor
        self.calcImageScaleNorm()

    def setAreaSize(self, areaSize):
        self.__areaSize = areaSize

    def setImageSize(self, imageSize):
        self.__imageSize

    def cutEdges(self,sourceImage,contours):
        copyImg = sourceImage.copy()
        copyImg = cv2.drawContours(copyImg, contours, -1, (255, 255, 255), self.__scaleFactor)
        return copyImg

    def normalizeImage(self, sourceImage):
        self.calcImageScaleNorm()
        print(self.__corImageScale)
        resizedImg = cv2.resize(sourceImage.copy(), (int(self.__imageSize[0]*self.__corImageScale), int(self.__imageSize[1]*self.__corImageScale)))
        return resizedImg

    def getAvgColor(self, sourceImage):
        avg_color_per_row = np.average(sourceImage, axis=0)
        avg_colors = np.average(avg_color_per_row, axis=0)
        return avg_colors
