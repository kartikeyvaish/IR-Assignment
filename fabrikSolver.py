from mpl_toolkits.mplot3d import axes3d
import numpy as np
import math
import sys
import copy
import matplotlib as mpl
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')


def unitVector(vector):
    return vector / np.linalg.norm(vector)


class Segment2D:
    def __init__(self, referenceX, referenceY, length, angle):
        self.angle = angle

        self.length = length

        deltaX = math.cos(math.radians(angle)) * length
        deltaY = math.sin(math.radians(angle)) * length

        newX = referenceX + deltaX
        newY = referenceY + deltaY

        self.point = np.array([newX, newY])

    def setPoint(self, a, b, reference):
        pass


class FabrikSolver2D:

    def __init__(self, baseX=0, baseY=0, marginOfError=0.01):
        self.basePoint = np.array([baseX, baseY])

        self.segments = []

        self.history = []

        self.armLength = 0

        self.marginOfError = marginOfError

    def addSegment(self, length, angle):
        if len(self.segments) > 0:

            segment = Segment2D(
                self.segments[-1].point[0], self.segments[-1].point[1], length, angle + self.segments[-1].angle)
        else:
            segment = Segment2D(
                self.basePoint[0], self.basePoint[1], length, angle)

        self.armLength += segment.length

        self.segments.append(segment)

    def isReachable(self, targetX, targetY):
        if np.linalg.norm(self.basePoint - np.array([targetX, targetY])) < self.armLength:
            return True
        return False

    def inMarginOfError(self, targetX, targetY):
        if np.linalg.norm(self.segments[-1].point - np.array([targetX, targetY])) < self.marginOfError:
            return True
        return False

    def iterate(self, targetX, targetY):
        target = np.array([targetX, targetY])

        for i in range(len(self.segments) - 1, 0, -1):
            if i == len(self.segments) - 1:
                self.segments[i-1].point = (unitVector(
                    self.segments[i-1].point - target) * self.segments[i].length) + target

            else:
                self.segments[i-1].point = (unitVector(self.segments[i-1].point -
                                                       self.segments[i].point) * self.segments[i].length) + self.segments[i].point

        for i in range(len(self.segments)):
            if i == 0:
                self.segments[i].point = (unitVector(
                    self.segments[i].point - self.basePoint) * self.segments[i].length) + self.basePoint

            elif i == len(self.segments) - 1:
                self.segments[i].point = (unitVector(
                    self.segments[i-1].point - target) * self.segments[i].length * -1) + self.segments[i-1].point

            else:
                self.segments[i].point = (unitVector(
                    self.segments[i].point - self.segments[i-1].point) * self.segments[i].length) + self.segments[i-1].point

    def compute(self, targetX, targetY):
        self.history.append(copy.deepcopy(self.segments))
        if self.isReachable(targetX, targetY):
            while not self.inMarginOfError(targetX, targetY):
                self.iterate(targetX, targetY)
                self.history.append(copy.deepcopy(self.segments))
        else:
            print('Target not reachable.')

    def plot(self, segments=None, save=False, name="graph", xMin=-300, xMax=300, yMin=-300, yMax=300):
        if segments == None:
            segments = self.segments

        for i in range(len(segments)):
            plt.plot([segments[i].point[0]], [
                     segments[i].point[1]], 'ro')

            plt.text(segments[i].point[0], segments[i].point[1] + 1, '(x:{}, y:{})'.format(
                int(segments[i].point[0]), int(segments[i].point[1])))

        plt.plot([self.basePoint[0]], [self.basePoint[1]], 'bo')
        plt.text(self.basePoint[0], self.basePoint[1], 'Base')

        plt.axis([xMin, xMax, yMin, yMax])
        plt.grid(True)

        if save == True:
            plt.savefig('{}.png'.format(name))

        plt.show(block=True)
