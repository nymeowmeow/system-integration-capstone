import math

class Util(object):
    @staticmethod
    def distance(pt1, pt2):
	return math.sqrt((pt1.x - pt2.x)**2 + (pt1.y - pt2.y)**2 + (pt1.z - pt2.z)**2)
