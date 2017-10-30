from styx_msgs.msg import TrafficLight
import numpy as np
import cv2


class TLClassifier(object):
    def __init__(self):
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
	decision = TrafficLight.UNKNOWN
	outim = image.copy();
	testim = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		
	lowBound1 = np.array([0,51,50])
        highBound1 = np.array([10,255,255])
        testim1 = cv2.inRange(testim, lowBound1 , highBound1)
		
	lowBound2 = np.array([170,51,50])
        highBound2 = np.array([180,255,255])
        testim2 = cv2.inRange(testim, lowBound2 , highBound2)
		
	testimcombined = cv2.addWeighted(testim1, 1.0, testim2, 1.0, 0.0)
	testimblur = cv2.GaussianBlur(testimcombined,(15,15),0)
	c = cv2.HoughCircles(testimblur,cv2.HOUGH_GRADIENT,0.5, 41, param1=70, param2=30,minRadius=7,maxRadius=150)

	if c is not None:
            decision = TrafficLight.RED
		
        return decision
		
		
