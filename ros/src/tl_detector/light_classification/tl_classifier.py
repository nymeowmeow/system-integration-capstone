from realworld_classifier import RealWorldClassifier
from simulator_classifier import SimulatorClassifier

class TLClassifier(object):
    def __init__(self, isSimulator):
        self.model = SimulatorClassifier() if isSimulator else RealWorldClassifier()


    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #predict traffic light color 
        return self.model.get_classification(image)
		
		
