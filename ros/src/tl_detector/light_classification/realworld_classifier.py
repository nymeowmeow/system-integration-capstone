from styx_msgs.msg import TrafficLight
import os
import numpy as np
import cv2
import tensorflow as tf
from time import gmtime, strftime, time
from timeit import default_timer as timer


DEBUG_MODE = False
PATH_TO_CKPT = os.path.dirname(os.path.realpath(__file__)) + '/frozen_inference_graph.pb'
# mapping between classifier class and TrafficLight
CLASSES = {1: TrafficLight.GREEN, 2: TrafficLight.RED, 3: TrafficLight.YELLOW, 4:TrafficLight.UNKNOWN}

# based on https://github.com/tensorflow/models/blob/master/object_detection/object_detection_tutorial.ipynb
class RealWorldClassifier(object):

    def __init__(self):
        self.load()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
        image (cv::Mat): image containing the traffic light

        Returns:
        int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # predict traffic light color
        class_index, probability = self.predict(image)
        return class_index


    def load(self):
        config = tf.ConfigProto()
        config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1

        self.detection_graph = tf.Graph()
        with tf.Session(graph=self.detection_graph, config=config) as sess:
            self.session = sess
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')


    def predict(self, image_np, min_score_thresh=0.5):
        image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

        start = timer()
        (boxes, scores, classes) = self.session.run(
                [detection_boxes, detection_scores, detection_classes],
                feed_dict={image_tensor: np.expand_dims(image_np, axis=0)})
        end = timer()

        scores = np.squeeze(scores)
        classes = np.squeeze(classes)
        boxes = np.squeeze(boxes)

        for i, box in enumerate(boxes):
            if scores[i] > min_score_thresh:
                light_class = CLASSES[classes[i]]
                return light_class, scores[i]
        return TrafficLight.UNKNOWN, 0.5
