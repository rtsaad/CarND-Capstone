import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
from PIL import Image
from PIL import ImageDraw
from PIL import ImageColor
import matplotlib.image as mpimg
import numpy as np

class TLClassifier(object):


    def __init__(self):

        self.detection_graph = self.load_graph('light_classification/frozen_inference_graph.pb')
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.detection_number = self.detection_graph.get_tensor_by_name('num_detections:0')

    
    def get_classification(self, image):

	img_expanded = np.expand_dims(np.asarray(image, dtype=np.uint8), 0) 
        # Get boxes for traffic lights
        boxes, scores, classes = self.get_boxes_for_traffic_lights(img_expanded)

	height, width, channels = image.shape
        #width, height = image.size
        box_coords = self.to_image_coords(boxes, height, width)

        for i in range(len(boxes)):
            ymin = int(box_coords[i][0])
            xmin = int(box_coords[i][1])
            ymax = int(box_coords[i][2])
            xmax = int(box_coords[i][3])

            image_a = np.asarray(image)
            cropped_image = image_a[ymin:ymax, xmin:xmax, :]
            image_resized = cv2.resize(cropped_image, (24, 72))

            light_color = self.get_light_classification(image_resized, boxes)
            if light_color == 0:
		rospy.loginfo('*********************************************')
		rospy.loginfo('RED LIGHT DETECTED')
                return TrafficLight.RED

	rospy.loginfo('UNKNOWN')
        return TrafficLight.UNKNOWN
    

    def get_boxes_for_traffic_lights(self, image):
        with tf.Session(graph=self.detection_graph) as sess:

            # img_expanded = np.expand_dims(np.asarray(image, dtype=np.uint8), 0) 

            # Actual detection.
            (boxes, scores, classes, num) = sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.detection_number], 
                                        feed_dict={self.image_tensor: image})

            # Remove unnecessary dimensions
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)
     
            # Filter boxes with a confidence score less than `CONFIDENCE_CUTOFF`
            return self.filter_boxes(0.5, boxes, scores, classes)


    def get_light_classification(self, image, boxes):

        with tf.Session(graph=self.detection_graph) as sess:
                
            saver = tf.train.import_meta_graph('light_classification/traffic_light.ckpt.meta')
            saver.restore(sess, 'light_classification/traffic_light.ckpt')

            input_image = tf.get_default_graph().get_tensor_by_name("input_image:0")
            model_output = tf.get_default_graph().get_tensor_by_name("model_output:0")

            classes = sess.run(model_output, {input_image: [image]})[0]
            return classes.tolist().index(max(classes))


    ################# Utils ########################

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score` and class == 10 (traffic light)""" 
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score and classes[i] == 10:
                idxs.append(i)
    
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes


    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
    
        This converts it back to the original coordinate based on the image size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width

        return box_coords


    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph
