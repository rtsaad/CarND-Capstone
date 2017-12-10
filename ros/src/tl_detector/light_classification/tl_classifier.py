from styx_msgs.msg import TrafficLight

class TLClassifier(object):

    MIN_THRESHOLD = 0.3

    # Classes 
    # LABEL_DICT =  {
    #     "Green" : 1,
    #     "Red" : 2,
    #     "GreenLeft" : 3,
    #     "GreenRight" : 4,
    #     "RedLeft" : 5,
    #     "RedRight" : 6,
    #     "Yellow" : 7,
    #     "off" : 8,
    #     "RedStraight" : 9,
    #     "GreenStraight" : 10,
    #     "GreenStraightLeft" : 11,
    #     "GreenStraightRight" : 12,
    #     "RedStraightLeft" : 13,
    #     "RedStraightRight" : 14
    # }

    def __init__(self):

        # Set default state
        self.current_light = TrafficLight.UNKNOWN

        # Download model and place it in the same folder
        PATH_TO_MODEL = 'frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            # Works up to here.
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)

    def get_classification(self, image):

        # Bounding Box Detection.
        with self.detection_graph.as_default():

        img_expanded = np.expand_dims(img, axis=0)  
        (boxes, scores, classes, num) = self.sess.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d],
            feed_dict={self.image_tensor: img_expanded})
        
        # Get the max
        max_score = scores[0]
        max_class = classes[0]

        self.current_light = TrafficLight.UNKNOWN

        if (max_score > MIN_THRESHOLD):
            print("Hay algo")
            if max_class in (1, 3, 4, 10, 11, 12):
                print("GREEN")
                self.current_light = TrafficLight.GREEN
            elif max_class in (2, 5, 6, 9, 13, 14):
                print("RED")
                self.current_light = TrafficLight.RED
            elif max_class in (7):
                print("YELLOW")
                self.current_light = TrafficLight.YELLOW

        return self.current_light
