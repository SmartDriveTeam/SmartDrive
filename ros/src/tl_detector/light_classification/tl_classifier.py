import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
from PIL import Image
import cv2
import numpy as np
from glob import glob
import os
from keras.models import load_model

import time

class TLClassifier(object):
    def __init__(self):

        tl_path = os.path.dirname(os.path.realpath(__file__))

        # Load Keras Traffic Light Color Classification Model
        self.class_model = load_model(tl_path + '/train_log/model_smartdrive_04262110.h5')
        self.class_graph = tf.get_default_graph()

        # Traffic Light Detection
        self.detection_graph = tf.Graph()

        with self.detection_graph.as_default():
            GraphDef = tf.GraphDef()
            with open(tl_path + "/models/frozen_inference_graph.pb", 'rb') as f:
                GraphDef.ParseFromString(f.read())
                tf.import_graph_def(GraphDef, name="")

            self.session = tf.Session(graph=self.detection_graph )
            self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.detection_boxes =  self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.num_detections    = self.detection_graph.get_tensor_by_name('num_detections:0')

        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light. OpenCV is BGR by default.

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        # Setting Parameters
        IMG_SAVE_FLAG = False #TEST SET - SAVE IMAGE
        DETECTION_SCORE = 0.3
        LIMIT_BOX_SIZE = 40 # 20-> 40 So Far Red Signal Found in simulator
        LIMIT_BOX_RATIO = 1.6

        with self.detection_graph.as_default():
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image_input = np.expand_dims(image,axis=0)

            # Traffic Light Detection
            (detection_boxes, detection_scores, detection_classes, num_detections) = self.session.run(
                    [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections],
                    feed_dict={self.image_tensor: image_input})

            detection_boxes = np.squeeze(detection_boxes)
            detection_classes = np.squeeze(detection_classes)
            detection_scores = np.squeeze(detection_scores)

            tl_box = None

            idx = -1
            for i, class_num in enumerate(detection_classes.tolist()):
                if class_num == 10: #Traffic Light Class
                    idx = i;
                    break;

            if idx == -1:
                pass
            elif detection_scores[idx] < DETECTION_SCORE:
                pass
            else:
                height, width = image.shape[0], image.shape[1]
                box_pixel = np.array([int(detection_boxes[idx][0]*height), int(detection_boxes[idx][1]*width),
                        int(detection_boxes[idx][2]*height), int(detection_boxes[idx][3]*width)])
      
                box_height, box_width  = (box_pixel[2] - box_pixel[0], box_pixel[3]-box_pixel[1])
                
                if (box_height < LIMIT_BOX_SIZE) or (box_width < LIMIT_BOX_SIZE): # Box small
                    pass
                elif (box_height/box_width < LIMIT_BOX_RATIO): # Box height, width ratio
                    pass
                else:                    
                    tl_box = box_pixel

        if tl_box is None:
            if IMG_SAVE_FLAG:
                image_save = image.copy()
                image_save = cv2.cvtColor(image_save, cv2.COLOR_RGB2BGR)
                tl_path = os.path.dirname(os.path.realpath(__file__))
                save_image_path = tl_path + '/image_save/' + str(time.time()) + '.png'
                cv2.imwrite(save_image_path, image_save)
            return TrafficLight.UNKNOWN


        # Keras Model Predict
        # Traffic Light Color Classification Model
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        box_image = image[tl_box[0]:tl_box[2], tl_box[1]:tl_box[3]]
        box_resize = cv2.resize(box_image, (32,32))
        box_resize_ = np.expand_dims(box_resize, axis=0).astype('float32')

        if IMG_SAVE_FLAG:
            image_save = image.copy()
            cv2.rectangle(image_save, (tl_box[1],tl_box[0]), (tl_box[3],tl_box[2]), (255,0,0), 5)
            tl_path = os.path.dirname(os.path.realpath(__file__))
            save_image_path = tl_path + '/image_save/' + str(time.time()) + '.png'
        
        with self.class_graph.as_default():
            predict = self.class_model.predict(box_resize_)
            predict_tl_color = np.argmax(predict)
            if predict_tl_color == 0:
                if IMG_SAVE_FLAG:
                    cv2.putText(image_save, "RED", (tl_box[3],tl_box[2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                    cv2.imwrite(save_image_path, image_save)
                return TrafficLight.RED
            elif predict_tl_color == 1:
                if IMG_SAVE_FLAG:
                    cv2.putText(image_save, "YELLOW", (tl_box[3],tl_box[2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                    cv2.imwrite(save_image_path, image_save)
                return TrafficLight.YELLOW
            elif predict_tl_color == 2:
                if IMG_SAVE_FLAG:
                    cv2.putText(image_save, "GREEN", (tl_box[3],tl_box[2]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)
                    cv2.imwrite(save_image_path, image_save)
                return TrafficLight.GREEN
            else:
                return TrafficLight.UNKNOWN


