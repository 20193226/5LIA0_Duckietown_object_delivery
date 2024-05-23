#!/usr/bin/env python3

import cv2
import numpy as np
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from duckietown_msgs.msg import Twist2DStamped
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage

from nn_model.constants import IMAGE_SIZE
from nn_model.model import Wrapper

from integration_activity import \
    NUMBER_FRAMES_SKIPPED, \
    filter_by_classes, \
    filter_by_bboxes, \
    filter_by_scores, \
    depth_estimation


class ObjectDetectionNode(DTROS):
    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(ObjectDetectionNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.initialized = False
        self.log("Initializing!")
        self.detection = 0

        self.veh = rospy.get_namespace().strip("/")
        
        # Construct publisher
        self._pub_nn_output = rospy.Publisher(
            'NN_output',
            Bool,
            queue_size=1,
        )

        # Construct subscriber
        self.sub_image = rospy.Subscriber(
            f"/{self.veh}/camera_node/image/compressed",
            CompressedImage,
            self.image_cb,
            buff_size=10000000,
            queue_size=1,
        )
        self._window = "camera-reader"
        cv2.namedWindow(self._window, cv2.WINDOW_AUTOSIZE)

        self.bridge = CvBridge()
        self.v = rospy.get_param("~speed", 0.4)
        aido_eval = False
        self.log(f"AIDO EVAL VAR: {aido_eval}")
        self.log("Starting model loading!")
        self._debug = rospy.get_param("~debug", False)
        self.model_wrapper = Wrapper()
        self.log("Finished model loading!")
        self.frame_id = 0
        self.first_image_received = False
        self.initialized = True
        self.log("Initialized!")

    def image_cb(self, image_msg):
        if not self.initialized:
            return
            
        self.frame_id += 1
        self.frame_id = self.frame_id % (1 + NUMBER_FRAMES_SKIPPED())
        if self.frame_id != 0:
            return

        # Decode from compressed image with OpenCV
        try:
            bgr = self.bridge.compressed_imgmsg_to_cv2(image_msg)
        except ValueError as e:
            self.logerr("Could not decode image: %s" % e)
            return
        #show image
        cv2.imshow(self._window, bgr)
        cv2.waitKey(1)

        rgb = bgr[..., ::-1]

        rgb = cv2.resize(rgb, (IMAGE_SIZE, IMAGE_SIZE))
        bboxes, classes, scores = self.model_wrapper.predict(rgb)

        self.detection, id = self.det2bool(bboxes, classes, scores)
        
        for new_id in id:
            if new_id == -1:
                break
            dist, angle = depth_estimation(bboxes[new_id])
            rospy.loginfo("duckie with r,theta, id: %.4f, %.4f, %d",dist, angle, new_id)
            
    def run(self):
    	# publish message every 0.1 second (10 Hz)
        rate = rospy.Rate(10)
        while not self.initialized:
            pass
            
        while not rospy.is_shutdown():
            #rospy.loginfo("Publishing message: '%s'" % self.detection)
            self._pub_nn_output.publish(self.detection)
            rate.sleep()


    def det2bool(self, bboxes, classes, scores):
        box_ids = np.array(list(map(filter_by_bboxes, bboxes))).nonzero()[0]
        cla_ids = np.array(list(map(filter_by_classes, classes))).nonzero()[0]
        sco_ids = np.array(list(map(filter_by_scores, scores))).nonzero()[0]

        box_cla_ids = set(list(box_ids)).intersection(set(list(cla_ids)))
        box_cla_sco_ids = set(list(sco_ids)).intersection(set(list(box_cla_ids)))

        if len(box_cla_sco_ids) > 0:
            return True, box_cla_sco_ids #list(box_cla_sco_ids).pop() #next(iter(box_cla_sco_ids))
        else:
            return False, {-1}


if __name__ == "__main__":
    # Initialize the node
    object_detection_node = ObjectDetectionNode(node_name="object_detection_node")
    # Keep it spinning
    object_detection_node.run()
    rospy.spin()
