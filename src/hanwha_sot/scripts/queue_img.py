#!/usr/bin/env python2.7
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import Queue as queue
import threading
import numpy as np
import time

class ImageFlipper:
    def __init__(self):
        self.queue = queue.Queue()
        self.thread = threading.Thread(target=self.process_queue)
        self.thread.daemon = True
        self.thread.start()
        self.bridge = CvBridge()
        self.publishers = {}

    def process_queue(self):
        while not rospy.is_shutdown():
            msg, topic = self.queue.get() 
            self.flip_image(msg, topic)
            self.queue.task_done()

    def flip_image(self, msg, topic):
        # Convert the ROS image message to an OpenCV image
        
        if topic in ["/ouster/nearir_image", "/ouster/signal_image", "/ouster/reflec_image", ]:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        else:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            cv_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            cv_image = cv_image.astype(np.uint8)
        
        # Flip the image. The flipCode value determines the flip direction:
        # 0: flip vertically, 1: flip horizontally, -1: flip both axes
        flipped_image = cv2.flip(cv_image, -1)
        # flipped_image = cv2.flip(cv_image, 1)
        
        # Convert the OpenCV image back to a ROS image message
        flipped_msg = self.bridge.cv2_to_imgmsg(flipped_image, encoding='mono8')
        
        # Publish the flipped image to the corresponding output topic
        if topic in self.publishers:
            self.publishers[topic].publish(flipped_msg)

    def callback(self, msg, topic):
        self.queue.put((msg, topic))  # Store both the message and the topic in the queue

if __name__ == '__main__':
    # time.sleep(10)
    rospy.init_node('flip_image_node')
    image_flipper = ImageFlipper()

    # Define input and output topic pairs
    topic_map = {
        "/ouster/nearir_image": "/arion/mtt/core/nearir_flipped_img",
        "/ouster/signal_image": "/arion/mtt/core/signal_flipped_img",
        "/ouster/reflec_image": "/arion/mtt/core/reflec_flipped_img",
        "/ouster/range_image": "/arion/mtt/core/range_flipped_img",
    }

    # Subscribe to input topics and create publishers for output topics
    for input_topic, output_topic in topic_map.items():
        rospy.Subscriber(input_topic, Image, lambda msg, t=input_topic: image_flipper.callback(msg, t))
        image_flipper.publishers[input_topic] = rospy.Publisher(output_topic, Image, queue_size=1)
        
    rospy.loginfo("set up image flipper")

    rospy.spin()
