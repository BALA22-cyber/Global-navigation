#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import rosbag
import cv2
import numpy as np

def publish_images_from_rosbag(rosbag_path, topic_name):
    rospy.init_node('rosbag_image_publisher', anonymous=True)
    pub = rospy.Publisher('image_from_rosbag', Image, queue_size=10)
    bridge = CvBridge()

    try:
        bag = rosbag.Bag(rosbag_path, 'r')
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            compressed_data = msg.data
            np_arr = np.fromstring(compressed_data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            image_msg = bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            pub.publish(image_msg)
            rospy.sleep(0.1)  # Optional delay to control the publishing rate
        bag.close()
    except rosbag.bag.ROSBagUnindexedException:
        rospy.logerr("The provided ROSBag is not indexed. Please run 'rosbag reindex <your_bag.bag>'.")
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    rosbag_path = 'irib2.bag'  # Provide the path to your ROSBag file
    topic_name = '/camera/color/image_raw/compressed'  # Replace with the actual topic name in your ROSBag

    try:
        publish_images_from_rosbag(rosbag_path, topic_name)
    except rospy.ROSInterruptException:
        pass
