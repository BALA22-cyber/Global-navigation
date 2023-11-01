#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import torch

# Global flags
crosswalk_detected = False
motion_detected = False
crosswalk_midpoint = None

class RealSenseYOLODetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Load YOLO models
        self.model_people = YOLO("/home/saketh/catkin_ws/src/Global-navigation/models/yolov8m.pt").to(self.device)
        self.model_crosswalk = YOLO("/home/saketh/catkin_ws/src/Global-navigation/models/yolov8_crosswalk_detection.pt").to(self.device)
        
        rospy.init_node('realsense_yolo_detector', anonymous=True)
        self.publisher = rospy.Publisher('processed_camera_image', Image, queue_size=10)
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        self.husky_cmd_pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
        self.state = "searching_crosswalk"  # To keep track of the current state

    
    def object_in_motion(self, current_frame, current_objects):
        previous_frames = []
        previous_objects = []
        moving_average_frames = 10
        motion_threshold = 1000
        if len(previous_frames) < moving_average_frames:
            previous_frames.append(current_frame)
            previous_objects.append(current_objects)
            return

        # Calculate the absolute difference between the current frame and the oldest frame in the moving average
        frame_diff = cv2.absdiff(current_frame, previous_frames[0])

        # Define a threshold for motion detection
        frame_diff_gray = cv2.cvtColor(frame_diff, cv2.COLOR_BGR2GRAY)

        # Apply a binary threshold to detect motion
        _, motion_mask = cv2.threshold(frame_diff_gray, 30, 255, cv2.THRESH_BINARY)

        # Count the number of non-zero pixels in the motion mask
        motion_pixel_count = np.count_nonzero(motion_mask)

        # If there is motion, set the flag to True
        if motion_pixel_count > motion_threshold:
            motion_detected = True
        else:
            motion_detected = False

        # Update the previous frames and objects by maintaining a moving average
        previous_frames.append(current_frame)
        previous_objects.append(current_objects)
        previous_frames.pop(0)
        previous_objects.pop(0)

        return motion_detected

    def image_callback(self, data):
        global crosswalk_detected, motion_detected, crosswalk_midpoint

        # Class names for visualization
        classNames = ["person", "bicycle", "car", "motorbike", "bus", "truck", "traffic light", "fire hydrant", "stop sign"]  

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results_people = self.model_people.predict(cv_image_rgb)
            results_crosswalk = self.model_crosswalk.predict(cv_image_rgb)

            # Process people detections
            for result in results_people:
                original_image = result.orig_img
                motion_text = "Stationary"

                for box, conf, class_id in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
                    class_id = int(class_id)
                    class_name = result.names[class_id]

                    if class_name in classNames and conf > 0.5:
                        x_min, y_min, x_max, y_max = map(int, box)
                        cv2.rectangle(original_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                        cv2.putText(original_image, class_name, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        motion_detected = self.object_in_motion(cv_image_rgb, result)

                        if motion_detected:
                            motion_text = "In Motion"

                        cv2.putText(original_image, motion_text, (x_max + 5, y_max), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Process crosswalk detections
            for result in results_crosswalk:
                for box, conf, class_id in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
                    if class_id == 0 and conf > 0.5:
                        crosswalk_detected = True
                        x_min, y_min, x_max, y_max = map(int, box)
                        crosswalk_midpoint = ((x_min + x_max) / 2, (y_min + y_max) / 2)
                        cv2.rectangle(cv_image_rgb, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                        cv2.putText(cv_image_rgb, "Crosswalk Detected", (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            annotated_image_np = np.array(cv_image_rgb)
            image_msg = self.bridge.cv2_to_imgmsg(annotated_image_np, encoding="bgr8")
            self.publisher.publish(image_msg)

            cv2.imshow("RealSense Image with People Detection", cv_image_rgb)
            cv2.waitKey(1)

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def navigate_husky(self):
        rate = rospy.Rate(10)
        
        if self.state == "searching_crosswalk":
            rotate_cmd = Twist()
            rotate_cmd.angular.z = 0.5  # Clockwise rotation
            self.husky_cmd_pub.publish(rotate_cmd)
            
            if crosswalk_detected:  # Transition to the next state
                self.state = "approaching_crosswalk"

        elif self.state == "approaching_crosswalk":
            approach_cmd = Twist()
            approach_cmd.linear.x = 0.5  # Forward motion
            self.husky_cmd_pub.publish(approach_cmd)

            # Assuming we switch state after moving forward for some time (this can be improved with better sensing)
            rospy.sleep(5)
            self.state = "aligning_to_crosswalk"

        elif self.state == "aligning_to_crosswalk":
            frame_midpoint = cv_image_rgb.shape[1] / 2  # Assuming cv_image_rgb is available here
            error = crosswalk_midpoint[0] - frame_midpoint

            # Adjust the alignment speed based on the error. You might want to add some scaling factor or PID control here.
            align_cmd = Twist()
            if error > 0:  # Crosswalk is to the right
                align_cmd.angular.z = -0.3
            elif error < 0:  # Crosswalk is to the left
                align_cmd.angular.z = 0.3
            else:
                align_cmd.angular.z = 0

            self.husky_cmd_pub.publish(align_cmd)

            # You can add a threshold to check if the robot is aligned enough
            if abs(error) < 10:  # Adjust the threshold as needed
                self.state = "stopping_and_checking"


        elif self.state == "stopping_and_checking":
            stop_cmd = Twist()  # Zero velocities
            self.husky_cmd_pub.publish(stop_cmd)

            if not motion_detected:  # If no vehicles are moving for a certain duration, proceed
                rospy.sleep(5)  # Wait for 5 seconds to ensure it's safe
                self.state = "crossing_road"

        elif self.state == "crossing_road":
            cross_cmd = Twist()
            cross_cmd.linear.x = 0.5
            self.husky_cmd_pub.publish(cross_cmd)

            # Assuming we switch state after moving forward for some time (adjust based on road width)
            rospy.sleep(10)
            stop_cmd = Twist()
            self.husky_cmd_pub.publish(stop_cmd)

    def run(self):
        while not rospy.is_shutdown():
            self.navigate_husky()
            rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        node = RealSenseYOLODetector()
        node.run()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()
