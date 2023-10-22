#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import torch
from sensor_msgs.msg import Image
import rosbag
import threading
import queue
import time

# Initialize global variables
crosswalk_detected = False
motion_detected = False


# Global variables to store the previous frames and objects
previous_frames = []
previous_objects = []

# Number of frames to use for moving average
moving_average_frames = 10
motion_threshold = 1000  # Adjust this threshold as needed

def object_in_motion(current_frame, current_objects):
    global previous_frames, previous_objects

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




# Function to command Spot to perform actions
def command_spot(pub, action):
    rate = rospy.Rate(10)  # 10 Hz
    msg = Twist()

    if action == 'walk':
        msg.linear.x = 0.5  # Speed value
    elif action == 'stop':
        # No values to be set, Twist() initializes to 0
        pass
    elif action == 'turn':
        msg.angular.z = 0.5  # Turn speed value

    pub.publish(msg)
    rate.sleep()

# Callback for processing the image data and detection
def image_callback(data):
    global crosswalk_detected, motion_detected
    publisher = rospy.Publisher('image_from_rosbag', Image, queue_size=10)

    bridge = CvBridge()
    model = YOLO("models/yolov8m.pt")
    model_cw = YOLO("models/yolov8_crosswalk_detection.pt")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device)
    model_cw = model_cw.to(device)

    # Class names for visualization
    classNames = ["person", "bicycle", "car", "motorbike", "bus", "truck", "traffic light", "fire hydrant", "stop sign"]  

    try:
        # Convert the image message to a format OpenCV can handle
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        # Convert the image from BGR to RGB
        cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Predict using the first YOLO model
        results = model.predict(cv_image_rgb)

            # Create a copy of the current frame to check for motion
        current_frame = cv_image_rgb.copy()

        for result in results:
            original_image = result.orig_img
            motion_text = "Stationary"  # Default text

            for box, conf, class_id in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
                class_id = int(class_id)
                class_name = result.names[class_id]
                
                if class_name in classNames and conf > 0.5:  # Filter by desired class and confidence threshold
                    x_min, y_min, x_max, y_max = box
                    x_min, y_min, x_max, y_max = map(int, [x_min, y_min, x_max, y_max])
                    cv2.rectangle(original_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  # Draw a green rectangle
                    cv2.putText(original_image, class_name, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Check for motion
                    motion_detected = object_in_motion(current_frame, result)
                    if motion_detected:
                        motion_text = "In Motion"

                    # Display motion status beside the class name
                    cv2.putText(original_image, motion_text, (x_max + 5, y_max), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            annotated_image_np = np.array(original_image)


            crosswalk_results = model_cw.predict(annotated_image_np)
            crosswalk_detected = False

            for crosswalk_result in crosswalk_results:
                for box, conf, class_id in zip(crosswalk_result.boxes.xyxy, crosswalk_result.boxes.conf, crosswalk_result.boxes.cls):
                    if class_id == 0 and conf > 0.5:  # Assuming class 0 is for crosswalk
                        crosswalk_detected = True
                        x_min, y_min, x_max, y_max = box
                        x_min, y_min, x_max, y_max = map(int, [x_min, y_min, x_max, y_max])
                        cv2.rectangle(annotated_image_np, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  # Draw a green rectangle
                        cv2.putText(annotated_image_np, "Crosswalk Detected", (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)



            # Create an Image message from the numpy array
            image_msg = bridge.cv2_to_imgmsg(annotated_image_np, encoding="bgr8")
            # Publish the annotated image to the topic
            publisher.publish(image_msg)



    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))




def navigate_spot(pub):
    global crosswalk_detected, motion_detected

    while not rospy.is_shutdown():
        if crosswalk_detected:
            if motion_detected == False:
                # If the path is clear, walk forward or cross the crosswalk
                command_spot(pub, 'walk')
            else:
                # If vehicles are moving, Spot should stop and wait
                command_spot(pub, 'stop')
        else:
            # If no crosswalk is detected, Spot can turn or search for a crosswalk
            command_spot(pub, 'turn')  # or 'walk', or any other logic you decide on

        rospy.sleep(1)  # Sleep for a short duration before the next iteration

def main():
    rospy.init_node('spot_navigator', anonymous=True)

    # Publisher for sending commands to Spot
    pub = rospy.Publisher('/spot/command', Twist, queue_size=10)

    # Subscriber for receiving the compressed images
    rospy.Subscriber('/camera/rgb/image_raw/compressed', CompressedImage, image_callback)

    try:
        navigate_spot(pub)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
