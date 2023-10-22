#!/usr/bin/env python

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch
from sensor_msgs.msg import Image
import rosbag
from task_manager.manager import TaskManager
import threading
import queue

exit_program = False
data_queue = queue.Queue()
def run_task_manager(task_manager, motion_text, crosswalk_detected, waypoint_file):
    task_manager.run(motion_text, crosswalk_detected, waypoint_file)
    global exit_program
    exit_program = True


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

def publish_images_from_rosbag(rosbag_path, topic_name, playback_speed=36):
    rospy.init_node('global_navigation', anonymous=True)
    pub = rospy.Publisher('image_from_rosbag', Image, queue_size=10)
    bridge = CvBridge()
    task_manager = TaskManager()

    model = YOLO("models/yolov8m.pt")
    model_cw = YOLO("models/yolov8_crosswalk_detection.pt")
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device)
    model_cw = model_cw.to(device)

    # Class names for visualization
    classNames = ["person", "bicycle", "car", "motorbike", "bus", "truck", "traffic light", "fire hydrant", "stop sign"]    
    # Calculate the playback rate
    rate = rospy.Rate(40 * playback_speed)  # Assuming the original rate is 10 Hz

    try:
        bag = rosbag.Bag(rosbag_path, 'r')
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            compressed_data = msg.data
            np_arr = np.fromstring(compressed_data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            results = model.predict(cv_image)

            # Create a copy of the current frame to check for motion
            current_frame = cv_image.copy()

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

                # Convert the annotated image to a numpy array
                annotated_image_np = np.array(original_image)

                # Pass the annotated image through the model_cw for crosswalk detection
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
                pub.publish(image_msg)
                # rate.sleep()  # Control the publishing rate based on the desired playback speed
        
            task_manager_thread = threading.Thread(target=run_task_manager, args=(task_manager, motion_text, crosswalk_detected), kwargs={'waypoint_file': 'rotated_waypoints3.txt'})
            task_manager_thread.start()        
        exit_program = True
        bag.close()
        task_manager_thread.join()
        # Check the exit_program flag and exit the program if it's True
        if exit_program:
            rospy.loginfo("All waypoints have been processed. Exiting the program.")
            rospy.signal_shutdown("All waypoints processed")
    except rosbag.bag.ROSBagUnindexedException:
        rospy.logerr("The provided ROSBag is not indexed. Please run 'rosbag reindex <your_bag.bag>'.")
    except rospy.ROSInterruptException:
        pass
