#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from ultralytics import YOLO
import torch
from sensor_msgs.msg import Image  # Updated from CompressedImage to Image
import roslaunch  # Import roslaunch API

# Global flags
crosswalk_detected = False
motion_detected = False
# def start_realsense_driver():
#     uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
#     roslaunch.configure_logging(uuid)
#     launch = roslaunch.parent.ROSLaunchParent(uuid, ["realsense2_camera", "rs_camera.launch"])
#     launch.start()
# Callback for processing the image data and detection
def image_callback(data):
    global crosswalk_detected, motion_detected
    publisher = rospy.Publisher('processed_camera_image', Image, queue_size=10)

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

def navigate_husky(pub):
    global crosswalk_detected, motion_detected
    rate = rospy.Rate(10)  # 10Hz

    move_cmd = Twist()
    move_cmd.linear.x = 1  # speed for Husky
    for _ in range(10):  # Assuming 1 m/s speed and 10Hz publishing rate
        pub.publish(move_cmd)
        # rate.sleep()

    start_time = rospy.get_time()
    while rospy.get_time() - start_time < 10:  # Assuming Husky covers 5 meters in 10 seconds at 0.5 m/s speed.
        rospy.sleep(0.1)
        if crosswalk_detected:
            stop_cmd = Twist()
            for _ in range(10):  # Assuming 1 m/s speed and 10Hz publishing rate
                pub.publish(move_cmd)
                rate.sleep(1)
            
            orient_cmd = Twist()
            orient_cmd.angular.z = 0.5  
            for _ in range(10):  # Assuming 1 m/s speed and 10Hz publishing rate
                pub.publish(orient_cmd)
                rate.sleep(1)
            
            stop_cmd = Twist()
            for _ in range(10):  # Assuming 1 m/s speed and 10Hz publishing rate
                pub.publish(stop_cmd)
                rate.sleep(1)

            while motion_detected:  
                rospy.loginfo("Waiting for vehicles to stop...")
                rospy.sleep(1)

            cross_cmd = Twist()
            cross_cmd.linear.x = 0.5
            for _ in range(10):  # Assuming 1 m/s speed and 10Hz publishing rate
                pub.publish(cross_cmd)
                rate.sleep(5)

            crosswalk_detected = False  
            return  

    turn_cmd = Twist()
    turn_cmd.angular.z = -0.5  
    for _ in range(10):  # Assuming 1 m/s speed and 10Hz publishing rate
        pub.publish(turn_cmd)
        rospy.sleep(2) 

    stop_cmd = Twist()
    pub.publish(stop_cmd)

def main():
    # start_realsense_driver()  # Start the RealSense driver

    rospy.init_node('husky_navigator', anonymous=True)
    pub = rospy.Publisher('/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    
    try:
        navigate_husky(pub)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
