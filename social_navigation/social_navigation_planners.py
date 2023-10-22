from utils.configurations import SocialNavigationConfiguration, SocialNavigationType, SocialNavigationStatus, PathPlannerConfiguration, ROSTopics
import rospy
import cv2
import numpy as np
from geometry_msgs.msg import Pose
from path_planner.path_planners import get_path_planner
from utils.functions import euler_from_quaternion
from sensor_msgs.msg import Image

def get_social_navigation_planner(configs:SocialNavigationConfiguration):
    if configs.type == SocialNavigationType.object_detection_navigation:
        return None
    else:
        raise ValueError("the path planner type {} is not defined".format(configs.type))
    












    
# def get_social_navigation_planner(configs: SocialNavigationConfiguration):
#     if configs.type == SocialNavigationType.object_detection_navigation:
#         rospy.init_node('global_navigation', anonymous=True)

#         planner = ObjectDetectionSocialNavigationPlanner(configs)


#         rate = rospy.Rate(10)  # Control loop rate

#         while not rospy.is_shutdown():
#             planner.navigate()
#             rate.sleep()

#         return planner
#     else:
#         raise ValueError("the social navigation type {} is not defined".format(configs.type))



# class ObjectDetection:
#     def __init__(self):
#         self.waypoints_jump_indices = 2
#         self.target_threshold = 5
#         self.path_planner = get_path_planner(PathPlannerConfiguration)
#         # self.social_navigation = get_social_navigation_planner(SocialNavigationConfiguration)

#         self.pose = None
#         self.target = None
#         self.waypoint_idx = None
#         self.prev_frame = None  # Store the previous frame
#         self.model = 'models/yolov8m.pt'

#         rospy.Subscriber(ROSTopics.pose, Pose, self.position_callback)
#         rospy.Subscriber(ROSTopics.target, Pose, self.target_callback)
#         # rospy.Subscriber(ROSTopics.camera_image, Image, self.camera_callback)
#     def initialize_social_navigation(self, social_navigation_config):
#         self.social_navigation = get_social_navigation_planner(social_navigation_config)



#     def target_callback(self, msg: Pose):
#         self.target = np.array([msg.position.x, msg.position.y, msg.position.z])

#     def position_callback(self, msg: Pose):
#         r, p, y = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
#         self.pose = np.array([msg.position.x, msg.position.y, msg.position.z, r, p, y])

#     def reset(self):
#         self.path_planner.reset()
#         self.social_navigation.reset()

#     def run_path_planner(self):
#         pass

#     def run_social_navigation(self, waypoints):
#         if self.social_navigation.status == SocialNavigationStatus.Running:
#             pass
#         elif self.social_navigation.status == SocialNavigationStatus.Arrived:
#             self.social_navigation.run(waypoints[self.waypoint_idx])
#         else:
#             raise ValueError("The social navigation status {} is not defined".format(self.social_navigation.status))

#     def find_nearest_waypoints(self, waypoints):
#         if self.waypoint_idx is None:
#             self.waypoint_idx = 0
#         else:
#             distances = np.linalg.norm(waypoints - waypoints[self.waypoint_idx], axis=1)
#             smallest_index = np.argmin(distances)
#             if self.waypoints_jump_indices >= smallest_index - self.waypoint_idx > 0:
#                 self.waypoint_idx = smallest_index

#     def camera_callback(self, msg: Image):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#             detected_objects = self.detect_objects_in_frame(frame)  # Detect objects
#         except Exception as e:
#             print(e)

#     def detect_objects_in_frame(self, frame):
#         results = self.model(frame)  # Implement object detection using YOLO model
#         classNames = ["person", "bicycle", "car", "motorbike", "bus", "truck", "traffic light", "fire hydrant", "stop sign"]

#         for result in results:
#             motion_detected = self.detect_motion(frame, result)

#             for box, conf, class_id in zip(result.boxes.xyxy, result.boxes.conf, result.boxes.cls):
#                 class_id = int(class_id)
#                 class_name = result.names[class_id]

#                 if class_name in classNames and conf > 0.5:  # Filter by desired class and confidence threshold
#                     x_min, y_min, x_max, y_max = map(int, box)
#                     cv2.rectangle(result.orig_img, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)  # Draw a green rectangle
#                     cv2.putText(result.orig_img, class_name, (x_min, y_min - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

#                     motion_text = "Stationary" if not motion_detected else "In Motion"
#                     cv2.putText(result.orig_img, motion_text, (x_max + 5, y_max), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#         # return results

#     def detect_motion(self, frame, result):
#         moving_average_frames = 10
#         motion_threshold = 10000
#         global previous_frames, previous_objects

#         if len(previous_frames) < moving_average_frames:
#             previous_frames.append(frame)
#             previous_objects.append(result)
#             return False

#         frame_diff = cv2.absdiff(frame, previous_frames[0])
#         frame_diff_gray = cv2.cvtColor(frame_diff, cv2.COLOR_BGR2GRAY)

#         _, motion_mask = cv2.threshold(frame_diff_gray, 30, 255, cv2.THRESH_BINARY)
#         motion_pixel_count = np.count_nonzero(motion_mask)

#         motion_detected = motion_pixel_count > motion_threshold

#         previous_frames.append(frame)
#         previous_objects.append(result)
#         previous_frames.pop(0)
#         previous_objects.pop(0)

#         return motion_detected

# class ObjectDetectionResult:
#     def __init__(self):
#         self.objects = []  # List of detected object classes

#     def add_detection(self, obj_class, confidence, bounding_box):
#         self.objects.append({
#             'class': obj_class,
#             'confidence': confidence,
#             'bounding_box': bounding_box
#         })

# class ObjectDetectionSocialNavigationPlanner:
#     def __init__(self, configs):
#         self.configs = configs
#         self.status = SocialNavigationStatus.Hanging
#         # self.object_detector = ObjectDetection()
#     def initialize_object_detector(self, object_detection):
#         # self.object_detector = object_detection
#         rospy.loginfo("Detecting Objects")
#         # pass
#     def navigate(self):
#         # rospy.loginfo("Navigating")
#         # Access object detection results from the ObjectDetector class
#         # detected_objects = self.object_detector.get_detected_objects()
#         # Add your navigation logic here
#         pass

# def get_social_navigation_planner(configs: SocialNavigationConfiguration):
#     if configs.type == SocialNavigationType.object_detection_navigation:
#         rospy.init_node('global_navigation', anonymous=True)
#         planner = ObjectDetectionSocialNavigationPlanner(configs)
#         object_detection = ObjectDetection()  # Create an instance of ObjectDetection
#         planner.initialize_object_detector(object_detection)
#         object_detection.initialize_social_navigation(configs)

#         rate = rospy.Rate(10)  # Control loop rate

#         while not rospy.is_shutdown():
#             planner.navigate()
#             rate.sleep()

#         return planner
#     else:
#         raise ValueError("The social navigation type {} is not defined".format(configs.type))