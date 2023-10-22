import rospy
from geometry_msgs.msg import Pose
import numpy as np
from tf.transformations import euler_from_quaternion  # Import quaternion to Euler angles conversion

from path_planner.path_planners import get_path_planner
from social_navigation.social_navigation_planners import get_social_navigation_planner
from utils.configurations import PathPlannerConfiguration, SocialNavigationConfiguration, SocialNavigationStatus, ROSTopics
from utils.functions import euler_from_quaternion

class TaskManager:
    def __init__(self):
        self.waypoints_jump_indices = 10  # Define your desired waypoints_jump_indices
        self.target_threshold = 0.1  # Define your desired target_threshold
        self.path_planner = get_path_planner(PathPlannerConfiguration)
        self.social_navigation = get_social_navigation_planner(SocialNavigationConfiguration)

        self.pose = None
        self.target = None
        self.waypoint_idx = None

        # Subscribe to Pose and Target topics
        rospy.Subscriber(ROSTopics.pose, Pose, self.position_callback)
        rospy.Subscriber(ROSTopics.target, Pose, self.target_callback)

    def set_target(self, target_pose):
        self.target = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])

    def target_callback(self, msg: Pose):
        self.set_target(msg)

    def position_callback(self, msg: Pose):
        r, p, y = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.pose = np.array([msg.position.x, msg.position.y, msg.position.z, r, p, y])

    def reset(self):
        self.path_planner.reset()
        self.social_navigation.reset()

    def run_path_planner(self):
        pass

    def run_social_navigation(self, waypoints):
        if self.social_navigation.status == SocialNavigationStatus.Running:
            pass
        elif self.social_navigation.status == SocialNavigationStatus.Arrived:
            self.social_navigation.run(waypoints[self.waypoint_idx])
        else:
            raise ValueError("the social navigation status {} is not defined".format(self.social_navigation.status))

    def find_nearest_waypoints(self, waypoints):
        if self.waypoint_idx is None:
            self.waypoint_idx = 0
        else:
            waypoints = np.array(waypoints)  # Convert the list of waypoints to a NumPy array
            distances = np.linalg.norm(waypoints - waypoints[self.waypoint_idx], axis=1)
            smallest_index = np.argmin(distances)
            if self.waypoints_jump_indices >= (smallest_index - self.waypoint_idx) > 0:
                self.waypoint_idx = smallest_index


    # def run(self):
    #     waypoints = self.path_planner.run()
    #     while np.linalg.norm(self.pose[:3] - self.target) > self.target_threshold:
    #         self.find_nearest_waypoints(waypoints=waypoints)
    #         self.run_social_navigation(waypoints=waypoints)



    def run(self, motion_text, crosswalk_detected, waypoint_file):
        # Load waypoints from the text file
        waypoints = self.load_waypoints_from_file(waypoint_file)
        if waypoints is None:
            rospy.logerr("Failed to load waypoints from the file.")
            return

        self.pose = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # Initial pose
        current_waypoint_index = 0  # Keep track of the current waypoint
        self.target = waypoints[current_waypoint_index]  # Set the initial target position to the first waypoint

        while current_waypoint_index < len(waypoints):
            self.find_nearest_waypoints(waypoints=waypoints)

            if crosswalk_detected:
                rospy.loginfo("Crosswalk detected. STOP!")
                # Handle the case when a crosswalk is detected (e.g., stop the robot)
            else:
                # if np.linalg.norm(self.pose[:3] - self.target) > self.target_threshold:
                #     if motion_text == 'Stationary':
                #         rospy.loginfo("Continue to next waypoint")
                #         # If no moving vehicles are detected, proceed to the next waypoint
                #         self.run_social_navigation(waypoints=waypoints)
                #     else:
                #         rospy.loginfo("Cars moving. STOP!")
                #         # If moving vehicles are detected, continue to wait
                # else:
                current_waypoint_index += 1  # Move to the next waypoint
                if current_waypoint_index < len(waypoints):
                    self.pose = self.target  # Update pose to the previous target
                    self.target = waypoints[current_waypoint_index]  # Update the target to the next waypoint
                    rospy.loginfo(f"Reached waypoint {current_waypoint_index}, target is {self.target}")
                else:
                    rospy.loginfo("Reached the final waypoint")
                    break

    # You may want to add some final actions or cleanup here


        # You may want to add some final actions or cleanup here

    def load_waypoints_from_file(self, waypoint_file):
        try:
            with open(waypoint_file, 'r') as file:
                lines = file.readlines()
                waypoints = []
                for line in lines:
                    parts = line.strip().split(',')
                    if len(parts) >= 3:
                        x, y, z = map(float, parts[:3])
                        waypoints.append([x, y, z])
                return waypoints
        except Exception as e:
            rospy.logerr(f"Error loading waypoints from the file: {str(e)}")
            return None
