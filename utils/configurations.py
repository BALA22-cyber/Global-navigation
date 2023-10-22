import math

from gym.spaces import Box
import numpy as np

from easydict import EasyDict as edict

"""
ROS Topics
"""
ROSTopics = edict()
ROSTopics.pose = "pose"   # message type: geometry_msgs/Pose
ROSTopics.target = "target"   # message type: geometry_msgs/Pose


"""
Configurations of motion planners
"""
action_linear_range = (0.0, 1.0)
action_angular_range = (-1.0, 1.0)

ActionSpace = Box(low=np.array([action_linear_range[0], action_angular_range[0]]),
                  high=np.array([action_linear_range[1], action_angular_range[1]]), shape=(2,), dtype=np.float32)


class MotionPlannerStatus:
    Hanging = "hanging"
    Arrived = "arrived"
    Running = "running"


class MotionPlannerType:
    dwa = "dwa"


DWAConfiguration = edict()
DWAConfiguration.v_range = (0., 1.)
DWAConfiguration.a_range = (-1.0, 1.0)
DWAConfiguration.a_max = (3., 5.)
DWAConfiguration.time_step = 0.1
DWAConfiguration.predict_time = 2.0
DWAConfiguration.to_goal_cost_gain = 3.0
DWAConfiguration.speed_cost_gain = 0.1
DWAConfiguration.obs_cost_gain = 0.3
DWAConfiguration.radius = 0.5
DWAConfiguration.goal_threshold = 0.5
DWAConfiguration.v_resolution = 0.3
DWAConfiguration.w_resolution = 0.3
DWAConfiguration.lidar_fov = (-2 * math.pi / 3, 2 * math.pi / 3)
DWAConfiguration.lidar_size = 341

MotionPlannerConfiguration = edict()
MotionPlannerConfiguration.type = MotionPlannerType.dwa
MotionPlannerConfiguration.configs = DWAConfiguration

"""
Configurations of sensor data
"""

lidar_range = (0., 10.)
lidar_shape = (341,)

GoalSpace = Box(high=np.inf, low=-np.inf, shape=(3,), dtype=np.float32)
PoseSpace = Box(high=np.inf, low=-np.inf, shape=(6,), dtype=np.float32)
LidarSpace = Box(high=lidar_range[1], low=lidar_range[0], shape=lidar_shape, dtype=np.float32)
VelocitySpace = Box(high=np.inf, low=-np.inf, shape=(2,), dtype=np.float32)


"""
"""

TaskManager = edict()
TaskManager.waypoints_jump_indices = 2
TaskManager.target_threshold = 5

"""
Configurations of path planner
"""


class PathPlannerType:
    a_star = "a_star"
    rrt_star = "rrt_star"


PathPlannerConfiguration = edict()
PathPlannerConfiguration.type = PathPlannerType.a_star
PathPlannerConfiguration.traversability_map_directory = ""

"""
Configurations of social navigation
"""


class SocialNavigationStatus:
    Hanging = "hanging"
    Arrived = "arrived"
    Running = "running"


class SocialNavigationType:
    object_detection_navigation = "object_detection_navigation"


SocialNavigationConfiguration = edict()
SocialNavigationConfiguration.type = SocialNavigationType.object_detection_navigation
