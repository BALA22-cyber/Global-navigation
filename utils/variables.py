from gym.spaces import Dict
from utils.configurations import *

"""
Variables of robots observation
"""


class ObservationKeys:
    lidar = "lidar"
    velocity = "velocity"
    imu = "imu"
    goal = "goal"
    pose = "pose"
    traversability_map = "traversability_map"
    gps = "gps"


ObservationSpace = Dict(
    {
        ObservationKeys.lidar: LidarSpace,
        ObservationKeys.velocity: VelocitySpace,
        ObservationKeys.goal: GoalSpace,
        ObservationKeys.pose: PoseSpace,
    }
)

