from abc import ABC, abstractmethod

import numpy as np


class BasePathPlanner(ABC):
    def __init__(self, map):
        self.map = map
        self.setup_subscriber()

    def setup_subscriber(self):
        """
        this method defines the subscribers and publisher
        """

    def run(self):
        """

        :return: this function returns the waypoints of the trajectory
        """