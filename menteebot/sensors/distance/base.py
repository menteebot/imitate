from abc import abstractmethod

import numpy as np

from .datatypes import DistanceData


class BaseDistance:
    GRAVITY = np.array([0, 0, -9.81])

    def __init__(self, name, cfg, ros_node):
        self.name = name
        self.cfg = cfg

        # IMU data
        self.data = DistanceData()

    @abstractmethod
    def get_data(self) -> DistanceData:
        """get distance data"""
