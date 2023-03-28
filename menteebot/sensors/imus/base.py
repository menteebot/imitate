from abc import abstractmethod
from collections import defaultdict
from typing import Tuple

import numpy as np

from .datatypes import IMUData


class BaseIMU:
    GRAVITY = np.array([0, 0, -9.81])

    def __init__(self, name, cfg, ros_node):
        self.name = name
        self.cfg = cfg

        # IMU data
        self.data = defaultdict(IMUData)

    @abstractmethod
    def imu_callback(self, msg) -> Tuple[str, IMUData]:
        """IMU ros subscriber function"""
