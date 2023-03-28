from abc import abstractmethod

import numpy as np

from .datatypes import ContactData


class BaseContact:
    def __init__(self, name, cfg, ros_node):
        self.name = name
        self.cfg = cfg

        self.data = ContactData()

    @abstractmethod
    def get_data(self) -> ContactData:
        """get contact data"""
