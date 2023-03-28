import logging
import time

import serial
from rich.console import Console

from ...utils.recorder import Recorder
from .base import BaseDistance
from .datatypes import DistanceData
from .Evo_single_point_display_range_py3 import Evo

console = Console()
logger = logging.getLogger("menteebot")


class TeraRangerEvo(BaseDistance):
    def __init__(self, name, cfg, ros_node=None):
        super(TeraRangerEvo, self).__init__(name, cfg, ros_node)
        self.evo = Evo()
        if self.evo.port == "NULL":
            logger.error("No Evo distance sensor was found")
        self.request_id = 0
        self.recorder = Recorder(
            f"{self.name}.csv",
            ros_node,
            fields_names=["frame_id", "time", "data"],
        )

    def get_data(self) -> DistanceData:
        try:
            self.data.distance = self.evo.get_evo_range()
            self.data.msg_stamp = time.time()
            self.data.msg_frame_id = self.request_id
            self.request_id += 1
            self.recorder.record_row(
                {
                    "frame_id": str(self.data.msg_frame_id),
                    "time": str(self.data.msg_stamp),
                    "data": str(self.data.distance),
                }
            )
        except serial.serialutil.SerialException:
            logger.error("Device disconnected (or multiple access on port).")

        return self.data

    # def __del__(self):
    #     self.evo.close()
