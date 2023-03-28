import logging
import time

import serial
from rich.console import Console

from ...utils.recorder import Recorder
from .base import BaseContact
from .datatypes import ContactData
from .read_hx711 import ContactArduino

console = Console()
logger = logging.getLogger("menteebot")


class ContactRead(BaseContact):
    def __init__(self, name, cfg, ros_node=None):
        super(ContactRead, self).__init__(name, cfg, ros_node)
        self.contact = ContactArduino()
        if self.contact.port == "NULL":
            logger.error("No Contact sensor was found")
        self.request_id = 0
        self.recorder = Recorder(
            f"{self.name}.csv",
            ros_node,
            fields_names=["frame_id", "time", "left_data", "right_data"],
        )

    def get_data(self) -> ContactData:
        try:
            self.data.contact_right, self.data.contact_left = self.contact.get_contact_data()
            self.data.msg_stamp = time.time()
            self.data.msg_frame_id = self.request_id
            self.request_id += 1
            self.recorder.record_row(
                {
                    "frame_id": str(self.data.msg_frame_id),
                    "time": str(self.data.msg_stamp),
                    "left_data": str(self.data.contact_left),
                    "right_data": str(self.data.contact_right),
                }
            )
        except serial.serialutil.SerialException:
            logger.error("Device disconnected (or multiple access on port).")

        return self.data

    # def __del__(self):
    #     self.evo.close()
