from dataclasses import dataclass, field
from typing import List


@dataclass
class ContactData:
    msg_frame_id: int = 0
    msg_stamp: float = 0.0
    contact_left: float = 0.0
    contact_right: float = 0.0
