from dataclasses import dataclass, field
from typing import List


@dataclass
class DistanceData:
    msg_frame_id: int = 0
    msg_stamp: float = 0.0
    distance: float = 0.0
