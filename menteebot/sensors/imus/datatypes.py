from dataclasses import dataclass, field
from typing import List


@dataclass
class CoordSys:
    local: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    lab: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class IMUData:
    msg_frame_id: int = 0
    msg_stamp: float = 0.0
    position: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    quaternion: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    linear_velocity: CoordSys = field(default_factory=CoordSys)
    angular_velocity: CoordSys = field(default_factory=CoordSys)
    acceleration: CoordSys = field(default_factory=CoordSys)
