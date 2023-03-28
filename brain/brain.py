import logging
import traceback

from rclpy.node import Node as RosNode
from std_srvs.srv import SetBool

from menteebot.state_manager.manager import StateManager

logger = logging.getLogger("menteebot")


class Brain:
    def __init__(self, cfg):

        self.cfg = cfg
        self.ros_node = RosNode("menteebot")
        self.state_manager = StateManager(cfg.states, self.ros_node)
        # Initiate the main network loop at 1/freq
        self.motion_timer = self.ros_node.create_timer(
            1 / self.cfg.frequency, self.perform_main_loop
        )
        self.action_dict = {"left_shoulder_pitch":0, "left_shoulder_roll":0, "left_shoulder_yaw":0,
                            "left_elbow":0, "left_wrist_pitch":0, "left_wrist_yaw":0,
                            "left_wrist_roll":0, "right_shoulder_pitch":0, "right_shoulder_roll":0, "right_shoulder_yaw":0,
                            "right_elbow":0, "right_wrist_pitch":0, "right_wrist_yaw":0,
                            "right_wrist_roll":0}
        self.req_id = 0

    def call_step(self):
        # status = self.state_manager.states["dofs"][:]
        self.action_dict["right_shoulder_pitch"] = -1* self.state_manager.states["dofs"].dofs["left_shoulder_pitch"].pos
        self.action_dict["right_shoulder_roll"] = -1*self.state_manager.states["dofs"].dofs["left_shoulder_roll"].pos
        self.action_dict["right_shoulder_yaw"] = -1*self.state_manager.states["dofs"].dofs["left_shoulder_yaw"].pos
        self.action_dict["right_elbow"] = -1*self.state_manager.states["dofs"].dofs["left_elbow"].pos
        # self.action_dict["left_wrist_pitch"] = self.state_manager.states["dofs"].dofs["left_wrist_pitch"].pos
        # self.action_dict["left_wrist_yaw"] = self.state_manager.states["dofs"].dofs["left_wrist_yaw"].pos
        self.action_dict["right_wrist_roll"] = -1*self.state_manager.states["dofs"].dofs["left_wrist_roll"].pos
        print(self.action_dict)
        # print(shoulder_pitch, shoulder_roll, shoulder_yaw, elbow, wrist_roll )

    def perform_main_loop(self, event=None) -> None:
        self.state_manager.update()
        self.call_step()

        # print("loop")
        self.state_manager.states["dofs"].publish_state(action_dict=self.action_dict, req_id=self.req_id)
        self.req_id +=1
