import time

from common.msg import RigidBodyState, StateEstimate, Vector3LL
from geometry_msgs.msg import Quaternion, QuaternionStamped, Vector3, Vector3Stamped
from rclpy.node import Node as RosNode
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from ...utils.recorder import Recorder
from ..utils.math_utils import *
from .base import BaseIMU


class IMUStateEstimation(BaseIMU):
    def __init__(self, name, cfg, ros_node: RosNode):
        super(IMUStateEstimation, self).__init__(name, cfg, ros_node)
        subs_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
        )
        self.imu_commander = ros_node.create_subscription(
            String, "/imu/commander", self.command_imu, subs_qos
        )
        self.imu_on = True
        self.lin_v_local = self.lin_v_lab = self.lin_a_local = self.lin_a_lab = [0, 0, 0]
        self.ang_v_local = self.ang_v_lab = [0, 0, 0]
        self.rot = [0, 0, 0, 1]
        self.pos = [0, 0, 0]
        self.step_zero = True

        self.state_estimation_info = ros_node.create_subscription(
            RigidBodyState, "/state_estimation/stater", self.imu_state_estimation, subs_qos
        )

        # Last
        self.prev_lin_a_local = self.prev_lin_a_lab = [0, 0, 0]
        self.prev_lin_v_local = self.prev_lin_v_lab = [0, 0, 0]
        self.prev_ang_v_local = self.prev_ang_v_lab = [0, 0, 0]
        self.prev_rot = [0, 0, 0, 1]
        self.prev_pos = [0, 0, 0]

        self.subscribers = []
        for topic_name in self.cfg.topics:
            self.subscribers.append(
                ros_node.create_subscription(Imu, topic_name, self.imu_callback, subs_qos)
            )

        self.recorder = Recorder(
            f"{self.name}.csv",
            ros_node,
            fields_names=[
                "frame_id",
                "which_topic",
                "stamp",
                "curr_time",
                "quaternion",
                "position",
                "angular_velocity.lab",
                "angular_velocity.local",
                "acceleration.lab",
                "acceleration.local",
                "linear_velocity.lab",
                "linear_velocity.local",
                "callback_time",
            ],
        )
        self.publisher = ros_node.create_publisher(RigidBodyState, f"/imu/state_estimater", subs_qos)

        # EMA
        self.do_ema = self.cfg.get("emma", False)
        self.alpha = 0.5

    def imu_callback(self, msg: Imu):
        curr_time = time.time()
        with self.recorder.timeit("callback_time"):
            msg_stamp = msg.header.stamp.sec + (1e-9 * msg.header.stamp.nanosec)
            if self.imu_on:
                # Quaternion
                self.quat = [
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                    msg.orientation.w,
                ]

                # Angular velocity
                self.ang_v_local = [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ]
                self.ang_v_lab = body_to_inertial_frame(
                    np.array(self.quat), np.array(self.ang_v_local)
                ).tolist()

                # Linear acceleration
                self.lin_a_lab = [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ]
                self.lin_a_local = inertial_to_body_frame(
                    np.array(self.quat), np.array(self.lin_a_lab)
                ).tolist()

                # Linear velocity
                if self.step_zero:
                    self.step_zero = False
                else:
                    self.lin_v_local = (
                        np.array(self.lin_v_local)
                        + (msg_stamp - self.prev_time) * self.prev_lin_a_local
                    ).tolist()
                    self.lin_v_lab = (
                        np.array(self.lin_v_lab)
                        + (msg_stamp - self.prev_time) * self.prev_lin_a_lab
                    ).tolist()

                # EMA
                if self.do_ema:
                    self.quat = (
                        np.array(self.quat) * self.alpha
                        + np.array(self.prev_quat) * (1 - self.alpha)
                    ).tolist()
                    self.ang_v_local = (
                        np.array(self.ang_v_local) * self.alpha
                        + np.array(self.prev_ang_v_local) * (1 - self.alpha)
                    ).tolist()
                    self.ang_v_lab = (
                        np.array(self.ang_v_lab) * self.alpha
                        + np.array(self.prev_ang_v_lab) * (1 - self.alpha)
                    ).tolist()
                    self.lin_a_lab = (
                        np.array(self.lin_a_lab) * self.alpha
                        + np.array(self.prev_lin_a_lab) * (1 - self.alpha)
                    ).tolist()
                    self.lin_a_local = (
                        np.array(self.lin_a_local) * self.alpha
                        + np.array(self.prev_lin_a_local) * (1 - self.alpha)
                    ).tolist()

                self.prev_quat = self.quat
                self.prev_ang_v_local = self.ang_v_local
                self.prev_ang_v_lab = self.ang_v_lab
                self.prev_lin_a_local = np.array(self.lin_a_local)
                self.prev_lin_a_lab = np.array(self.lin_a_lab)
                self.prev_lin_v_local = np.array(self.lin_v_local)
                self.prev_lin_v_lab = np.array(self.lin_v_lab)
                self.prev_time = msg_stamp

                self.data[self.name].msg_frame_id = msg.header.frame_id
                self.data[self.name].msg_stamp = msg_stamp
                self.data[self.name].quaternion = self.quat
                self.data[self.name].angular_velocity.lab = self.ang_v_lab
                self.data[self.name].angular_velocity.local = self.ang_v_local
                self.data[self.name].acceleration.lab = self.lin_a_lab
                self.data[self.name].acceleration.local = self.lin_a_local
                self.data[self.name].linear_velocity.lab = self.lin_v_lab
                self.data[self.name].linear_velocity.local = self.lin_v_local

            else:

                self.data[self.name].quaternion = [0, 0, 0, 1]
                self.data[self.name].angular_velocity.lab = [0, 0, 0]
                self.data[self.name].angular_velocity.local = [0, 0, 0]
                self.data[self.name].acceleration.lab = [0, 0, 0]
                self.data[self.name].acceleration.local = [0, 0, 0]
                self.data[self.name].linear_velocity.lab = [0, 0, 0]
                self.data[self.name].linear_velocity.local = [0, 0, 0]

        # self.imu_publish()
        self.recorder.add_dict_to_buffer(
            {
                "frame_id": str(msg.header.frame_id),
                "which_topic": "imu_data",
                "stamp": str(msg_stamp),
                "curr_time": str(curr_time),
                "quaternion": str(self.data[self.name].quaternion),
                "angular_velocity.lab": str(self.data[self.name].angular_velocity.lab),
                "angular_velocity.local": str(self.data[self.name].angular_velocity.local),
                "acceleration.lab": str(self.data[self.name].acceleration.lab),
                "acceleration.local": str(self.data[self.name].acceleration.local),
                "linear_velocity.lab": str(self.data[self.name].linear_velocity.lab),
                "linear_velocity.local": str(self.data[self.name].linear_velocity.local),
            }
        )
        self.recorder.flush()
        return self.name, self.data[self.name]

    def imu_publish(self):

        imu_state_msg = RigidBodyState()
        imu_state_msg.name.append("base_link")

        rot = Quaternion()
        rot.x = self.data[self.name].quaternion[0]
        rot.y = self.data[self.name].quaternion[1]
        rot.z = self.data[self.name].quaternion[2]
        rot.w = self.data[self.name].quaternion[3]
        imu_state_msg.rot.append(rot)

        ang_vel = Vector3LL()
        ang_vel.local.x = self.data[self.name].angular_velocity.local[0]
        ang_vel.local.y = self.data[self.name].angular_velocity.local[1]
        ang_vel.local.z = self.data[self.name].angular_velocity.local[2]
        ang_vel.lab.x = self.data[self.name].angular_velocity.lab[0]
        ang_vel.lab.y = self.data[self.name].angular_velocity.lab[1]
        ang_vel.lab.z = self.data[self.name].angular_velocity.lab[2]
        imu_state_msg.ang_vel.append(ang_vel)

        acc = Vector3LL()
        acc.local.x = self.data[self.name].acceleration.local[0]
        acc.local.y = self.data[self.name].acceleration.local[1]
        acc.local.z = self.data[self.name].acceleration.local[2]
        acc.lab.x = self.data[self.name].acceleration.lab[0]
        acc.lab.y = self.data[self.name].acceleration.lab[1]
        acc.lab.z = self.data[self.name].acceleration.lab[2]
        imu_state_msg.acc.append(acc)

        lin_vel = Vector3LL()
        lin_vel.local.x = self.data[self.name].linear_velocity.local[0]
        lin_vel.local.y = self.data[self.name].linear_velocity.local[1]
        lin_vel.local.z = self.data[self.name].linear_velocity.local[2]
        lin_vel.lab.x = self.data[self.name].linear_velocity.lab[0]
        lin_vel.lab.y = self.data[self.name].linear_velocity.lab[1]
        lin_vel.lab.z = self.data[self.name].linear_velocity.lab[2]
        imu_state_msg.lin_vel.append(lin_vel)

        pos = Vector3()
        pos.x = self.data[self.name].position[0]
        pos.y = self.data[self.name].position[1]
        pos.z = self.data[self.name].position[2]
        imu_state_msg.pos.append(pos)

        self.publisher.publish(imu_state_msg)

    def command_imu(self, msg):

        case = msg.data
        if case == "imu_off":
            self.imu_on = False
            print("IMU OFF")

        elif case == "imu_on":
            self.imu_on = True
            print("IMU ON")

    def imu_state_estimation(self, msg: RigidBodyState):

        curr_time = time.time()
        with self.recorder.timeit("callback_time"):
            msg_stamp = msg.header.stamp.sec + (1e-9 * msg.header.stamp.nanosec)
            if self.imu_on:
                # Quaternion
                self.rot = [
                    msg.rot.x,
                    msg.rot.y,
                    msg.rot.z,
                    msg.rot.w,
                ]

                # Position
                self.pos = [
                    msg.pos.x,
                    msg.pos.y,
                    msg.pos.z,
                ]

                # Angular velocity local
                self.ang_v_local = [
                    msg.ang_vel.local.x,
                    msg.ang_vel.local.y,
                    msg.ang_vel.local.z,
                ]
                # Angular velocity lab
                self.ang_v_local = [
                    msg.ang_vel.lab.x,
                    msg.ang_vel.lab.y,
                    msg.ang_vel.lab.z,
                ]
                # self.ang_v_lab = body_to_inertial_frame(
                #     np.array(self.quat), np.array(self.ang_v_local)
                # ).tolist()

                # Linear acceleration lab
                self.lin_a_lab = [
                    msg.acc.lab.x,
                    msg.acc.lab.y,
                    msg.acc.lab.z,
                ]
                # Linear acceleration local
                self.lin_a_local = [
                    msg.acc.local.x,
                    msg.acc.local.y,
                    msg.acc.local.z,
                ]
                # self.lin_a_local = inertial_to_body_frame(
                #     np.array(self.quat), np.array(self.lin_a_lab)
                # ).tolist()

                # Linear velocity lab
                self.lin_v_lab = [
                    msg.lin_vel.lab.x,
                    msg.lin_vel.lab.y,
                    msg.lin_vel.lab.z,
                ]
                # Linear velocity local
                self.lin_v_local = [
                    msg.lin_vel.local.x,
                    msg.lin_vel.local.y,
                    msg.lin_vel.local.z,
                ]
                # if self.step_zero:
                #     self.step_zero = False
                # else:
                #     self.lin_v_local = (
                #         np.array(self.lin_v_local)
                #         + (msg_stamp - self.prev_time) * self.prev_lin_a_local
                #     ).tolist()
                #     self.lin_v_lab = (
                #         np.array(self.lin_v_lab)
                #         + (msg_stamp - self.prev_time) * self.prev_lin_a_lab
                #     ).tolist()

                # EMA
                if self.do_ema:
                    self.rot = (
                        np.array(self.rot) * self.alpha + np.array(self.prev_rot) * (1 - self.alpha)
                    ).tolist()
                    self.pos = (
                        np.array(self.pos) * self.alpha + np.array(self.prev_pos) * (1 - self.alpha)
                    ).tolist()
                    self.ang_v_local = (
                        np.array(self.ang_v_local) * self.alpha
                        + np.array(self.prev_ang_v_local) * (1 - self.alpha)
                    ).tolist()
                    self.ang_v_lab = (
                        np.array(self.ang_v_lab) * self.alpha
                        + np.array(self.prev_ang_v_lab) * (1 - self.alpha)
                    ).tolist()
                    self.lin_a_lab = (
                        np.array(self.lin_a_lab) * self.alpha
                        + np.array(self.prev_lin_a_lab) * (1 - self.alpha)
                    ).tolist()
                    self.lin_a_local = (
                        np.array(self.lin_a_local) * self.alpha
                        + np.array(self.prev_lin_a_local) * (1 - self.alpha)
                    ).tolist()
                    self.lin_v_local = (
                        np.array(self.lin_v_local) * self.alpha
                        + np.array(self.prev_lin_v_local) * (1 - self.alpha)
                    ).tolist()
                    self.lin_v_lab = (
                        np.array(self.lin_v_lab) * self.alpha
                        + np.array(self.prev_ang_v_lab) * (1 - self.alpha)
                    ).tolist()

                self.prev_rot = self.rot
                self.prev_pos = self.pos
                self.prev_ang_v_local = np.array(self.ang_v_local)
                self.prev_ang_v_lab = np.array(self.ang_v_lab)
                self.prev_lin_a_local = np.array(self.lin_a_local)
                self.prev_lin_a_lab = np.array(self.lin_a_lab)
                self.prev_lin_v_local = np.array(self.lin_v_local)
                self.prev_lin_v_lab = np.array(self.lin_v_lab)
                self.prev_time = msg_stamp

                #self.data[self.name].msg_frame_id = msg.header.frame_id
                self.data[self.name].msg_stamp = msg_stamp
                self.data[self.name].quaternion = self.rot
                self.data[self.name].position = self.pos
                self.data[self.name].angular_velocity.lab = self.ang_v_lab
                self.data[self.name].angular_velocity.local = self.ang_v_local
                self.data[self.name].acceleration.lab = self.lin_a_lab
                self.data[self.name].acceleration.local = self.lin_a_local
                self.data[self.name].linear_velocity.lab = self.lin_v_lab
                self.data[self.name].linear_velocity.local = self.lin_v_local

            else:

                self.data[self.name].quaternion = [0, 0, 0, 1]
                self.data[self.name].position = [0, 0, 0]
                self.data[self.name].angular_velocity.lab = [0, 0, 0]
                self.data[self.name].angular_velocity.local = [0, 0, 0]
                self.data[self.name].acceleration.lab = [0, 0, 0]
                self.data[self.name].acceleration.local = [0, 0, 0]
                self.data[self.name].linear_velocity.lab = [0, 0, 0]
                self.data[self.name].linear_velocity.local = [0, 0, 0]

        # self.imu_publish()
        self.recorder.add_dict_to_buffer(
            {
                #"frame_id": str(msg.header.frame_id),
                "which_topic": "imu_state_estimation_data",
                #"stamp": str(msg_stamp),
                "curr_time": str(curr_time),
                "quaternion": str(self.data[self.name].quaternion),
                "position": str(self.data[self.name].position),
                "angular_velocity.lab": str(self.data[self.name].angular_velocity.lab),
                "angular_velocity.local": str(self.data[self.name].angular_velocity.local),
                "acceleration.lab": str(self.data[self.name].acceleration.lab),
                "acceleration.local": str(self.data[self.name].acceleration.local),
                "linear_velocity.lab": str(self.data[self.name].linear_velocity.lab),
                "linear_velocity.local": str(self.data[self.name].linear_velocity.local),
            }
        )
        self.recorder.flush()
        return self.name, self.data[self.name]
