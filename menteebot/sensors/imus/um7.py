import time

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from um7.srv import Reset

from ..utils.math_utils import *
from .base import BaseIMU

GRAVITY = np.array([0, 0, -9.81])


class UM7(BaseIMU):
    def __init__(self, name, cfg):
        super(UM7, self).__init__(name, cfg)

        self.subscribers = []
        for topic_name in self.cfg.topics:
            self.subscribers.append(rospy.Subscriber(topic_name, Imu, self.imu_callback))

        self.imu_commander = rospy.Subscriber(
            "/imu/commander", String, self.command_imu, queue_size=10
        )
        self.imu_reset = rospy.ServiceProxy("/imu/reset", Reset)
        self.imu_on = True
        self.calibration_mode = True
        self.imu_reset_flag = True
        self.gravity_offset = GRAVITY
        self.calib_len = 100
        self.acc_offset = []
        self.lin_v_local = self.lin_v_lab = np.zeros(3)
        self.step_zero = True

    def imu_callback(self, msg: Imu):

        msg_frame_id = msg.header.frame_id
        msg_stamp = msg.header.stamp.to_time()

        if self.imu_on:

            (
                quat,
                ang_v_local,
                ang_v_lab,
                lin_a_local,
                lin_a_lab,
                lin_v_local,
                lin_v_lab,
            ) = self.update_imu_state(msg)

            self.data[self.name].quaternion = quat
            self.data[self.name].angular_velocity.lab = ang_v_lab
            self.data[self.name].angular_velocity.local = ang_v_local
            self.data[self.name].acceleration.lab = lin_a_lab
            self.data[self.name].acceleration.local = lin_a_local
            self.data[self.name].linear_velocity.lab = lin_v_lab
            self.data[self.name].linear_velocity.local = lin_v_local
            self.data[self.name].msg_frame_id = msg_frame_id
            self.data[self.name].msg_stamp = msg_stamp

        else:

            self.data[self.name].quaternion = [0, 0, 0, 1]
            self.data[self.name].angular_velocity.lab = [0, 0, 0]
            self.data[self.name].angular_velocity.local = [0, 0, 0]
            self.data[self.name].acceleration.lab = [0, 0, 0]
            self.data[self.name].acceleration.local = [0, 0, 0]
            self.data[self.name].linear_velocity.lab = [0, 0, 0]
            self.data[self.name].linear_velocity.local = [0, 0, 0]
            self.data[self.name].msg_frame_id = msg_frame_id
            self.data[self.name].msg_stamp = msg_stamp

        return self.name, self.data[self.name]

    def update_imu_state(self, imu_msg):

        ###############################################################################################
        ### valid for um7 driver params: tf_ned_to_enu = False ; orientation_in_robot_frame = False ###
        ###############################################################################################
        self.time_stamp = time.time()

        quat = [
            imu_msg.orientation.x,
            -imu_msg.orientation.y,
            -imu_msg.orientation.z,
            imu_msg.orientation.w,
        ]

        ang_v_local = [
            imu_msg.angular_velocity.x,
            -imu_msg.angular_velocity.y,
            -imu_msg.angular_velocity.z,
        ]

        ang_v_lab = body_to_inertial_frame(np.array(quat), np.array(ang_v_local)).tolist()

        lin_a = [
            imu_msg.linear_acceleration.x,
            -imu_msg.linear_acceleration.y,
            -imu_msg.linear_acceleration.z,
        ]

        if self.calibration_mode:
            self.gravity_offset = self.acc_offset_calib(lin_a)

        lin_a_local = (
            np.array(lin_a) + inertial_to_body_frame(np.array(quat), self.gravity_offset)
        ).tolist()
        lin_a_lab = (
            body_to_inertial_frame(np.array(quat), np.array(lin_a)) + self.gravity_offset
        ).tolist()

        if self.step_zero:
            self.step_zero = False

        else:
            self.lin_v_local = (
                self.lin_v_local + (self.time_stamp - self.prev_time) * self.prev_lin_a_local
            )
            self.lin_v_lab = (
                self.lin_v_lab + (self.time_stamp - self.prev_time) * self.prev_lin_a_lab
            )

        self.prev_lin_a_local = np.array(lin_a_local)
        self.prev_lin_a_lab = np.array(lin_a_lab)
        self.prev_time = self.time_stamp

        return (
            quat,
            ang_v_local,
            ang_v_lab,
            lin_a_local,
            lin_a_lab,
            self.lin_v_local.tolist(),
            self.lin_v_lab.tolist(),
        )

    def acc_offset_calib(self, lin_a):

        ###########################################################################
        ### imu linear acceleration calibration - IMU must be leveled & static! ###
        ###########################################################################

        if self.imu_reset_flag:
            print("IMU RESET")
            self.imu_reset(zero_gyros=True, reset_ekf=True, set_mag_ref=True)
            self.imu_reset_flag = False
            time.sleep(1)

        if not len(self.acc_offset):
            print(f"IMU CALIBRATION - {self.calib_len} STEPS")

        if len(self.acc_offset) < self.calib_len:
            self.acc_offset.append(lin_a)
            return self.gravity_offset

        else:
            self.acc_offset = np.array(self.acc_offset)
            self.calibration_mode = False
            self.gravity_offset = -self.acc_offset.mean(axis=0)
            # self.gravity_offset[0] = self.gravity_offset[1] = 0
            print(f"GRAVITY offset: {self.gravity_offset}")
            return self.gravity_offset

    def command_imu(self, msg):

        case = msg.data
        if case == "reset":
            self.acc_offset = []
            self.calibration_mode = True
            self.imu_reset_flag = True
            self.lin_v_local = self.lin_v_lab = np.zeros(3)
            self.step_zero = True

        elif case == "imu_off":
            self.imu_on = False
            print("IMU OFF")

        elif case == "imu_on":
            self.imu_on = True
            print("IMU ON")
