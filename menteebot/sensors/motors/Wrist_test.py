#
import menteebot_pybind.pyCandle as pyCandle
# import polish_ros as pCan
#
motor_id_list  = [204,205,206]
motor_dic = {204:{'kd': 5, 'kp': 1000.0, 'max_position': 0.5, 'min_position': -0.5, 'pos_percent_wanted': 0.9, 'soft_limit': 1.0, 'torque_offset': 0},
             205:{'kd': 5, 'kp': 1000.0, 'max_position': 0.5, 'min_position': -0.5, 'pos_percent_wanted': 0.9, 'soft_limit': 1.0, 'torque_offset': 0},
             206:{'kd': 5, 'kp': 1000.0, 'max_position': 0.5, 'min_position': -0.5, 'pos_percent_wanted': 0.9, 'soft_limit': 1.0, 'torque_offset': 0}}

can_obj = pyCandle.CandleHandler(pyCandle.CANdleBaudrate_E.CAN_BAUD_8M, pyCandle.BusType_E.USB, "", True)

can_obj.addMd80s(motor_dic)
can_obj.setModeMd80(motor_id_list, pyCandle.Md80Mode_E.IMPEDANCE)
can_obj.enableAllMotors()

# print("we got ros")