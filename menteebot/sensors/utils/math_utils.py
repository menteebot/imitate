import numpy as np


def quat_rotate_inverse(q, v):
    q_w = q[-1]
    q_vec = q[:3]
    a = v * (2.0 * q_w**2 - 1.0)
    b = np.cross(q_vec, v, axis=-1) * q_w * 2.0
    c = q_vec * (q_vec.reshape([1, 3]) @ v.reshape([3, 1])).squeeze(-1) * 2.0
    return a - b + c


# Taken from ref_1 - https://redshiftlabs.com.au/wp-content/uploads/2018/02/an-1006-understandingquaternions.pdf
def quat_to_rot_mat(q):

    rot_mat = np.zeros((3, 3))
    a = q[3]
    b = q[0]
    c = q[1]
    d = q[2]
    rot_mat[0, 0] = a**2 + b**2 - c**2 - d**2
    rot_mat[1, 1] = a**2 - b**2 + c**2 - d**2
    rot_mat[2, 2] = a**2 - b**2 - c**2 + d**2
    rot_mat[0, 1] = 2 * b * c - 2 * a * d
    rot_mat[0, 2] = 2 * b * d + 2 * a * c
    rot_mat[1, 0] = 2 * b * c + 2 * a * d
    rot_mat[1, 2] = 2 * c * d - 2 * a * b
    rot_mat[2, 0] = 2 * b * d - 2 * a * c
    rot_mat[2, 1] = 2 * c * d + 2 * a * b

    return rot_mat


def inverse_quat(q):

    inv_quat = -q
    inv_quat[3] *= -1

    return inv_quat


# opposite to the definitions in ref_1 (because of th NED to ENU transform) - empirically tested


def inertial_to_body_frame(q, v):

    v_inertial = quat_to_rot_mat(inverse_quat(q)) @ v

    return v_inertial


def body_to_inertial_frame(q, v):

    v_body = quat_to_rot_mat(q) @ v

    return v_body
