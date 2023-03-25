"""
Log: 23/03/23 The first version of EKF for agri-machinery self-navigation, 
currently a framework. The next step is to tune the filter with IMU and 
GNSS module using realsense2 (https://github.com/IntelRealSense/librealsense) 
and pynmea2 (https://github.com/Knio/pynmea2).


Update date: 23th Mar, 2023 by ychen441
"""

import math
import numpy as np
import matplotlib.pyplot as plt


class EkfMethods:
    """
    EkfMethods class fuses IMU and GNSS data by inertial_pred (prediction)
    and inertial_crc (correction) functions. Additional visual measurements
    are fused with the inertial data using vi_pred and vi_crc functions.
    X and y used in the following functions may be changed according to the practical IMU setting.
    """

    def __init__(self):
        pass

    def body_to_world(self, x_body_curr, y_body_curr, x_world_pst, y_world_pst, theta_curr):
        """
        :param x_body_curr: x-axis position (body), time t
        :param y_body_curr: y-axis position (body), time t
        :param x_world_pst: x-axis position (world), time t-1
        :param y_world_pst: y-axis position (world), time t-1
        :param theta_curr: yaw angle, y-axis (body) vs x-axis (world), time t
        :return: 2D position x and y (world)
        Possibly introduce bias to this function after test.
        """
        r = np.sqrt(x_body_curr ** 2 + y_body_curr ** 2)  # Distance between positions at time t and t+1
        phi = theta_curr + math.atan2(x_body_curr, y_body_curr)  # Find yaw angle in world frame
        x_world_curr = x_world_pst + r * np.cos(phi)  # X-axis position of the vehicle in world frame
        y_world_curr = y_world_pst + r * np.sin(phi)  # Y-axis position of the vehicle in world frame
        return x_world_curr, y_world_curr

    def inertial_pred(self, x_body_pst, y_body_pst, v_x_pst, v_y_pst, a_x_pst, a_y_pst,
                      theta_pst, w_pst, var_a, var_w, cov_pst, T):
        """
        The prediction stage of Inertial EKF.
        :param x_body_pst: x-axis position (body), time t-1
        :param y_body_pst: y-axis position (body), time t-1
        :param v_x_pst: velocity (x, body) time t-1
        :param v_y_pst: velocity (y, body) time t-1
        :param a_x_pst: acceleration (x, body), time t-1
        :param a_y_pst: acceleration (y, body), time t-1
        :param theta_pst: yaw angle, y-axis (body) vs x-axis (world), time t-1
        :param w_pst: angular velocity (yaw), time t-1
        :param var_a: process noise in acceleration
        :param var_w: process noise in angular rate
        :param T: sampling rate
        :param cov_pst: covariance matrix of the input state
        :return: predicted state estimate miu_pred,
                 predicted covariance estimate cov_pred
        """
        # Here we build a dynamic model.
        a_x_curr = a_x_pst  # Const x-axis acceleration
        a_y_curr = a_y_pst  # Const y-axis acceleration
        w_curr = w_pst  # Const angular rate (yaw)

        # Predictions in kinematics
        theta_curr = theta_pst + w_pst * T
        v_x_curr = v_x_pst + a_x_pst * T
        v_y_curr = v_y_pst + a_y_pst * T
        x_body_curr = x_body_pst + v_x_pst * T
        y_body_curr = y_body_pst + v_y_pst * T
        v_pst = np.sqrt(v_x_pst ** 2 + v_y_pst ** 2)  # Body velocity, time t-1
        v_curr = np.sqrt(v_x_curr ** 2 + v_y_curr ** 2)  # Body velocity, time t
        a_curr = np.sqrt(a_x_curr ** 2 + a_y_curr ** 2)  # Cody acceleration

        # Predicted state estimate (6*1) miu_pred
        miu_pred = [x_body_curr, y_body_curr, v_curr, a_curr, theta_curr, w_curr]

        # State transition matrix (6*6) f_pred
        f = [[1, 0, np.cos(theta_pst) * T, 0, -v_pst * T * np.sin(theta_pst)],
             [0, 1, np.sin(theta_pst) * T, 0, v_pst * T * np.cos(theta_pst)],
             [0, 0, 1, T, 0, 0], [0, 0, 0, 1, 0, 0],
             [0, 0, 0, 0, 1, T], [0, 0, 0, 0, 0, 1]]
        """
        Here I use a really nasty way to compute the Jacobian, which is certainly not an appropriate 
        way to derive large-scale matrices in visual-inertial fusion (store landmarks). A possible 
        solution is to write a loop and find the elements' Jacobians in a row-to-column manner.
        """

        # Process noise covariance (6*6) q
        s = [[0, 0], [0, 0], [0, 0], [1, 0], [0, 0], [0, 1]]  # Shaping matrix
        noise_p = [[var_a, 0], [0, var_w]]  # Noise matrix
        q = s * noise_p * np.transpose(s)

        # Predicted covariance estimate (6*6) cov_pred
        cov_pred = f * cov_pst * np.transpose(f) + q

        return miu_pred, cov_pred

    def inertial_crc(self, x_gnss, y_gnss, a_x_msd, a_y_msd, w_msd, var_x, var_y, var_a_m, var_w_m,
                     x_body_temp, y_body_temp, v_temp, a_temp, theta_temp, w_temp, cov_temp):
        """
        The correction stage of Inertial EKF.
        ***Here I add some temporary variables to replace miu_pred and cov_pred to avoid error reports.
        :param x_gnss: x-axis position (GNSS), time t
        :param y_gnss: y-axis position (GNSS), time t
        :param a_x_msd: acceleration measured (x, body), time t
        :param a_y_msd: acceleration measured (y, body), time t
        :param w_msd: angular velocity measured (yaw), time t
        :param var_x: measurement noise in GNSS, x-axis
        :param var_y: measurement noise in GNSS, y-axis
        :param var_a_m: measurement noise in accelerometer
        :param var_w_m: measurement noise in gyroscope
        :return: corrected state estimate miu_crc,
                 corrected covariance estimate cov_crc
        """
        # Observation matrix (4*1) z
        a_msd = np.sqrt(a_x_msd ** 2 + a_y_msd ** 2)
        z = [x_gnss, y_gnss, a_msd, w_msd]

        # Observation model matrix (4*6) h
        h = [[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1]]

        # Innovation of measurement residual (4*1) y
        miu_temp = [x_body_temp, y_body_temp, v_temp, a_temp, theta_temp, w_temp]
        y = z - h * miu_temp

        # Measurement noise covariance matrix (4*4) r
        r = [[var_x, 0, 0, 0], [0, var_y, 0, 0], [0, 0, var_a_m, 0], [0, 0, 0, var_w_m]]

        # Kalman gain (6*4) k
        k = cov_temp * np.transpose(h) * np.linalg.inv(h * cov_temp * np.transpose(h) + r)

        # Corrected state estimate (6*1) miu_crc
        miu_crc = miu_temp + k * y

        # Corrected covariance matrix (6*6) cov_crc
        cov_crc = (np.identity(6) - k * h) * cov_temp

        return miu_crc, cov_crc

