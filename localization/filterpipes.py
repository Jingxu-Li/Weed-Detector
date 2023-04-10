"""An extended Kalman filter for computing intertial-gps (finished) and 
visual-inertial-gps (ongoing) sensor fusion.

The kinetic model used in ekf prediction is the main body moving vertically 
and horizontally in a bird's-eye view. Inertial-gps part is basically 
finished at present but will possibly be modified if needed later in 
real-time tests.

Updated date: 10/04/23 by ychen441
"""

import math
import numpy as np
import time as time
import matplotlib.pyplot as plt


class Filters:
    """Fuses IMU and gps data by inertial_pred (prediction) and 
    inertial_crc (correction) functions. inertial_crc2 function is 
    to check performances between different measurement states and 
    covariance matrices.
    """

    def __init__(self, x_init, y_init, v_init, a_init, theta_init, w_init):
        """ekf initialisation.

        Attributes: 
        x_init: longitude, initial x-axis position in Cartesian frame
        y_init: latitude, initial y-axis position in Cartesian frame
        v_init: initial velocity
        a_init: initial planar acceleration
        theta_init: initial yaw angle
        w_init: initial angular rate on YAW
        """
        self.theta = theta_init
        self.velocity = v_init
        # State vector
        self.state = np.array(
            [x_init, y_init, self.velocity, a_init, self.theta, w_init])
        # Initial guess for covariance matrix
        self.cov_pred = np.identity(6)

    def inertial_pred(self, var_a, var_w, T):
        """The prediction stage of Inertial EKF.

        Kinetic model computes distance in METRES under Cartesian longitude 
        latitude frame also in METRES.

        Args:
          var_a: accelerometer noise
          var_w: gyroscope noise
          T: sampling interval
        """
        # Updated state
        self.state_pred = np.matmul([[
            1, 0, T * np.cos(self.theta), 0.5 * T * T * np.cos(self.theta), 0,
            0
        ],
                                     [
                                         0, 1, T * np.sin(self.theta),
                                         0.5 * T * T * np.sin(self.theta), 0, 0
                                     ], [0, 0, 1, T, 0, 0], [0, 0, 0, 1, 0, 0],
                                     [0, 0, 0, 0, 1, T], [0, 0, 0, 0, 0, 1]],
                                    self.state)
        # State transition matrix
        f = np.array([[
            1, 0, T * np.cos(self.theta), 0.5 * T * T * np.cos(self.theta),
            -self.velocity * T * np.sin(self.theta),
            -0.5 * T * T * np.sin(self.theta)
        ],
                      [
                          0, 1, T * np.sin(self.theta),
                          0.5 * T * T * np.sin(self.theta),
                          self.velocity * T * np.cos(self.theta),
                          0.5 * T * T * np.cos(self.theta)
                      ], [0, 0, 1, T, 0, 0], [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, T], [0, 0, 0, 0, 0, 1]])
        # Noise covariance
        noise_q = np.array([[var_a, 0], [0, var_w]])
        # shaping matrix
        s = np.array([[0, 0], [0, 0], [0, 0], [1, 0], [0, 0], [0, 1]])
        # Process noise covariance matrix
        q = np.matmul(s, np.matmul(noise_q, np.transpose(s)))
        # Predicted covariance estimate
        self.cov_pred = np.add(
            np.matmul(f, np.matmul(self.cov_pred, np.transpose(f))), q)

    def inertial_crc(self, x_gps, y_gps, a_msd, w_msd, var_x, var_y, var_a_m,
                     var_w_m):
        """The correction stage of Inertial EKF.All measurements are done in 
        current instance.

        Args:
          x_gps: longitude
          y_gps: latitude
          a_msd: acceleration
          w_msd: angular rate in YAW
          var_x: gps module noise
          var_y: gps module noise
          var_a_m: accelerometer noise
          var_w_m: gyroscope noise

        Returns:
          corrected state estimate state_crc,
          corrected covariance estimate cov_crc
        """
        # Observation matrix
        z = np.array([x_gps, y_gps, a_msd, w_msd])
        # Observation model matrix
        h = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1]])
        # Innovation of measurement residual
        y = np.subtract(z, np.matmul(h, self.state_pred))
        # Measurement noise covariance
        noise_r = np.array([[var_x, 0, 0, 0], [0, var_y, 0, 0],
                            [0, 0, var_a_m, 0], [0, 0, 0, var_w_m]])
        # Kalman gain
        k = np.matmul(
            self.cov_pred,
            np.matmul(
                np.transpose(h),
                np.linalg.inv(
                    np.add(
                        np.matmul(h, np.matmul(self.cov_pred,
                                               np.transpose(h))), noise_r))))
        # Corrected state estimate
        state_crc = np.add(self.state_pred, np.matmul(k, y))
        # Corrected covariance matrix
        cov_crc = np.matmul(np.subtract(np.identity(6), np.matmul(k, h)),
                            self.cov_pred)

        return state_crc, cov_crc

    def inertial_crc2(self, x_gps, y_gps, var_x, var_y):
        """Use this approx identical func to check filter 
        performance using alternative settings.

        Args:
          x_gps: longitude
          y_gps: latitude
          var_x: gps module noise
          var_y: gps module noise
          
        Returns:
          corrected state estimate state_crc,
          corrected covariance estimate cov_crc
        """
        # Observation matrix
        z = np.array([x_gps, y_gps])
        # Observation model matrix
        h = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])
        # Innovation of measurement residual (4*1) y
        y = np.subtract(z, np.matmul(h, self.state_pred))
        # Measurement noise covariance
        noise_r = np.array([[var_x, 0], [0, var_y]])
        # Kalman gain
        k = np.matmul(
            self.cov_pred,
            np.matmul(
                np.transpose(h),
                np.linalg.inv(
                    np.add(
                        np.matmul(h, np.matmul(self.cov_pred,
                                               np.transpose(h))), noise_r))))
        # Corrected state estimate
        state_crc = np.add(self.state_pred, np.matmul(k, y))
        # Corrected covariance matrix
        cov_crc = np.matmul(np.subtract(np.identity(6), np.matmul(k, h)),
                            self.cov_pred)

        return state_crc, cov_crc


"""test using sensor data acquired by Sarthak:
https://github.com/smahajan07/sensor-fusion
"""
if __name__ == '__main__':
    import json
    import numpy as np
    import matplotlib.pyplot as plt
    from filterpipes import Filters
    from classOfuncs import helpers

    file_name = 'pos_final.json'
    with open(file_name) as data_file:
        data = json.load(data_file)
        #initialData = data[0]

    # Buffers for plotting elements
    org_lon = []
    org_lat = []
    ekf_lon1 = []
    ekf_lat1 = []
    ekf_lon2 = []
    ekf_lat2 = []

    # Vectorially-added accels
    a_sum = []
    for i in range(0, len(data)):
        accel = math.sqrt(data[i]["abs_east_acc"]**2 +
                          data[i]["abs_north_acc"]**2)
        if (data[i]["vel_east"] * data[i]["vel_north"] < 0):
            accel *= -1
        a_sum.append(accel)

    geo = helpers()
    x_init = geo.Lon2Cartesian(data[0]["gps_lon"])  # degrees to metres
    y_init = geo.Lat2Cartesian(data[0]["gps_lat"])
    v_init = 0
    theta_init = 0
    w_init = data[0]["yaw"]
    state = np.array([x_init, y_init, v_init, theta_init,
                      w_init])  # for inertial_crc
    state2 = np.array([x_init, y_init, v_init, theta_init,
                       w_init])  # for inertial_crc2

    for i in range(1, len(data)):
        if (data[i]["gps_lon"] != 0):
            currentData = data[i]
            filtertest = Filters(x_init=state[0],
                                 y_init=state[1],
                                 v_init=state[2],
                                 a_init=a_sum[0],
                                 theta_init=state[3],
                                 w_init=state[4])
            filtertest2 = Filters(x_init=state2[0],
                                  y_init=state2[1],
                                  v_init=state2[2],
                                  a_init=a_sum[0],
                                  theta_init=state2[3],
                                  w_init=state2[4])

            # Prediction
            t = currentData["timestamp"] - data[i - 1]["timestamp"]
            var_a = 9.80665 * 0.043
            var_w = 0
            pred = filtertest.inertial_pred(var_a=var_a, var_w=var_w, T=t)
            pred2 = filtertest2.inertial_pred(var_a=var_a, var_w=var_w, T=t)

            # Correction
            lon_msd = currentData["gps_lon"]
            lat_msd = currentData["gps_lat"]
            x_gps = geo.Lon2Cartesian(lon_msd)
            y_gps = geo.Lat2Cartesian(lat_msd)
            w_msd = currentData["yaw"]
            var_x = 2
            var_y = 2
            var_a_m = var_a
            var_w_m = var_w
            upd = filtertest.inertial_crc(x_gps=x_gps,
                                          y_gps=y_gps,
                                          a_msd=a_sum[i],
                                          w_msd=w_msd,
                                          var_x=var_x,
                                          var_y=var_y,
                                          var_a_m=var_a_m,
                                          var_w_m=var_w_m)
            state = upd[0]
            upd2 = filtertest2.inertial_crc2(x_gps=x_gps,
                                             y_gps=y_gps,
                                             var_x=var_x,
                                             var_y=var_y)
            state2 = upd2[0]
            ekf_lon1.append(upd[0][0])
            ekf_lat1.append(upd[0][1])
            ekf_lon2.append(upd2[0][0])
            ekf_lat2.append(upd2[0][1])

            i += 1
        else:
            continue

    # Original trajectory
    for i in range(0, len(data)):
        orglonMtrs = geo.Lon2Cartesian(data[i]["gps_lon"])
        orglatMtrs = geo.Lat2Cartesian(data[i]["gps_lat"])
        if (data[i]["gps_lon"] != 0):
            org_lon.append(orglonMtrs)
            org_lat.append(orglatMtrs)

    # Visualisation
    plt.plot(org_lon, org_lat, label='org')
    plt.plot(ekf_lon1, ekf_lat1, label='ekf')
    plt.plot(ekf_lon2, ekf_lat2, label='ekf2')
    plt.legend()

    plt.show()
