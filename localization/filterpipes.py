

import math
import numpy as np
import time as time
import matplotlib.pyplot as plt


class Filters:
    """Fuses IMU and GNSS data by inertial_pred (prediction) and 
    inertial_crc (correction) functions. Additional visual measurements
    are fused with the inertial data using vi_pred and vi_crc functions.
    """

    def __init__(self, x_init, y_init, v_init, a_init, theta_init, w_init):
        """Define system and measurement variances.

        Attributes: 
        var_a: system noise, accelerometer
        var_w: system noise, gyroscope
        var_x: measurement noise, longitude
        var_y: measurement noise, latitude
        var_a_m: measurement noise, accelerometer
        var_w_m: measurement noise, gyroscope
        """
        #a_init = math.sqrt(a_x_init**2 + a_y_init**2)
        self.theta = theta_init
        self.velocity = v_init
        # State vector
        self.state = np.array(
            [x_init, y_init, self.velocity, a_init, self.theta, w_init])
        # Initial guess for covariance matrix
        self.cov_pred = np.identity(6)

    def inertial_pred(self, var_a, var_w, T):
        """The prediction stage of Inertial EKF.

        We don't consider an additional body coordinate here in the inertial-GNSS system since 
        we can handle everything using solely the longitude-latitude Cartesian coordinate and 
        also convert the location back to geographical representation (degrees/radians).

        Kinetic model computes distance in METRES under Cartesian longitude 
        latitude frame also in METRES.

        Args:
          T: sampling rate
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
        # State transition matrix (6*6) f
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
        # Process noise covariance (6*6) q
        # Process noise covariance
        noise_q = np.array([[var_a, 0], [0, var_w]])
        s = np.array([[0, 0], [0, 0], [0, 0], [1, 0], [0, 0],
                      [0, 1]])  # Shaping matrix
        q = np.matmul(s, np.matmul(noise_q, np.transpose(s)))

        # Predicted covariance estimate (6*6) cov_pred
        self.cov_pred = np.add(
            np.matmul(f, np.matmul(self.cov_pred, np.transpose(f))), q)

    def inertial_crc(self, x_gnss, y_gnss, a_msd, w_msd, var_x, var_y, var_a_m,
                     var_w_m):
        """The correction stage of Inertial EKF. Data fed into the filter 
        should be synchronised and unit-uniformed, so additional conversions 
        are not considered here. All measurements are done in current instance.

        Args:
          x_gnss: longitude
          y_gnss: latitude
          a_x_msd: acceleration along x-axis
          a_y_msd: acceleration along y-axis
          w_msd: angular velocity (yaw)

        Returns:
          corrected state estimate miu_crc,
          corrected covariance estimate cov_crc
        """
        # Observation matrix (4*1) z
        #a_msd = math.sqrt(a_x_msd**2 + a_y_msd**2)
        z = np.array([x_gnss, y_gnss, a_msd, w_msd])

        # Observation model matrix (4*6) h
        h = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 0, 1]])

        # Innovation of measurement residual (4*1) y
        y = np.subtract(z, np.matmul(h, self.state_pred))

        # Measurement noise covariance
        noise_r = np.array([[var_x, 0, 0, 0], [0, var_y, 0, 0],
                            [0, 0, var_a_m, 0], [0, 0, 0, var_w_m]])

        # Kalman gain (6*4) k
        k = np.matmul(
            self.cov_pred,
            np.matmul(
                np.transpose(h),
                np.linalg.inv(
                    np.add(
                        np.matmul(h, np.matmul(self.cov_pred,
                                               np.transpose(h))), noise_r))))

        # Corrected state estimate (6*1) miu_crc
        state_crc = np.add(self.state_pred, np.matmul(k, y))

        # Corrected covariance matrix (6*6) cov_crc
        cov_crc = np.matmul(np.subtract(np.identity(6), np.matmul(k, h)),
                            self.cov_pred)

        # Replace the former state and its Cov with the corrected one
        #self.state_pred = state_crc
        #self.cov_pred = cov_crc

        #return self.state_pred, self.cov_pred
        return state_crc, cov_crc
    
    def inertial_crc2(self, x_gnss, y_gnss, var_x, var_y):
        """The correction stage of Inertial EKF. Data fed into the filter 
        should be synchronised and unit-uniformed, so additional conversions 
        are not considered here. All measurements are done in current instance.

        Args:
          x_gnss: longitude
          y_gnss: latitude
          a_x_msd: acceleration along x-axis
          a_y_msd: acceleration along y-axis
          w_msd: angular velocity (yaw)

        Returns:
          corrected state estimate miu_crc,
          corrected covariance estimate cov_crc
        """
        # Observation matrix (4*1) z
        #a_msd = math.sqrt(a_x_msd**2 + a_y_msd**2)
        z = np.array([x_gnss, y_gnss])

        # Observation model matrix (4*6) h
        h = np.array([[1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0]])

        # Innovation of measurement residual (4*1) y
        y = np.subtract(z, np.matmul(h, self.state_pred))

        # Measurement noise covariance
        noise_r = np.array([[var_x, 0], [0, var_y]])

        # Kalman gain (6*4) k
        k = np.matmul(
            self.cov_pred,
            np.matmul(
                np.transpose(h),
                np.linalg.inv(
                    np.add(
                        np.matmul(h, np.matmul(self.cov_pred,
                                               np.transpose(h))), noise_r))))

        # Corrected state estimate (6*1) miu_crc
        state_crc = np.add(self.state_pred, np.matmul(k, y))

        # Corrected covariance matrix (6*6) cov_crc
        cov_crc = np.matmul(np.subtract(np.identity(6), np.matmul(k, h)),
                            self.cov_pred)

        # Replace the former state and its Cov with the corrected one
        #self.state_pred = state_crc
        #self.cov_pred = cov_crc

        #return self.state_pred, self.cov_pred
        return state_crc, cov_crc


if __name__ == '__main__':
    from filterpipes import Filters
    from classOfuncs import helpers
    import numpy as np
    import json
    import matplotlib.pyplot as plt

    file_name = 'pos_final.json'
    with open(file_name) as data_file:
        data = json.load(data_file)
        # read initial data
        initialData = data[0]

    a_sum = []
    plot_lon = []
    plot_lat = []
    plot_lon2 = []
    plot_lat2 = []
    plot_orglon = []
    plot_orglat = []
    for i in range(0, len(data)):
        accel = math.sqrt(data[i]["abs_east_acc"]**2 +
                          data[i]["abs_north_acc"]**2)
        if (data[i]["vel_east"] * data[i]["vel_north"] < 0):
            accel *= -1
        a_sum.append(accel)

    geo = helpers()
    x_init = geo.Lon2Cartesian(initialData["gps_lon"])
    y_init = geo.Lat2Cartesian(initialData["gps_lat"])
    v_init = 0
    theta_init = 0
    w_init = initialData["yaw"]
    state = np.array([x_init, y_init, v_init, theta_init, w_init])
    state2 = np.array([x_init, y_init, v_init, theta_init, w_init])

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
            t = currentData["timestamp"] - data[i - 1]["timestamp"]
            var_a = 9.80665 * 0.043
            var_w = 0
            pred = filtertest.inertial_pred(var_a=var_a, var_w=var_w, T=t)
            pred2 = filtertest2.inertial_pred(var_a=var_a, var_w=var_w, T=t)
            lon_msd = currentData["gps_lon"]
            lat_msd = currentData["gps_lat"]
            x_gnss = geo.Lon2Cartesian(lon_msd)
            y_gnss = geo.Lat2Cartesian(lat_msd)
            w_msd = currentData["yaw"]
            var_x = 2
            var_y = 2
            var_a_m = var_a
            var_w_m = var_w
            upd = filtertest.inertial_crc(x_gnss=x_gnss,
                                          y_gnss=y_gnss,
                                          a_msd=a_sum[i],
                                          w_msd=w_msd,
                                          var_x=var_x,
                                          var_y=var_y,
                                          var_a_m=var_a_m,
                                          var_w_m=var_w_m)
            state = upd[0]
            upd2 = filtertest2.inertial_crc2(x_gnss=x_gnss,
                                          y_gnss=y_gnss,
                                          var_x=var_x,
                                          var_y=var_y)
            state2 = upd2[0]

            plot_lon.append(upd[0][0])
            plot_lat.append(upd[0][1])
            plot_lon2.append(upd2[0][0])
            plot_lat2.append(upd2[0][1])

            i += 1
            #print(upd[0][0], upd[0][1])
        else:
            continue

    for i in range(0, len(data)):
        orglonMtrs = geo.Lon2Cartesian(data[i]["gps_lon"])
        orglatMtrs = geo.Lat2Cartesian(data[i]["gps_lat"])
        if (data[i]["gps_lon"] != 0):
            plot_orglon.append(orglonMtrs)
            plot_orglat.append(orglatMtrs)
    
    plt.plot(plot_orglon, plot_orglat, label='org')
    plt.plot(plot_lon, plot_lat, label='ekf')
    plt.plot(plot_lon2, plot_lat2, label='ekf2')
    plt.legend()

    plt.show()
