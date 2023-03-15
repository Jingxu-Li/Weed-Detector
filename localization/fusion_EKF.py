"""
This is an extended Kalman filter for IMU-GNSS sensor fusion. The IMU has 6 DoF 
with an accelerometer and a gyroscope inside. The future version of this filter
may involve a visual odometry-IMU-GNSS fusion.
version 15/03/23: initial framework, main body to be added.
"""

import numpy as np

# Earth frame-body frame conversion
def gnss_to_body(px_gnss, py_gnss, pz_gnss):
    return px_body, py_body, pz_body
"""The conversion can also be defined more generally, e,g, lat/lon_to_mtrs."""
    
# Radius to degree conversion
def rad_to_deg(latnlon):
    return latnlon * 180 / np.pi
    
# Degree to radius conversion
def deg_to_radius(latnlon):
    return latnlon * np.pi / 180    

"""
Initialisation, i,e, initial 3-axis position, covariance P of the system estimate, process noise and 
measurement noise covariance matrix Q and R, will be programmed as the starting point of the loop that fetches sensor inputs.
"""

"""timestamp get current time, do the calculus, and update timestamp"""

"""
    # Dynamic model
    # 3-axis position
    px_next = px_now + vx_now * delta_t + 0.5 * ax_now * delta_t * delta_t
    py_next = py_now + vy_now * delta_t + 0.5 * ay_now * delta_t * delta_t
    pz_next = pz_now + vz_now * delta_t + 0.5 * az_now * delta_t * delta_t
    
    # 3-axis velocity
    vx_next = vx_now + ax_now * gravity * np.sin(phiy_now) * delta_t
    vy_next = vy_now + ay_now * gravity * np.sin(phix_now) * delta_t
    vz_next = vz_now + az_now * gravity * np.cos(phiy_now) * np.cos(phix_now) * delta_t
    
    # 3-axis accelerations by accelerometer with zero mean Gaussian noise.
    ax_next = ax_now + sigma_a * delta_t
    ay_next = ay_now + sigma_a * delta_t
    az_next = az_now + sigma_a * delta_t
    
    # Roll, pitch, yaw
    phix_next = phix_now + wx_now * delta_t
    phiy_next = phiy_now + wy_now * delta_t
    phiz_next = phiz_now + wz_now * delta_t
    
    # 3-axis angular velocity by gyroscope with zero mean Gaussian noise.
    wx_next = wx_now + sigma_w * delta_t
    wy_next = wy_now + sigma_w * delta_t
    wz_next = wz_now + sigma_w * delta_t
"""
    
"""
The dynamic model includes noise in each axis of accelerometer and gyroscope. 
The state vector is built by the expectation of the dynamic model, which means sigma_a and sigma_w are ignored.
sigma_a and sigma_w here are system noise of the dynamic model, tuning them to adjust an appropriate gain.
"""

    
# Prediction
def prediction(px_now, py_now, pz_now, vx_now, vy_now, vz_now, ax_now, ay_now, az_now, sigma_a, 
               phix_now, phiy_now, phiz_now, wx_now, wy_now, wz_now, sigma_w, delta_t):
    
    gravity = 9.8067

    
    # State vector (15 * 1)
    x_pred = [px_now + vx_now * delta_t + 0.5 * ax_now * delta_t * delta_t,
              py_now + vy_now * delta_t + 0.5 * ay_now * delta_t * delta_t,
              pz_now + vz_now * delta_t + 0.5 * az_now * delta_t * delta_t,
              vx_now + ax_now * gravity * np.sin(phiy_now) * delta_t,
              vy_now + ay_now * gravity * np.sin(phix_now) * delta_t,
              vz_now + az_now * gravity * np.cos(phiy_now) * np.cos(phix_now) * delta_t,
              ax_now, ay_now, az_now,
              phix_now + wx_now * delta_t, phiy_now + wy_now * delta_t, phiz_now + wz_now * delta_t,
              wx_now, wy_now, wz_now]
    
    # Partial derivative to get transition matrix a_pred (15 * 15)
    
    # Process noise covariance matrix q_pred (2 * 2)
    q_pred = [[sigma_a * sigma_a, 0],[0, sigma_w * sigma_w]]
    
    # Partial derivativeto get process noise pair matrix g_pred (15 * 2)
    g_pred = [[0, 0, 0, 0, 0, 0, delta_t, delta_t, delta_t, 0, 0, 0, 0, 0, 0],
              [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, delta_t, delta_t, delta_t]]
    
    # Predict system covariance matrix P using A, P_now (P_0), and Q (15 * 15)
    p_pred = a_pred * p_now * a_pred.transpose() + q_pred
    


# Update
def update(px_msd, py_msd, pz_msd, ax_msd, ay_msd, az_msd, wx_msd, wy_msd, wz_msd, 
           sigma_px, sigma_py, sigma_pz, sigma_acc, sigma_gyro):
    
    # Measurement vector z (9 * 1)
    z_upd = [px_msd, py_msd, pz_msd, ax_msd, ay_msd, az_msd, wx_msd, wy_msd, wz_msd]
    """px_msd, py_msd, and pz_msd are 3-axis positions measured by GNSS and converted by gnss_to_body function."""
    
    # Partial derivative to get the transformation matrix h_upd from state vector to measurement vector (9 * 15)
    
    # Measurement noise covariance matrix r_upd (9*9)
    r_upd = [[sigma_px * sigma_px, 0, 0, 0, 0, 0, 0, 0, 0],
             [0, sigma_py * sigma_py, 0, 0, 0, 0, 0, 0, 0], 
             [0, 0, sigma_pz * sigma_pz, 0, 0, 0, 0, 0, 0], 
             [0, 0, 0, sigma_acc * sigma_acc, 0, 0, 0, 0, 0], 
             [0, 0, 0, 0, sigma_acc * sigma_acc, 0, 0, 0, 0], 
             [0, 0, 0, 0, 0, sigma_acc * sigma_acc, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, sigma_gyro * sigma_gyro, 0, 0], 
             [0, 0, 0, 0, 0, 0, 0, sigma_gyro * sigma_gyro, 0], 
             [0, 0, 0, 0, 0, 0, 0, 0, sigma_gyro * sigma_gyro]]
             
    # Kalman gain k_upd (15 * 9)
    k_upd = p_pred * h_upd.transpose() * np.linalg.inv(h_upd * p_pred * h_upd.transpose() + r_upd)
    
    # State update x_upd (15 * 1)
    x_upd = x_pred + k_upd * (z_upd - h_upd * x_pred)
    
    # Covariance update p_upd (15 * 15)
    p_upd = (np.identity(15) - k_upd * h_upd) * p_pred
    

"""Then the system do the iteration"""

