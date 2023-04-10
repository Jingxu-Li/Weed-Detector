"""Run camera, fetch inertial and visual data and save them.

Update date: 10/04/23 by ychen441
"""


import cv2
import time
import numpy as np
from tryCam import rs2stream

# Buffers for unsynchronised spatial measurements
acc_buffer = []
gyro_buffer = []
# Buffers for temporal measurements
t_unsync_acc = []
t_unsync_gyro = []

# Camera initialisation
enable_rgb = True
enable_IMU = True
width = 640
height = 480
cam_rate = 30
accel_rate = 100  # or 200Hz
gyro_rate = 200  # or 400Hz
max_frame_num = 0  # Set maximum frames (if needed)


cam = rs2stream(frame_width=width,
                    frame_height=height,
                    cam_framerate=cam_rate,
                    acc_framerate=accel_rate,
                    gyro_framerate=gyro_rate,
                    enable_rgb=enable_rgb,
                    enable_imu=enable_IMU)

# Set a counter to count frames and corresponding timestamps
counter = 0
t_init = time.time()
t_frame = t_init

# RGB saving configs
rgb_writeto = 'rs2.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
rgbwriter = cv2.VideoWriter(rgb_writeto, fourcc, 30.0, (640, 480), 1)

try:
    while True:
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = cam.run_imu()
        counter += 1
        t_last = t_frame
        t_frame = time.time()
        # Feed spatial and temporal data into their buffers
        acc_buffer.append([acc_x, acc_y, acc_z])
        t_unsync_acc.append(t_last)
        gyro_buffer.append([gyro_x, gyro_y, gyro_z])
        t_unsync_gyro.append(t_last)
        # Save inertial data for temporal sync
        with open("acc_readout.txt", 'a') as f1:
            print("acc:", acc_buffer[counter-1], file=f1)
            print("acc time:", t_unsync_acc[counter-1], file=f1)
        with open("gyro_readout.txt", 'a') as f2:
            print("gyro:", gyro_buffer[counter-1], file=f2)
            print("gyro time:", t_unsync_gyro[counter-1], file=f2)       
        
        # Read RGB frames and save them
        color_img = cam.run_rgb()
        rgbwriter.write(color_img)
        cv2.imshow('rs2', color_img)
        # Break when press "esc"
        key = cv2.waitKey(1)
        if key == 27:
            break
finally:
    rgbwriter.release()
    cam.shutdown()
