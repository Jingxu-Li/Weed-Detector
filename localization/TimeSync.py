"""Temporal synchronisation of accelerometer, gyroscope and 
rgb frames with reference to gps module.

GPS and Cam inputs are disabled in order to test the sync module 
using hand-written buffer inputs, ref stamps and unsynced stamps.

Updated date: 11/04/23 by ychen441
"""

import io
import sys
import cv2
import time
import serial
import pynmea2
import numpy as np
from cam_funcs import rs2stream
from classOfuncs import helpers

# Buffers for unsynchronised measurements
#gps_buffer = []
#acc_buffer = []
gyro_buffer = np.array([[3, 0, 0], [3, 0, 0], [1, 0, 0], [1, 0, 0], [26, 0, 0],
                        [26, 0, 0], [50, 0, 0], [1, 0, 0], [1, 0, 0],
                        [1, 0, 0]])
acc_buffer = np.array([[5, 0, 0], [5, 0, 0], [1, 0, 0], [1, 0, 0], [13, 0, 0],
                       [13, 0, 0], [25, 0, 0], [1, 0, 0], [1, 0, 0], [1, 0, 0]])

# Buffers for temporal measurements
#t_std = []
#t_unsync = []
t_std = np.array([1, 2, 3, 4, 4.6])
t_unsync = np.array([1.95, 2.05, 2.5, 2.8, 2.95, 3.08, 3.95, 4.2, 4.4, 4.8])

# Buffers for synchronised data
sync_acc = []
sync_gyro = []

"""GPS data (lon, lat, time) fetching
geo = helpers()
ser = serial.Serial('/dev/ttyS0', 9600, timeout=5.0)
gps_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser))

for sentence in gps_io:

    try:
        msg = pynmea2.parse(sentence)
        if isinstance(msg, pynmea2.types.talker.RMC):
            lon_raw = msg.longitude
            lon = geo.Lon2Cartesian(lon_raw)
            lat_raw = msg.latitude
            lat = geo.Lat2Cartesian(lat_raw)
            t_epoch = msg.datetime.timestamp()
            gps_buffer.append([lon, lat])
            t_std.append(t_epoch)

    except serial.SerialException as e:
        #print('Device error: {}'.format(e))
        break

    except pynmea2.ParseError as e:
        #print('Parse error: {}'.format(e))
        continue
        
Camera data (accel, gyro, rgb, time) fetching, already synced
enable_rgb = True
enable_IMU = True
width = 640
height = 480
cam_rate = 30
accel_rate = 100
gyro_rate = 200
max_frame_num = 0

cam = rs2stream(frame_width=width,
                    frame_height=height,
                    cam_framerate=cam_rate,
                    acc_framerate=accel_rate,
                    gyro_framerate=gyro_rate,
                    enable_rgb=enable_rgb,
                    enable_imu=enable_IMU)

counter = 0
t_init = time.time()
t_frame = t_init

rgb_writeto = 'rs2.avi'
fourcc = cv2.VideoWriter_fourcc(*'XVID')
rgbwriter = cv2.VideoWriter(rgb_writeto, fourcc, 30.0, (640, 480), 1)

try:
    while True:
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = cam.run_imu()
        counter += 1
        t_last = t_frame
        t_frame = time.time()
        acc_buffer.append([acc_x, acc_y, acc_z])
        gyro_buffer.append([gyro_x, gyro_y, gyro_z])
        t_unsync.append(t_last)

        # Save measurements and their timestamps
        with open("acc_readout.txt", 'a') as f1:
            print("acc:", acc_buffer[counter-1], file=f1)
            print("acc time:", t_unsync[counter-1], file=f1)
        with open("gyro_readout.txt", 'a') as f2:
            print("gyro:", gyro_buffer[counter-1], file=f2)
            print("gyro time:", t_unsync[counter-1], file=f2)       
        
        color_img = cam.run_rgb()
        rgbwriter.write(color_img)
        cv2.imshow('rs2', color_img)
        key = cv2.waitKey(1)
        if key == 27:
            break
finally:
    rgbwriter.release()
    cam.shutdown()"""

"""Sync IMU w.r.t. GPS module """
# Initialisation
i = 0
j = 0
threshold = 0.1  # pseudo setting fpr test
false_counter = 0  # Check data loss

for i in range(0, len(t_std)):
    ref_time = t_std[i]  # gps reference timestamp
    for j in range(0, len(t_unsync) - 1):
        unsync_front = t_unsync[j]
        unsync_back = t_unsync[j + 1]
        front_diff = ref_time - unsync_front
        back_diff = unsync_back - ref_time
        # Define scales for interpolation
        front_scale = (unsync_back - ref_time) / (unsync_back - unsync_front)
        back_scale = (ref_time - unsync_front) / (unsync_back - unsync_front)
        # unsync_front and back both later than ref_time
        if unsync_front > ref_time and unsync_back > ref_time:
            i += 1
            break
        # unsync_front earlier than ref_time, back later than ref
        if unsync_front < ref_time and unsync_back > ref_time:
            # front and back_diff within the threshold
            if front_diff < threshold and back_diff < threshold:
                sync_acc_x = front_scale * acc_buffer[j][
                    0] + back_scale * acc_buffer[j + 1][0]
                sync_acc_y = front_scale * acc_buffer[j][
                    1] + back_scale * acc_buffer[j + 1][1]
                sync_acc_z = front_scale * acc_buffer[j][
                    2] + back_scale * acc_buffer[j + 1][2]
                sync_gyro_x = front_scale * gyro_buffer[j][
                    0] + back_scale * gyro_buffer[j + 1][0]
                sync_gyro_y = front_scale * gyro_buffer[j][
                    1] + back_scale * gyro_buffer[j + 1][1]
                sync_gyro_z = front_scale * gyro_buffer[j][
                    2] + back_scale * gyro_buffer[j + 1][2]
                sync_acc.append([sync_acc_x, sync_acc_y, sync_acc_z])
                sync_gyro.append([sync_gyro_x, sync_gyro_y, sync_gyro_z])
                i += 1
                break
            # front OR back diff surpasses the threshold
            if front_diff > threshold and back_diff < threshold:
                sync_acc_x = acc_buffer[j + 1][0]
                sync_acc_y = acc_buffer[j + 1][1]
                sync_acc_z = acc_buffer[j + 1][2]
                sync_gyro_x = gyro_buffer[j + 1][0]
                sync_gyro_y = gyro_buffer[j + 1][1]
                sync_gyro_z = gyro_buffer[j + 1][2]
                sync_acc.append([sync_acc_x, sync_acc_y, sync_acc_z])
                sync_gyro.append([sync_gyro_x, sync_gyro_y, sync_gyro_z])
                i += 1
                break
            if front_diff < threshold and back_diff > threshold:
                sync_acc_x = acc_buffer[j][0]
                sync_acc_y = acc_buffer[j][1]
                sync_acc_z = acc_buffer[j][2]
                sync_gyro_x = gyro_buffer[j][0]
                sync_gyro_y = gyro_buffer[j][1]
                sync_gyro_z = gyro_buffer[j][2]
                sync_acc.append([sync_acc_x, sync_acc_y, sync_acc_z])
                sync_gyro.append([sync_gyro_x, sync_gyro_y, sync_gyro_z])
                i += 1
                break
            # front AND back surpasses the threshold
            if front_diff >= threshold and back_diff >= threshold:
                false_counter += 1
                if false_counter == 3:
                    sys.exit('accelerometer error.')
                else:
                    i += 1
                    break
        # unsync_front and back both earlier than ref_time
        if unsync_front < ref_time and unsync_back < ref_time:
            j += 1
            continue

print("Synced accel:", sync_acc)
print("Synced gyro:", sync_gyro)
print("Check nums:", len(sync_acc), "and", len(sync_gyro))
print("Times out of bounds:", false_counter)
