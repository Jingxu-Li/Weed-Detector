import io
import time
import sys
import serial
import pynmea2
from tryCam import rs2stream
from classOfuncs import helpers
from datetime import time as dtime

# Buffers for unsynchronised spatial measurements
gnss_buffer = []
acc_buffer = []
gyro_buffer = []
# Buffers for temporal measurements
t_std = []
t_unsync_acc = []
t_unsync_gyro = []
# Buffers for synchronised data
sync_acc = []
sync_gyro = []

# Stream initialisation
# Primarily check /dev/ttyS* in the terminal. Here I use a pseudo one.
ser = serial.Serial('/dev/ttyS0', 9600, timeout=5.0)
gnss_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
# Read RMC sentence to get full, uniform timestamps.
for sentence in gnss_io:
    try:
        msg = pynmea2.parse(sentence)
        if isinstance(msg, pynmea2.types.talker.RMC):
            lon_raw = msg.longitude
            lon = helpers.Lon2Cartesian(lon_raw)
            lat_raw = msg.latitude
            lat = helpers.Lat2Cartesian(lat_raw)
            t_epoch = msg.datetime.timestamp()
            gnss_buffer.append([lon, lat])
            t_std.append(t_epoch)
    except serial.SerialException as e:
        #print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        #print('Parse error: {}'.format(e))
        continue

# Camera initialisation
enable_rgb = True
enable_IMU = True
width = 1280
height = 800
accel_rate = 63  # or 250Hz
gyro_rate = 200  # or 400Hz
max_frame_num = 0  # Set maximum frames (if needed) that the camera accepts

try:
    cam = rs2stream(frame_width=width,
                    frame_height=height,
                    acc_framerate=accel_rate,
                    gyro_framerate=gyro_rate,
                    enable_rgb=False,
                    enable_imu=True)

    # Set a counter for frames and time
    counter = 0
    t_init = time.time()
    t_frame = t_init

    # Main, grab IMU accel and gyro data and send to buffers.
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
        # Break if reaching maximum_frame_num
        #if max_frame_num > 0:
            #if counter == max_frame_num:
                #break
        #else:
            #continue
            # or time.sleep(0.02)
except:
    cam.shutdown()

# Sync Initialistion
i = 0
j = 0
threshold_acc = 0.2  # pseudo setting
threshold_gyro = 0.05  # pseudo setting
false_counter_acc = 0
false_counter_gyro = 0

# IMU-accelerometer TimeSync
while i < len(t_std):
    while j < len(t_unsync_acc) - 1:
        """Fetch two measurements, compute temporal distance to ref_time, and 
        make a decision about the synchronised equivalent measurement value. 
        A threshold is set to discarded measurements that loosely relate to 
        the reference stamp.

        A false_counter is used here to count how many times both front and 
        back point out of bound, there may be a severe data loss when the 
        counter reaches 3 to 5.
        """
        ref_time = t_std[i]  # GNSS reference timestamp
        unsync_front = t_unsync_acc[j]
        unsync_back = t_unsync_acc[j + 1]
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
            if front_diff < 0.2 and back_diff < 0.2:
                sync_acc_x = front_scale * acc_buffer[j][
                    0] + back_scale * acc_buffer[j + 1][0]
                sync_acc_y = front_scale * acc_buffer[j][
                    1] + back_scale * acc_buffer[j + 1][1]
                sync_acc_z = front_scale * acc_buffer[j][
                    2] + back_scale * acc_buffer[j + 1][2]
                sync_acc.append([sync_acc_x, sync_acc_y, sync_acc_z])
                j += 1
                i += 1
                break
            """When measurement(s) out of threshold, we have two solutions:
            1. choose the one inside our acceptance as equivalent value;
            2. Discard current ref point and move to the next, if continuously 
            discard 3 to 5 points, stop and check hardware issue.

            Currently, the first solution is used in this OR case, and when both 
            front AND back is out of bound, we shutdown the process.
            """
            # front OR back diff surpasses the threshold
            if front_diff >= 0.2 and back_diff < 0.2:
                sync_acc_x = acc_buffer[j + 1][0]
                sync_acc_y = acc_buffer[j + 1][1]
                sync_acc_z = acc_buffer[j + 1][2]
                sync_acc.append([sync_acc_x, sync_acc_y, sync_acc_z])
                j += 1
                i += 1
                break
            if front_diff < 0.2 and back_diff >= 0.2:
                sync_acc_x = acc_buffer[j][0]
                sync_acc_y = acc_buffer[j][1]
                sync_acc_z = acc_buffer[j][2]
                sync_acc.append([sync_acc_x, sync_acc_y, sync_acc_z])
                j += 1
                i += 1
                break
            # front AND back surpasses the threshold
            if front_diff >= 0.2 and back_diff >= 0.2:
                false_counter_acc += 1
                i += 1
                break
        # unsync_front and back both earlier than ref_time
        if unsync_front < ref_time and unsync_back < ref_time:
            j += 1
            break
    # Data loss decision making
    if false_counter_acc == 3:
        sys.exit('accelerometer error.')
    else:
        continue

# IMU-gyroscope TimeSync
while i < len(t_std):
    while j < len(t_unsync_gyro) - 1:
        ref_time = t_std[i]  # GNSS reference timestamp
        unsync_front = t_unsync_gyro[j]
        unsync_back = t_unsync_gyro[j + 1]
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
            if front_diff < 0.2 and back_diff < 0.2:
                sync_gyro_x = front_scale * gyro_buffer[j][
                    0] + back_scale * gyro_buffer[j + 1][0]
                sync_gyro_y = front_scale * gyro_buffer[j][
                    1] + back_scale * gyro_buffer[j + 1][1]
                sync_gyro_z = front_scale * gyro_buffer[j][
                    2] + back_scale * gyro_buffer[j + 1][2]
                sync_gyro.append([sync_gyro_x, sync_gyro_y, sync_gyro_z])
                j += 1
                i += 1
                break
            # front OR back diff surpasses the threshold
            if front_diff >= 0.2 and back_diff < 0.2:
                sync_gyro_x = gyro_buffer[j + 1][0]
                sync_gyro_y = gyro_buffer[j + 1][1]
                sync_gyro_z = gyro_buffer[j + 1][2]
                sync_gyro.append([sync_gyro_x, sync_gyro_y, sync_gyro_z])
                j += 1
                i += 1
                break
            if front_diff < 0.2 and back_diff >= 0.2:
                sync_gyro_x = gyro_buffer[j][0]
                sync_gyro_y = gyro_buffer[j][1]
                sync_gyro_z = gyro_buffer[j][2]
                sync_gyro.append([sync_gyro_x, sync_gyro_y, sync_gyro_z])
                j += 1
                i += 1
                break
            # front AND back surpasses the threshold
            if front_diff >= 0.2 and back_diff >= 0.2:
                false_counter_gyro += 1
                i += 1
                break
        # unsync_front and back both earlier than ref_time
        if unsync_front < ref_time and unsync_back < ref_time:
            j += 1
            break
    # Data loss decision making
    if false_counter_gyro == 3:
        sys.exit('gyroscope error.')
    else:
        continue

    """So now the synchronised data we have:
    GNSS: gnssbuffer([lon,lat]) t_std
    IMU_accel: sync_acc([acc_x, acc_y, acc_y]) t_std
    IMU_gyro: sync_gyro([gyro_x, gyro_y, gyro_z]) t_std

    init_pred: position: lon, lat >> gnssbuffer[0][0], gnssbuffer[0][1];
               accel: acc_x, acc_y >> sync_acc[0][0], sync_acc[0][1];
               yaw: gyro_z >> sync_gyro[0][2];
               velocity & theta = 0
    init_crc: lon, lat >> gnssbuffer[1][0], gnssbuffer[1][1];
         accel >> sync_acc[1][0], sync_acc[1][1];
         gyro >> sync_gyro[1][2]
    
    Then iterates.
    """
