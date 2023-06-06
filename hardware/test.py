import numpy as np
import time
from can_tool import  driveFrame, sprayerFrame

# con = driveFrame('192.168.1.10', 8080, 0x0201)
# drive_status = 3
# header_percent = 98
# engine_rpm = 0x1111
# turn_angle = 155
# con.change_drive_status(drive_status)
# con.change_header_percent(header_percent)
# con.change_engine_rpm(engine_rpm)
# con.change_turn_angle(turn_angle)
# con.send('192.111.111.11')

def set_bits_open(arr, low, high):
    arr[low:high] = 1
    return arr

def set_bits_close(arr, low, high):
    arr[low:high] = 0
    return arr


low = 1
high = 10

con1 = sprayerFrame('192.168.1.6', 8001, 0x0202)
b = np.zeros(22, dtype=int)
b = set_bits_open(b, low, high)
server = ('192.168.1.10', 4001)
con1.change_spray_head(b)
con1.send_message(server)
time.sleep(2)
b = set_bits_close(b, low, high)
con1.change_spray_head(b)
con1.send_message(server)