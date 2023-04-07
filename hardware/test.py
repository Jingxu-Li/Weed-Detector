import numpy as np

from can_tool import  driveFrame, sprayerFrame

con = driveFrame('192.168.1.10', 8080, 0x0201)
drive_status = 3
header_percent = 98
engine_rpm = 0x1111
turn_angle = 155
con.change_drive_status(drive_status)
con.change_header_percent(header_percent)
con.change_engine_rpm(engine_rpm)
con.change_turn_angle(turn_angle)
con.send('192.111.111.11')

con1 = sprayerFrame('192.168.1.6', 8001, 0x0202)
b = np.zeros(22, dtype=int)
b[4:8]=1
server = ('192.168.1.10', 4001)
con1.change_spray_head(b)
con1.send_message(server)