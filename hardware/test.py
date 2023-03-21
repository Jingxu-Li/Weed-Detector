from hardware.can_tool import CANConverter
from hardware.can_tool import BasicFrame
con = CANConverter('192.168.1.62', 8080)
con.send(b'12345678876578374875', ('192.168.1.10', 4001))
con.receive()

BasicFrame('192.168.1.62', 8080)
BasicFrame.change_drive_status(1)
assert BasicFrame.message == [xxxxxx]