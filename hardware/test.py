from hardware.can_tool import CANConverter

con = CANConverter('192.168.1.62', 8080)
con.send(b'12345678876578374875', ('192.168.1.10', 4001))
con.receive()