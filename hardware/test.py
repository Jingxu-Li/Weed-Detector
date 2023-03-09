from hardware.can_tool import CANConverter

con = CANConverter('127.0.0.1', 8081)
con.receive()