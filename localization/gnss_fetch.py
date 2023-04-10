import io
import serial
import pynmea2
from classOfuncs import helpers

# Buffers for unsynchronised spatial measurements
gnss_buffer = []
# Buffers for temporal measurements
t_std = []

# Stream initialisation
# Primarily check /dev/ttyS* in the terminal. Here I use a pseudo one.
geo = helpers()
ser = serial.Serial('/dev/ttyS0', 9600, timeout=5.0)
gnss_io = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
# Read RMC sentence to get full, uniform timestamps.
for sentence in gnss_io:
    try:
        msg = pynmea2.parse(sentence)
        if isinstance(msg, pynmea2.types.talker.RMC):
            lon_raw = msg.longitude
            lon = geo.Lon2Cartesian(lon_raw)
            lat_raw = msg.latitude
            lat = geo.Lat2Cartesian(lat_raw)
            t_epoch = msg.datetime.timestamp()
            gnss_buffer.append([lon, lat])
            t_std.append(t_epoch)
    except serial.SerialException as e:
        #print('Device error: {}'.format(e))
        break
    except pynmea2.ParseError as e:
        #print('Parse error: {}'.format(e))
        continue