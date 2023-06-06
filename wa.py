import queue
import numpy as np
import threading
import time
from copy import deepcopy

from hardware.can_tool import sprayerFrame

q = queue.Queue()
debug = False
def send_to_sprayer():
    while True:
        try:
            cmd = q.get()
            time.sleep(1)
            if not debug:
                con.change_spray_head(cmd)
                con.send_message(server)
                print(f"True put:{cmd}")
            else:
                print(f"Debug put:{cmd}")
        except Exception as e:
            print(e)


if not debug:
    con = sprayerFrame('192.168.1.6', 8001, 0x0202)
server = ('192.168.1.10', 4001)
zero_msg = np.zeros(22, dtype=int)
t = threading.Thread(target=send_to_sprayer, daemon=True)

def set_bits_open(arr, low, high):
    arr[low:high] = 1
    return arr

def set_bits_close(arr, low, high):
    arr[low:high] = 0
    return arr
map_c = {
    0:18,
    1:20,
    2:21,
    3:16,
    4:17,
    5:8,
    6:9,
    7:10,
    8:11,
    9:12,
    10:14,
    11:13,
    12:15,
    13:0,
    14:1,
    15:2,
    16:3,
    17:4,
    18:5,
    19:6,
    20:7
}
if __name__ == "__main__":
    t.start()
    while True:
        try:
            bit = int(input("Bit: ")) 
            # low = map_c[2*bit]
            low = bit
            high = low + 1
            msg = np.zeros(22, dtype=int)
            msg_on = set_bits_open(msg, low, high)
            msg_put = deepcopy(msg_on)
            q.put(msg_put)
            msg_off = set_bits_close(msg, low, high)
            msg_put_off = deepcopy(msg_off)
            q.put(msg_put_off)
        except Exception as e:
            print(e)
        
        