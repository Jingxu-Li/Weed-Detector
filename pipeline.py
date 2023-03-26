'''
main pipeline of sprayer with depth camera
'''
from hardware.gps_tool import GNSS
from hardware.depth_camera import VideoStream
if __name__ == "__main__":
    gnss_receiver = GNSS('COM5', 115200)
    depth_camera = VideoStream()
    while(1):
        print(f"Current lat:{gnss_receiver.lat}")
        print(f"Current lon:{gnss_receiver.lon}")
        print(f"Current acc:{depth_camera.acc}")
        print(f"Current gyro:{depth_camera.gyro}")