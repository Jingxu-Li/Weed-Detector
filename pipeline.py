'''
main pipeline of sprayer with depth camera
'''

from apscheduler.schedulers.blocking import BlockingScheduler

from hardware.can_tool import sprayerCAN
from detection.detector import detector

det = detector()
sprayer_can = sprayerCAN("COM5")

def core_task():
    det.detect()
    sprayer_can.send_spray(1)

if __name__ == '__main__':
    scheduler = BlockingScheduler()
    scheduler.add_job(core_task, "interval", seconds=2)
    scheduler.start()