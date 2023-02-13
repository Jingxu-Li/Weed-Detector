from dataclasses import dataclass
from queue import Queue


@dataclass
class SensorsDataItem:
    '''
    Data package with timestamp for processing
    '''
    images: list


message_queue = Queue()
