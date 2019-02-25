"""
This class is responsible for I/O with FMS and controlling LEDs based on
field/bot state

"""
from multiprocessing import Process
import threading
from networktables import NetworkTables

SERVER_ADDRESS = "10.15.18.2"  # IP address of the robot


class Nettables(Process):
    def __init__(self, read_queue=None, write_queue=None, stop_queue=None, **kwargs):
        super(Nettables, self).__init__()
        self.read_queue = read_queue
        self.write_queue = write_queue
        self.stop_queue = stop_queue
        self.kwargs = kwargs
        NetworkTables.initialize(server=SERVER_ADDRESS)
        self.smartdashboard = NetworkTables.getTable("SmartDashboard")

    def run(self):
        t = threading.Thread(target=self.ntables)
        t.daemon = True
        t.start()

    def ntables(self):
        # read values from the smartdashboard for use by the python scripts
        field_data = {
            "match_started": self.smartdashboard.getNumber("match_started"),
            "alliance": self.smartdashboard.getNumber("alliance")
        }
        self.read_queue.put(field_data)
        # get data from python and put it to the smartdashboard for use by the rio
        if self.write_queue.empty() is False:
            target = self.write_queue.get()
            self.smartdashboard.putNumber("distance", target["avg_distance"])
            self.smartdashboard.putNumber("offset", target["avg_offset"])
