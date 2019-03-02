"""
This class is responsible for I/O with FMS and controlling LEDs based on
field/bot state

"""
from multiprocessing import Process
from networktables import NetworkTables

SERVER_ADDRESS = "10.15.18.2"  # IP address of the robot


class Nettables(Process):
    def __init__(self, fms_queues=None, stop_pipe=None, **kwargs):
        super(Nettables, self).__init__()
        self.fms_queues = fms_queues
        self.stop_pipe = stop_pipe
        self.kwargs = kwargs
        NetworkTables.initialize(server=SERVER_ADDRESS)
        self.smartdashboard = NetworkTables.getTable("SmartDashboard")

    def run(self):
        while True:
            if self.stop_pipe.poll():
                # try reading from the stop pipe; if it's not empty
                # this block will work, and we'll exit the while
                # loop and terminate the script
                stop = self.stop_pipe.recv()
                if stop == "stop":
                    break
            # read values from the smartdashboard for use by the python scripts
            field_data = {
                "match_started": self.smartdashboard.getNumber("match_started"),
                "alliance": self.smartdashboard.getNumber("alliance")
            }
            for q in self.fms_queues:
                # write the field data out to all the other processes to receive
                # and act upon
                q.put(field_data)
