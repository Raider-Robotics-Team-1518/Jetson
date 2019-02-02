import cscore as cs
import cv2
import numpy as np
import robovision as rv
from fielddata import FieldData
from multiprocessing import Process
from networktables import NetworkTables
from queue import Empty

SERVER_ADDRESS = "10.15.18.2"  # IP address of the robot


class Camstreamer(Process):
    def __init__(self, targeting_queue=None, stop_pipe=None, **kwargs):
        super(Camstreamer, self).__init__()
        self.targeting_queue = targeting_queue
        self.stop_pipe = stop_pipe
        self.kwargs = kwargs
        NetworkTables.initialize(server=SERVER_ADDRESS)
        self.smartdashboard = NetworkTables.getTable("SmartDashboard")

    def run(self):
        camserver = cs.CameraServer.getInstance()
        camera = cs.UsbCamera(name="lifecam", dev=1)
        camera.setResolution(640, 480)
        camserver.addCamera(camera)

        inputStream = camserver.getVideo()
        outputStream = camserver.putVideo("LifeCam", 640, 480)

        frame = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        while True:
            if self.stop_pipe.poll():
                # try reading from the stop pipe; if it's not empty
                # this block will work, and we'll exit the while
                # loop and terminate the script
                stop = self.stop_pipe.recv()
                if stop == "stop":
                    break
            # if pipe is empty, do the real work of this loop
            # the next two lines will be used to signal the Arduino to control the blinkies
            # alliance = self.smartdashboard.getNumber("alliance")
            # match_started = self.smartdashboard.getNumber("match_started")
            success, frame = inputStream.grabFrame(frame)
            if success:
                cv2.imshow("LifeCam", frame)
                try:
                    field_data = self.targeting_queue.get_nowait()
                    # cv2.rectangle(frame, (100, 100), (200, 200), (0, 0, 255), 4)
                    # use the field data to draw the overlays
                    if field_data.target_in_view is True:
                        if field_data.is_centered is True:
                            # draw a green border around the frame
                            frame = rv.draw_border(frame, (0, 255, 0), thickness=20)
                        elif field_data.is_left is True:
                            # draw red border and arrow
                            frame = rv.draw_border(frame, (0, 0, 255), thickness=20)
                            frame = rv.draw_arrow(frame, (255, 0, 0), direction=90, thickness=40)
                        elif field_data.is_right is True:
                            # draw red border and arrow
                            frame = rv.draw_border(frame, (0, 0, 255), thickness=20)
                            frame = rv.draw_arrow(frame, (255, 0, 0), direction=270, thickness=40)
                except Empty:
                    # exception thrown if there's nothing in the queue to read
                    pass
                outputStream.putFrame(frame)
            cv2.waitKey(1)  # pause for 1 millisecond to not max out the CPU


if __name__ == "__main__":
    print("This script does not support being run from the command line")
    exit()
