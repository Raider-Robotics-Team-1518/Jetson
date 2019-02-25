import cscore as cs
import cv2
from multiprocessing import Process
import numpy as np
import robovision as rv
from networktables import NetworkTables


class Camstreamer(Process):
    def __init__(self, queue=None, stop_queue=None, **kwargs):
        super(Camstreamer, self).__init__()
        self.queue = queue
        self.stop_queue = stop_queue
        self.kwargs = kwargs
        NetworkTables.initialize(server="10.15.18.2")

    def run(self):
        camserver = cs.CameraServer.getInstance()
        camera = cs.UsbCamera(name="lifecam", dev=1)
        camera.setResolution(320, 240)
        camserver.addCamera(camera)

        inputStream = camserver.getVideo()
        outputStream = camserver.putVideo("LifeCam", 320, 240)

        frame = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
        while True:
            if self.stop_queue.empty() is False:
                stop = self.stop_queue.get()
                if stop == 1:
                    break
            success, frame = inputStream.grabFrame(frame)
            if success:
                if self.queue.empty() is False:
                    field_data = self.queue.get()
                    # use the field data to draw the overlays
                    if field_data["target_in_view"] is True:
                        if field_data["is_centered"] is True:
                            # draw a green border around the frame
                            frame = rv.draw_border(frame, (0, 255, 0), thickness=20)
                        elif field_data["is_left"] is True:
                            # draw red border and arrow
                            frame = rv.draw_border(frame, (0, 0, 255), thickness=20)
                            frame = rv.draw_arrow(frame, (255, 0, 0), direction=90, thickness=40)
                        elif field_data["is_right"] is True:
                            # draw red border and arrow
                            frame = rv.draw_border(frame, (0, 0, 255), thickness=20)
                            frame = rv.draw_arrow(frame, (255, 0, 0), direction=270, thickness=40)
                outputStream.putFrame(frame)
            cv2.waitKey(1)  # pause for 1 millisecond to not max out the CPU


if __name__ == "__main__":
    print("This script does not support being run from the command line")
    exit()
