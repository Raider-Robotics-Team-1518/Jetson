import cscore as cs
import cv2
import numpy as np
import robovision as rv
from fielddata import FieldData
from multiprocessing import Process
from networktables import NetworkTables
from queue import Empty

SERVER_ADDRESS = "10.15.18.2"  # IP address of the robot
empty_image_shape = (120, 160, 3)
h, w, _ = empty_image_shape
frame_rate = 15  # default is 7


class Camstreamer(Process):
    def __init__(self, targeting_queue=None, stop_pipe=None, **kwargs):
        super(Camstreamer, self).__init__()
        self.targeting_queue = targeting_queue
        self.stop_pipe = stop_pipe
        self.kwargs = kwargs

    def run(self):
        camserver = cs.CameraServer.getInstance()
        # determine the unique camera names by using the following command
        # in a terminal:
        # find /dev/v4l
        # then use that address in the `path` param
        clawcam = cs.UsbCamera(name="clawcam",
                               path="/dev/v4l/by-path/platform-3530000.xhci-usb-0:2.2:1.0-video-index0")
        clawcam.setResolution(w, h)
        clawcam.setFPS(frame_rate)
        camserver.addCamera(clawcam)

        drivecam = cs.UsbCamera(name="drivecam",
                                path="/dev/v4l/by-path/platform-3530000.xhci-usb-0:2.1:1.0-video-index0")
        drivecam.setResolution(w, h)
        drivecam.setFPS(frame_rate)
        camserver.addCamera(drivecam)

        clawcam_input_stream = camserver.getVideo(camera=clawcam)
        drivecam_input_stream = camserver.getVideo(camera=drivecam)
        clawcam_output_stream = camserver.putVideo("ClawCam", w, h)
        drivecam_output_stream = camserver.putVideo("DriveCam", w, h)

        claw_img = np.zeros(shape=empty_image_shape, dtype=np.uint8)
        drive_img = np.zeros(shape=empty_image_shape, dtype=np.uint8)

        while True:
            if self.stop_pipe.poll():
                # try reading from the stop pipe; if it's not empty
                # this block will work, and we'll exit the while
                # loop and terminate the script
                stop = self.stop_pipe.recv()
                if stop == "stop":
                    break

            # For the claw cam, we just stream the video without modification
            claw_success, claw_img = clawcam_input_stream.grabFrame(claw_img)
            if claw_success:
                clawcam_output_stream.putFrame(claw_img)

            # For the drive cam, we will add the frame and arrow overlays
            drive_success, drive_img = drivecam_input_stream.grabFrame(drive_img)
            if drive_success:
                try:
                    field_data = self.targeting_queue.get_nowait()
                    # use the field data to draw the overlays
                    if field_data.target_in_view is True:
                        if field_data.is_centered is True:
                            # draw a green border around the frame
                            drive_img = rv.draw_border(drive_img, (0, 255, 0), thickness=20)
                        elif field_data.is_left is True:
                            # draw red border and arrow
                            drive_img = rv.draw_border(drive_img, (0, 0, 255), thickness=20)
                            drive_img = rv.draw_arrow(drive_img, (255, 0, 0), direction=90, thickness=20)
                        elif field_data.is_right is True:
                            # draw red border and arrow
                            drive_img = rv.draw_border(drive_img, (0, 0, 255), thickness=20)
                            drive_img = rv.draw_arrow(drive_img, (255, 0, 0), direction=270, thickness=20)
                except Empty:
                    # exception thrown if there's nothing in the queue to read
                    pass
                drivecam_output_stream.putFrame(drive_img)
            cv2.waitKey(1)  # pause for 1 millisecond to not max out the CPU


if __name__ == "__main__":
    print("This script does not support being run from the command line")
    exit()
