#!/usr/bin/env python3
#
# WARNING: You should only use this approach for testing cscore on platforms that
#          it doesn't support using UsbCamera (Windows or OSX).
#

import cscore as cs
import cv2
import numpy as np

from networktables import NetworkTables


def main():
    empty_image_shape = (120, 160, 3)
    h, w, _ = empty_image_shape
    frame_rate = 15  # default is 7

    camserver = cs.CameraServer.getInstance()
    clawcam = cs.UsbCamera(name="clawcam", path="/dev/v4l/by-path/platform-3530000.xhci-usb-0:2.2:1.0-video-index0")
    # clawcam = cs.UsbCamera(name="clawcam", dev=1)
    # clawcam.setResolution(w, h)
    # clawcam.setFPS(15)
    camserver.addCamera(clawcam)

    drivecam = cs.UsbCamera(name="drivecam", path="/dev/v4l/by-path/platform-3530000.xhci-usb-0:2.1:1.0-video-index0")
    # drivecam = cs.UsbCamera(name="drivecam", dev=2)
    # drivecam.setResolution(w, h)
    drivecam.setFPS(frame_rate)
    camserver.addCamera(drivecam)

    # cap = cv2.VideoCapture(1)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    clawcam_input_stream = camserver.getVideo(camera=clawcam)
    drivecam_input_stream = camserver.getVideo(camera=drivecam)
    clawcam_output_stream = camserver.putVideo("ClawCam", w, h)
    drivecam_output_stream = camserver.putVideo("DriveCam", w, h)

    claw_img = np.zeros(shape=empty_image_shape, dtype=np.uint8)
    drive_img = np.zeros(shape=empty_image_shape, dtype=np.uint8)
    while True:
        claw_success, claw_img = clawcam_input_stream.grabFrame(claw_img)
        if claw_success:
            cv2.rectangle(claw_img, (100, 100), (200, 200), (0, 0, 255), 4)
            clawcam_output_stream.putFrame(claw_img)
        drive_success, drive_img = drivecam_input_stream.grabFrame(drive_img)
        if drive_success:
            drivecam_output_stream.putFrame(drive_img)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    NetworkTables.initialize(server="10.15.18.2")
    main()
