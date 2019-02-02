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
    # camera = cs.UsbCamera("usbcam", 1)
    # camera.setVideoMode(cs.VideoMode.PixelFormat.kMJPEG, 320, 240, 20)

    camserver = cs.CameraServer.getInstance()
    # camera = camserver.startAutomaticCapture()
    camera = cs.UsbCamera(name="lifecam", dev=1)
    camera.setResolution(320, 240)
    camserver.addCamera(camera)

    # cap = cv2.VideoCapture(1)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

    inputStream = camserver.getVideo()
    outputStream = camserver.putVideo("LifeCam", 320, 240)

    img = np.zeros(shape=(240, 320, 3), dtype=np.uint8)
    while True:
        success, img = inputStream.grabFrame(img)
        if success:
            cv2.rectangle(img, (100, 100), (200, 200), (0, 0, 255), 4)
            outputStream.putFrame(img)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            break


if __name__ == "__main__":
    NetworkTables.initialize(server="10.15.18.2")
    main()
