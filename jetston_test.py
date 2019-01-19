"""
Script for viewing live camera output from a network (RTSP), USB, or onboard
camera with a Jetson board.
"""
import cv2


def main():
    cam = open_cam('http://10.15.18.100/mjpg/video.mjpg')
    # cam = open_cam(1)  # if this doesn't work, try a 0
    # cam = open_cam('/dev/video1')  # also works, may be /dev/video0
    #cam = open_cam_onboard(1024, 768)  # Use Jetson onboard camera
    if cam.isOpened() is False:
        print('failed to open camera')
        exit()
    while True:
        _, img = cam.read()
        cv2.imshow("frame", img)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            break


def open_cam(device):
    return cv2.VideoCapture(device)


def open_cam_onboard(width, height):
    # On versions of L4T previous to L4T 28.1, flip-method=2
    # Use Jetson onboard camera
    gst_str = ("nvcamerasrc ! video/x-raw(memory:NVMM), "
               "width=(int)2592, height=(int)1458, format=(string)I420, framerate=(fraction)30/1 ! "
               "nvvidconv ! video/x-raw, width=(int){}, height=(int){}, format=(string)BGRx ! "
               "videoconvert ! appsink").format(width, height)
    return cv2.VideoCapture(gst_str, cv2.CAP_GSTREAMER)


if __name__ == "__main__":
    main()