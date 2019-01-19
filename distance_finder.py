"""
Get the HSV colors of a region you select from a live image
"""
import argparse
import cv2
import numpy as np
import os
import sys

import robovision as rv  # noqa: E402

lower = 60
upper = 100

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--source", required=False,
                    help="Video source, e.g. the webcam number")
    args = vars(ap.parse_args())
    source = args["source"]
    vs = None
    try:
        webcam = int(source)
        vs = rv.VideoStream(source="webcam", cam_id=webcam)
    except ValueError:
        if source == "picam":
            vs = rv.VideoStream(source="picam")
        else:
            vs = rv.VideoStream(source="ipcam", ipcam_url=source)
    if source is None:
        print("Invalid source")
        exit()
    vs.start()
    cv2.namedWindow('CapturedImage', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('hue_lower', 'CapturedImage', lower, 255, set_lower)
    cv2.createTrackbar('hue_upper', 'CapturedImage', upper, 255, set_upper)
    target = rv.Target()
    params = rv.load_camera_params('params.pickle')
    while True:
        frame = vs.read_frame()
        # frame = rv.flatten(frame, params)
        target.set_color_range(lower=(lower, 100, 100), upper=(upper, 255, 255))
        contours = target.get_contours(frame)
        if len(contours) > 0:
            cv2.drawContours(frame, contours, 0, (0, 0, 255), 3)
            cv2.drawContours(frame, contours, 1, (0, 255, 0), 3)
            print('angle 0 {}'.format(target.get_skew_angle(contours[0])))
            print('angle 1 {}'.format(target.get_skew_angle(contours[1])))
        cv2.imshow('CapturedImage', frame)
        # wait for Esc or q key and then exit
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            vs.stop()
            break


def set_lower(val):
    global lower
    lower = int(val)

def set_upper(val):
    global upper
    upper = int(val)

class Object(object):
    """
    An empty, generic object constructor required for de-pickling
    the camera parameters file
    """
    pass


if __name__ == "__main__":
    main()
