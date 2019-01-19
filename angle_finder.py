"""
Get the HSV colors of a region you select from a live image
"""
import argparse
import cv2
import numpy as np
import os
import sys

# If you've `git cloned` the repo and are running the examples locally
# you'll need the next line so that Python can find the robovision library
# Otherwise, comment out the sys.path... line
sys.path.append(os.path.dirname(os.path.realpath('.')))
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
            vs = rv.VideoStream(source="ipcam", ipcam_url=source, resolution=(640,480))
    if source is None:
        print("Invalid source")
        exit()
    vs.start()
    cv2.namedWindow('CapturedImage', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('hue_lower', 'CapturedImage', lower, 255, set_lower)
    cv2.createTrackbar('hue_upper', 'CapturedImage', upper, 255, set_upper)
    target = rv.Target()
    while True:
        frame = vs.read_frame()
        target.set_color_range(lower=(lower, 100, 100), upper=(upper, 255, 255))
        contours = target.find_border_contours(frame)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        if len(contours) > 0:
            cv2.drawContours(frame, contours, -1, (0, 0, 255), 3)
            print('angle {}'.format(target.get_skew_angle(contours[0])))
            print('angle {}'.format(target.get_skew_angle(contours[1])))
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

if __name__ == "__main__":
    main()
