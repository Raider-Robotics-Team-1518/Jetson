"""
Get the HSV colors of a region you select from a live image
"""
import argparse
import cv2
import numpy as np
import os
import sys

import robovision as rv

coords = []
drawing = False


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
    # params = rv.load_camera_params('params.pickle')
    cv2.namedWindow('CapturedImage', cv2.WINDOW_NORMAL)
    while True:
        frame = vs.read_frame()
        # frame = rv.flatten(frame, params)
        cv2.imshow('CapturedImage', frame)
        cv2.setMouseCallback('CapturedImage', click_and_measure, frame)
        # wait for Esc or q key and then exit
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            print('Colors measured at coordinates: {}'.format(coords))
            cv2.destroyAllWindows()
            vs.stop()
            break


def click_and_measure(event, x, y, flag, image):
    """
    Callback function, called by OpenCV when the user interacts
    with the window using the mouse. This function will be called
    repeatedly as the user interacts.
    """
    # get access to a couple of global variables we'll need
    global coords, drawing
    if event == cv2.EVENT_LBUTTONDOWN:
        # user has clicked the mouse's left button
        drawing = True
        # save those starting coordinates
        coords = [(x, y)]
    elif event == cv2.EVENT_MOUSEMOVE:
        # user is moving the mouse within the window
        if drawing is True:
            # if we're in drawing mode, we'll draw a green rectangle
            # from the starting x,y coords to our current coords
            clone = image.copy()
            cv2.rectangle(clone, coords[0], (x, y), (0, 255, 0), 2)
            cv2.imshow('CapturedImage', clone)
    elif event == cv2.EVENT_LBUTTONUP:
        # user has released the mouse button, leave drawing mode
        # and crop the photo
        drawing = False
        # save our ending coordinates
        coords.append((x, y))
        if len(coords) == 2:
            # calculate the four corners of our region of interest
            ty, by, tx, bx = coords[0][1], coords[1][1], coords[0][0], coords[1][0]
            # crop the image using array slicing
            roi = image[ty:by, tx:bx]
            height, width = roi.shape[:2]
            if width > 0 and height > 0:
                # make sure roi has height/width to prevent imshow error
                # and show the cropped image in a new window
                print("Height: {}".format(height))
                print("Width: {}".format(width))
                cv2.namedWindow("ROI", cv2.WINDOW_NORMAL)
                cv2.imshow("ROI", roi)
                height, width, _ = np.shape(roi)

class Object(object):
    """
    An empty, generic object constructor required for de-pickling
    the camera parameters file
    """
    pass

if __name__ == "__main__":
    main()
