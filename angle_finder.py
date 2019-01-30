"""
Gets the tape angles and uses that to calculate distance to the targets

Note that the camera will be mounted 11.25" back from the front of the camera
(including the bumpers).
"""
import argparse
import cv2
import math
import numpy as np

import robovision as rv

lower = 60         # HSV lower hue color
upper = 100        # HSV upper hue color
tolerance = 4   # angle tolerance, in degrees
tape_angle = 14.0  # degrees
tape_separation_distance = 8.0  # distance between tape points in inches
fl = 840     # perceived focal length calculated with distance_calibration.py
last_known_distance = 500  # start assuming target is far away

dist_coeff = np.zeros((4, 1), np.float64)
dist_coeff[0, 0] = -5e-07
dist_coeff[1, 0] = 0.0
dist_coeff[2, 0] = 8.5e-05
dist_coeff[3, 0] = 1.4e-05

cam_matrix = np.eye(3, dtype=np.float32)
cam_matrix[0, 2] = 320.0
cam_matrix[1, 2] = 240.0
cam_matrix[0, 0] = 8   # define focal length x
cam_matrix[1, 1] = 8   # define focal length y


def main():
    global last_known_distance
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--source", required=False,
                    help="Video source, e.g. the webcam number")
    args = vars(ap.parse_args())
    source = args["source"]
    if source is None:
        source = "http://10.15.18.100/mjpg/video.mjpg"
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
    target = rv.Target()
    while True:
        frame = vs.read_frame()
        frame = rv.flatten(frame, cam_matrix, dist_coeff)
        target.set_color_range(lower=(lower, 100, 100), upper=(upper, 255, 255))
        contours = target.get_contours(frame)
        if len(contours) > 1:
            contours, _ = sort_contours(contours, method="left-to-right")
            left_contour = None
            right_contour = None
            left_angle = 0
            right_angle = 0
            left_center = (0, 0)
            for cnt in contours:
                angle = target.get_skew_angle(cnt)
                rect = cv2.minAreaRect(cnt)
                center = rect[0]
                if left_contour is None and math.isclose(angle, tape_angle, abs_tol=tolerance):
                    left_contour = cnt
                    left_angle = angle
                    left_center = center
                    print("left_center: {}".format(left_center))
                elif left_contour is not None and \
                        right_contour is None and \
                        angle < 0 and \
                        left_center[0] < center[0] and \
                        math.isclose(angle, -tape_angle, abs_tol=tolerance):
                    right_contour = cnt
                    right_angle = angle
                    print("right_center: {}".format(center))
            if left_contour is not None and \
                    right_contour is not None and \
                    math.isclose(left_angle + abs(right_angle), tape_angle * 2, abs_tol=tolerance):
                print(left_angle + abs(right_angle))
                cv2.drawContours(frame, [left_contour], -1, (0, 0, 255), 3)
                cv2.drawContours(frame, [right_contour], -1, (0, 255, 0), 3)
                _, left_rightmost, _, _ = target.get_extreme_points(left_contour)
                _, _, right_leftmost, _ = target.get_extreme_points(right_contour)
                print('left angle {}'.format(left_angle))
                print('right angle {}'.format(right_angle))
                point_spacing = right_leftmost[0] - left_rightmost[0]
                # Distance from formula: D’ = (W x F) / P
                dist = tape_separation_distance * fl / point_spacing
                last_known_distance = dist
                print("Distance: {}".format(dist))
            else:
                last_known_distance = 500
        cv2.imshow('CapturedImage', frame)
        # wait for Esc or q key and then exit
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            vs.stop()
            break


def sort_contours(contours, method="left-to-right"):
    reverse = False
    i = 0
    if method == "right-to-left" or method == "bottom-to-top":
        reverse = True
    if method == "top-bottom" or method == "bottom-to-top":
        i = 1
    bounding_boxes = [cv2.boundingRect(c) for c in contours]
    (contours, boundingBoxes) = zip(*sorted(zip(contours, bounding_boxes), key=lambda b: b[1][i], reverse=reverse))
    return contours, bounding_boxes


if __name__ == "__main__":
    main()
