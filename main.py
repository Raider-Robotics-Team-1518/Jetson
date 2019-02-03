"""
Gets the tape angles and uses that to calculate distance to the targets

Note that the camera will be mounted 11.25" back from the front of the camera
(including the bumpers).
"""
import argparse
import cv2
import logging
import math
import numpy as np
import robovision as rv
from networktables import NetworkTables

# DEFAULT / STARTING VALUES
SERVER_ADDRESS = "10.15.18.2"  # IP address of the robot
# I think we could use 'roborio-XXX-frc.local' instead
LOWER = 60         # HSV lower hue color
UPPER = 100        # HSV upper hue color
TOLERANCE = 10   # angle tolerance, in degrees
TAPE_ANGLE = 14.5  # degrees
ANGLE_ADJUSTMENT = 0.8  # calculated angle is off by small amount
TAPE_SEPARATION_DISTANCE = 8.0  # distance between tape points in inches
FOCAL_LENGTH = 665     # perceived focal length calculated with distance_calibration.py
TARGET_ASPECT_RATIO = 0.40  # aspect ratio of tape (2 / 5.5 = 0.36)

# CAMERA/LENS PARAMETERS FOR FIELD FLATTENING
dist_coeff = np.zeros((4, 1), np.float64)
dist_coeff[0, 0] = -2e-06
dist_coeff[1, 0] = 0.0
dist_coeff[2, 0] = 6e-06
dist_coeff[3, 0] = 0.0

cam_matrix = np.eye(3, dtype=np.float32)
cam_matrix[0, 2] = 400.0
cam_matrix[1, 2] = 300.0
cam_matrix[0, 0] = 2   # define focal length x
cam_matrix[1, 1] = 2   # define focal length y

# GLOBAL OBJECT CONFIGURATIONS
logging.basicConfig(level=logging.DEBUG)  # needed to get logging info from NetworkTables
NetworkTables.initialize(server=SERVER_ADDRESS)
nettable = NetworkTables.getTable("SmartDashboard")
target = rv.Target()
target.set_color_range(lower=(LOWER, 100, 100), upper=(UPPER, 255, 255))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-s", "--source", required=False,
                    help="Video source, e.g. the webcam number")
    args = vars(ap.parse_args())
    source = args["source"]
    if source is None or source == "":
        source = "http://10.15.18.100/mjpg/video.mjpg"
    vs = rv.get_video_stream(source)
    vs.start()
    cv2.namedWindow('CapturedImage', cv2.WINDOW_NORMAL)
    distance_deck = rv.Deck(maxlen=5)
    l_angle_deck = rv.Deck(maxlen=5)
    r_angle_deck = rv.Deck(maxlen=5)
    while True:
        frame = vs.read_frame()
        frame = rv.flatten(frame, cam_matrix, dist_coeff)
        contours = target.get_contours(frame)
        target_in_view, distance, angle, lr_angle = process_contours(contours,
                                                                     frame,
                                                                     show_preview=True)
        distance_deck.push(distance)
        avg_distance = distance_deck.average(precision=1)
        print("Distance: {}".format(avg_distance))
        l_angle_deck.push(lr_angle[0])
        r_angle_deck.push(lr_angle[1])
        print("Angles: L: {}, R: {}".format(l_angle_deck.average(precision=1), r_angle_deck.average(precision=1)))
        if target_in_view:
            # write to network tables
            nettable.putNumber("distance", avg_distance)
            nettable.putNumber("angle", angle)
        else:
            nettable.putNumber("distance", -1)
            nettable.putNumber("angle", 0)
        # wait for Esc or q key and then exit
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            vs.stop()
            break


def process_contours(contours, frame, show_preview=False):
    """
    Helper function to process contours, determine key parameters
    and return results to main() script

    :param contours: list of contours as returned from target.get_contours()
    :param frame: image (frame of video)
    :param show_preview: Boolean, default is False
    :return: Tuple of target_in_view (Boolean), distance (in same units as
             TAPE_SEPARATION_DISTANCE), angle (robot angle to targets, degrees)
    """
    target_in_view = False
    distance = -1
    angle = 0
    bot_angle = 0
    lr_angle = (-1, -1)
    if len(contours) > 1:
        contours, _ = sort_contours(contours, method="left-to-right")
        left_contour = None
        right_contour = None
        left_angle = 0
        right_angle = 0
        left_center = (0, 0)
        for cnt in contours:
            angle = target.get_skew_angle(cnt)
            _, _, w, h = target.get_rectangle(cnt)
            aspect_ratio = float(w) / h
            if aspect_ratio < TARGET_ASPECT_RATIO:
                # filter out non-rectangular contours by their aspect ratio
                continue
            solidity = target.get_solidity(cnt)
            if solidity < 0.9:
                # non-solid contour is probably not one of the tapes
                continue
            rect = cv2.minAreaRect(cnt)
            center = rect[0]
            if left_contour is None and math.isclose(angle, TAPE_ANGLE, abs_tol=TOLERANCE):
                left_contour = cnt
                left_angle = angle + ANGLE_ADJUSTMENT
                left_center = center
            elif left_contour is not None and \
                    right_contour is None and \
                    angle < 0 and \
                    left_center[0] < center[0] and \
                    math.isclose(angle, -TAPE_ANGLE, abs_tol=TOLERANCE):
                right_contour = cnt
                right_angle = angle - ANGLE_ADJUSTMENT
        if left_contour is not None and \
                right_contour is not None and \
                math.isclose(left_angle + abs(right_angle), TAPE_ANGLE * 2, abs_tol=TOLERANCE):
            target_in_view = True
            _, left_rightmost, _, _ = target.get_extreme_points(left_contour)
            _, _, right_leftmost, _ = target.get_extreme_points(right_contour)
            point_spacing = right_leftmost[0] - left_rightmost[0]
            # Distance from formula: D’ = (W x F) / P
            distance = TAPE_SEPARATION_DISTANCE * FOCAL_LENGTH / point_spacing
            bot_angle = estimate_robot_angle(left_angle, right_angle)
            lr_angle = (left_angle, right_angle)
        if show_preview:
            show_preview_window(frame, left_contour, right_contour)

    return target_in_view, distance, bot_angle, lr_angle


def estimate_robot_angle(left_angle, right_angle):
    """
    Estimate the angle at which the robot is facing the targets

    :param left_angle: Apparent angle of the left target tape
    :param right_angle: Apparent angle of the right target tape
    :return: angle (robot angle to targets, degrees)
    """
    return 0


def sort_contours(contours, method="left-to-right"):
    """
    Sorts contours according to the method specified

    :param contours: list of contours as returned from target.get_contours()
    :param method: string, how to sort
    :return: list of sorted contours
    """
    reverse = False
    i = 0
    if method == "right-to-left" or method == "bottom-to-top":
        reverse = True
    if method == "top-bottom" or method == "bottom-to-top":
        i = 1
    bounding_boxes = [cv2.boundingRect(c) for c in contours]
    contours, boundingBoxes = zip(*sorted(zip(contours, bounding_boxes),
                                          key=lambda b: b[1][i],
                                          reverse=reverse))
    return contours, bounding_boxes


def show_preview_window(frame, left_contour, right_contour):
    """
    Shows an OpenCV window with the contours outlined in red/green

    :param frame: image (frame of video)
    :param left_contour: single contour, associated with left target
    :param right_contour: single contour, associated with right target
    """
    if left_contour is not None:
        cv2.drawContours(frame, [left_contour], -1, (0, 0, 255), 3)
    if right_contour is not None:
        cv2.drawContours(frame, [right_contour], -1, (255, 0, 0), 3)
    cv2.imshow('CapturedImage', frame)


if __name__ == "__main__":
    main()
