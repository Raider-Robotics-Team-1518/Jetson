"""
Goal of this script was to determine the bot's angle to the wall by
comparing the perceived geometry of the tapes to the actual geometry.

The script functions, but does not provide useful information. More
exploration & research is needed to make this technique work.

rotation vectors are calculated with values like:
RVEC angles: [179.9179794], [3.32004522], [-0.31692297]

However, rotating the camera does not appreciably change any of those
or at least not in a way that appears to be proportional to the real
angle.

"""

import cv2
import numpy as np
import robovision as rv
import math

model_points = np.array([
    (0, -464, 0),
    (109, -305, 0),
    (45, 0, 0),
    (64, -480, 0),
    (371, -305, 0),
    (640, -464, 0),
    (595, 0, 0),
    (576, -480, 0)
], dtype="double")

# {'p1': 6.000000000000001e-05, 'p2': 0.0, 'k1': -2e-06, 'k2': 0.0, 'fl': 2.0, 'center_y': 300.0, 'center_x': 400.0}


dist_coeff = np.zeros((4, 1), np.float64)
dist_coeff[0, 0] = -2e-06
dist_coeff[1, 0] = 0.0
dist_coeff[2, 0] = 6e-05
dist_coeff[3, 0] = 0.0

cam_matrix = np.eye(3, dtype=np.float32)
cam_matrix[0, 2] = 400.0
cam_matrix[1, 2] = 300.0
cam_matrix[0, 0] = 2   # define focal length x
cam_matrix[1, 1] = 2   # define focal length y

LOWER = 60         # HSV lower hue color
UPPER = 100        # HSV upper hue color
TOLERANCE = 10   # angle tolerance, in degrees
TAPE_ANGLE = 14.5  # degrees
TAPE_SEPARATION_DISTANCE = 8.0  # distance between tape points in inches
ANGLE_ADJUSTMENT = 0.8  # calculated angle is off by small amount
FOCAL_LENGTH = 840     # perceived focal length calculated with distance_calibration.py
TARGET_ASPECT_RATIO = 0.40
target = rv.Target()
target.set_color_range(lower=(LOWER, 100, 100), upper=(UPPER, 255, 255))

def main():
    source = "http://10.15.18.100/mjpg/video.mjpg"
    vs = rv.get_video_stream(source)
    vs.start()
    cv2.namedWindow('CapturedImage', cv2.WINDOW_NORMAL)
    while True:
        frame = vs.read_frame()
        frame = rv.flatten(frame, cam_matrix, dist_coeff)
        contours = target.get_contours(frame)
        target_in_view, distance, angle, lr_angle = process_contours(contours,
                                                                     frame,
                                                                     show_preview=True)
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
        right_center = (0, 0)
        imgpts = None
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
                right_center = center
        if left_contour is not None and \
                right_contour is not None and \
                math.isclose(left_angle + abs(right_angle), TAPE_ANGLE * 2, abs_tol=TOLERANCE):
            target_in_view = True
            left_leftmost, left_rightmost, left_topmost, left_bottommost = target.get_extreme_points(left_contour)
            # return leftmost, rightmost, topmost, bottommost
            right_leftmost, right_rightmost, right_topmost, right_bottommost = target.get_extreme_points(right_contour)
            point_spacing = right_leftmost[0] - left_rightmost[0]
            # Distance from formula: D’ = (W x F) / P
            distance = TAPE_SEPARATION_DISTANCE * FOCAL_LENGTH / point_spacing
            bot_angle, imgpts = estimate_robot_angle(left_leftmost, left_rightmost, left_topmost, left_bottommost, right_leftmost, right_rightmost, right_topmost, right_bottommost)
            lr_angle = (left_angle, right_angle)
        if show_preview:
            if imgpts is not None:
                origin = tuple([400, 300])
                frame = cv2.line(frame, origin, tuple(imgpts[0][0].ravel()), (255, 0, 0), 4)
                frame = cv2.line(frame, origin, tuple(imgpts[0][1].ravel()), (0, 255, 0), 4)
                frame = cv2.line(frame, origin, tuple(imgpts[0][2].ravel()), (0, 0, 255), 4)
            show_preview_window(frame, left_contour, right_contour)

    return target_in_view, distance, bot_angle, lr_angle


def estimate_robot_angle(left_leftmost, left_rightmost, left_topmost, left_bottommost, right_leftmost, right_rightmost, right_topmost, right_bottommost):
    """
    Estimate the angle at which the robot is facing the targets

    :param left_angle: Apparent angle of the left target tape yee haw
    :param right_angle: Apparent angle of the right target tape haw yee
    :return: angle (robot angle to targets, degrees) yaw hee
    """
    image_points = np.array([
        left_leftmost,
        left_rightmost,
        left_topmost,
        left_bottommost,
        right_leftmost,
        right_rightmost,
        right_topmost,
        right_bottommost
    ], dtype="double")

    axis = np.float32([[-2, 0, 0], [0, -2, 0], [0, 0, -2]]).reshape(-1, 3)
    (success, rvec, tvec) = cv2.solvePnP(model_points, image_points, cam_matrix, dist_coeff, flags=cv2.SOLVEPNP_ITERATIVE)
    imgpts = cv2.projectPoints(axis, rvec, tvec, cam_matrix, dist_coeff)
    rv0 = rvec[0] * 180/math.pi
    rv1 = rvec[1] * 180/math.pi
    rv2 = rvec[2] * 180/math.pi
    print("RVEC angles: {}, {}, {}".format(rv0, rv1, rv2))
    return rvec[0]*180/math.pi, imgpts


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




if __name__=='__main__':
    main()


