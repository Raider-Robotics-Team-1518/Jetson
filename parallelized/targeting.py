"""
Locate the targets, calculates the distance and offset

Note that the camera will be mounted 11.25" back from the front of the camera
(including the bumpers).
"""
import cv2
import math
from multiprocessing import Process
import numpy as np
import robovision as rv
from fielddata import FieldData
from networktables import NetworkTables
import serial

# DEFAULT / STARTING VALUES
SERVER_ADDRESS = "10.15.18.2"  # IP address of the robot
TOLERANCE = 5                  # angle tolerance, in degrees
TAPE_ANGLE = 14.5              # degrees
ANGLE_ADJUSTMENT = 0.8         # calculated angle is off by small amount
TAPE_HEIGHT = 8.0              # distance between tape points in inches
FOCAL_LENGTH = 681             # perceived focal length calculated with distance_calibration.py
TARGET_ASPECT_RATIO = 0.40     # aspect ratio of tape (2 / 5.5 = 0.36)
FOV = 67                       # actual camera FOV
CAMERA_SETBACK = 22.5          # how far back the camera is mounted
CENTER_TOLERANCE = 10          # tolerance for being centered, in pixels
DECK_LENGTH = 5                # deque length for rolling average

BLACK = 0
RED = 1
BLUE = 2
YELLOW = 3
GREEN = 4
RANDOM = 5

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


class Targeting(Process):
    def __init__(self, targeting_queue=None, stop_pipe=None, source="http://10.15.18.11/mjpg/video.mjpg",
                 lower_hsv=60, upper_hsv=100):
        super(Targeting, self).__init__()
        self.targeting_queue = targeting_queue
        self.stop_pipe = stop_pipe
        NetworkTables.initialize(server="10.15.18.2")
        self.smart_dashboard = NetworkTables.getTable("SmartDashboard")
        self.fmsinfo = NetworkTables.getTable("FMSInfo")
        self.distance_deck = rv.Deck(maxlen=5)
        self.offset_deck = rv.Deck(maxlen=5)
        self.camera = cv2.VideoCapture("http://10.15.18.11/mjpg/video.mjpg")
        # self.vs = rv.get_video_stream(source)
        # self.vs.start()
        self.target = rv.Target()
        self.target.set_color_range(lower=(lower_hsv, 100, 100), upper=(upper_hsv, 255, 255))
        self.ser = serial.Serial('/dev/ttyUSB0')
        self.ser.flushInput()

    def run(self):
        while True:
            if self.stop_pipe.poll():
                # try reading from the stop pipe; if it's not empty
                # this block will work, and we'll exit the while
                # loop and terminate the script
                stop = self.stop_pipe.recv()
                if stop == "stop":
                    break
            field_data = FieldData()
            # frame = self.vs.read_frame()
            success, frame = self.camera.read()
            # if frame is None:
            #     print('wut?')
            #     continue
            if success is False:
                print('wut?')
                continue
            # frame = rv.flatten(frame, cam_matrix, dist_coeff)
            contours = self.target.get_contours(frame)
            target_in_view, distance, offset, offset_direction = self.process_contours(contours)
            is_red_alliance, is_game_started = self.get_field_info()

            if is_game_started:
                if target_in_view:
                    if offset_direction == 0:
                        self.set_colors(GREEN)
                    else:
                        self.set_colors(YELLOW)
                else:
                    if is_red_alliance:
                        self.set_colors(RED)
                    else:
                        self.set_colors(BLUE)
            else:
                self.set_colors(RANDOM)

            self.distance_deck.push(distance)
            self.offset_deck.push(offset)
            avg_distance = self.distance_deck.average(precision=1)
            avg_offset = self.offset_deck.average(precision=3)
            field_data.target_in_view = target_in_view
            if target_in_view:
                print("target is in view")
                # write to network tables
                self.smart_dashboard.putNumber("distance", avg_distance)
                self.smart_dashboard.putNumber("offset", avg_offset)
                field_data.distance = avg_distance
                field_data.offset = avg_offset
                if offset_direction == -1:
                    field_data.is_right = False
                    field_data.is_left = True
                    field_data.is_centered = False
                elif offset_direction == 1:
                    field_data.is_right = True
                    field_data.is_left = False
                    field_data.is_centered = False
                else:
                    field_data.is_right = False
                    field_data.is_left = False
                    field_data.is_centered = True
            else:
                print("no target")
                self.smart_dashboard.putNumber("distance", -1)
                self.smart_dashboard.putNumber("offset", -1)
                field_data.distance = -1
                field_data.offset = -1
                field_data.is_right = False
                field_data.is_left = False
                field_data.is_centered = False
            self.targeting_queue.put(field_data)

    def process_contours(self, contours):
        """
        Helper function to process contours, determine key parameters
        and return results to main() script

        :param contours: list of contours as returned from target.get_contours()
        :return: Tuple of target_in_view (Boolean), distance (in same units as
                 TAPE_SEPARATION_DISTANCE), offset (in inches), offset_direction
                 (-1 if bot must move right, 1 if left, 0 if centered)
        """
        target_in_view = False
        distance = -1
        display_distance = 0
        offset = 0
        offset_direction = 0
        if len(contours) > 1:
            contours, _ = self.sort_contours(contours, method="left-to-right")
            left_contour = None
            right_contour = None
            left_angle = 0
            right_angle = 0
            left_center = (0, 0)
            for cnt in contours:
                angle = self.target.get_skew_angle(cnt)
                _, _, w, h = self.target.get_rectangle(cnt)
                aspect_ratio = float(w) / h
                if aspect_ratio < TARGET_ASPECT_RATIO:
                    # filter out non-rectangular contours by their aspect ratio
                    continue
                solidity = self.target.get_solidity(cnt)
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
                _, left_rightmost, _, _ = self.target.get_extreme_points(left_contour)
                _, _, right_leftmost, _ = self.target.get_extreme_points(right_contour)
                point_spacing = right_leftmost[0] - left_rightmost[0]
                # Distance from formula: D’ = (W x F) / P
                distance = (TAPE_HEIGHT * FOCAL_LENGTH / point_spacing)
                display_distance = distance - CAMERA_SETBACK
                px_per_inch = (5801 / 12) / distance
                dist = point_spacing / 2
                pmid = right_leftmost[0] - dist
                diff = 320 - pmid
                # print("pixel difference", diff, 'px_per_inch', px_per_inch)
                offset = diff / px_per_inch / 1.2
                offset_direction = 0
                if distance > 0:
                    # inches = ((np.log(23.75-diff))/(np.log(0.9714))) * 0.0305
                    if diff < -CENTER_TOLERANCE:
                        offset_direction = -1
                        # print("shift to the right", offset, "inches")
                    elif diff > CENTER_TOLERANCE:
                        offset_direction = 1
                        # print("shift to the left", offset, "inches")
                    # else:
                    #     print("you have reached the unattainable center")
                else:
                    distance = 0
        return target_in_view, display_distance, offset, offset_direction

    @staticmethod
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

    def get_field_info(self):
        is_red_alliance = self.fmsinfo.getBoolean('IsRedAlliance')
        is_game_started = self.smart_dashboard.getBoolean('isGameStarted')
        return is_red_alliance, is_game_started

    def set_colors(self, color):
        self.ser.write(str(color).encode())
        self.ser.write(str("\n").encode())


if __name__ == "__main__":
    print("This script does not support being run from the command line")
    exit()
