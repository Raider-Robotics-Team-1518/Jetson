import argparse
import cv2
import logging
import math
import numpy as np
import robovision as rv

# DEFAULT / STARTING VALUES
SERVER_ADDRESS = "10.15.18.2"  # IP address of the robot
# I think we could use 'roborio-XXX-frc.local' instead
LOWER = 60         # HSV lower hue color
UPPER = 100        # HSV upper hue color
TOLERANCE = 10   # angle tolerance, in degrees
TAPE_ANGLE = 14.5  # degrees
ANGLE_ADJUSTMENT = 0.8  # calculated angle is off by small amount
TAPE_SEPARATION_DISTANCE = 14.625  # distance between tape points in inches
FOCAL_LENGTH = 480     # perceived focal length calculated with distance_calibration.py
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
    while True:
        frame = vs.read_frame()
        frame = rv.flatten(frame, cam_matrix, dist_coeff)
        contours = target.get_contours(frame)
        cv2.drawContours(frame, contours, 0, (0, 0, 255), 3)
        cv2.imshow("CapturedImage", frame)
        for cnt in contours:
            _, _, w, h = target.get_rectangle(cnt)
            distance = 12 * FOCAL_LENGTH / w
            distance_deck.push(distance)
            print('width (px): ', w, "distance:", distance_deck.average(precision=1))
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            cv2.destroyAllWindows()
            vs.stop()
            break

if __name__ == "__main__":
    main()